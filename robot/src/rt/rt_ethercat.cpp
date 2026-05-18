#include "rt/rt_ethercat.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h> // what is this?
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <mutex>
#include "soem/soem.h"
#include "rt/mit_motor_protocol.h"
#include "SimUtilities/ti_boardcontrol.h"

// SOEM v2 exposes context-based ecx_* APIs. Keep existing call sites by
// bridging to a single process-wide context.
static ecx_contextt ec_context;

#define ec_slave     (ec_context.slavelist)
#define ec_slavecount (ec_context.slavecount)
#define ec_group     (ec_context.grouplist)

static int ec_init(const char* ifname) { return ecx_init(&ec_context, ifname); }
static int ec_config_init(boolean) { return ecx_config_init(&ec_context); }
static int ec_config_map(void* pIOmap) { return ecx_config_map_group(&ec_context, pIOmap, 0); }
static boolean ec_configdc() { return ecx_configdc(&ec_context); }
static uint16 ec_statecheck(uint16 slave, uint16 reqstate, int timeout) { return ecx_statecheck(&ec_context, slave, reqstate, timeout); }
static int ec_writestate(uint16 slave) { return ecx_writestate(&ec_context, slave); }
static int ec_readstate() { return ecx_readstate(&ec_context); }
static int ec_send_processdata() { return ecx_send_processdata(&ec_context); }
static int ec_receive_processdata(int timeout) { return ecx_receive_processdata(&ec_context, timeout); }
static void ec_close() { ecx_close(&ec_context); }


#define EC_TIMEOUTMON 500
#define PROCESSDATA_INTERVAL_US 250
#define OP_TIMEOUT_US (10 * 1000 * 1000)
#define PREOP_PRIME_FRAMES 100

#define CAN_CHANNEL_COUNT 4
#define CAN_CHANNEL_SIZE 75
#define CAN_DLC_FIXED 8
#define CAN_DATA_LEN_FIXED 8

#ifndef ECAT_ENABLE_MOTOR_FEEDBACK_LOG
#define ECAT_ENABLE_MOTOR_FEEDBACK_LOG 1
#endif

static char IOmap[4096];
static int expectedWKC;
static volatile int wkc;
static boolean inOP;
static bool ecat_initialized = false;
// Single source of truth for EtherCAT NIC name.
// Change only this value when switching network adapters.
static const char* kDefaultAdapter = "enp86s0";
static const uint16 g_channel_offsets[CAN_CHANNEL_COUNT] = {0, 75, 150, 225};
static uint8_t g_joint_rr_phase = 0;
static TiBoardData g_latest_board_data[CAN_CHANNEL_COUNT];
static bool g_board_data_inited = false;
static bool g_shutdown_hook_registered = false;
static std::mutex command_mutex, data_mutex;
static bool g_feedback_monitor_active = false;

#if ECAT_ENABLE_MOTOR_FEEDBACK_LOG
static uint32_t g_feedback_rx_count_1s[CAN_CHANNEL_COUNT][mit_motor_protocol::kJointCountPerLeg];
static uint64_t g_feedback_rx_count_total[CAN_CHANNEL_COUNT][mit_motor_protocol::kJointCountPerLeg];
static uint64_t g_feedback_window_start_ns = 0;
#endif

constexpr float kMitPMin = -12.5f;
constexpr float kMitPMax = 12.5f;
constexpr float kMitVMin = -30.0f;
constexpr float kMitVMax = 30.0f;
constexpr float kMitTMin = -32.0f;
constexpr float kMitTMax = 32.0f;

static float uint_to_float(uint32_t x_int, float x_min, float x_max, int bits) {
  const float span = x_max - x_min;
  const float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static int feedback_motor_id_to_joint(uint8_t motor_id) {
  // MIT dog feedback IDs on each CAN leg bus are commonly 1/3/5.
  switch (motor_id) {
    case 1:
      return 0;
    case 2:
      return 1;
    case 3:
      return 2;
    default:
      return -1;
  }
}

static int decode_mit_feedback(const uint8 data[CAN_DATA_LEN_FIXED], int* joint, float* q,
                               float* dq, float* tau) {
  if (data == NULL || joint == NULL || q == NULL || dq == NULL || tau == NULL) {
    return 0;
  }

  const int joint_idx = feedback_motor_id_to_joint(data[0]);
  if (joint_idx < 0) {
    return 0;
  }

  const uint32_t p_uint = ((uint32_t)data[2]) | (((uint32_t)data[1]) << 8);
  const uint32_t v_uint = ((uint32_t)(data[4] >> 4)) | (((uint32_t)data[3]) << 4);
  const uint32_t t_uint = ((uint32_t)data[5]) | (((uint32_t)(data[4] & 0x0F)) << 8);

  *joint = joint_idx;
  *q = uint_to_float(p_uint, kMitPMin, kMitPMax, 16);
  *dq = uint_to_float(v_uint, kMitVMin, kMitVMax, 12);
  *tau = uint_to_float(t_uint, kMitTMin, kMitTMax, 12);
  return 1;
}

#if ECAT_ENABLE_MOTOR_FEEDBACK_LOG
static uint64_t monotonic_time_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static int motor_linear_id(int leg, int joint) {
  return leg * mit_motor_protocol::kJointCountPerLeg + joint + 1;
}

static void feedback_monitor_reset_window() {
  memset(g_feedback_rx_count_1s, 0, sizeof(g_feedback_rx_count_1s));
  g_feedback_window_start_ns = monotonic_time_ns();
}

static void feedback_monitor_on_rx(int leg, int joint) {
  if (!g_feedback_monitor_active) {
    return;
  }
  if (leg < 0 || leg >= CAN_CHANNEL_COUNT || joint < 0 ||
      joint >= mit_motor_protocol::kJointCountPerLeg) {
    return;
  }
  g_feedback_rx_count_1s[leg][joint]++;
  g_feedback_rx_count_total[leg][joint]++;
}

static void feedback_monitor_tick_print() {
  if (!g_feedback_monitor_active) {
    return;
  }

  const uint64_t now_ns = monotonic_time_ns();
  if (g_feedback_window_start_ns == 0) {
    g_feedback_window_start_ns = now_ns;
    return;
  }
  if (now_ns - g_feedback_window_start_ns < 1000000000ULL) {
    return;
  }

  printf("[EtherCAT][FB] 1s rx:");
  for (int leg = 0; leg < CAN_CHANNEL_COUNT; leg++) {
    for (int joint = 0; joint < mit_motor_protocol::kJointCountPerLeg; joint++) {
      const int mid = motor_linear_id(leg, joint);
      printf(" M%02d=%" PRIu32, mid, g_feedback_rx_count_1s[leg][joint]);
    }
  }
  printf("\n");

  for (int leg = 0; leg < CAN_CHANNEL_COUNT; leg++) {
    for (int joint = 0; joint < mit_motor_protocol::kJointCountPerLeg; joint++) {
      if (g_feedback_rx_count_1s[leg][joint] == 0) {
        const int mid = motor_linear_id(leg, joint);
        printf("[EtherCAT][FB] motor %d offline in last 1s (leg=%d joint=%d)\n",
               mid, leg, joint);
      }
    }
  }

  feedback_monitor_reset_window();
}
#endif

// EtherCAT process image for this slave stores CAN ID in big-endian order.
static void write_u32_be(uint8 *dst, uint32 value) {
  dst[0] = (uint8)((value >> 24) & 0xFFU);
  dst[1] = (uint8)((value >> 16) & 0xFFU);
  dst[2] = (uint8)((value >> 8) & 0xFFU);
  dst[3] = (uint8)(value & 0xFFU);
}

static uint32 read_u32_be(const uint8 *src) {
  return ((uint32)src[0] << 24) |
         ((uint32)src[1] << 16) |
         ((uint32)src[2] << 8) |
         (uint32)src[3];
}

static int get_channel_offset(int channel, uint16 *offset) {
  if ((channel < 1) || (channel > CAN_CHANNEL_COUNT)) {
    return 0;
  }
  *offset = g_channel_offsets[channel - 1];
  return 1;
}

static int can_send_fixed8(ec_slavet *slave, int channel, uint32 can_id,
                           const uint8 data[CAN_DATA_LEN_FIXED]) {
  uint16 offset = 0;
  uint8 *base = NULL;

  if ((slave == NULL) || (data == NULL) || (slave->outputs == NULL)) {
    return 0;
  }
  if (!get_channel_offset(channel, &offset)) {
    return 0;
  }
  if (slave->Obytes < (uint32)(offset + CAN_CHANNEL_SIZE)) {
    return 0;
  }

  base = slave->outputs + offset;
  memset(base, 0x00, CAN_CHANNEL_SIZE);
  write_u32_be(base, can_id);                  // 0~3: CAN ID (big-endian)
  base[4] = CAN_DLC_FIXED;                     // 4: DLC
  memcpy(&base[5], data, CAN_DATA_LEN_FIXED);  // 5~12: 有效负载 8 字节
  return 1;
}

static int can_recv_fixed8(const ec_slavet *slave, int channel, uint32 *can_id,
                           uint8 data[CAN_DATA_LEN_FIXED]) {
  uint16 offset = 0;
  const uint8 *base = NULL;

  if ((slave == NULL) || (can_id == NULL) || (data == NULL) || (slave->inputs == NULL)) {
    return 0;
  }
  if (!get_channel_offset(channel, &offset)) {
    return 0;
  }
  if (slave->Ibytes < (uint32)(offset + CAN_CHANNEL_SIZE)) {
    return 0;
  }

  base = slave->inputs + offset;
  if (base[4] != CAN_DLC_FIXED) {
    return 0;
  }

  *can_id = read_u32_be(base);
  memcpy(data, &base[5], CAN_DATA_LEN_FIXED);
  return 1;
}

static void send_raw_frame_all_motors(const uint8 data[CAN_DATA_LEN_FIXED], int full_sweeps,
                                      const char* tag) {
  if (data == NULL || full_sweeps <= 0 || ec_slavecount < 1 || !ecat_initialized || !inOP) {
    return;
  }

  ec_slavet* slave = &ec_slave[1];
  for (int sweep = 0; sweep < full_sweeps; sweep++) {
    for (int joint = 0; joint < mit_motor_protocol::kJointCountPerLeg; joint++) {
      const uint32 can_id = mit_motor_protocol::kJointCanIds[joint];
      for (int leg = 0; leg < CAN_CHANNEL_COUNT; leg++) {
        can_send_fixed8(slave, leg + 1, can_id, data);
      }
      ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
      osal_usleep(PROCESSDATA_INTERVAL_US);
    }
  }

  if (tag != NULL) {
    printf("[EtherCAT] %s done (sweeps=%d)\n", tag, full_sweeps);
  }
}

static void rt_ethercat_shutdown() {
  command_mutex.lock();
  if (ecat_initialized && inOP && ec_slavecount >= 1) {
    const uint8 zero_frame[CAN_DATA_LEN_FIXED] = {0};
    send_raw_frame_all_motors(zero_frame, 3, "sent zero frame to all motors");
  }
  command_mutex.unlock();

  inOP = FALSE;
  ecat_initialized = false;
  g_feedback_monitor_active = false;
  ec_close();
}

static void rt_ethercat_shutdown_hook() { rt_ethercat_shutdown(); }

static void degraded_handler() {
  //shut of gpio enables
  // estop();
  printf("[EtherCAT Error] Logging error...\n");
  time_t current_time = time(NULL);
  char* time_str = ctime(&current_time);
  printf("ESTOP. EtherCAT became degraded at %s.\n", time_str);
  rt_ethercat_shutdown();
  printf("[EtherCAT Error] Stopping RT process.\n");
  exit(0);
}

void rt_ethercat_init() { rt_ethercat_init(kDefaultAdapter); }

void rt_ethercat_init(const char* ifname) {
  const char* adapter = ifname ? ifname : kDefaultAdapter;
  printf("[EtherCAT] Initializing EtherCAT on %s\n", adapter);
  const int op_cycles = OP_TIMEOUT_US / PROCESSDATA_INTERVAL_US;

  int attempt = 0;
  while (true) {
    attempt++;
    inOP = FALSE;
    ecat_initialized = false;
    wkc = 0;

    if (!ec_init(adapter)) {
      printf("[EtherCAT Error] No socket connection on %s.\n", adapter);
      osal_usleep(1000000);
      continue;
    }

    if (ec_config_init(FALSE) <= 0) {
      printf("[EtherCAT Error] No slaves found.\n");
      ec_close();
      osal_usleep(1000000);
      continue;
    }

    int dc_capable = 0;
    for (int i = 1; i <= ec_slavecount; i++) {
      ec_slavet* s = &ec_slave[i];
      s->mbx_l = 0;
      s->mbx_rl = 0;
      if (s->hasdc) {
        dc_capable++;
      }
    }

    ec_context.overlappedMode = TRUE;

    if (ec_config_map(&IOmap) <= 0) {
      printf("[EtherCAT Error] Failed to map process data.\n");
      ec_close();
      osal_usleep(1000000);
      continue;
    }

    if (dc_capable > 0) {
      ec_configdc();
    }

    ec_statecheck(0, EC_STATE_SAFE_OP, OP_TIMEOUT_US);

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

    for (int i = 0; i < PREOP_PRIME_FRAMES; i++) {
      ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
      osal_usleep(PROCESSDATA_INTERVAL_US);
    }

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);

    int chk = op_cycles;
    do {
      ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
      ec_statecheck(0, EC_STATE_OPERATIONAL, PROCESSDATA_INTERVAL_US);
      osal_usleep(PROCESSDATA_INTERVAL_US);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
      inOP = TRUE;
      ecat_initialized = true;
      if (!g_shutdown_hook_registered) {
        atexit(rt_ethercat_shutdown_hook);
        g_shutdown_hook_registered = true;
      }
      const uint8 motor_mode_frame[CAN_DATA_LEN_FIXED] = {
          0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
      command_mutex.lock();
      send_raw_frame_all_motors(motor_mode_frame, 3, "sent motor-mode frame to all motors");
      command_mutex.unlock();
#if ECAT_ENABLE_MOTOR_FEEDBACK_LOG
      memset(g_feedback_rx_count_total, 0, sizeof(g_feedback_rx_count_total));
      feedback_monitor_reset_window();
#endif
      g_feedback_monitor_active = true;
      printf("[EtherCAT] OP reached (attempt %d), WKC=%d\n", attempt, wkc);
      return;
    }

    printf("[EtherCAT Error] OP not reached on attempt %d, state=0x%2.2x\n", attempt, ec_slave[0].state);
    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++) {
      if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
        printf("[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
               i, ec_slave[i].state, ec_slave[i].ALstatuscode,
               ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
      }
    }
    ec_close();
    osal_usleep(1000000);
  }
}

/**@brief EtherCAT errors are measured over this period of loop iterations */
#define K_ETHERCAT_ERR_PERIOD 100

/**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
#define K_ETHERCAT_ERR_MAX 20

static int wkc_err_count = 0;
static int wkc_err_iteration_count = 0;

//initiate etherCAT communication
/** @brief Send and receive data over EtherCAT
 *
 * In Simulation, send data over LCM
 * On the robt, verify the EtherCAT connection is still healthy, send data, receive data, and check for lost packets
 */
void rt_ethercat_run()
{
  if(!ecat_initialized || !inOP)
  {
    printf("[EtherCAT Error] rt_ethercat_run called before OP-ready initialization.\n");
    degraded_handler();
  }

  //check connection
  if(wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
  {
    wkc_err_count = 0;
    wkc_err_iteration_count = 0;
  }
  if(wkc_err_count > K_ETHERCAT_ERR_MAX)
  {
    printf("[EtherCAT Error] Error count too high!\n");
    //program terminates in degraded handler.
    degraded_handler();
  }

  //send
  command_mutex.lock();
  ec_send_processdata();
  command_mutex.unlock();

  //receive
  data_mutex.lock();
  wkc = ec_receive_processdata(EC_TIMEOUTRET);
  data_mutex.unlock();

  //check for dropped packet
  if(wkc < expectedWKC)
  {
    printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
    wkc_err_count++;
  }
  wkc_err_iteration_count++;
}

void rt_ethercat_get_data(TiBoardData* data) {
  data_mutex.lock();

  if (data == nullptr || ec_slavecount < 1) {
    data_mutex.unlock();
    return;
  }

  const ec_slavet* slave = &ec_slave[1];
  if (!g_board_data_inited) {
    memset(g_latest_board_data, 0, sizeof(g_latest_board_data));
    g_board_data_inited = true;
  }

  for (int leg = 0; leg < CAN_CHANNEL_COUNT; leg++) {
    uint32 can_id = 0;
    uint8 can_data[CAN_DATA_LEN_FIXED] = {0};
    if (can_recv_fixed8(slave, leg + 1, &can_id, can_data)) {
      // New MIT feedback format:
      //   CAN ID is 0 for all motors; payload byte0 carries motor ID.
      if (can_id == 0) {
        int joint = -1;
        float q = 0.f, dq = 0.f, tau = 0.f;
        if (decode_mit_feedback(can_data, &joint, &q, &dq, &tau)) {
          #if ECAT_ENABLE_MOTOR_FEEDBACK_LOG
          feedback_monitor_on_rx(leg, joint);
          #endif
          g_latest_board_data[leg].q[joint] = q;
          g_latest_board_data[leg].dq[joint] = dq;
          g_latest_board_data[leg].tau[joint] = tau;

          // Keep these aliases consistent for existing debug/LCM outputs.
          g_latest_board_data[leg].position[joint] = q;
          g_latest_board_data[leg].velocity[joint] = dq;
          g_latest_board_data[leg].force[joint] = tau;
        }
      } else {
        // Backward compatibility for older diagnostic frame payloads.
        g_latest_board_data[leg].loop_count_ti = can_id;
        g_latest_board_data[leg].ethercat_count_ti = read_u32_be(can_data);
        g_latest_board_data[leg].microtime_ti = read_u32_be(can_data + 4);
      }
    }

    // Always publish accumulated state so 3 cycles complete all 12 motors.
    data[leg] = g_latest_board_data[leg];
  }

  #if ECAT_ENABLE_MOTOR_FEEDBACK_LOG
  feedback_monitor_tick_print();
  #endif

  data_mutex.unlock();
}

void rt_ethercat_set_command(TiBoardCommand* command) {
  command_mutex.lock();

  if (command != nullptr && ec_slavecount >= 1) {
    ec_slavet* slave = &ec_slave[1];
// 已通过上层腿索引验证：
//   leg[0]=前右, leg[1]=前左, leg[2]=后右, leg[3]=后左
//（通过 Quadruped::getHipLocation 和 rt_spi 侧符号映射交叉检查）。
// 因此我们假设 EtherCAT-CAN 通道的接线方式为：
//   CAN1->前右, CAN2->前左, CAN3->后右, CAN4->后左
//
// 一个周期每个通道只能发送一个 CAN 帧，因此所有 12 个电机
// 将在 3 个周期内按固定轮转顺序更新：
//   阶段 0：髋关节（id=1），阶段 1：大腿（id=2），阶段 2：小腿（id=3）。
    const int joint = g_joint_rr_phase % mit_motor_protocol::kJointCountPerLeg;
    for (int leg = 0; leg < CAN_CHANNEL_COUNT; leg++) {
      uint8 can_data[CAN_DATA_LEN_FIXED] = {0};
      uint32 can_id = mit_motor_protocol::kJointCanIds[joint];
      const TiBoardCommand& leg_cmd = command[leg];

      if (leg_cmd.enable) {
        mit_motor_protocol::pack_control_command(
            leg_cmd.q_des[joint], leg_cmd.qd_des[joint], leg_cmd.kp_joint[joint],
            leg_cmd.kd_joint[joint], leg_cmd.tau_ff[joint], can_data);
      } else {
        mit_motor_protocol::pack_control_command(0.f, 0.f, 0.f, 0.f, 0.f, can_data);
      }

      can_send_fixed8(slave, leg + 1, can_id, can_data);
    }
    g_joint_rr_phase =
        static_cast<uint8_t>((g_joint_rr_phase + 1) % mit_motor_protocol::kJointCountPerLeg);
  }

  command_mutex.unlock();
}
