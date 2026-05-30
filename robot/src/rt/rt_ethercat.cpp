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
#define PROCESSDATA_INTERVAL_US 10000
#define OP_TIMEOUT_US (10 * 1000 * 1000)
#define PREOP_PRIME_FRAMES 100

#define CAN_CHANNEL_COUNT 4
#define CAN_CHANNEL_SIZE 75
#define CAN_DLC_FIXED 8
#define CAN_DATA_LEN_FIXED 8

#ifndef ECAT_ENABLE_MOTOR_FEEDBACK_LOG
#define ECAT_ENABLE_MOTOR_FEEDBACK_LOG 0
#endif

static char IOmap[4096];                 // SOEM 过程数据映射区（主站PDO内存镜像）
static int expectedWKC;                  // 期望工作计数值（用于链路健康检查）
static volatile int wkc;                 // 当前周期实际工作计数值
static boolean inOP;                     // EtherCAT 网络是否已进入 OP 状态
static bool ecat_initialized = false;    // EtherCAT 初始化是否完成
// Single source of truth for EtherCAT NIC name.
// Change only this value when switching network adapters.
static const char* kDefaultAdapter = "enp86s0";
static const uint16 g_channel_offsets[CAN_CHANNEL_COUNT] = {0, 75, 150, 225};  // 4路CAN在从站PDO内的偏移
static uint8_t g_joint_rr_phase __attribute__((unused)) = 0;                    // 关节轮转发送阶段（控制发送被注释后保留）
static TiBoardData g_latest_board_data[CAN_CHANNEL_COUNT];                      // 每条CAN通道最近一次聚合后的关节状态缓存
static bool g_board_data_inited = false;                                        // 状态缓存是否已清零初始化
static bool g_shutdown_hook_registered = false;                                 // 退出钩子（atexit）是否已注册
static bool g_enable_edge_inited = false;                                       // 是否已初始化使能边沿状态机
static bool g_prev_enable_cmd = false;                                          // 上一次收到的enable状态（来自上层_legsEnabled）
static int g_transition_mode = 0;                                               // 0:无特殊帧 1:ENTER(FC) 2:EXIT(FD)
static int g_transition_step = 0;                                               // 特殊帧序列当前步(0..2)
// 四路CAN发送开关（1=发送, 0=关闭该通道发送）。
// 通道映射：CH1->FR, CH2->FL, CH3->RR, CH4->RL
// 测试单腿时，把其它腿对应开关置0即可（会把该通道DLC改为0）。
static int g_can_ch1_tx_enable = 1;
static int g_can_ch2_tx_enable = 1;
static int g_can_ch3_tx_enable = 1;
static int g_can_ch4_tx_enable = 1;
static std::mutex command_mutex, data_mutex;                                    // 命令区/数据区互斥锁

constexpr float kMitPMin = -12.5f;
constexpr float kMitPMax = 12.5f;
constexpr float kMitVMin = -30.0f;
constexpr float kMitVMax = 30.0f;
constexpr float kMitTMin = -32.0f;
constexpr float kMitTMax = 32.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kKneeOffsetPos = 2.52f;

// 与 SPI 版本保持一致的“安装方向/零位偏置”参数
static const float kAbadSideSign[4] = {-1.f, -1.f, 1.f, 1.f};
static const float kHipSideSign[4] = {-1.f, 1.f, -1.f, 1.f};
static const float kKneeSideSign[4] = {-1.f, 1.f, -1.f, 1.f};
static const float kAbadOffset[4] = {0.f, 0.f, 0.f, 0.f};
static const float kHipOffset[4] = {
    kPi / 2.f - 0.08f, -kPi / 2.f + 0.08f, -kPi / 2.f + 0.08f, kPi / 2.f - 0.08f};
static const float kKneeOffset[4] = {
    kKneeOffsetPos, -kKneeOffsetPos, kKneeOffsetPos, -kKneeOffsetPos};

static float uint_to_float(uint32_t x_int, float x_min, float x_max, int bits) {
  const float span = x_max - x_min;
  const float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static int motor_id_to_joint_idx(uint8_t motor_id) {
  // 电机ID与关节索引映射（每条腿CAN总线一致）：
  // ID1->joint0, ID2->joint1, ID3->joint2
  switch (motor_id) {
    case 1: return 0;
    case 2: return 1;
    case 3: return 2;
    default: return -1;
  }
}

static int channel_to_leg_idx(int channel_idx) {
  // 通道与腿映射（按当前工程约定）：
  // channel0(CAN1)->FR(leg0), channel1(CAN2)->FL(leg1),
  // channel2(CAN3)->RR(leg2), channel3(CAN4)->RL(leg3)
  switch (channel_idx) {
    case 0: return 0;
    case 1: return 1;
    case 2: return 2;
    case 3: return 3;
    default: return -1;
  }
}

static float motor_pos_to_robot_q(int leg_idx, int joint_idx, float motor_q) {
  switch (joint_idx) {
    case 0:
      return (motor_q - kAbadOffset[leg_idx]) * kAbadSideSign[leg_idx];
    case 1:
      return (motor_q - kHipOffset[leg_idx]) * kHipSideSign[leg_idx];
    case 2:
      return (motor_q - kKneeOffset[leg_idx]) * kKneeSideSign[leg_idx];
    default:
      return motor_q;
  }
}

static float motor_vel_to_robot_dq(int leg_idx, int joint_idx, float motor_dq) {
  switch (joint_idx) {
    case 0:
      return motor_dq * kAbadSideSign[leg_idx];
    case 1:
      return motor_dq * kHipSideSign[leg_idx];
    case 2:
      return motor_dq * kKneeSideSign[leg_idx];
    default:
      return motor_dq;
  }
}

static float robot_q_to_motor_pos(int leg_idx, int joint_idx, float robot_q) {
  switch (joint_idx) {
    case 0:
      return (robot_q * kAbadSideSign[leg_idx]) + kAbadOffset[leg_idx];
    case 1:
      return (robot_q * kHipSideSign[leg_idx]) + kHipOffset[leg_idx];
    case 2:
      return (robot_q / kKneeSideSign[leg_idx]) + kKneeOffset[leg_idx];
    default:
      return robot_q;
  }
}

static float robot_dq_to_motor_vel(int leg_idx, int joint_idx, float robot_dq) {
  switch (joint_idx) {
    case 0:
      return robot_dq * kAbadSideSign[leg_idx];
    case 1:
      return robot_dq * kHipSideSign[leg_idx];
    case 2:
      return robot_dq / kKneeSideSign[leg_idx];
    default:
      return robot_dq;
  }
}

static float robot_tau_to_motor_tauff(int leg_idx, int joint_idx, float robot_tau) {
  switch (joint_idx) {
    case 0:
      return robot_tau * kAbadSideSign[leg_idx];
    case 1:
      return robot_tau * kHipSideSign[leg_idx];
    case 2:
      return robot_tau * kKneeSideSign[leg_idx];
    default:
      return robot_tau;
  }
}

static int decode_mit_feedback_with_channel(int channel_idx,
                                            const uint8 data[CAN_DATA_LEN_FIXED],
                                            int* leg_idx, int* joint_idx,
                                            float* q, float* dq, float* tau) {
  if (data == NULL || leg_idx == NULL || joint_idx == NULL ||
      q == NULL || dq == NULL || tau == NULL) {
    return 0;
  }

  const int mapped_leg = channel_to_leg_idx(channel_idx);
  const int mapped_joint = motor_id_to_joint_idx(data[0]);
  if (mapped_leg < 0 || mapped_joint < 0) {
    return 0;
  }

  const uint32_t p_uint = ((uint32_t)data[2]) | (((uint32_t)data[1]) << 8);
  const uint32_t v_uint = ((uint32_t)(data[4] >> 4)) | (((uint32_t)data[3]) << 4);
  const uint32_t t_uint = ((uint32_t)data[5]) | (((uint32_t)(data[4] & 0x0F)) << 8);

  *leg_idx = mapped_leg;
  *joint_idx = mapped_joint;
  *q = uint_to_float(p_uint, kMitPMin, kMitPMax, 16);
  *dq = uint_to_float(v_uint, kMitVMin, kMitVMax, 12);
  *tau = uint_to_float(t_uint, kMitTMin, kMitTMax, 12);
  return 1;
}

#if ECAT_ENABLE_MOTOR_FEEDBACK_LOG
static uint64_t g_joint_q_print_last_ns = 0;  // 关节角打印上次时间戳（1Hz）
static uint64_t g_raw_input_print_last_ns = 0;  // 原始输入PDO打印上次时间戳（1Hz）

static uint64_t monotonic_time_ns_always() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static void print_all_joint_q_once_per_sec(const TiBoardData* data) {
  if (data == NULL) {
    return;
  }

  const uint64_t now_ns = monotonic_time_ns_always();
  if (g_joint_q_print_last_ns != 0 &&
      (now_ns - g_joint_q_print_last_ns) < 1000000000ULL) {
    return;
  }

  g_joint_q_print_last_ns = now_ns;
  printf("[EtherCAT][Q] FR: %.6f %.6f %.6f | FL: %.6f %.6f %.6f | RR: %.6f %.6f %.6f | RL: %.6f %.6f %.6f\n",
         data[0].q[0], data[0].q[1], data[0].q[2],
         data[1].q[0], data[1].q[1], data[1].q[2],
         data[2].q[0], data[2].q[1], data[2].q[2],
         data[3].q[0], data[3].q[1], data[3].q[2]);
}

static void print_raw_input_pdo_once_per_sec(const ec_slavet* slave) {
  if (slave == NULL || slave->inputs == NULL) {
    return;
  }

  const uint64_t now_ns = monotonic_time_ns_always();
  if (g_raw_input_print_last_ns != 0 &&
      (now_ns - g_raw_input_print_last_ns) < 1000000000ULL) {
    return;
  }

  g_raw_input_print_last_ns = now_ns;
  const int raw_len = (slave->Ibytes >= 300) ? 300 : (int)slave->Ibytes;
  const uint8* raw = slave->inputs;
  printf("[EtherCAT][RAW] Slave->Master input PDO (%d bytes):\n", raw_len);
  for (int i = 0; i < raw_len; i++) {
    if ((i % 16) == 0) {
      printf("  [%03d] ", i);
    }
    printf("%02X ", raw[i]);
    if ((i % 16) == 15 || i == raw_len - 1) {
      printf("\n");
    }
  }
}

static void feedback_monitor_tick_print(const TiBoardData* data,
                                        const ec_slavet* slave) {
  print_all_joint_q_once_per_sec(data);
  print_raw_input_pdo_once_per_sec(slave);
}
#else
static void feedback_monitor_tick_print(const TiBoardData* data,
                                        const ec_slavet* slave) {
  (void)data;
  (void)slave;
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
  int ch_enable = 1;

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
  switch (channel) {
    case 1: ch_enable = g_can_ch1_tx_enable; break;
    case 2: ch_enable = g_can_ch2_tx_enable; break;
    case 3: ch_enable = g_can_ch3_tx_enable; break;
    case 4: ch_enable = g_can_ch4_tx_enable; break;
    default: ch_enable = 1; break;
  }

  memset(base, 0x00, CAN_CHANNEL_SIZE);
  write_u32_be(base, can_id);                  // 0~3: CAN ID (big-endian)
  base[4] = ch_enable ? CAN_DLC_FIXED : 0;     // 4: DLC=0 表示该通道本周期不发CAN
  if (ch_enable) {
    memcpy(&base[5], data, CAN_DATA_LEN_FIXED);  // 5~12: 有效负载 8 字节
  }
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
  // 回报帧可能是 6 字节（MIT 常见），不一定固定 8 字节。
  // 这里只做“至少有 6 字节”的校验，随后按实际 DLC 拷贝到本地 8 字节缓冲。
  const uint8 dlc = base[4];
  if (dlc < 6) {
    return 0;
  }

  *can_id = read_u32_be(base);
  memset(data, 0, CAN_DATA_LEN_FIXED);
  const uint8 copy_len = (dlc < CAN_DATA_LEN_FIXED) ? dlc : CAN_DATA_LEN_FIXED;
  memcpy(data, &base[5], copy_len);
  return 1;
}
// 发送ID轮转槽位：0->ID1, 1->ID2, 2->ID3，然后回到0。
static int g_motor_slot = 0;

static void print_tx_frame(const char* prefix, uint32 can_id,
                           const uint8 data[CAN_DATA_LEN_FIXED]) {
  printf("%s ID=0x%08" PRIX32 " DATA=", prefix, can_id);
  for (int i = 0; i < CAN_DATA_LEN_FIXED; i++) {
    printf("%02X", data[i]);
    if (i + 1 < CAN_DATA_LEN_FIXED) {
      printf(" ");
    }
  }
  printf("\n");
}

// 固定场景发送函数：
// 1 个 EtherCAT 从站，4 条 CAN（对应 4 条腿），每条 CAN 上 3 个电机 ID（1/2/3）。
// 每次调用只在 PDO 输出区写入 1 个指定槽位(slot)的 ID 到 4 条 CAN 通道。
// 真正上总线发送由 rt_ethercat_run() 统一调用 ec_send_processdata() 完成。
static void stage_same_frame_to_all_12_motors(const uint8 data[CAN_DATA_LEN_FIXED],
                                              int slot,
                                              const char* tag) {
  if (data == NULL || ec_slavecount < 1 || !ecat_initialized || !inOP) {
    return;
  }

  if (slot < 0 || slot >= mit_motor_protocol::kJointCountPerLeg) {
    return;
  }
  g_motor_slot = slot;

  ec_slavet* slave = &ec_slave[1];
  const uint32 can_id = mit_motor_protocol::kJointCanIds[g_motor_slot];
  for (int leg = 0; leg < 4; leg++) {
    can_send_fixed8(slave, leg + 1, can_id, data);
    if (leg == 0) {
      print_tx_frame("[EtherCAT] TX motor1", can_id, data);
    }
  }

  if (tag != NULL) {
    printf("[EtherCAT] %s staged (slot=%d)\n", tag, slot);
  }
}

static void rt_ethercat_shutdown() {
  // command_mutex.lock();
  // if (ecat_initialized && inOP && ec_slavecount >= 1) {
  //   const uint8 zero_frame[CAN_DATA_LEN_FIXED] = {0};
  //   send_same_frame_to_all_12_motors(zero_frame, "sent zero frame to all motors");
  // }
  // command_mutex.unlock();

  inOP = FALSE;
  ecat_initialized = false;
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
      g_enable_edge_inited = false;
      g_prev_enable_cmd = false;
      g_transition_mode = 0;
      g_transition_step = 0;
      g_motor_slot = 0;
      printf("[EtherCAT] enable-edge state reset\n");
      if (!g_shutdown_hook_registered) {
        atexit(rt_ethercat_shutdown_hook);
        g_shutdown_hook_registered = true;
      }
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

  // 1) 按顺序读取 4 路 CAN 通道的从站输入PDO
  for (int channel_idx = 0; channel_idx < CAN_CHANNEL_COUNT; channel_idx++) {
    uint32 can_id = 0;
    uint8 can_data[CAN_DATA_LEN_FIXED] = {0};
    if (can_recv_fixed8(slave, channel_idx + 1, &can_id, can_data)) {
      // 2) 按协议判断消息类型：
      //    电机回报帧: CAN ID=0, 数据第1字节(data[0])是电机ID
      if (can_id == 0) {
        int leg_idx = -1;
        int joint_idx = -1;
        float q = 0.f, dq = 0.f, tau = 0.f;
        // 3) 解包MIT回报数据，并根据“通道+电机ID”映射到 leg/joint
        if (decode_mit_feedback_with_channel(channel_idx, can_data, &leg_idx,
                                             &joint_idx, &q, &dq, &tau)) {
          // 3.1) 电机坐标 -> 程序关节坐标（迁移 SPI 的 side_sign + offset）
          q = motor_pos_to_robot_q(leg_idx, joint_idx, q);
          dq = motor_vel_to_robot_dq(leg_idx, joint_idx, dq);

          // 4) 将解包后的角度/角速度/电流写入对应腿与关节
          g_latest_board_data[leg_idx].q[joint_idx] = q;
          g_latest_board_data[leg_idx].dq[joint_idx] = dq;
          g_latest_board_data[leg_idx].tau[joint_idx] = tau;

          // Keep these aliases consistent for existing debug/LCM outputs.
          g_latest_board_data[leg_idx].position[joint_idx] = q;
          g_latest_board_data[leg_idx].velocity[joint_idx] = dq;
          g_latest_board_data[leg_idx].force[joint_idx] = tau;
        }
      } else {
        // 兼容旧诊断帧：非电机回报时按原格式记录计数信息
        const int leg_idx = channel_to_leg_idx(channel_idx);
        if (leg_idx >= 0) {
          g_latest_board_data[leg_idx].loop_count_ti = can_id;
          g_latest_board_data[leg_idx].ethercat_count_ti = read_u32_be(can_data);
          g_latest_board_data[leg_idx].microtime_ti = read_u32_be(can_data + 4);
        }
      }
    }

    // 5) 将每条腿聚合后的最新状态复制到输出参数
    const int leg_idx = channel_to_leg_idx(channel_idx);
    if (leg_idx >= 0) {
      data[leg_idx] = g_latest_board_data[leg_idx];
    }
  }

  feedback_monitor_tick_print(data, slave);

  data_mutex.unlock();
}

void rt_ethercat_set_command(TiBoardCommand* command) {
  command_mutex.lock();

  // 边沿逻辑直接使用上层 command.enable（来源于 _legsEnabled）：
  // false->true 发送 ENTER-MODE，true->false 发送 EXIT-MODE。
  // 这样 EtherCAT->CAN 只做转发与边沿动作，不再重复 Runner 里的 count_ini 逻辑。

  if (command != nullptr && ec_slavecount >= 1) {
    ec_slavet* slave = &ec_slave[1];
    bool all_enable_equal = true;
    const bool enable_cmd = (command[0].enable != 0);
    for (int leg = 1; leg < CAN_CHANNEL_COUNT; leg++) {
      if ((command[leg].enable != 0) != enable_cmd) {
        all_enable_equal = false;
        break;
      }
    }
    if (!all_enable_equal) {
      printf("[EtherCAT] Warning: per-leg enable mismatch, using leg0 enable=%d\n",
             (int)enable_cmd);
    }

    if (!g_enable_edge_inited) {
      g_enable_edge_inited = true;
      g_prev_enable_cmd = false;
    }

    if (enable_cmd != g_prev_enable_cmd) {
      g_transition_mode = enable_cmd ? 1 : 2;
      g_transition_step = 0;
      // 与 SPI->CAN 固件一致：收到边沿后立即更新本地 enable 状态。
      g_prev_enable_cmd = enable_cmd;
      printf("[EtherCAT] enable-edge detected, transition=%s\n",
             enable_cmd ? "ENTER(FC)" : "EXIT(FD)");
    }

    // 特殊帧序列执行期间暂停普通控制帧，保证每个周期总线负载稳定。
    if (g_transition_mode != 0) {
      const uint8 enter_motor_mode_frame[CAN_DATA_LEN_FIXED] = {
          0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
      const uint8 exit_motor_mode_frame[CAN_DATA_LEN_FIXED] = {
          0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
      // 与 SPI->CAN 旧固件顺序对齐：
      // enable: 1->3->2; disable: 1->2->3
      const int enable_slot_order[mit_motor_protocol::kJointCountPerLeg] = {0, 2, 1};
      const int disable_slot_order[mit_motor_protocol::kJointCountPerLeg] = {0, 1, 2};
      const int slot = (g_transition_mode == 1)
                           ? enable_slot_order[g_transition_step]
                           : disable_slot_order[g_transition_step];
      stage_same_frame_to_all_12_motors(
          (g_transition_mode == 1) ? enter_motor_mode_frame : exit_motor_mode_frame,
          slot,
          "[transition] special-frame");
      g_transition_step++;
      if (g_transition_step >= mit_motor_protocol::kJointCountPerLeg) {
        g_transition_mode = 0;
        g_transition_step = 0;
        printf("[EtherCAT] transition finished\n");
      }
      (void)slave;
      command_mutex.unlock();
      return;
    }

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
      const float motor_q_des = robot_q_to_motor_pos(leg, joint, leg_cmd.q_des[joint]);
      const float motor_qd_des = robot_dq_to_motor_vel(leg, joint, leg_cmd.qd_des[joint]);
      const float motor_tau_ff = robot_tau_to_motor_tauff(leg, joint, leg_cmd.tau_ff[joint]);

      const float kp_cmd = leg_cmd.enable ? leg_cmd.kp_joint[joint] : 0.f;
      const float kd_cmd = leg_cmd.enable ? leg_cmd.kd_joint[joint] : 0.f;
      const float tau_cmd = leg_cmd.enable ? motor_tau_ff : 0.f;
      // 与 SPI 行为对齐：enable=0 时不发全零目标，仍下发经过方向/零位映射后的目标值；
      // 同时将 kp/kd/tau 置零，避免闭环增益与前馈力矩生效。
      mit_motor_protocol::pack_control_command(
          motor_q_des, motor_qd_des, kp_cmd, kd_cmd, tau_cmd, can_data);

      can_send_fixed8(slave, leg + 1, can_id, can_data);
    }
    g_joint_rr_phase =
        static_cast<uint8_t>((g_joint_rr_phase + 1) % mit_motor_protocol::kJointCountPerLeg);
  }
  (void)command;

  command_mutex.unlock();
}
