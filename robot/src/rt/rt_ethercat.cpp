#include "rt/rt_ethercat.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h> // what is this?
#include <stdlib.h>
#include <time.h>
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

static char IOmap[4096];
static int expectedWKC;
static volatile int wkc;
static boolean inOP;
static bool ecat_initialized = false;
static const char* kDefaultAdapter = "enp0s31f6";
static const uint16 g_channel_offsets[CAN_CHANNEL_COUNT] = {0, 75, 150, 225};
static uint8_t g_leg_motor_rr_idx[CAN_CHANNEL_COUNT] = {0, 0, 0, 0};

static void write_u32_le(uint8 *dst, uint32 value) {
  dst[0] = (uint8)(value & 0xFFU);
  dst[1] = (uint8)((value >> 8) & 0xFFU);
  dst[2] = (uint8)((value >> 16) & 0xFFU);
  dst[3] = (uint8)((value >> 24) & 0xFFU);
}

static uint32 read_u32_le(const uint8 *src) {
  return (uint32)src[0] |
         ((uint32)src[1] << 8) |
         ((uint32)src[2] << 16) |
         ((uint32)src[3] << 24);
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
  write_u32_le(base, can_id);                  // 0~3: CAN ID
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

  *can_id = read_u32_le(base);
  memcpy(data, &base[5], CAN_DATA_LEN_FIXED);
  return 1;
}

static void degraded_handler() {
  //shut of gpio enables
  // estop();
  printf("[EtherCAT Error] Logging error...\n");
  time_t current_time = time(NULL);
  char* time_str = ctime(&current_time);
  printf("ESTOP. EtherCAT became degraded at %s.\n", time_str);
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

static std::mutex command_mutex, data_mutex;

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
  for (int leg = 0; leg < CAN_CHANNEL_COUNT; leg++) {
    uint32 can_id = 0;
    uint8 can_data[CAN_DATA_LEN_FIXED] = {0};
    memset(&data[leg], 0, sizeof(TiBoardData));
    if (can_recv_fixed8(slave, leg + 1, &can_id, can_data)) {
      data[leg].loop_count_ti = can_id;
      data[leg].ethercat_count_ti = read_u32_le(can_data);
      data[leg].microtime_ti = read_u32_le(can_data + 4);
    }
  }

  data_mutex.unlock();
}

void rt_ethercat_set_command(TiBoardCommand* command) {
  command_mutex.lock();

  if (command != nullptr && ec_slavecount >= 1) {
    ec_slavet* slave = &ec_slave[1];
    for (int leg = 0; leg < CAN_CHANNEL_COUNT; leg++) {
      uint8 can_data[CAN_DATA_LEN_FIXED] = {0};
      uint32 can_id = mit_motor_protocol::kJointCanIds[0];

      const int joint = g_leg_motor_rr_idx[leg] % mit_motor_protocol::kJointCountPerLeg;
      g_leg_motor_rr_idx[leg] =
          static_cast<uint8_t>((g_leg_motor_rr_idx[leg] + 1) % mit_motor_protocol::kJointCountPerLeg);

      const TiBoardCommand& leg_cmd = command[leg];
      can_id = mit_motor_protocol::kJointCanIds[joint];

      if (leg_cmd.enable) {
        mit_motor_protocol::pack_control_command(
            leg_cmd.q_des[joint], leg_cmd.qd_des[joint], leg_cmd.kp_joint[joint],
            leg_cmd.kd_joint[joint], leg_cmd.tau_ff[joint], can_data);
      } else {
        mit_motor_protocol::pack_control_command(0.f, 0.f, 0.f, 0.f, 0.f, can_data);
      }

      can_send_fixed8(slave, leg + 1, can_id, can_data);
    }
  }

  command_mutex.unlock();
}
