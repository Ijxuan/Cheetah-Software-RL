#include "LordImu.h"
#include <cstdio>
#include <stdexcept>
#include <thread>
#include "../../common/include/cppTypes.h"
#include "../../common/include/Dynamics/spatial.h"
#include "../../common/include/Math/orientation_tools.h"

int p1=0;
float gAcc[3]={0,0,0};
const size_t YIS_HEADER_1ST = 0x59;
const size_t YIS_HEADER_2ND = 0x53;
const size_t PROTOCOL_MIN_LEN = 7;
const size_t PROTOCOL_TID_LEN = 2;
const size_t PROTOCOL_PAYLOAD_LEN = 1;
const size_t PROTOCOL_CHECKSUM_LEN = 2;
const size_t PROTOCOL_TID_POS = 2;
const size_t PROTOCOL_PAYLOAD_LEN_POS = 4;
const size_t CRC_CALC_START_POS = 2;
const size_t PAYLOAD_POS = 5;
const size_t TLV_HEADER_LEN = 2;

const size_t acc_id = 0x10;
const size_t gyro_id = 0x20;
const size_t euler_id = 0x40;
const size_t quaternion_id = 0x41;

const size_t acc_len = 0x0C;
const size_t gyro_len = 0x0C;
const size_t euler_len = 0x0C;
const size_t quaternion_len = 0x10;

const double data_factor = 0.000001;


std::vector<int> buf(512);
size_t buf_len = 0;

struct YISOut {
    int tid = 1;
    float roll, pitch, yaw;
    float q0, q1, q2, q3;
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
};

bool decode_data(const std::vector<int>& data, size_t num, YISOut& info, bool debug_flg);
void clear_data(size_t clr_len);
bool parse_data_by_id(int id, size_t len, const std::vector<int>& payload, YISOut& info);
int calc_checksum(const std::vector<int>& data, size_t len);
int get_int32_lit(const std::vector<int>& data);

bool decode_data(const std::vector<int>& data, size_t num, YISOut& info, bool debug_flg) {
    size_t pos = 0;
    size_t cnt = 0;
    size_t data_len = 0;
    int check_sum = 0;


    std::copy(data.begin(), data.begin() + num, buf.begin() + buf_len);
    buf_len += num;


    if (buf_len < PROTOCOL_MIN_LEN) {
        if (debug_flg) {
            std::cout << "len not enough" << std::endl;
        }
        return false;
    }

    cnt = buf_len;


    while (cnt > 0) {
        if (YIS_HEADER_1ST == buf[pos] && YIS_HEADER_2ND == buf[pos + 1]) {
            break;
        } else {
            cnt--;
            pos++;
        }
    }

    if (debug_flg) {
        std::cout << "start pos = " << pos << std::endl;
    }

    if (cnt < PROTOCOL_MIN_LEN) {
        if (debug_flg) {
            std::cout << "clear_data" << std::endl;
        }
        clear_data(pos);
        return false;
    }

    data_len = buf[pos + PROTOCOL_PAYLOAD_LEN_POS];
    if (debug_flg) {
        std::cout << "payload_len = " << data_len << std::endl;
    }


    if (PROTOCOL_MIN_LEN + data_len > cnt) {
        clear_data(pos);
        return false;
    }

    check_sum = calc_checksum(std::vector<int>(buf.begin() + pos + CRC_CALC_START_POS, buf.end()), data_len + PROTOCOL_TID_LEN + PROTOCOL_PAYLOAD_LEN);
    const size_t PROTOCOL_CRC_DATA_POS = CRC_CALC_START_POS + data_len + PROTOCOL_TID_LEN + PROTOCOL_PAYLOAD_LEN;
    if (check_sum != (buf[pos + PROTOCOL_CRC_DATA_POS] + (buf[pos + PROTOCOL_CRC_DATA_POS + 1] << 8))) {
        clear_data(pos + data_len + PROTOCOL_MIN_LEN);
        return false;
    }

    info.tid = buf[pos + PROTOCOL_TID_POS] + (buf[pos + PROTOCOL_TID_POS + 1] << 8);
    cnt = data_len;


    pos += PAYLOAD_POS;
    while (data_len > 0 && pos <= buf_len) {
        int id = buf[pos];
        int len = buf[pos + 1];
        bool ret = parse_data_by_id(id, len, std::vector<int>(buf.begin() + pos + TLV_HEADER_LEN, buf.end()), info);
        if (debug_flg) {
            std::cout << "id: " << static_cast<int>(id) << ", len: " << static_cast<int>(len) << ", ret = " << ret << std::endl;
        }
        if (ret) {
            pos += len + TLV_HEADER_LEN;
            data_len -= len + TLV_HEADER_LEN;
        } else {
            pos += 1;
            data_len -= 1;
        }
    }

    if (debug_flg) {
        std::cout << "total len : " << buf_len << std::endl;
    }

    clear_data(pos + PROTOCOL_CHECKSUM_LEN);
    if (debug_flg) {
        std::cout << "anlysis done, pos = " << pos << ", buf_len left " << buf_len << std::endl;
    }
    return true;
}

void clear_data(size_t clr_len) {
    if (clr_len == 0) {
        return;
    }

    std::fill(buf.begin(), buf.begin() + clr_len, 0);

    if (buf_len > clr_len) {
        std::copy(buf.begin() + clr_len, buf.begin() + buf_len, buf.begin());
        std::fill(buf.begin() + buf_len - clr_len, buf.end(), 0);
        buf_len -= clr_len;
    } else {
        buf_len = 0;
    }
}

bool parse_data_by_id(int id, size_t len, const std::vector<int>& payload, YISOut& info) {
    switch (id) {
        case acc_id:
            if (len == acc_len) {
                info.acc_x = get_int32_lit(std::vector<int>(payload.begin(), payload.begin() + 4)) * data_factor;
                info.acc_y = get_int32_lit(std::vector<int>(payload.begin() + 4, payload.begin() + 8)) * data_factor;
                info.acc_z = get_int32_lit(std::vector<int>(payload.begin() + 8, payload.begin() + 12)) * data_factor;
                return true;
            }
            break;
        case gyro_id:
            if (len == gyro_len) {
                info.gyro_x = get_int32_lit(std::vector<int>(payload.begin(), payload.begin() + 4)) * data_factor;
                info.gyro_y = get_int32_lit(std::vector<int>(payload.begin() + 4, payload.begin() + 8)) * data_factor;
                info.gyro_z = get_int32_lit(std::vector<int>(payload.begin() + 8, payload.begin() + 12)) * data_factor;
                return true;
            }
            break;
        case euler_id:
            if (len == euler_len) {
                info.pitch = get_int32_lit(std::vector<int>(payload.begin(), payload.begin() + 4)) * data_factor;
                info.roll = get_int32_lit(std::vector<int>(payload.begin() + 4, payload.begin() + 8)) * data_factor;
                info.yaw = get_int32_lit(std::vector<int>(payload.begin() + 8, payload.begin() + 12)) * data_factor;
                return true;
            }
            break;
        case quaternion_id:
            if (len == quaternion_len) {
                info.q0 = get_int32_lit(std::vector<int>(payload.begin(), payload.begin() + 4)) * data_factor;
                info.q1 = get_int32_lit(std::vector<int>(payload.begin() + 4, payload.begin() + 8)) * data_factor;
                info.q2 = get_int32_lit(std::vector<int>(payload.begin() + 8, payload.begin() + 12)) * data_factor;
                info.q3 = get_int32_lit(std::vector<int>(payload.begin() + 12, payload.begin() + 16)) * data_factor;
                return true;
            }
            break;
        default:
            return false;
    }
    return false;
}

int calc_checksum(const std::vector<int>& data, size_t len) {
    int check_a = 0x00;
    int check_b = 0x00;
    for (size_t i = 0; i < len; ++i) {
        check_a += data[i];
        check_b += check_a;
    }
    return ((check_b % 256) << 8) + (check_a % 256);
}

int get_int32_lit(const std::vector<int>& data) {
    int temp = 0;
    for (size_t i = 0; i < 4; ++i) {
        temp += data[i] << (i * 8);
    }
    //if (temp & 0x80000000) {
        //temp = (temp ^ 0xFFFFFFFF) + 1;
    //}
    return temp;
}


constexpr u32 IMU_PACKET_TIMEOUT_MS = 1000;
//constexpr u32 MIP_SDK_GX4_25_IMU_DIRECT_MODE = 0x02;
constexpr u32 MIP_SDK_STANDARD_MODE = 0x01;

static std::mutex dataMutex;

bool LordImu::tryInit(u32 port, u32 baud_rate) {
  try {
    init(port, baud_rate);
  } catch(std::exception& e) {
    printf("[LordIMU] failed to initialize: %s\n", e.what());
    return false;
  }

  return true;
}

void LordImu::init(u32 port, u32 baud_rate) {
#ifdef USE_LordIMU
  printf("[Lord IMU] Open port %d, baud rate %d\n", port, baud_rate);

  if(mip_interface_init(port, baud_rate, &device_interface, IMU_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
    throw std::runtime_error("Failed to initialize MIP interface for IMU\n");
  }

  printf("[Lord IMU] Port open. Mode setup...\n");
  mode_setup();
  printf("[Lord IMU] Get info...\n");
  get_device_info();
  printf("[Lord IMU] Self test...\n");
  //self_test();
  printf("[Lord IMU] Basic report...\n");
  basic_report();
//  printf("[Lord IMU] Zero Gyro...\n");
//  zero_gyro();
  printf("[Lord IMU] Setup IMU...\n");
  setup_streaming();
  printf("[Lord IMU] Enable Data...\n");
  enable();
#endif

#ifdef USE_SelfIMU
    printf("[Lord IMU] Open port %d, baud rate %d\n", port, baud_rate);

    if(mip_interface_init(port, baud_rate, &device_interface, IMU_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
        throw std::runtime_error("Failed to initialize MIP interface for IMU\n");
    }


    printf("[Lord IMU] Port open. Mode setup...\n");
    //mode_setup();
    printf("[Lord IMU] Get info...\n");
    //get_device_info();
    printf("[Lord IMU] Self test...\n");
    //self_test();
    printf("[Lord IMU] Basic report...\n");
    // basic_report();
//  printf("[Lord IMU] Zero Gyro...\n");
//  zero_gyro();
    printf("[Lord IMU] Setup IMU...\n");
    setup_streaming();
    printf("[Lord IMU] Enable Data...\n");
    //enable();
#endif
}

void LordImu::mode_setup() {
  u8 com_mode = MIP_SDK_STANDARD_MODE;
  printf("[Lord IMU] Set direct mode\n");

  while(mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK) {
    printf("failed to set com mode\n");
  }

  while(mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK) {
    printf("failed to read com mode\n");
  }

  if(com_mode != MIP_SDK_STANDARD_MODE) {
    printf("failed to set mode, wanted %d, got %d\n", MIP_SDK_STANDARD_MODE, com_mode);
  } else {
    printf("[Lord IMU] Done\n");
  }

  usleep(100000);


  while(mip_base_cmd_idle(&device_interface) != MIP_INTERFACE_OK){
    printf("idle fail\n");
  }

  usleep(100000);

  printf("[Lord IMU] Ping...\n");

  while(mip_base_cmd_ping(&device_interface) != MIP_INTERFACE_OK){
    printf("fail\n");
  }
}

static std::string dev_string(u16* str) {
  char temp[32];
  char* in = (char*)str;

  u32 i = 0;
  while((!in[i] || in[i] == ' ') && i < 16) {
    i++;
  }

  char* charp = temp;
  while(i < 16) {
    *charp = in[i];
    charp++;
    i++;
  }
  *charp = 0;
  return std::string(temp);
}

void LordImu::get_device_info() {
  base_device_info_field device_info;
  while(mip_base_cmd_get_device_info(&device_interface, &device_info) != MIP_INTERFACE_OK){
    printf("fail\n");
  }

  u16 base_rate = 0;
  while(mip_3dm_cmd_get_ahrs_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK){
    printf("fail\n");
  }

  u16 filter_rate = 0;
  while(mip_3dm_cmd_get_filter_base_rate(&device_interface, &filter_rate) != MIP_INTERFACE_OK){
    printf("fail\n");
  }



  deviceInfo.modelName = dev_string(device_info.model_name);
  deviceInfo.modelNumber = dev_string(device_info.model_number);
  deviceInfo.serialNumber = dev_string(device_info.serial_number);
  deviceInfo.lotNumber = dev_string(device_info.lotnumber);
  deviceInfo.deviceOptions = dev_string(device_info.device_options);
  deviceInfo.firmwareVersion = device_info.firmware_version;
  deviceInfo.ahrs_base_rate = base_rate;
  deviceInfo.filter_rate = filter_rate;

  printf("[Lord IMU] Got device info:\n");
  printf("  name: %s\n"
         "  model: %s\n"
         "  serial: %s\n"
         "  lot: %s\n"
         "  options: %s\n"
         "  firmware: %d\n"
         "  ahrs rate: %d Hz\n"
         "  filter rate: %d Hz\n",
         deviceInfo.modelName.c_str(),
         deviceInfo.modelNumber.c_str(),
         deviceInfo.serialNumber.c_str(),
         deviceInfo.lotNumber.c_str(),
         deviceInfo.deviceOptions.c_str(),
         deviceInfo.firmwareVersion,
         deviceInfo.ahrs_base_rate,
         deviceInfo.filter_rate);
}


void LordImu::self_test() {
  u32 bit_result;
  while(mip_base_cmd_built_in_test(&device_interface, &bit_result) != MIP_INTERFACE_OK){
    printf("fail\n");
  }

  if(bit_result) {
    printf("[Lord IMU] Self test failed: 0x%x\n", bit_result);
    throw std::runtime_error("self test fail\n");
  } else {
    printf("[Lord IMU] Self test passed\n");
  }
}


void LordImu::basic_report() {
//  while(mip_3dm_cmd_hw_specific_imu_device_status(&device_interface, 6237, 1, &basicStatus) != MIP_INTERFACE_OK){
//    printf("fail\n");
//  }
}
static LordImu* gLordImu;
/* static void filter_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type) {
  (void)user_ptr;
  (void)packet_size;

  mip_field_header* field_header;
  u8* field_data;
  u16 field_offset = 0;
  mip_filter_attitude_quaternion quat;

//  Mat3<float> R = Mat3<float>::Identity();
  //R << 0, -1, 0, -1, 0, 0, 0, 0, 1;

  switch(callback_type) {
    case MIP_INTERFACE_CALLBACK_VALID_PACKET:
      while(mip_get_next_field(packet, &field_header,
          &field_data, &field_offset) == MIP_OK) {
        switch(field_header->descriptor) {
          case MIP_FILTER_DATA_ATT_QUATERNION:
          {
            memcpy(&quat, field_data, sizeof(mip_filter_attitude_quaternion));
            mip_filter_attitude_quaternion_byteswap(&quat);
            dataMutex.lock();
            gLordImu->quat = Vec4<float>(quat.q);
            Mat3<float> g_R_imu, r_R_imup;

            g_R_imu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
            r_R_imup << 0, 0, 1, 0, -1, 0, 1, 0, 0;

            Vec4<float> ql = ori::rotationMatrixToQuaternion(g_R_imu.transpose());
            Vec4<float> qr = ori::rotationMatrixToQuaternion(r_R_imup);
            gLordImu->quat = ori::quatProduct(ql, ori::quatProduct(gLordImu->quat, qr));

            gLordImu->good_packets++;
            dataMutex.unlock();
          }
            break;
          default:
            printf("[Lord IMU] Unknown FILTER packet %d\n", field_header->descriptor);
            break;
        }
      }
      break;
    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
      gLordImu->invalid_packets++;
      break;
    case MIP_INTERFACE_CALLBACK_TIMEOUT:
      gLordImu->timeout_packets++;
      break;
    default:
      gLordImu->unknown_packets++;
      break;
  }

} */



static void ahrs_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type) {
#ifdef USE_LordIMU
  (void)user_ptr;
  (void)packet_size;

  mip_field_header* field_header;
  u8* field_data;
  u16 field_offset = 0;
  mip_ahrs_scaled_accel accel;
  mip_ahrs_scaled_gyro gyro;



  switch(callback_type) {
    case MIP_INTERFACE_CALLBACK_VALID_PACKET:
      //gLordImu->good_packets++;
      while(mip_get_next_field(packet, &field_header,
          &field_data, &field_offset) == MIP_OK) {
        switch(field_header->descriptor) {
          case MIP_AHRS_DATA_ACCEL_SCALED:
            memcpy(&accel, field_data, sizeof(mip_ahrs_scaled_accel));
            mip_ahrs_scaled_accel_byteswap(&accel);
            //gLordImu->acc = Vec3<float>(accel.scaled_accel);
            dataMutex.lock();
            gLordImu->acc[0] = 9.81f * accel.scaled_accel[2];
            gLordImu->acc[1] = -9.81f * accel.scaled_accel[1];
            gLordImu->acc[2] = 9.81f * accel.scaled_accel[0];
            gLordImu->good_packets++;
            dataMutex.unlock();
            break;
          case MIP_AHRS_DATA_GYRO_SCALED:
            dataMutex.lock();
            memcpy(&gyro, field_data, sizeof(mip_ahrs_scaled_gyro));
            mip_ahrs_scaled_gyro_byteswap(&gyro);
            //gLordImu->gyro = Vec3<float>(gyro.scaled_gyro);
            gLordImu->gyro[0] = gyro.scaled_gyro[2];
            gLordImu->gyro[1] = -gyro.scaled_gyro[1];
            gLordImu->gyro[2] = gyro.scaled_gyro[0];
            gLordImu->good_packets++;
            dataMutex.unlock();
            break;
          default:
            printf("[Lord IMU] Unknown AHRS packet %d\n", field_header->descriptor);
            break;
        }
      }
      break;
    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
      gLordImu->invalid_packets++;
      break;
    case MIP_INTERFACE_CALLBACK_TIMEOUT:
      gLordImu->timeout_packets++;
      break;
    default:
      gLordImu->unknown_packets++;
      break;
  }
#endif

#ifdef USE_SelfIMU
    (void)user_ptr;
    (void)packet_size;

    mip_field_header* field_header;
    u8* field_data;
    u16 field_offset = 0;

    switch(callback_type) {
        case MIP_INTERFACE_CALLBACK_VALID_PACKET:
            //gLordImu->good_packets++;
            while(mip_get_next_field(packet, &field_header,
                                     &field_data, &field_offset) == MIP_OK) {
                switch(field_header->descriptor) {

                    default:

                        printf("[Lord IMU] Unknown AHRS packet %d\n", field_header->descriptor);
                        break;
                }
            }
            break;
        case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
            gLordImu->invalid_packets++;
            break;
        case MIP_INTERFACE_CALLBACK_TIMEOUT:
            gLordImu->timeout_packets++;
            break;
        default:
            gLordImu->unknown_packets++;
            break;
    }
#endif
}


void LordImu::setup_streaming() {
#ifdef USE_LordIMU
  gLordImu = this;

  u8 enable = MIP_3DM_CONING_AND_SCULLING_DISABLE;

  while(mip_3dm_cmd_coning_sculling_compensation(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &enable) != MIP_INTERFACE_OK){
    printf("fail\n");
  }

  // callbacks
  if(mip_interface_add_descriptor_set_callback(&device_interface,
      MIP_FILTER_DATA_SET, nullptr, filter_callback) != MIP_INTERFACE_OK) {
    throw std::runtime_error("failed to set IMU filter callback");
  }

  if(mip_interface_add_descriptor_set_callback(&device_interface,
      MIP_AHRS_DATA_SET, nullptr, ahrs_callback) != MIP_INTERFACE_OK) {
    throw std::runtime_error("failed to set IMU ahrs callback");
  }

  printf("[Lord IMU] Setup message types...\n");
  u8 data_types[3] = {MIP_AHRS_DATA_GYRO_SCALED, MIP_AHRS_DATA_ACCEL_SCALED};
  u16 data_downsampling[3] = {1, 1};

  u8 num_entries = 2;
  while(mip_3dm_cmd_ahrs_message_format(&device_interface,
      MIP_FUNCTION_SELECTOR_WRITE, &num_entries, data_types, data_downsampling) != MIP_INTERFACE_OK) {
    printf("fail\n");
  }

  data_types[0] = {MIP_FILTER_DATA_ATT_QUATERNION};
  num_entries = 1;

  while(mip_3dm_cmd_filter_message_format(&device_interface,
       MIP_FUNCTION_SELECTOR_WRITE, &num_entries, data_types, data_downsampling) != MIP_INTERFACE_OK) {
    printf("fail\n");
  }
#endif

#ifdef USE_SelfIMU
    gLordImu = this;
    // if(mip_interface_add_descriptor_set_callback(&device_interface,
    //                                              MIP_FILTER_DATA_SET, nullptr, filter_callback) != MIP_INTERFACE_OK) {
    //     throw std::runtime_error("failed to set IMU filter callback");
    // }

    if(mip_interface_add_descriptor_set_callback(&device_interface,
                                                 MIP_AHRS_DATA_SET, nullptr, ahrs_callback) != MIP_INTERFACE_OK) {
        throw std::runtime_error("failed to set IMU ahrs callback");
    }
#endif
}

void LordImu::enable() {

//  u8 hs = MIP_FILTER_HEADING_SOURCE_MAGNETOMETER;
//  while(mip_filter_heading_source(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &hs) != MIP_INTERFACE_OK) {
//    printf("hs fail\n");
//  }

  while(mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK){
    printf("reset fail\n");
  }

  float angles[3];
  angles[0] = angles[1] = angles[2] = 0;

  while(mip_filter_set_init_attitude(&device_interface, angles) != MIP_INTERFACE_OK){
    printf("init fail\n");
  }

  u8 option = 0; // no magnetometer
  while(mip_filter_heading_source(&device_interface, 1, &option) != MIP_INTERFACE_OK) {
    printf("setup heading fail\n");
  }

  u8 enable_value = 1;
  while(mip_3dm_cmd_continuous_data_stream(&device_interface,
      MIP_FUNCTION_SELECTOR_WRITE,
      MIP_3DM_AHRS_DATASTREAM,
      &enable_value) != MIP_INTERFACE_OK) {
    printf("fail\n");
  }

  while(mip_3dm_cmd_continuous_data_stream(&device_interface,
                                           MIP_FUNCTION_SELECTOR_WRITE,
                                           MIP_3DM_INS_DATASTREAM,
                                           &enable_value) != MIP_INTERFACE_OK) {
    printf("fail\n");
  }

  printf("[Lord IMU] Ready to run!\n");
}

void LordImu::run() {
  for(u32 i = 0; i < 32; i++){
    int dat[170];
    mip_interface_update(&device_interface,dat);

    p1++;

    if (p1%250==0){
      //printf("      %d %d\n",p1,dat[65]);
      YISOut imu;
      std::vector<int> data(dat,dat+67);
      size_t num = 67;//data.size();
      bool ret = decode_data(data, num, imu, false);   
      //printf("acc (%.2f,%.2f,%.2f)\n",gLordImu->acc[0],gLordImu->acc[1],gLordImu->acc[2]);    
      if (ret ) {

        dataMutex.lock();
        gLordImu->good_packets++;
        gAcc[0]=gLordImu->acc[0] = -imu.acc_x;
        gAcc[1]=gLordImu->acc[1] = -imu.acc_y;
        gAcc[2]=gLordImu->acc[2] = imu.acc_z;
        
        gLordImu->gyro[0] = -imu.gyro_x/100;
        gLordImu->gyro[1] = -imu.gyro_y/100;
        gLordImu->gyro[2] = imu.gyro_z/100;

        gLordImu->quat[0] = -imu.q0;
        gLordImu->quat[1] = imu.q1;
        gLordImu->quat[2] = imu.q2;
        gLordImu->quat[3] = -imu.q3;
        dataMutex.unlock();

      }

        dataMutex.lock();
        gLordImu->good_packets++;
        gLordImu->acc[0] = gAcc[0];
        gLordImu->acc[1] = gAcc[1];
        gLordImu->acc[2] = gAcc[2];
        dataMutex.unlock();

    }

  }
  usleep(100);
}

void LordImu::updateLCM(microstrain_lcmt *message) {
  dataMutex.lock();
  for(u32 i = 0; i < 4; i++) {
    message->quat[i] = quat[i];
  }

  Vec3<float> rpy = ori::quatToRPY(quat);
  for(u32 i = 0; i < 3; i++) {
    message->rpy[i] = rpy[i];
    message->acc[i] = acc[i];
    message->omega[i] = gyro[i];
  }

  message->good_packets = good_packets;
  message->bad_packets = invalid_packets + unknown_packets + timeout_packets;
  dataMutex.unlock();
}

void LordImu::print_packet_stats() {
  printf("-- IMU Packets --\n"
         "good: %d\n"
         "invalid: %d\n"
         "timeout: %d\n"
         "unknown: %d\n",
         good_packets, invalid_packets,
         timeout_packets, unknown_packets);
}

void LordImu::zero_gyro() {
  u16 duration = 5000; //milliseconds
  float bias_vector[3];

  printf("please wait...\n");
  while(mip_3dm_cmd_capture_gyro_bias(&device_interface, duration, bias_vector) != MIP_INTERFACE_OK){
    printf("fail\n");
  }

  printf("Gyro Bias Captured:\nbias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0], bias_vector[1], bias_vector[2]);
}
