#ifndef PROJECT_RT_MIT_MOTOR_PROTOCOL_H
#define PROJECT_RT_MIT_MOTOR_PROTOCOL_H

#include <cstdint>

namespace mit_motor_protocol {

constexpr int kJointCountPerLeg = 3;
constexpr uint32_t kJointCanIds[kJointCountPerLeg] = {0x01, 0x03, 0x05};

void pack_control_command(float p, float v, float kp, float kd, float t, uint8_t out[8]);

}  // namespace mit_motor_protocol

#endif  // PROJECT_RT_MIT_MOTOR_PROTOCOL_H
