#include "rt/mit_motor_protocol.h"

namespace {

constexpr float P_MIN = -12.5f;
constexpr float P_MAX = 12.5f;
constexpr float V_MIN = -30.0f;
constexpr float V_MAX = 30.0f;
constexpr float KP_MIN = 0.0f;
constexpr float KP_MAX = 500.0f;
constexpr float KD_MIN = 0.0f;
constexpr float KD_MAX = 5.0f;
constexpr float T_MIN = -32.0f;
constexpr float T_MAX = 32.0f;

inline float clampf(float x, float mn, float mx) {
  if (x < mn) return mn;
  if (x > mx) return mx;
  return x;
}

inline uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
  const float span = x_max - x_min;
  const float offset = x_min;
  return static_cast<uint16_t>((x - offset) * ((1u << bits) - 1) / span);
}

}  // namespace

namespace mit_motor_protocol {

void pack_control_command(float p_in, float v_in, float kp_in, float kd_in, float t_in,
                          uint8_t out[8]) {
  const float p_f = clampf(p_in, P_MIN, P_MAX);
  const float v_f = clampf(v_in, V_MIN, V_MAX);
  const float kp_f = clampf(kp_in, KP_MIN, KP_MAX);
  const float kd_f = clampf(kd_in, KD_MIN, KD_MAX);
  const float t_f = clampf(t_in, T_MIN, T_MAX);

  const uint16_t p = float_to_uint(p_f, P_MIN, P_MAX, 16);
  const uint16_t v = float_to_uint(v_f, V_MIN, V_MAX, 12);
  const uint16_t kp = float_to_uint(kp_f, KP_MIN, KP_MAX, 12);
  const uint16_t kd = float_to_uint(kd_f, KD_MIN, KD_MAX, 12);
  const uint16_t t = float_to_uint(t_f, T_MIN, T_MAX, 12);

  out[0] = p >> 8;
  out[1] = p & 0xFF;
  out[2] = v >> 4;
  out[3] = static_cast<uint8_t>(((v & 0xF) << 4) | (kp >> 8));
  out[4] = kp & 0xFF;
  out[5] = kd >> 4;
  out[6] = static_cast<uint8_t>(((kd & 0xF) << 4) | (t >> 8));
  out[7] = t & 0xFF;
}

}  // namespace mit_motor_protocol
