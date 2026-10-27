/**
 * @file rt_rc_interface.h
 *
 */
#ifndef _RT_RC_INTERFACE
#define _RT_RC_INTERFACE
#include <chrono>
#include "RLControllerSelection.h"

// Captured with the same lock/snapshot as the RC velocity used by the controller.
struct rc_input_diagnostics {
    bool joystick_valid = false;
    int lx = 0, ly = 0, rx = 0, rb = 0, rt = 0;
    uint64_t event_sequence = 0;
    double event_age_ms = -1.;
    int navigation_state = 0; // NavigationCommand::State
};

class rc_control_settings {
  public:
    double     mode;
    double     p_des[2]; // (x, y) -1 ~ 1
    double     height_variation; // -1 ~ 1
    double     v_des[3]; // -1 ~ 1 * (scale 0.5 ~ 1.5)
    double     rpy_des[3]; // -1 ~ 1
    double     omega_des[3]; // -1 ~ 1
    double     variable[3];
    double     step_height;
    uint64_t rl_selection_sequence = 0;
    rc_input_diagnostics input_diagnostics;
};

// Thread-safe navigation input; only accepted after RB arms RL navigation.
void set_cmd_vel_command(double vx, double wz);


namespace RC_mode{
  constexpr int OFF = 0;
  constexpr int QP_STAND = 3;
  constexpr int BACKFLIP_PRE = 4;
  constexpr int BACKFLIP = 5;
  constexpr int VISION = 6;
  constexpr int LOCOMOTION = 11;
  constexpr int RECOVERY_STAND = 12;
  constexpr int RL_STAIRS = 53;
  constexpr int RL_JOINT_PD = RL_STAIRS;
  constexpr int RL_FLAT = 54;
  inline bool isRL(double mode) { return mode == RL_STAIRS || mode == RL_FLAT; }

  // Experiment Mode
  constexpr int TWO_LEG_STANCE_PRE = 20;
  constexpr int TWO_LEG_STANCE = 21;
};

void sbus_packet_complete();


//rlh 2023.6.10
void js_complete(int port);
int init_js();
void auto_speed();

void keyboard_ing();


void get_rc_control_settings(void* settings);
// Same OFF request as the remote; normal operator mode selection can recover.
void request_rc_off();
//void get_rc_channels(void* settings);

void* v_memcpy(void* dest, volatile void* src, size_t n);

float deadband(float command, float deadbandRegion, float minVal, float maxVal);

#endif
