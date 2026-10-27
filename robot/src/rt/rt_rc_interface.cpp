#include <mutex>
#include <rt/NavigationCommand.h>
#include <rt/rt_rc_interface.h>
#include "Utilities/EdgeTrigger.h"
#include <string.h> // memcpy
#include <stdio.h>
#include <rt/rt_sbus.h>
#include <algorithm>
#include <cmath>
static std::mutex lcm_get_set_mutex;

// Controller Settings
rc_control_settings rc_control;
static NavigationCommand navigation;
static double last_joystick_event_time = 0.;

static double navigation_time()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void set_cmd_vel_command(double vx, double wz)
{
    std::lock_guard<std::mutex> lock(lcm_get_set_mutex);
    if (RC_mode::isRL(rc_control.mode))
        navigation.receive(vx, wz, navigation_time());
}

/* ------------------------- HANDLERS ------------------------- */

// Controller Settings
void get_rc_control_settings(void *settings)
{
    std::lock_guard<std::mutex> lock(lcm_get_set_mutex);
    if (!RC_mode::isRL(rc_control.mode)) navigation.cancel();
    auto snapshot = rc_control;
    double vx = 0., wz = 0.;
    if (navigation.sample(navigation_time(), &vx, &wz)) {
        snapshot.v_des[0] = vx;
        snapshot.v_des[1] = snapshot.v_des[2] = 0.;
        snapshot.omega_des[0] = snapshot.omega_des[1] = 0.;
        snapshot.omega_des[2] = wz;
    }
    snapshot.input_diagnostics.navigation_state = static_cast<int>(navigation.state());
    if (snapshot.input_diagnostics.joystick_valid)
        snapshot.input_diagnostics.event_age_ms =
            (navigation_time() - last_joystick_event_time) * 1000.;
    static NavigationCommand::State reported = NavigationCommand::State::Manual;
    if (reported != navigation.state()) {
        reported = navigation.state();
        const char* names[] = {"manual", "waiting for new command", "active",
                               "stale: zero velocity", "timeout: zero velocity; RB re-arm or RT manual"};
        printf("[Navigation] %s\n", names[static_cast<int>(reported)]);
    }
    v_memcpy(settings, &snapshot, sizeof(rc_control_settings));
}

void request_rc_off()
{
    std::lock_guard<std::mutex> lock(lcm_get_set_mutex);
    rc_control.mode = RC_mode::OFF;
    navigation.cancel();
    for (int i = 0; i < 3; ++i) {
        rc_control.v_des[i] = 0.;
        rc_control.omega_des[i] = 0.;
    }
}

// void get_rc_channels(void *settings) {
// pthread_mutex_lock(&lcm_get_set_mutex);
// v_memcpy(settings, &rc_channels, sizeof(rc_channels));
// pthread_mutex_unlock(&lcm_get_set_mutex);
// }

EdgeTrigger<int> mode_edge_trigger(0);
EdgeTrigger<TaranisSwitchState> backflip_prep_edge_trigger(SWITCH_UP);
EdgeTrigger<TaranisSwitchState> experiment_prep_edge_trigger(SWITCH_UP);
TaranisSwitchState initial_mode_go_switch = SWITCH_DOWN;

// rlh 2023.6.10
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/joystick.h>

#define XBOX_TYPE_BUTTON 0x01
#define XBOX_TYPE_AXIS 0x02

#define XBOX_BUTTON_A 0x00
#define XBOX_BUTTON_B 0x01
#define XBOX_BUTTON_X 0x02
#define XBOX_BUTTON_Y 0x03
#define XBOX_BUTTON_LB 0x04
#define XBOX_BUTTON_RB 0x05
#define XBOX_BUTTON_START 0x06
#define XBOX_BUTTON_BACK 0x07
#define XBOX_BUTTON_HOME 0x08
#define XBOX_BUTTON_LO 0x09 /* 左摇杆按键 */
#define XBOX_BUTTON_RO 0x0a /* 右摇杆按键 */

#define XBOX_BUTTON_ON 0x01
#define XBOX_BUTTON_OFF 0x00

#define XBOX_AXIS_LX 0x00 /* 左摇杆X轴 */
#define XBOX_AXIS_LY 0x01 /* 左摇杆Y轴 */
#define XBOX_AXIS_RX 0x03 /* 右摇杆X轴 */
#define XBOX_AXIS_RY 0x04 /* 右摇杆Y轴 */
#define XBOX_AXIS_LT 0x02
#define XBOX_AXIS_RT 0x05
#define XBOX_AXIS_XX 0x06 /* 方向键X轴 */
#define XBOX_AXIS_YY 0x07 /* 方向键Y轴 */

#define XBOX_AXIS_VAL_UP -32767
#define XBOX_AXIS_VAL_DOWN 32767
#define XBOX_AXIS_VAL_LEFT -32767
#define XBOX_AXIS_VAL_RIGHT 32767

#define XBOX_AXIS_VAL_MIN -32767
#define XBOX_AXIS_VAL_MAX 32767
#define XBOX_AXIS_VAL_MID 0x00

typedef struct xbox_map
{
    int time;
    int a;
    int b;
    int x;
    int y;
    int lb;
    int rb;
    int start;
    int back;
    int home;
    int lo;
    int ro;

    int lx;
    int ly;
    int rx;
    int ry;
    int lt;
    int rt;
    int xx;
    int yy;

} xbox_map_t;

int xbox_open(const char *file_name)
{
    int xbox_fd;

    xbox_fd = open(file_name, O_RDONLY);
    if (xbox_fd < 0)
    {
        perror("open");
        return -1;
    }

    return xbox_fd;
}

int xbox_map_read(int xbox_fd, xbox_map_t *map)
{
    int len, type, number, value;
    struct js_event js;

    len = read(xbox_fd, &js, sizeof(struct js_event));
    if (len < 0)
    {
        perror("read");
        return -1;
    }

    type = js.type;
    number = js.number;
    value = js.value;

    map->time = js.time;

    if (type == JS_EVENT_BUTTON)
    {
        switch (number)
        {
        case XBOX_BUTTON_A:
            map->a = value;
            break;

        case XBOX_BUTTON_B:
            map->b = value;
            break;

        case XBOX_BUTTON_X:
            map->x = value;
            break;

        case XBOX_BUTTON_Y:
            map->y = value;
            break;

        case XBOX_BUTTON_LB:
            map->lb = value;
            break;

        case XBOX_BUTTON_RB:
            map->rb = value;
            break;

        case XBOX_BUTTON_START:
            // printf("XBOX_BUTTON_START is down\n");对应罗技手柄的back键
            map->start = value;
            break;

        case XBOX_BUTTON_BACK:
            // printf("XBOX_BUTTON_BACK is down\n");对应罗技手柄的START键
            map->back = value;
            break;

        case XBOX_BUTTON_HOME:
            map->home = value;
            break;

        case XBOX_BUTTON_LO:
            map->lo = value;
            break;

        case XBOX_BUTTON_RO:
            map->ro = value;
            break;

        default:
            break;
        }
    }
    else if (type == JS_EVENT_AXIS)
    {
        switch (number)
        {
        case XBOX_AXIS_LX:
            map->lx = value;
            break;

        case XBOX_AXIS_LY:
            map->ly = value;
            break;

        case XBOX_AXIS_RX:
            map->rx = value;
            break;

        case XBOX_AXIS_RY:
            map->ry = value;
            break;

        case XBOX_AXIS_LT:
            map->lt = value;
            break;

        case XBOX_AXIS_RT:
            map->rt = value;
            break;

        case XBOX_AXIS_XX:
            map->xx = value;
            break;

        case XBOX_AXIS_YY:
            map->yy = value;
            break;

        default:
            break;
        }
    }
    else
    {
        /* Init do nothing */
    }

    return len;
}

void xbox_close(int xbox_fd)
{
    close(xbox_fd);
    return;
}

xbox_map_t map;
int js_gait = 3;
int yaw_times = 0;
int back_time = 0;
int advance_or_retreat = 1;
int js_print_count = 0;

template <typename T>
T clamp(T value, T min_val, T max_val)
{
    if (value < min_val)
        return min_val;
    else if (value > max_val)
        return max_val;
    else
        return value;
}

void js_complete(int port)
{
    const bool previous_start = map.start;
    const bool previous_back = map.back;
    int len = xbox_map_read(port, &map);
    if (len < 0)
        return;

    std::lock_guard<std::mutex> lock(lcm_get_set_mutex);

    auto& diagnostic = rc_control.input_diagnostics;
    diagnostic.joystick_valid = true;
    diagnostic.lx = map.lx; diagnostic.ly = map.ly; diagnostic.rx = map.rx;
    diagnostic.rb = map.rb; diagnostic.rt = map.rt;
    ++diagnostic.event_sequence;
    last_joystick_event_time = navigation_time();

    if (++js_print_count >= 100)
    {
        js_print_count = 0;
        printf("[Joystick] raw left=(%d, %d), right=(%d, %d) | norm left=(%.3f, %.3f), right=(%.3f, %.3f)\n",
               map.lx, map.ly, map.rx, map.ry,
               (float)map.lx / 32768.f, (float)map.ly / 32768.f,
               (float)map.rx / 32768.f, (float)map.ry / 32768.f);
    }

    if (map.rb)
    {
        yaw_times = 2800;
        back_time = 0;
        advance_or_retreat = 1;
        printf("yaw_times is %d\n", yaw_times);
    }

    if (map.lt > 30000 && map.a) // LT+A，起立
        rc_control.mode = RC_mode::RECOVERY_STAND;
    else if (map.lt > 30000 && map.b) // LT+B，失能
        rc_control.mode = RC_mode::OFF;
    else if (rc_control.mode != RC_mode::OFF && map.lt > 30000 && map.x)
        rc_control.mode = RC_mode::RECOVERY_STAND;
    else if (rc_control.mode != RC_mode::OFF &&
             ((map.back && !previous_back) || (map.start && !previous_start))) {
        // BACK wins if both are pressed. Held buttons never retry a switch.
        ++rc_control.rl_selection_sequence;
        rc_control.mode = map.back ? RC_mode::RL_STAIRS : RC_mode::RL_FLAT;
    }
    else if (rc_control.mode == RC_mode::QP_STAND)
    {
        rc_control.rpy_des[0] = (float)map.lx / 32768; // 如果是站立模式，设置目标角度
        rc_control.rpy_des[1] = -(float)map.ry / 32768;
        rc_control.rpy_des[2] = (float)map.rx / 32768;
        rc_control.height_variation = std::max(-map.ly / 32768.f, -0.8f); // 可以设置目标高度，最高为0.8

        rc_control.omega_des[0] = 0; // 目标角速度为0
        rc_control.omega_des[1] = 0;
        rc_control.omega_des[2] = 0;
    }
    navigation.buttons(RC_mode::isRL(rc_control.mode),
                       map.rb, map.rt > 15000, navigation_time());
    if (RC_mode::isRL(rc_control.mode)) {
        // Always refresh manual input. Navigation overrides only the snapshot,
        // so RT takes effect on this event without retaining a navigation speed.
        rc_control.v_des[0] = -(double)map.ly / 32768;
        rc_control.v_des[1] = -0.5 * (double)map.lx / 32768;
        rc_control.v_des[2] = 0.;
        rc_control.omega_des[0] = rc_control.omega_des[1] = 0.;
        rc_control.omega_des[2] = -(double)map.rx / 32768;
    }
}

int init_js()
{
    int fd = xbox_open("/dev/input/js0");
    if (fd > 0) {
        memset(&map, 0, sizeof(xbox_map_t));
        printf("[Joystick] opened /dev/input/js0\n");
    } else {
        printf("[Joystick] failed to open /dev/input/js0\n");
    }
    rc_control.step_height = 0.4;
    rc_control.height_variation = 0;
    js_gait = 3;
    return fd;
}
void sbus_packet_complete()
{
    Taranis_X7_data data;
    update_taranis_x7(&data);

    std::lock_guard<std::mutex> lock(lcm_get_set_mutex);

    float v_scale = data.knobs[0] * 1.5f + 2.0f; // from 0.5 to 3.5
    float w_scale = 2. * v_scale;                // from 1.0 to 7.0
                                                 // printf("v scale: %f\n", v_scale);

    auto estop_switch = data.right_lower_right_switch;

    int selected_mode = 0;

    switch (estop_switch)
    {

    case SWITCH_UP: // ESTOP
        selected_mode = RC_mode::OFF;
        break;

    case SWITCH_MIDDLE: // recover
        selected_mode = RC_mode::RECOVERY_STAND;
        break;

    case SWITCH_DOWN: // run
        selected_mode = RC_mode::RL_JOINT_PD;

        // Deadband
        for (int i(0); i < 2; ++i)
        {
            data.left_stick[i] = deadband(data.left_stick[i], 0.1, -1., 1.);
            data.right_stick[i] = deadband(data.right_stick[i], 0.1, -1., 1.);
        }

        rc_control.v_des[0] = v_scale * data.left_stick[1];
        rc_control.v_des[1] = -v_scale * data.left_stick[0] / (data.knobs[0] * 1.5f + 2.0f);
        rc_control.v_des[2] = 0;

        rc_control.omega_des[0] = 0;
        rc_control.omega_des[1] = 0;
        rc_control.omega_des[2] = -w_scale * data.right_stick[0];

        break;
    }

    bool trigger = mode_edge_trigger.trigger(selected_mode);
    if (trigger || selected_mode == RC_mode::OFF || selected_mode == RC_mode::RECOVERY_STAND)
    {
        if (trigger)
        {
            printf("MODE TRIGGER!\n");
        }
        rc_control.mode = selected_mode;
    }
    if (!RC_mode::isRL(rc_control.mode)) navigation.cancel();
}

void *v_memcpy(void *dest, volatile void *src, size_t n)
{
    void *src_2 = (void *)src;
    return memcpy(dest, src_2, n);
}

float deadband(float command, float deadbandRegion, float minVal, float maxVal)
{
    if (command < deadbandRegion && command > -deadbandRegion)
    {
        return 0.0;
    }
    else
    {
        return (command / (2)) * (maxVal - minVal);
    }
}
