#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "xl2515.h"


// モーターのPin設定や、CANFrane関係の設定。
#define LED_PIN 25
// M15_212のプロトコル
#define MOTOR1_ID 1
#define MOTOR2_ID 2
// FBは、FB_ID_BASE+IDで増えていく。
#define FB_ID_BASE 0x96
// SET MODEで、電流制御や位置制御、速度制御に切り替えることができる。
//しかし、モード切り替えごとに速度が0になるので注意が必要。
#define CMD_SET_MODE 0x105
#define CMD_SET_FB_METHOD 0x106
#define CMD_SET_VALUE_1_4 0x32
#define CMD_SET_VALUE_5_8 0x33

#define MODE_VOLT_OPEN 0x00
#define MODE_CURRENT 0x01
#define MODE_VELOCITY 0x02
#define MODE_POSITION 0x03
#define MODE_DISABLE 0x09
// 電流制御の際、指令値自体は-33A~33Aまで出せる。
// 16Aぐらい実際に流れると、普通にMDがエラーを吐く。
// ただ、33Aぐらいの指令を送ると、210rpmが無負荷回転で出る
#define I_FULL_SCALE_A 33.0f
// 実際に指定する電流の制限(絶対値)
#define I_LIMIT_A 33.0f
// CANbusから流れてくるFBの時間
#define FB_PERIOD_MS 1
//PIDを計算して、制御する時間。
#define CONTROL_PERIOD_MS 1
// PIDのゲイン
#define PID_KP 0.65f
#define PID_KI 0.035f
#define PID_KD 0.015f
// USBのシリアル経由でながす時間の制限。
// これをつけている理由は、PC側でコンソールに値を表示すると、
//キーボード入力が効かない時があるから。
#define USB_FB_PERIOD_DEFAULT_MS 100
#define USB_FB_PERIOD_NONE -1
// LEDチカチカの周期。CANを受け取ったら光るようにしてある。
#define LED_IDLE_OFF_MS 20

#define ROBOT_WHEEL_RADIUS_M 0.01f
#define ROBOT_WHEEL_BASE_M 0.2f

// USB経由でのフィードバックデータのフォーマット
typedef enum {
    USB_FB_MIRROR = 0,
    USB_FB_COMPACT,
    USB_FB_PID_ONLY,
} usb_fb_format_t;

// PIDコントローラの状態
typedef struct {
    float kp;
    float ki;
    float kd;
    float i_limit_a;
    float integral;
    float prev_error;
    bool has_prev;
} pid_controller_t;

// 最新のフィードバック情報
typedef struct {
    int16_t velocity;
    int32_t pid_error;
    float pid_output;
    uint8_t mode;
} latest_feedback_t;

//ロボットのおどめとり情報
typedef struct {
    float x_m;
    float y_m;
    float theta_rad;
} odometry_t;

// モーター情報に関するフィードバック情報
typedef struct {
    uint8_t motor_id;
    int16_t angle;
    int16_t velocity;
    int16_t current;
    uint8_t fault;
    uint8_t mode;
} feedback_t;

//モーターに送るコマンドがどのようなものか、また、USB経由で受け取ったコマンドの種類
typedef enum {
    CMD_INVALID = 0,
    CMD_FEEDBACK_CONFIG,
    CMD_DISABLE,
    CMD_VOLTAGE_OPEN_LOOP,
    CMD_CURRENT_LOOP,
    CMD_VELOCITY_LOOP,
    CMD_POSITION_LOOP,
    CMD_RPM_PID,
} command_mode_t;

// USB経由でのコマンド情報
typedef struct {
    command_mode_t mode;
    double value1;
    double value2;
    usb_fb_format_t format;
    int32_t period_ms;
    bool has_format;
    bool has_period;
} command_t;

static usb_fb_format_t g_usb_fb_format = USB_FB_MIRROR;
static int32_t g_usb_fb_period_ms = USB_FB_PERIOD_DEFAULT_MS;
static uint64_t g_last_usb_fb_ms = 0;

static latest_feedback_t g_latest_feedback[3];
static feedback_t g_latest_motor_feedback[3];
static bool g_has_motor_feedback[3];
static int8_t g_pending_mode[3];
static int32_t g_pid_targets[3];
static bool g_pid_enabled = false;
static pid_controller_t g_pid_motor1;
static pid_controller_t g_pid_motor2;
static odometry_t g_odometry;

static bool g_led_state = false;
static uint64_t g_last_comm_ms = 0;


static uint64_t now_ms(void)
{
    return time_us_64() / 1000;
}

static void note_communication(void)
{
    g_led_state = !g_led_state;
    gpio_put(LED_PIN, g_led_state);
    g_last_comm_ms = now_ms();
}

static void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->has_prev = false;
}

static float pid_update(pid_controller_t *pid, float error, float dt_s)
{   //積分の計算
    //誤差x過ぎた時間
    pid->integral += error * dt_s;
    //積分項の値の制限
    //いらない説もある。(Anti-Windup対策のため)実験しよう。
    if (pid->ki != 0.0f) {
        float max_int = pid->i_limit_a / pid->ki;
        if (pid->integral > max_int) {
            pid->integral = max_int;
        } else if (pid->integral < -max_int) {
            pid->integral = -max_int;
        }

    }

    // 微分項の計算
    float derivative = 0.0f;
    if (pid->has_prev && dt_s > 0.0f) {
    //一見すると、誤差があって、増加方向に働くと思いがちだが、
    //実際には、誤差が減少する方向に働く。なぜなら、 error = target - measuredで、
    // prev_error - error = (target - prev_measured) - (target - measured)
    // となり、targetとprev_targetが等しい場合、それらが打ち消しあって、
    // = measured - prev_measured となり、実際の変化量になる
    // 今回は、モーターの速度制御なので、target-previous_target=0であることが多い。
        derivative = (error - pid->prev_error) / dt_s;
    }
    pid->prev_error = error;
    pid->has_prev = true;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    if (output > pid->i_limit_a) {
        output = pid->i_limit_a;
    } else if (output < -pid->i_limit_a) {
        output = -pid->i_limit_a;
    }
    return output;
}

static int16_t s16_from_bigendian(uint8_t msb, uint8_t lsb)
{
    return (int16_t)((msb << 8) | lsb);
}

static void s16_to_bigendian(int32_t v, uint8_t *msb, uint8_t *lsb)
{
    if (v > 32767) {
        v = 32767;
    } else if (v < -32767) {
        v = -32767;
    }
    uint16_t u = (uint16_t)(v & 0xFFFF);
    *msb = (u >> 8) & 0xFF;
    *lsb = u & 0xFF;
}

static int16_t current_a_to_raw(float i_a)
{
    if (i_a > I_LIMIT_A) {
        i_a = I_LIMIT_A;
    } else if (i_a < -I_LIMIT_A) {
        i_a = -I_LIMIT_A;
    }
    return (int16_t)(i_a / I_FULL_SCALE_A * 32767.0f);
}

static void can_send(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    xl2515_send(can_id, (uint8_t *)data, len);
    note_communication();
}

static bool mode_change_needed(uint8_t target1, uint8_t target2)
{
    if (g_pending_mode[MOTOR1_ID] == (int8_t)target1 ||
        g_pending_mode[MOTOR2_ID] == (int8_t)target2) {
        return false;
    }
    if (g_latest_feedback[MOTOR1_ID].mode != target1) {
        return true;
    }
    if (g_latest_feedback[MOTOR2_ID].mode != target2) {
        return true;
    }
    return false;
}

static void mark_pending_mode(uint8_t target1, uint8_t target2)
{
    g_pending_mode[MOTOR1_ID] = (int8_t)target1;
    g_pending_mode[MOTOR2_ID] = (int8_t)target2;
}

static void set_feedback_method(void)
{
    uint8_t p = FB_PERIOD_MS;
    if (p < 1) {
        p = 1;
    } else if (p > 127) {
        p = 127;
    }
    uint8_t data[8] = {p, p, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80};
    can_send(CMD_SET_FB_METHOD, data, 8);
}

static void send_disable_commands(void)
{
    if (!mode_change_needed(MODE_DISABLE, MODE_DISABLE)) {
        return;
    }
    uint8_t data[8] = {MODE_DISABLE, MODE_DISABLE, MODE_DISABLE, MODE_DISABLE,
                       MODE_DISABLE, MODE_DISABLE, MODE_DISABLE, MODE_DISABLE};
    can_send(CMD_SET_MODE, data, 8);
    mark_pending_mode(MODE_DISABLE, MODE_DISABLE);
}

static void set_mode_current_loop(void)
{
    if (!mode_change_needed(MODE_CURRENT, MODE_CURRENT)) {
        return;
    }
    uint8_t data[8] = {MODE_CURRENT, MODE_CURRENT, MODE_DISABLE, MODE_DISABLE,
                       MODE_DISABLE, MODE_DISABLE, MODE_DISABLE, MODE_DISABLE};
    can_send(CMD_SET_MODE, data, 8);
    mark_pending_mode(MODE_CURRENT, MODE_CURRENT);
}

static void set_mode_voltage_open_loop(void)
{
    if (!mode_change_needed(MODE_VOLT_OPEN, MODE_VOLT_OPEN)) {
        return;
    }
    uint8_t data[8] = {MODE_VOLT_OPEN, MODE_VOLT_OPEN, MODE_DISABLE, MODE_DISABLE,
                       MODE_DISABLE, MODE_DISABLE, MODE_DISABLE, MODE_DISABLE};
    can_send(CMD_SET_MODE, data, 8);
    mark_pending_mode(MODE_VOLT_OPEN, MODE_VOLT_OPEN);
}

static void set_mode_position_loop(void)
{
    if (!mode_change_needed(MODE_POSITION, MODE_POSITION)) {
        return;
    }
    uint8_t data[8] = {MODE_POSITION, MODE_POSITION, MODE_DISABLE, MODE_DISABLE,
                       MODE_DISABLE, MODE_DISABLE, MODE_DISABLE, MODE_DISABLE};
    can_send(CMD_SET_MODE, data, 8);
    mark_pending_mode(MODE_POSITION, MODE_POSITION);
}

static void set_mode_velocity_loop(void)
{
    if (!mode_change_needed(MODE_VELOCITY, MODE_VELOCITY)) {
        return;
    }
    uint8_t data[8] = {MODE_VELOCITY, MODE_VELOCITY, MODE_DISABLE, MODE_DISABLE,
                       MODE_DISABLE, MODE_DISABLE, MODE_DISABLE, MODE_DISABLE};
    can_send(CMD_SET_MODE, data, 8);
    mark_pending_mode(MODE_VELOCITY, MODE_VELOCITY);
}

static void send_current_commands(int16_t raw1, int16_t raw2)
{
    uint8_t m1h, m1l, m2h, m2l;
    s16_to_bigendian(raw1, &m1h, &m1l);
    s16_to_bigendian(raw2, &m2h, &m2l);
    uint8_t data[8] = {m1h, m1l, m2h, m2l, 0x00, 0x00, 0x00, 0x00};
    can_send(CMD_SET_VALUE_1_4, data, 8);
}

static void send_voltage_commands(int16_t raw1, int16_t raw2)
{
    uint8_t m1h, m1l, m2h, m2l;
    s16_to_bigendian(raw1, &m1h, &m1l);
    s16_to_bigendian(raw2, &m2h, &m2l);
    uint8_t data[8] = {m1h, m1l, m2h, m2l, 0x00, 0x00, 0x00, 0x00};
    can_send(CMD_SET_VALUE_1_4, data, 8);
}

static void send_position_commands(int16_t pos1, int16_t pos2)
{
    uint8_t m1h, m1l, m2h, m2l;
    s16_to_bigendian(pos1, &m1h, &m1l);
    s16_to_bigendian(pos2, &m2h, &m2l);
    uint8_t data[8] = {m1h, m1l, m2h, m2l, 0x00, 0x00, 0x00, 0x00};
    can_send(CMD_SET_VALUE_1_4, data, 8);
}

static void send_velocity_commands(int16_t vel1, int16_t vel2)
{
    uint8_t m1h, m1l, m2h, m2l;
    s16_to_bigendian(vel1, &m1h, &m1l);
    s16_to_bigendian(vel2, &m2h, &m2l);
    uint8_t data[8] = {m1h, m1l, m2h, m2l, 0x00, 0x00, 0x00, 0x00};
    can_send(CMD_SET_VALUE_1_4, data, 8);
}

static void enable_rpm_pid(int32_t target1_rpm, int32_t target2_rpm)
{
    g_pid_targets[MOTOR1_ID] = target1_rpm;
    g_pid_targets[MOTOR2_ID] = target2_rpm;
    pid_reset(&g_pid_motor1);
    pid_reset(&g_pid_motor2);
    g_pid_enabled = true;
    set_mode_current_loop();
}

static void disable_rpm_pid(void)
{
    g_pid_enabled = false;
}

static bool usb_feedback_due(uint64_t now_ms_value)
{
    if (g_usb_fb_period_ms == USB_FB_PERIOD_NONE) {
        return true;
    }
    if ((int64_t)(now_ms_value - g_last_usb_fb_ms) >= g_usb_fb_period_ms) {
        g_last_usb_fb_ms = now_ms_value;
        return true;
    }
    return false;
}

static void calc_robot_odometry(const feedback_t *fb1, const feedback_t *fb2,
                                odometry_t *odometry)
{
    float vel1_rpm = (float)fb1->velocity;
    float vel2_rpm = (float)fb2->velocity;
    float vel1_mps = (vel1_rpm / 60.0f) * (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS_M);
    float vel2_mps = (vel2_rpm / 60.0f) * (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS_M);

    float v_linear = (vel1_mps + vel2_mps) / 2.0f;
    float v_angular = (vel2_mps - vel1_mps) / ROBOT_WHEEL_BASE_M;

    float dt_s = (float)FB_PERIOD_MS / 1000.0f;

    //ひとまず速度データのみでオドメトリを計算する。（積分誤差あり。）
    odometry->theta_rad += v_angular * dt_s;
    odometry->x_m += v_linear * cos(odometry->theta_rad) * dt_s;
    odometry->y_m += v_linear * sin(odometry->theta_rad) * dt_s;
}

static void send_usb_feedback(const feedback_t *fb, const odometry_t *odometry)
{
    int32_t pid_error = g_latest_feedback[fb->motor_id].pid_error;
    float pid_output = g_latest_feedback[fb->motor_id].pid_output;

    if (g_usb_fb_format == USB_FB_COMPACT) {
        printf("{\"motor_id\":%u,\"velocity\":%d,\"current\":%d,"
               "\"pid_error\":%ld,\"pid_output\":%.6f}\n",
               fb->motor_id, fb->velocity, fb->current,
               (long)pid_error, (double)pid_output);
    } else if (g_usb_fb_format == USB_FB_PID_ONLY) {
        printf("{\"motor_id\":%u,\"pid_error\":%ld,\"pid_output\":%.6f}\n",
               fb->motor_id, (long)pid_error, (double)pid_output);
    } else {
        printf("{\"motor_id\":%u,\"angle\":%d,\"velocity\":%d,\"current\":%d,"
               "\"fault\":%u,\"mode\":%u,\"pid_error\":%ld,\"pid_output\":%.6f,"
               "\"x_m\":%.6f,\"y_m\":%.6f,\"theta_rad\":%.6f}\n",
               fb->motor_id, fb->angle, fb->velocity, fb->current,
               fb->fault, fb->mode, (long)pid_error, (double)pid_output, odometry->x_m, odometry->y_m, odometry->theta_rad);
    }
    fflush(stdout);
}


static bool parse_can_feedback_message(uint32_t can_id, const uint8_t *data, uint8_t len,
                                       feedback_t *fb)
{
    if (len < 8) {
        return false;
    }
    int32_t motor_id = (int32_t)can_id - FB_ID_BASE;
    if (motor_id != MOTOR1_ID && motor_id != MOTOR2_ID) {
        return false;
    }
    fb->motor_id = (uint8_t)motor_id;
    fb->velocity = s16_from_bigendian(data[0], data[1]);
    fb->current = s16_from_bigendian(data[2], data[3]);
    fb->angle = s16_from_bigendian(data[4], data[5]);
    fb->fault = data[6];
    fb->mode = data[7];
    return true;
}

static const char *skip_ws(const char *p)
{
    while (p && *p && isspace((unsigned char)*p)) {
        p++;
    }
    return p;
}

static const char *find_json_key(const char *json, const char *key)
{
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    return strstr(json, pattern);
}

static bool json_get_string(const char *json, const char *key, char *out, size_t out_size)
{
    const char *p = find_json_key(json, key);
    if (!p) {
        return false;
    }
    p = strchr(p, ':');
    if (!p) {
        return false;
    }
    p = skip_ws(p + 1);
    if (!p || *p != '"') {
        return false;
    }
    p++;
    const char *end = strchr(p, '"');
    if (!end) {
        return false;
    }
    size_t len = (size_t)(end - p);
    if (len >= out_size) {
        len = out_size - 1;
    }
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static bool json_get_number(const char *json, const char *key, double *out)
{
    const char *p = find_json_key(json, key);
    if (!p) {
        return false;
    }
    p = strchr(p, ':');
    if (!p) {
        return false;
    }
    p = skip_ws(p + 1);
    if (!p) {
        return false;
    }
    char *endptr = NULL;
    double val = strtod(p, &endptr);
    if (endptr == p) {
        return false;
    }
    *out = val;
    return true;
}

static bool parse_json_command_line(const char *line, command_t *cmd)
{
    char mode[32];
    if (!json_get_string(line, "mode", mode, sizeof(mode))) {
        return false;
    }

    memset(cmd, 0, sizeof(*cmd));
    cmd->mode = CMD_INVALID;

    if (strcmp(mode, "feedback_config") == 0) {
        cmd->mode = CMD_FEEDBACK_CONFIG;
        char fmt[16];
        if (json_get_string(line, "format", fmt, sizeof(fmt))) {
            if (strcmp(fmt, "compact") == 0) {
                cmd->format = USB_FB_COMPACT;
                cmd->has_format = true;
            } else if (strcmp(fmt, "pid_only") == 0) {
                cmd->format = USB_FB_PID_ONLY;
                cmd->has_format = true;
            } else if (strcmp(fmt, "mirror") == 0) {
                cmd->format = USB_FB_MIRROR;
                cmd->has_format = true;
            }
        }
        double period_val = 0.0;
        if (json_get_number(line, "period_ms", &period_val)) {
            cmd->has_period = true;
            cmd->period_ms = (int32_t)period_val;
        }
        return true;
    }

    if (strcmp(mode, "disable") == 0) {
        cmd->mode = CMD_DISABLE;
    } else if (strcmp(mode, "voltage_open_loop") == 0) {
        cmd->mode = CMD_VOLTAGE_OPEN_LOOP;
    } else if (strcmp(mode, "current_loop") == 0) {
        cmd->mode = CMD_CURRENT_LOOP;
    } else if (strcmp(mode, "velocity_loop") == 0) {
        cmd->mode = CMD_VELOCITY_LOOP;
    } else if (strcmp(mode, "position_loop") == 0) {
        cmd->mode = CMD_POSITION_LOOP;
    } else if (strcmp(mode, "rpm_pid") == 0) {
        cmd->mode = CMD_RPM_PID;
    } else {
        return false;
    }

    cmd->value1 = 0.0;
    cmd->value2 = 0.0;
    json_get_number(line, "value1", &cmd->value1);
    json_get_number(line, "value2", &cmd->value2);
    return true;
}

static void apply_command(const command_t *cmd)
{
    switch (cmd->mode) {
    case CMD_FEEDBACK_CONFIG:
        if (cmd->has_format) {
            g_usb_fb_format = cmd->format;
        }
        if (cmd->has_period) {
            if (cmd->period_ms <= 0) {
                g_usb_fb_period_ms = USB_FB_PERIOD_NONE;
            } else {
                g_usb_fb_period_ms = cmd->period_ms;
            }
        }
        break;
    case CMD_DISABLE:
        disable_rpm_pid();
        send_disable_commands();
        break;
    case CMD_VOLTAGE_OPEN_LOOP: {
        disable_rpm_pid();
        set_mode_voltage_open_loop();
        int16_t raw1 = current_a_to_raw((float)cmd->value1);
        int16_t raw2 = current_a_to_raw((float)cmd->value2);
        send_voltage_commands(raw1, raw2);
        break;
    }
    case CMD_CURRENT_LOOP: {
        disable_rpm_pid();
        set_mode_current_loop();
        int16_t raw1 = current_a_to_raw((float)cmd->value1);
        int16_t raw2 = current_a_to_raw((float)cmd->value2);
        send_current_commands(raw1, raw2);
        break;
    }
    case CMD_VELOCITY_LOOP:
        disable_rpm_pid();
        set_mode_velocity_loop();
        send_velocity_commands((int16_t)cmd->value1, (int16_t)cmd->value2);
        break;
    case CMD_POSITION_LOOP:
        disable_rpm_pid();
        set_mode_position_loop();
        send_position_commands((int16_t)cmd->value1, (int16_t)cmd->value2);
        break;
    case CMD_RPM_PID:
        enable_rpm_pid((int32_t)cmd->value1, (int32_t)cmd->value2);
        break;
    default:
        break;
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(200);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, false);

    xl2515_init(KBPS500);

    memset(g_latest_feedback, 0, sizeof(g_latest_feedback));
    g_latest_feedback[MOTOR1_ID].mode = 0xFF;
    g_latest_feedback[MOTOR2_ID].mode = 0xFF;
    g_pending_mode[MOTOR1_ID] = -1;
    g_pending_mode[MOTOR2_ID] = -1;

    g_pid_motor1.kp = PID_KP;
    g_pid_motor1.ki = PID_KI;
    g_pid_motor1.kd = PID_KD;
    g_pid_motor1.i_limit_a = I_LIMIT_A;
    pid_reset(&g_pid_motor1);

    g_pid_motor2.kp = PID_KP;
    g_pid_motor2.ki = PID_KI;
    g_pid_motor2.kd = PID_KD;
    g_pid_motor2.i_limit_a = I_LIMIT_A;
    pid_reset(&g_pid_motor2);

    set_feedback_method();

    char line_buf[256];
    size_t line_len = 0;
    uint64_t last_pid_ms = now_ms();
    uint64_t last_mode_refresh_ms = 0;

    while (true) {
        uint64_t current_ms = now_ms();

        int ch = getchar_timeout_us(0);
        while (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\r') {
                /* ignore */
            } else if (ch == '\n') {
                line_buf[line_len] = '\0';
                if (line_len > 0) {
                    command_t cmd;
                    if (parse_json_command_line(line_buf, &cmd)) {
                        apply_command(&cmd);
                    }
                }
                line_len = 0;
            } else {
                if (line_len < sizeof(line_buf) - 1) {
                    line_buf[line_len++] = (char)ch;
                } else {
                    line_len = 0;
                }
            }
            ch = getchar_timeout_us(0);
        }

        uint32_t can_id = 0;
        uint8_t data[8];
        uint8_t len = 0;
        while (xl2515_recv(&can_id, data, &len)) {
            note_communication();
            feedback_t fb;
            if (parse_can_feedback_message(can_id, data, len, &fb)) {
                g_latest_feedback[fb.motor_id].velocity = fb.velocity;
                g_latest_feedback[fb.motor_id].mode = fb.mode;
                g_latest_motor_feedback[fb.motor_id] = fb;
                g_has_motor_feedback[fb.motor_id] = true;
                if (g_has_motor_feedback[MOTOR1_ID] && g_has_motor_feedback[MOTOR2_ID]) {
                    calc_robot_odometry(&g_latest_motor_feedback[MOTOR1_ID],
                                        &g_latest_motor_feedback[MOTOR2_ID],
                                        &g_odometry);
                }
                if (g_pending_mode[fb.motor_id] == (int8_t)fb.mode) {
                    g_pending_mode[fb.motor_id] = -1;
                }
                uint64_t fb_ms = now_ms();
                if (usb_feedback_due(fb_ms)) {
                    send_usb_feedback(&fb, &g_odometry);
                }
            }
        }

        if ((current_ms - last_pid_ms) >= CONTROL_PERIOD_MS) {
            float dt = (float)(current_ms - last_pid_ms) / 1000.0f;
            last_pid_ms = current_ms;
            if (g_pid_enabled && dt > 0.0f) {
                if ((current_ms - last_mode_refresh_ms) >= 500) {
                    set_mode_current_loop();
                    last_mode_refresh_ms = current_ms;
                }

                int16_t v1 = g_latest_feedback[MOTOR1_ID].velocity;
                int16_t v2 = g_latest_feedback[MOTOR2_ID].velocity;

                int32_t err1 = g_pid_targets[MOTOR1_ID] - v1;
                int32_t err2 = g_pid_targets[MOTOR2_ID] - v2;

                float cmd1_a = pid_update(&g_pid_motor1, (float)err1, dt);
                float cmd2_a = pid_update(&g_pid_motor2, (float)err2, dt);

                g_latest_feedback[MOTOR1_ID].pid_error = err1;
                g_latest_feedback[MOTOR1_ID].pid_output = cmd1_a;
                g_latest_feedback[MOTOR2_ID].pid_error = err2;
                g_latest_feedback[MOTOR2_ID].pid_output = cmd2_a;

                int16_t raw1 = current_a_to_raw(cmd1_a);
                int16_t raw2 = current_a_to_raw(cmd2_a);
                send_current_commands(raw1, raw2);
            }
        }

        if (g_led_state && (current_ms - g_last_comm_ms) > LED_IDLE_OFF_MS) {
            g_led_state = false;
            gpio_put(LED_PIN, false);
        }

        sleep_ms(1);
    }
}
