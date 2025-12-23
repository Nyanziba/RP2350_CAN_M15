#include <stdint.h>
// 最新のフィードバック情報
typedef struct {
    int16_t velocity;
    int32_t pid_error;
    float pid_output;
    uint8_t mode;
} latest_feedback_t;

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