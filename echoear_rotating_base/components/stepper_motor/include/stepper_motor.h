#pragma once

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IN1_PIN (GPIO_NUM_28)
#define IN2_PIN (GPIO_NUM_27)
#define IN3_PIN (GPIO_NUM_26)
#define IN4_PIN (GPIO_NUM_25)

// 步进电机速度配置（每步延时，单位：微秒 μs）
// 注意：使用半步驱动模式，精度提高2倍，运动更平滑
#define STEPPER_SPEED_ULTRA_FAST    600     // 超快速模式：800μs/步 (0.8ms)
#define STEPPER_SPEED_FAST          1000    // 快速模式：1000μs/步 (1ms)
#define STEPPER_SPEED_NORMAL        1500    // 正常模式：1500μs/步 (1.5ms)
#define STEPPER_SPEED_SLOW          2000    // 慢速模式：2000μs/步 (2ms)

// 加减速配置
#define STEPPER_START_DELAY_US      1500    // 启动延时（较慢，确保启动）
#define STEPPER_ACCEL_STEPS         30     // 加速步数
#define STEPPER_DECEL_STEPS         30     // 减速步数

// 步进电机动作类型枚举
typedef enum {
    STEPPER_ACTION_SHAKE_HEAD,          // 摇头动作（固定幅度和次数）
    STEPPER_ACTION_SHAKE_HEAD_DECAY,    // 渐变递减摇头动作
    STEPPER_ACTION_LOOK_AROUND,         // 左顾右盼观察动作
    STEPPER_ACTION_BEAT_SWING,          // 跟随鼓点左右摆头
    STEPPER_ACTION_CAT_NUZZLE,          // 猫咪蹭手动作
    STEPPER_ACTION_MAX                  // 动作类型数量（用于边界检查）
} stepper_action_type_t;

void stepper_rotate_angle(float angle, int delay_us);
void stepper_rotate_angle_with_accel(float angle, int target_delay_us);
void stepper_shake_head(float amplitude, int cycles, int speed_us);
void stepper_shake_head_decay(float initial_amplitude, float decay_rate, int speed_us);
void stepper_look_around(float left_angle, float right_angle, float scan_angle, int pause_ms, int large_speed_us, int small_speed_us);
void stepper_beat_swing(float angle, int speed_us);
void stepper_cat_nuzzle(float angle, int cycles, int speed_us);
void stepper_motor_power_off(void);
void stepper_motor_gpio_init(void);

#ifdef __cplusplus
}
#endif

