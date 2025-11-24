#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "stepper_motor.h"

static const char *TAG = "Stepper Motor";

// 8拍半步进序列 - 正转 (交替单相和双相励磁)
static const int step_sequence_cw[8][4] = {
    {1, 0, 0, 0},  // Step 1: IN1 (单相)
    {1, 1, 0, 0},  // Step 2: IN1+IN2 (双相)
    {0, 1, 0, 0},  // Step 3: IN2 (单相)
    {0, 1, 1, 0},  // Step 4: IN2+IN3 (双相)
    {0, 0, 1, 0},  // Step 5: IN3 (单相)
    {0, 0, 1, 1},  // Step 6: IN3+IN4 (双相)
    {0, 0, 0, 1},  // Step 7: IN4 (单相)
    {1, 0, 0, 1}   // Step 8: IN4+IN1 (双相)
};

// 8拍半步进序列 - 反转 (交替单相和双相励磁)
static const int step_sequence_ccw[8][4] = {
    {1, 0, 0, 1},  // Step 1: IN4+IN1 (双相)
    {0, 0, 0, 1},  // Step 2: IN4 (单相)
    {0, 0, 1, 1},  // Step 3: IN3+IN4 (双相)
    {0, 0, 1, 0},  // Step 4: IN3 (单相)
    {0, 1, 1, 0},  // Step 5: IN2+IN3 (双相)
    {0, 1, 0, 0},  // Step 6: IN2 (单相)
    {1, 1, 0, 0},  // Step 7: IN1+IN2 (双相)
    {1, 0, 0, 0}   // Step 8: IN1 (单相)
};

// 设置步进电机引脚状态
static void set_motor_pins(int in1, int in2, int in3, int in4)
{
    gpio_set_level(IN1_PIN, in1);
    gpio_set_level(IN2_PIN, in2);
    gpio_set_level(IN3_PIN, in3);
    gpio_set_level(IN4_PIN, in4);
}

// 步进电机正转一步
static void stepper_step_cw(int step)
{
    step = step % 8;  // 确保步数在0-7范围内（半步模式8步）
    set_motor_pins(step_sequence_cw[step][0], 
                   step_sequence_cw[step][1], 
                   step_sequence_cw[step][2], 
                   step_sequence_cw[step][3]);
}

// 步进电机反转一步
static void stepper_step_ccw(int step)
{
    step = step % 8;  // 确保步数在0-7范围内（半步模式8步）
    set_motor_pins(step_sequence_ccw[step][0], 
                   step_sequence_ccw[step][1], 
                   step_sequence_ccw[step][2], 
                   step_sequence_ccw[step][3]);
}

// 微秒级精确延时函数
static inline void precise_delay_us(int delay_us)
{
    if (delay_us > 0) {
        esp_rom_delay_us(delay_us);
    }
}

// 步进电机正转指定步数
static void stepper_rotate_cw(int steps, int delay_us)
{
    ESP_LOGD(TAG, "Stepper motor rotating clockwise %d steps", steps);
    for (int i = 0; i < steps; i++) {
        stepper_step_cw(i);
        precise_delay_us(delay_us);
    }
}

// 步进电机反转指定步数
static void stepper_rotate_ccw(int steps, int delay_us)
{
    ESP_LOGD(TAG, "Stepper motor rotating counter-clockwise %d steps", steps);
    for (int i = 0; i < steps; i++) {
        stepper_step_ccw(i);
        precise_delay_us(delay_us);
    }
}

// 步进电机按指定角度转动（固定速度，无加减速）
// angle: 转动角度，正值向右，负值向左
// delay_us: 每步之间的延时（微秒）
void stepper_rotate_angle(float angle, int delay_us)
{
    // 半步模式：4128步 = 360度 (原4拍2064步的2倍)
    // 计算所需步数：steps = angle * 4128 / 360
    int steps = (int)(angle * 4128.0 / 360.0 + 0.5); // +0.5用于四舍五入
    
    if (steps == 0) {
        ESP_LOGI(TAG, "Angle too small, no rotation needed");
        return;
    }
    
    if (steps > 0) {
        // 正角度，向右转动
        ESP_LOGD(TAG, "Stepper motor rotating %.1f degrees clockwise (%d half-steps, %dus/step)", angle, steps, delay_us);
        stepper_rotate_cw(steps, delay_us);
    } else {
        // 负角度，向左转动
        ESP_LOGD(TAG, "Stepper motor rotating %.1f degrees counter-clockwise (%d half-steps, %dus/step)", angle, -steps, delay_us);
        stepper_rotate_ccw(-steps, delay_us);
    }
}

// 带加减速的正转函数
static void stepper_rotate_cw_with_accel(int steps, int target_delay_us)
{
    int start_delay_us = STEPPER_START_DELAY_US;
    int accel_steps = STEPPER_ACCEL_STEPS;
    int decel_steps = STEPPER_DECEL_STEPS;
    
    // 如果总步数太少，调整加减速步数
    if (steps < (accel_steps + decel_steps)) {
        accel_steps = steps / 3;
        decel_steps = steps / 3;
    }
    
    int constant_steps = steps - accel_steps - decel_steps;
    int current_delay_us;
    
    ESP_LOGD(TAG, "Accel-Decel profile: accel=%d, constant=%d, decel=%d steps", 
             accel_steps, constant_steps, decel_steps);
    
    // 加速阶段
    for (int i = 0; i < accel_steps; i++) {
        // 线性插值：从 start_delay 逐渐减小到 target_delay
        current_delay_us = start_delay_us - (start_delay_us - target_delay_us) * i / accel_steps;
        stepper_step_cw(i);
        precise_delay_us(current_delay_us);
    }
    
    // 匀速阶段
    for (int i = accel_steps; i < accel_steps + constant_steps; i++) {
        stepper_step_cw(i);
        precise_delay_us(target_delay_us);
    }
    
    // 减速阶段
    for (int i = accel_steps + constant_steps; i < steps; i++) {
        // 线性插值：从 target_delay 逐渐增大到 start_delay
        int decel_progress = i - (accel_steps + constant_steps);
        current_delay_us = target_delay_us + (start_delay_us - target_delay_us) * decel_progress / decel_steps;
        stepper_step_cw(i);
        precise_delay_us(current_delay_us);
    }
}

// 带加减速的反转函数
static void stepper_rotate_ccw_with_accel(int steps, int target_delay_us)
{
    int start_delay_us = STEPPER_START_DELAY_US;
    int accel_steps = STEPPER_ACCEL_STEPS;
    int decel_steps = STEPPER_DECEL_STEPS;
    
    // 如果总步数太少，调整加减速步数
    if (steps < (accel_steps + decel_steps)) {
        accel_steps = steps / 3;
        decel_steps = steps / 3;
    }
    
    int constant_steps = steps - accel_steps - decel_steps;
    int current_delay_us;
    
    ESP_LOGD(TAG, "Accel-Decel profile: accel=%d, constant=%d, decel=%d steps", 
             accel_steps, constant_steps, decel_steps);
    
    // 加速阶段
    for (int i = 0; i < accel_steps; i++) {
        current_delay_us = start_delay_us - (start_delay_us - target_delay_us) * i / accel_steps;
        stepper_step_ccw(i);
        precise_delay_us(current_delay_us);
    }
    
    // 匀速阶段
    for (int i = accel_steps; i < accel_steps + constant_steps; i++) {
        stepper_step_ccw(i);
        precise_delay_us(target_delay_us);
    }
    
    // 减速阶段
    for (int i = accel_steps + constant_steps; i < steps; i++) {
        int decel_progress = i - (accel_steps + constant_steps);
        current_delay_us = target_delay_us + (start_delay_us - target_delay_us) * decel_progress / decel_steps;
        stepper_step_ccw(i);
        precise_delay_us(current_delay_us);
    }
}

// 步进电机按指定角度转动（带加减速）
// angle: 转动角度，正值向右，负值向左
// target_delay_us: 目标速度延时（微秒），启动时会自动从慢速加速到此速度
void stepper_rotate_angle_with_accel(float angle, int target_delay_us)
{
    // 半步模式：4128步 = 360度
    int steps = (int)(angle * 4128.0 / 360.0 + 0.5);
    
    if (steps == 0) {
        ESP_LOGI(TAG, "Angle too small, no rotation needed");
        return;
    }
    
    if (steps > 0) {
        ESP_LOGI(TAG, "Rotating %.1f° CW with acceleration (%d steps, target %dus/step)", 
                 angle, steps, target_delay_us);
        stepper_rotate_cw_with_accel(steps, target_delay_us);
    } else {
        ESP_LOGI(TAG, "Rotating %.1f° CCW with acceleration (%d steps, target %dus/step)", 
                 angle, -steps, target_delay_us);
        stepper_rotate_ccw_with_accel(-steps, target_delay_us);
    }
}

// 摇头动作函数
// amplitude: 摇头幅度（单边角度），例如30表示左右各摇30度
// cycles: 摇头次数，完整的左右摇动算1次
// speed_us: 摇头速度（微秒延时），建议使用 STEPPER_SPEED_ULTRA_FAST 等宏
void stepper_shake_head(float amplitude, int cycles, int speed_us)
{
    if (amplitude <= 0 || cycles <= 0) {
        ESP_LOGW(TAG, "Invalid shake head parameters: amplitude=%.1f, cycles=%d", amplitude, cycles);
        return;
    }
    
    ESP_LOGI(TAG, "Shake head started: amplitude=%.1f°, cycles=%d, speed=%dus/step", 
             amplitude, cycles, speed_us);
    
    // 第一步：从中心向左转到起始位置
    // ESP_LOGI(TAG, "Moving to left start position (%.1f°)", -amplitude);
    stepper_rotate_angle_with_accel(-amplitude, speed_us);
    vTaskDelay(pdMS_TO_TICKS(10));  // 短暂停顿
    
    // 摇头循环
    for (int i = 0; i < cycles; i++) {
        // ESP_LOGI(TAG, "Shake cycle %d/%d", i + 1, cycles);
        
        // 向右转（从左到右，转动 2*amplitude 度）
        stepper_rotate_angle_with_accel(2 * amplitude, speed_us);
        vTaskDelay(pdMS_TO_TICKS(10));  // 短暂停顿
        
        // 向左转（从右到左，转动 2*amplitude 度）
        stepper_rotate_angle_with_accel(-2 * amplitude, speed_us);
        vTaskDelay(pdMS_TO_TICKS(10));  // 短暂停顿
    }
    
    // 最后一步：从左边回到中心位置
    // ESP_LOGI(TAG, "Returning to center position");
    stepper_rotate_angle_with_accel(amplitude, speed_us);
    
    ESP_LOGI(TAG, "Shake head completed");
}

// 关闭步进电机所有线圈（断电）
// 调用此函数后，电机将不再保持位置，可被外力转动
void stepper_motor_power_off(void)
{
    set_motor_pins(0, 0, 0, 0);
    ESP_LOGI(TAG, "Stepper motor powered off, all coils released");
}

void stepper_motor_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) | (1ULL << IN3_PIN) | (1ULL << IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    gpio_config(&io_conf);
    
    // 设置所有引脚为低电平
    gpio_set_level(IN1_PIN, 0);
    gpio_set_level(IN2_PIN, 0);
    gpio_set_level(IN3_PIN, 0);
    gpio_set_level(IN4_PIN, 0);
    
    ESP_LOGI(TAG, "GPIO initialization completed, IN1-IN4 pins set as output mode with default low level");
}
