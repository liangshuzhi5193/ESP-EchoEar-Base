#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_random.h"
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
    // ESP_LOGD(TAG, "Stepper motor rotating clockwise %d steps", steps);
    for (int i = 0; i < steps; i++) {
        stepper_step_cw(i);
        precise_delay_us(delay_us);
    }
}

// 步进电机反转指定步数
static void stepper_rotate_ccw(int steps, int delay_us)
{
    // ESP_LOGD(TAG, "Stepper motor rotating counter-clockwise %d steps", steps);
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
        // ESP_LOGD(TAG, "Stepper motor rotating %.1f degrees clockwise (%d half-steps, %dus/step)", angle, steps, delay_us);
        stepper_rotate_cw(steps, delay_us);
    } else {
        // 负角度，向左转动
        // ESP_LOGD(TAG, "Stepper motor rotating %.1f degrees counter-clockwise (%d half-steps, %dus/step)", angle, -steps, delay_us);
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
    
    // ESP_LOGD(TAG, "Accel-Decel profile: accel=%d, constant=%d, decel=%d steps", 
    //          accel_steps, constant_steps, decel_steps);
    
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
    
    // ESP_LOGD(TAG, "Accel-Decel profile: accel=%d, constant=%d, decel=%d steps", 
    //          accel_steps, constant_steps, decel_steps);
    
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
        // ESP_LOGI(TAG, "Rotating %.1f° CW with acceleration (%d steps, target %dus/step)", 
        //          angle, steps, target_delay_us);
        stepper_rotate_cw_with_accel(steps, target_delay_us);
    } else {
        // ESP_LOGI(TAG, "Rotating %.1f° CCW with acceleration (%d steps, target %dus/step)", 
        //          angle, -steps, target_delay_us);
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
    
    // ESP_LOGI(TAG, "Shake head started: amplitude=%.1f°, cycles=%d, speed=%dus/step", 
    //          amplitude, cycles, speed_us);
    
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
    
    // ESP_LOGI(TAG, "Shake head completed");
}

// 渐变递减摇头动作函数
// initial_amplitude: 起始摇头幅度（单边角度），例如30表示起始左右各摇30度
// decay_rate: 衰减率，范围0.0~1.0，例如0.8表示每次摇动后幅度变为原来的80%
//             或者 >1.0 表示每次减少的固定角度，例如5.0表示每次减少5度
// speed_us: 摇头速度（微秒延时），建议使用 STEPPER_SPEED_ULTRA_FAST 等宏
void stepper_shake_head_decay(float initial_amplitude, float decay_rate, int speed_us)
{
    if (initial_amplitude <= 0) {
        ESP_LOGW(TAG, "Invalid amplitude: %.1f", initial_amplitude);
        return;
    }
    
    if (decay_rate <= 0) {
        ESP_LOGW(TAG, "Invalid decay rate: %.3f", decay_rate);
        return;
    }
    
    // ESP_LOGI(TAG, "Decay shake head started: initial_amplitude=%.1f°, decay_rate=%.3f, speed=%dus/step", 
    //          initial_amplitude, decay_rate, speed_us);
    
    float current_amplitude = initial_amplitude;
    float min_amplitude = 5.0;  // 最小幅度阈值（度），小于此值停止摇动，避免小角度抖动
    float smooth_threshold = 8.0;  // 平滑阈值（度），小于此值时降低速度
    int cycle_count = 0;
    bool use_percentage_decay = (decay_rate > 0.0 && decay_rate < 1.0);  // 判断是百分比衰减还是固定值衰减
    float current_position = 0.0;  // 记录当前相对中心的位置
    
    // 第一步：从中心向左转到起始位置
    // ESP_LOGD(TAG, "Moving to left start position (%.1f°)", -current_amplitude);
    stepper_rotate_angle_with_accel(-current_amplitude, speed_us);
    current_position = -current_amplitude;
    vTaskDelay(pdMS_TO_TICKS(10));  // 短暂停顿
    
    // 摇头循环，直到幅度小于阈值
    while (current_amplitude >= min_amplitude) {
        cycle_count++;
        
        // 根据当前幅度动态调整速度，幅度越小速度越慢，避免抖动
        int current_speed_us = speed_us;
        if (current_amplitude < smooth_threshold) {
            // 线性插值：幅度从smooth_threshold降到min_amplitude时，速度因子从1.0增加到1.3（最多降速30%）
            float speed_factor = 1.0 + 0.3 * (smooth_threshold - current_amplitude) / (smooth_threshold - min_amplitude);
            current_speed_us = (int)(speed_us * speed_factor);
            // ESP_LOGI(TAG, "Decay shake cycle %d: amplitude=%.1f°, speed adjusted to %dus/step (factor=%.2f)", 
            //          cycle_count, current_amplitude, current_speed_us, speed_factor);
        } else {
            // ESP_LOGI(TAG, "Decay shake cycle %d: amplitude=%.1f°", cycle_count, current_amplitude);
        }
        
        // 向右转（从左到右，转动 2*current_amplitude 度）
        stepper_rotate_angle_with_accel(2 * current_amplitude, current_speed_us);
        current_position += 2 * current_amplitude;
        vTaskDelay(pdMS_TO_TICKS(15));  // 短暂停顿，幅度小时增加停顿时间
        
        // 计算下一次的幅度
        float next_amplitude;
        if (use_percentage_decay) {
            // 百分比衰减模式：幅度 = 当前幅度 × 衰减率
            next_amplitude = current_amplitude * decay_rate;
            // ESP_LOGD(TAG, "Percentage decay: next amplitude=%.1f° (%.1f%%)", 
            //          next_amplitude, decay_rate * 100);
        } else {
            // 固定值衰减模式：幅度 = 当前幅度 - 衰减值
            next_amplitude = current_amplitude - decay_rate;
            // ESP_LOGD(TAG, "Fixed decay: next amplitude=%.1f° (-%.1f°)", 
            //          next_amplitude, decay_rate);
        }
        
        // 检查是否需要继续摇动
        if (next_amplitude < min_amplitude) {
            // ESP_LOGD(TAG, "Next amplitude too small (%.1f° < %.1f°), stopping at position %.1f°", 
            //          next_amplitude, min_amplitude, current_position);
            break;
        }
        
        current_amplitude = next_amplitude;
        
        // 根据当前幅度动态调整速度
        current_speed_us = speed_us;
        if (current_amplitude < smooth_threshold) {
            float speed_factor = 1.0 + 0.3 * (smooth_threshold - current_amplitude) / (smooth_threshold - min_amplitude);
            current_speed_us = (int)(speed_us * speed_factor);
        }
        
        // 向左转（从右到左，转动 2*current_amplitude 度）
        stepper_rotate_angle_with_accel(-2 * current_amplitude, current_speed_us);
        current_position -= 2 * current_amplitude;
        vTaskDelay(pdMS_TO_TICKS(15));  // 短暂停顿
        
        // 再次计算下一次的幅度（对称衰减）
        if (use_percentage_decay) {
            next_amplitude = current_amplitude * decay_rate;
        } else {
            next_amplitude = current_amplitude - decay_rate;
        }
        
        // 检查下一次幅度
        if (next_amplitude < min_amplitude) {
            // ESP_LOGD(TAG, "Next amplitude too small (%.1f° < %.1f°), stopping at position %.1f°", 
            //          next_amplitude, min_amplitude, current_position);
            break;
        }
        
        current_amplitude = next_amplitude;
    }
    
    // 最后一步：回到中心位置
    // ESP_LOGI(TAG, "Returning to center from position %.1f°", current_position);
    
    // 回中心时使用稍慢的速度，确保平滑
    float abs_position = (current_position > 0) ? current_position : -current_position;
    int return_speed_us = speed_us;
    if (abs_position < smooth_threshold) {
        // 如果剩余角度较小，稍微降低回中心的速度（最多降速20%）
        float speed_factor = 1.2;
        return_speed_us = (int)(speed_us * speed_factor);
        // ESP_LOGD(TAG, "Using slower speed for return: %dus/step", return_speed_us);
    }
    
    stepper_rotate_angle_with_accel(-current_position, return_speed_us);
    
    // ESP_LOGI(TAG, "Decay shake head completed: total cycles=%d", cycle_count);
}

// 生成随机角度偏移（±max_offset范围内）
static float get_random_angle_offset(float max_offset)
{
    if (max_offset <= 0) {
        return 0;
    }
    
    // 生成0到1之间的随机数
    uint32_t random_value = esp_random();
    float normalized = (float)(random_value % 10000) / 10000.0;  // 0.0 ~ 1.0
    
    // 转换为-max_offset到+max_offset的范围
    float offset = (normalized * 2.0 - 1.0) * max_offset;
    
    return offset;
}

// 左顾右盼动作函数（带小幅度扫视观察 + 随机偏移）
// left_angle: 向左转的主要角度（正值），例如45表示向左转45度
// right_angle: 向右转的主要角度（正值），例如45表示向右转45度
// scan_angle: 在左右两侧小幅度扫视的角度（正值），例如10表示小幅度左右扫10度
// pause_ms: 每次停顿的时间（毫秒），模拟"观察"的动作
// large_speed_us: 大幅度转动速度（微秒延时），用于主要位置转动
// small_speed_us: 小幅度扫视速度（微秒延时），用于小范围观察，建议比large_speed_us慢
//
// 注意：每次转动都会添加随机偏移，让动作更自然
//   - 大幅度转动：随机偏移±20°
//   - 小幅度扫视：随机偏移±10°
//
// 动作流程（以left_angle=45, scan_angle=10为例，实际会有随机偏移）：
// 1. 向左转到-45°±20° (大速度) → 停顿
// 2. 向左小扫到-55°±10° (小速度) → 停顿
// 3. 向右小扫到-35°±10° (小速度) → 停顿
// 4. 向右大转到+45°±20° (大速度) → 停顿
// 5. 向右小扫到+55°±10° (小速度) → 停顿
// 6. 向左小扫到+35°±10° (小速度) → 停顿
// 7. 回到中心0° (大速度)
void stepper_look_around(float left_angle, float right_angle, float scan_angle, int pause_ms, int large_speed_us, int small_speed_us)
{
    if (left_angle < 0 || right_angle < 0 || scan_angle < 0) {
        ESP_LOGW(TAG, "Invalid angles: left=%.1f, right=%.1f, scan=%.1f (should be positive)", 
                 left_angle, right_angle, scan_angle);
        return;
    }
    
    if (pause_ms < 0) {
        pause_ms = 0;
    }
    
    // ESP_LOGI(TAG, "Look around started: left=%.1f°, right=%.1f°, scan=%.1f°, pause=%dms, large_speed=%dus, small_speed=%dus", 
    //          left_angle, right_angle, scan_angle, pause_ms, large_speed_us, small_speed_us);
    
    // 累积位置追踪（相对于起始中心位置）
    float accumulated_position = 0.0;
    
    // ========== 第一阶段：向左看 ==========
    if (left_angle > 0) {
        // ESP_LOGI(TAG, "Phase 1: Looking left");
        
        // 1. 向左转到主要位置（大幅度，快速）+ 随机偏移
        float left_offset = get_random_angle_offset(20.0);  // ±20度随机
        float actual_left_angle = left_angle + left_offset;
        // ESP_LOGD(TAG, "Turn left: target=%.2f°, offset=%.2f°, actual=%.2f°", 
        //          -left_angle, left_offset, -actual_left_angle);
        stepper_rotate_angle_with_accel(-actual_left_angle, large_speed_us);
        accumulated_position -= actual_left_angle;  // 记录累积位置
        // ESP_LOGD(TAG, "After left turn: accumulated_position=%.2f°", accumulated_position);
        vTaskDelay(pdMS_TO_TICKS(pause_ms));
        
        // 2. 向左小幅度扫视（慢速）+ 随机偏移
        if (scan_angle > 0) {
            float scan_left_offset = get_random_angle_offset(10.0);  // ±10度随机
            float actual_scan_left = scan_angle + scan_left_offset;
            
            // 确保小幅度转动不小于5度
            if (actual_scan_left < 5.0) {
                actual_scan_left = 5.0;
                scan_left_offset = actual_scan_left - scan_angle;
            }
            
            // ESP_LOGD(TAG, "Scan left: -%.2f° (scan_angle=%.2f, offset=%.2f)", 
            //          actual_scan_left, scan_angle, scan_left_offset);
            stepper_rotate_angle_with_accel(-actual_scan_left, small_speed_us);
            accumulated_position -= actual_scan_left;  // 记录累积位置
            // ESP_LOGD(TAG, "After scan left: accumulated_position=%.2f°", accumulated_position);
            vTaskDelay(pdMS_TO_TICKS(pause_ms));
            
            // 3. 直接向右小幅度扫视（慢速，不回主位置）+ 随机偏移
            float scan_right_offset = get_random_angle_offset(10.0);  // ±10度随机
            float actual_scan_range = 2 * scan_angle + scan_left_offset - scan_right_offset;
            
            // 确保小幅度转动不小于5度
            if (actual_scan_range < 5.0) {
                actual_scan_range = 5.0;
            }
            
            // ESP_LOGD(TAG, "Scan right: +%.2f°", actual_scan_range);
            stepper_rotate_angle_with_accel(actual_scan_range, small_speed_us);
            accumulated_position += actual_scan_range;  // 记录累积位置
            // ESP_LOGD(TAG, "After scan right: accumulated_position=%.2f°", accumulated_position);
            vTaskDelay(pdMS_TO_TICKS(pause_ms));
        }
    }
    
    // ========== 第二阶段：向右看 ==========
    if (right_angle > 0) {
        // ESP_LOGI(TAG, "Phase 2: Looking right");
        
        // 4. 从当前位置大幅度转到右侧主要位置（快速）+ 随机偏移
        float right_offset = get_random_angle_offset(20.0);  // ±20度随机
        float actual_right_angle = right_angle + right_offset;
        
        // 计算转动角度：从当前累积位置转到右侧
        float turn_angle = actual_right_angle - accumulated_position;
        
        // ESP_LOGD(TAG, "Turn right: target=%.2f°, current_pos=%.2f°, turn=%.2f°", 
        //          actual_right_angle, accumulated_position, turn_angle);
        stepper_rotate_angle_with_accel(turn_angle, large_speed_us);
        accumulated_position += turn_angle;  // 累加实际转动的角度，避免精度误差
        // ESP_LOGD(TAG, "After right turn: accumulated_position=%.2f°", accumulated_position);
        vTaskDelay(pdMS_TO_TICKS(pause_ms));
        
        // 5. 向右小幅度扫视（慢速）+ 随机偏移
        if (scan_angle > 0) {
            float scan_right2_offset = get_random_angle_offset(10.0);  // ±10度随机
            float actual_scan_right = scan_angle + scan_right2_offset;
            
            // 确保小幅度转动不小于5度
            if (actual_scan_right < 5.0) {
                actual_scan_right = 5.0;
                scan_right2_offset = actual_scan_right - scan_angle;
            }
            
            // ESP_LOGD(TAG, "Scan right: +%.2f° (scan_angle=%.2f, offset=%.2f)", 
            //          actual_scan_right, scan_angle, scan_right2_offset);
            stepper_rotate_angle_with_accel(actual_scan_right, small_speed_us);
            accumulated_position += actual_scan_right;  // 记录累积位置
            // ESP_LOGD(TAG, "After scan right: accumulated_position=%.2f°", accumulated_position);
            vTaskDelay(pdMS_TO_TICKS(pause_ms));
            
            // 6. 直接向左小幅度扫视（慢速，不回主位置）+ 随机偏移
            float scan_left2_offset = get_random_angle_offset(10.0);  // ±10度随机
            float actual_scan_range2 = 2 * scan_angle + scan_right2_offset - scan_left2_offset;
            
            // 确保小幅度转动不小于5度
            if (actual_scan_range2 < 5.0) {
                actual_scan_range2 = 5.0;
            }
            
            // ESP_LOGD(TAG, "Scan left: -%.2f°", actual_scan_range2);
            stepper_rotate_angle_with_accel(-actual_scan_range2, small_speed_us);
            accumulated_position -= actual_scan_range2;  // 记录累积位置
            // ESP_LOGD(TAG, "After scan left: accumulated_position=%.2f°", accumulated_position);
            vTaskDelay(pdMS_TO_TICKS(pause_ms));
        }
    }
    
    // ========== 第三阶段：回到中心 ==========
    // ESP_LOGI(TAG, "Phase 3: Returning to center");
    
    // 使用累积位置直接回到中心（容差：0.5度）
    float abs_position = (accumulated_position > 0) ? accumulated_position : -accumulated_position;
    if (abs_position > 0.5) {
        // ESP_LOGD(TAG, "Returning to center from %.2f°", accumulated_position);
        stepper_rotate_angle_with_accel(-accumulated_position, large_speed_us);
        // ESP_LOGD(TAG, "Returned to center");
    } else {
        // ESP_LOGD(TAG, "Already near center (offset=%.2f°)", accumulated_position);
    }
    
    // ESP_LOGI(TAG, "Look around completed");
}

// 跟随鼓点左右摆头函数
// angle: 每次摆动的角度（正值），例如10表示左右各摆10度
// speed_us: 转动速度（微秒延时）
//
// 每次调用此函数，会自动切换方向：
// 第1次调用：向左转
// 第2次调用：向右转
// 第3次调用：向左转
// ...依此类推
void stepper_beat_swing(float angle, int speed_us)
{
    // 静态变量记录当前摆动方向，false=向左，true=向右
    static bool swing_direction = false;
    
    if (angle <= 0) {
        ESP_LOGW(TAG, "Invalid angle: %.1f (should be positive)", angle);
        return;
    }
    
    if (swing_direction) {
        // 向右转
        // ESP_LOGD(TAG, "Beat swing: turning right +%.1f° at %dus/step", angle, speed_us);
        stepper_rotate_angle_with_accel(angle, speed_us);
    } else {
        // 向左转
        // ESP_LOGD(TAG, "Beat swing: turning left -%.1f° at %dus/step", angle, speed_us);
        stepper_rotate_angle_with_accel(-angle, speed_us);
    }
    
    // 切换方向
    swing_direction = !swing_direction;
}

// 猫咪蹭手动作函数（温柔的向左扭头并回中心，反复几次）
// angle: 向左扭头的角度（正值），例如20表示向左扭20度
// cycles: 蹭头次数，每次包含一个完整的"扭头-回中心"动作
// speed_us: 转动速度（微秒延时），建议使用较慢的速度如 STEPPER_SPEED_SLOW 来营造温柔感
//
// 动作流程（以angle=20, cycles=3为例）：
// 1. 向左扭头到-20° (缓慢) → 短暂停顿 → 回到中心0° (缓慢) → 短暂停顿
// 2. 向左扭头到-20° (缓慢) → 短暂停顿 → 回到中心0° (缓慢) → 短暂停顿
// 3. 向左扭头到-20° (缓慢) → 短暂停顿 → 回到中心0° (缓慢)
//
// 特点：
// - 全程使用缓慢平滑的加减速，营造温柔感
// - 每次扭头后有短暂停顿，模拟猫咪蹭手时的接触感
// - 回中心时也很平缓，不会突然
void stepper_cat_nuzzle(float angle, int cycles, int speed_us)
{
    if (angle <= 0) {
        ESP_LOGW(TAG, "Invalid angle: %.1f (should be positive)", angle);
        return;
    }
    
    if (cycles <= 0) {
        ESP_LOGW(TAG, "Invalid cycles: %d (should be positive)", cycles);
        return;
    }
    
    // ESP_LOGI(TAG, "Cat nuzzle started: angle=%.1f°, cycles=%d, speed=%dus/step", 
    //          angle, cycles, speed_us);
    
    // 执行蹭头循环
    for (int i = 0; i < cycles; i++) {
        // ESP_LOGI(TAG, "Nuzzle cycle %d/%d", i + 1, cycles);
        
        // 向左缓慢扭头
        stepper_rotate_angle_with_accel(-angle, speed_us);
        vTaskDelay(pdMS_TO_TICKS(100));  // 停顿100ms，模拟蹭手时的接触感
        
        // 缓慢回到中心
        stepper_rotate_angle_with_accel(angle, speed_us);
        
        // 在非最后一次循环时，添加短暂停顿
        if (i < cycles - 1) {
            vTaskDelay(pdMS_TO_TICKS(50));  // 每次蹭完短暂停顿，准备下一次
        }
    }
    
    // ESP_LOGI(TAG, "Cat nuzzle completed");
}

// 关闭步进电机所有线圈（断电）
// 调用此函数后，电机将不再保持位置，可被外力转动
void stepper_motor_power_off(void)
{
    set_motor_pins(0, 0, 0, 0);
    // ESP_LOGI(TAG, "Stepper motor powered off, all coils released");
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
    
    // ESP_LOGI(TAG, "GPIO initialization completed, IN1-IN4 pins set as output mode with default low level");
}
