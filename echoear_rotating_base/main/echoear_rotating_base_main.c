#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "iot_button.h"
#include "nvs_flash.h"

#include "control_serial.h"
#include "stepper_motor.h"
#include "magnetic_slide_switch.h"

static const char *TAG = "Echoear Rotating Base";

#define BASE_ANGLE_LIMIT_SWITCH_GPIO (GPIO_NUM_1)
#define BOOT_BUTTON_GPIO             (GPIO_NUM_9)
#define BASE_CALIBRATION_TASK_STACK_SIZE    (1024 * 3)

static bool s_base_angle_limit_switch_pressed = false;

static void button_event_cb(void *arg, void *data)
{
    button_event_t event = iot_button_get_event(arg);
    if (event == BUTTON_PRESS_DOWN) {
        s_base_angle_limit_switch_pressed = true;
    }
}

// Boot 按键事件回调函数
static void boot_button_event_cb(void *arg, void *data)
{
    button_event_t event = iot_button_get_event(arg);
    if (event == BUTTON_LONG_PRESS_START) {
        ESP_LOGI(TAG, "Boot button long press detected, starting recalibration...");
        magnetic_slide_switch_start_recalibration();
    }
}

static void base_angle_limit_switch_init(void)
{
    button_handle_t btn = NULL;

    button_config_t cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = BASE_ANGLE_LIMIT_SWITCH_GPIO,
            .active_level = 0,
        },
    };
    btn = iot_button_create(&cfg);
    assert(btn != NULL);
    iot_button_register_cb(btn, BUTTON_PRESS_DOWN, button_event_cb, NULL);
    iot_button_register_cb(btn, BUTTON_PRESS_UP, button_event_cb, NULL);
}

static void boot_button_init(void)
{
    button_handle_t boot_btn = NULL;

    button_config_t boot_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,  // 使用默认长按时间（通常是 5000ms）
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = BOOT_BUTTON_GPIO,
            .active_level = 0,  // 低电平有效
        },
    };
    boot_btn = iot_button_create(&boot_cfg);
    assert(boot_btn != NULL);
    iot_button_register_cb(boot_btn, BUTTON_LONG_PRESS_START, boot_button_event_cb, NULL);
    
    ESP_LOGI(TAG, "Boot button initialized on GPIO%d (long press to recalibrate)", BOOT_BUTTON_GPIO);
}

static void base_calibration_task(void *arg)
{
    ESP_LOGI(TAG, "Base calibration task started");
    base_angle_limit_switch_init();

    while (1) {
        if (!s_base_angle_limit_switch_pressed) {
            stepper_rotate_angle(-5, STEPPER_SPEED_FAST);
            vTaskDelay(pdMS_TO_TICKS(2));
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
            stepper_rotate_angle_with_accel(95.0, STEPPER_SPEED_ULTRA_FAST);
            stepper_motor_power_off();
            vTaskDelete(NULL);
        }
    }
}

void app_main(void)
{
    // 初始化 NVS (用于保存校准数据)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS 分区已满或版本不匹配，擦除重新初始化
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
    
    stepper_motor_gpio_init();
    base_angle_limit_switch_init();
    boot_button_init();  // 初始化 Boot 按键
    control_serial_init();

    xTaskCreate(base_calibration_task, "base_calibration_task", BASE_CALIBRATION_TASK_STACK_SIZE, NULL, 10, NULL);
    magnetic_slide_switch_start();
}
