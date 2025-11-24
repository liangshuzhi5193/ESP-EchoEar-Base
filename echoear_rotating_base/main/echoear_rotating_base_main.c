#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "iot_button.h"

#include "control_serial.h"
#include "stepper_motor.h"
#include "magnetic_slide_switch.h"

static const char *TAG = "Echoear Rotating Base";

#define BASE_ANGLE_LIMIT_SWITCH_GPIO (GPIO_NUM_1)
#define BASE_CALIBRATION_TASK_STACK_SIZE    (1024 * 3)

static bool s_base_angle_limit_switch_pressed = false;

static void button_event_cb(void *arg, void *data)
{
    button_event_t event = iot_button_get_event(arg);
    if (event == BUTTON_PRESS_DOWN) {
        s_base_angle_limit_switch_pressed = true;
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

static void base_calibration_task(void *arg)
{
    ESP_LOGI(TAG, "Base calibration task started");
    base_angle_limit_switch_init();

    while (1) {
        if (!s_base_angle_limit_switch_pressed) {
            stepper_rotate_angle(-5, STEPPER_SPEED_FAST);
        } else {
            stepper_rotate_angle_with_accel(95.0, STEPPER_SPEED_ULTRA_FAST);
            stepper_motor_power_off();
            vTaskDelay(pdMS_TO_TICKS(100));
            control_serial_init();
            vTaskDelete(NULL);
        }
    }
}

void app_main(void)
{
    stepper_motor_gpio_init();
    base_angle_limit_switch_init();

    xTaskCreate(base_calibration_task, "base_calibration_task", BASE_CALIBRATION_TASK_STACK_SIZE, NULL, 10, NULL);
    magnetic_slide_switch_start();

    while (0) {
        stepper_shake_head(12.0, 2, 600);
        stepper_motor_power_off();
        vTaskDelay(pdMS_TO_TICKS(1000));

        stepper_shake_head(15.0, 2, 1000);
        stepper_motor_power_off();
        vTaskDelay(pdMS_TO_TICKS(1000));

        stepper_shake_head(8.0, 2, 600);
        stepper_motor_power_off();
        vTaskDelay(pdMS_TO_TICKS(1000));

        stepper_shake_head(20.0, 2, 1000);
        stepper_motor_power_off();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
