#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "stepper_motor.h"
#include "control_serial.h"

static const char *TAG = "Control Serial";

static uint16_t s_base_angle = 0;

static void uart_cmd_receive_task(void *arg)
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            // Print received data in hexadecimal format
            // printf("data: %02X %02X %02X %02X %02X %02X %02X %02X\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
            // Check frame header
            if (data[0] == 0xAA && data[1] == 0x55) {
                // Get data length
                uint8_t data_len = data[3];
                // Get command code
                uint8_t cmd = data[4];
                uint8_t calc_checksum = cmd;

                // Parse data according to command type
                if (cmd == 0x01 && len >= 8) {  // Base angle control
                    // Get data value (combine two bytes)
                    uint16_t value = (data[5] << 8) | data[6];
                    // Calculate checksum
                    calc_checksum += data[5] + data[6];
                    // printf("calc_checksum: 0x%02X\n", calc_checksum);
                    uint8_t received_checksum = data[7];

                    // Print parsing results
                    ESP_LOGI(TAG, "=== Base Angle Control Command ===");
                    ESP_LOGI(TAG, "Frame header: 0x%02X 0x%02X", data[0], data[1]);
                    ESP_LOGI(TAG, "Data length: %d", data_len);
                    ESP_LOGI(TAG, "Command: 0x%02X", cmd);
                    ESP_LOGI(TAG, "received angle value: %d", value);
                    ESP_LOGI(TAG, "Checksum: 0x%02X (calculated: 0x%02X)", received_checksum, calc_checksum);

                    // Checksum verification
                    if (calc_checksum == received_checksum) {
                        ESP_LOGI(TAG, "Checksum verification passed");
                        int16_t diff_angle = value - 90;
                        printf("diff_angle: %d\n", diff_angle);
                        if (s_base_angle > 0 && s_base_angle < 180) {
                            stepper_rotate_angle_with_accel(diff_angle, STEPPER_SPEED_FAST);
                            stepper_motor_power_off();
                            ESP_LOGI(TAG, "Base angle: %d\n", s_base_angle);
                        }
                        s_base_angle += diff_angle;
                    } else {
                        ESP_LOGW(TAG, "Checksum verification failed");
                    }
                } else {
                    ESP_LOGW(TAG, "Invalid command or frame length");
                }
            }
        }
    }
}

esp_err_t control_serial_init(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, UART_ECHO_TXD, UART_ECHO_RXD, UART_ECHO_RTS, UART_ECHO_CTS));

    xTaskCreate(uart_cmd_receive_task, "uart_cmd_receive_task", UART_CMD_TASK_STACK_SIZE, NULL, 10, NULL);
    return ESP_OK;
}
