#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "stepper_motor.h"
#include "control_serial.h"
#include "magnetic_slide_switch.h"

static const char *TAG = "Control Serial";

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
                if (cmd == CMD_BASE_ANGLE_CONTROL && len >= 8) {  // Base angle control
                    // Get data value (combine two bytes)
                    uint16_t value = (data[5] << 8) | data[6];
                    // Calculate checksum
                    calc_checksum += data[5] + data[6];
                    // printf("calc_checksum: 0x%02X\n", calc_checksum);
                    uint8_t received_checksum = data[7];

                    // Print parsing results
                    // ESP_LOGI(TAG, "=== Base Angle Control Command ===");
                    // ESP_LOGI(TAG, "Frame header: 0x%02X 0x%02X", data[0], data[1]);
                    // ESP_LOGI(TAG, "Data length: %d", data_len);
                    // ESP_LOGI(TAG, "Command: 0x%02X", cmd);
                    ESP_LOGI(TAG, "received angle value: %d", value);
                    // ESP_LOGI(TAG, "Checksum: 0x%02X (calculated: 0x%02X)", received_checksum, calc_checksum);

                    // Checksum verification
                    if (calc_checksum == received_checksum) {
                        static int16_t last_diff_angle = 0;
                        // ESP_LOGI(TAG, "Checksum verification passed");
                        int16_t diff_angle = value - 90;
                        printf("diff_angle: %d\n", diff_angle);
                        stepper_rotate_angle_with_accel(diff_angle - last_diff_angle, STEPPER_SPEED_ULTRA_FAST);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        stepper_motor_power_off();
                        last_diff_angle = diff_angle;
                        // printf("last_diff_angle: %d\n", last_diff_angle);
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

    xTaskCreate(uart_cmd_receive_task, "uart_cmd_receive_task", UART_CMD_TASK_STACK_SIZE, NULL, 12, NULL);
    return ESP_OK;
}

/**
 * @brief 发送磁吸滑动开关事件通知
 * 
 * @param event 事件类型 (magnetic_slide_switch_event_t)
 * @return esp_err_t 
 * 
 * 数据包格式：
 * AA 55 00 03 03 00 01 04
 * [0-1]: 帧头 (0xAA 0x55)
 * [2-3]: 数据长度 (0x00 0x03 = 3字节)
 * [4]:   命令码 (0x03 = 磁吸滑动开关事件)
 * [5-6]: 事件数据 (0x00 0x01 = SLIDE_DOWN)
 * [7]:   校验和 (命令码 + 数据字节之和)
 */
esp_err_t control_serial_send_magnetic_switch_event(uint16_t event)
{
    uint8_t tx_buffer[8];
    
    // 帧头
    tx_buffer[0] = UART_FRAME_HEAD_H;  // 0xAA
    tx_buffer[1] = UART_FRAME_HEAD_L;  // 0x55
    
    // 数据长度 (命令码1字节 + 数据2字节 = 3字节)
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x03;
    
    // 命令码
    tx_buffer[4] = CMD_MAGNETIC_SWITCH_EVENT;  // 0x03
    
    // 事件数据 (大端序，高字节在前)
    tx_buffer[5] = (event >> 8) & 0xFF;  // 高字节
    tx_buffer[6] = event & 0xFF;          // 低字节
    
    // 校验和 (命令码 + 数据高字节 + 数据低字节)
    tx_buffer[7] = tx_buffer[4] + tx_buffer[5] + tx_buffer[6];
    
    // 通过UART发送
    int sent = uart_write_bytes(ECHO_UART_PORT_NUM, tx_buffer, sizeof(tx_buffer));
    
    if (sent == sizeof(tx_buffer)) {
        ESP_LOGI(TAG, "Magnetic switch event sent: event=%d", event);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send magnetic switch event: sent=%d", sent);
        return ESP_FAIL;
    }
}
