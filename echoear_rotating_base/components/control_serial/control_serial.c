#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "stepper_motor.h"
#include "control_serial.h"

// #include "magnetic_slide_switch.h"

static const char *TAG = "Control Serial";
extern void app_csi_uart_cb(uint8_t *data, size_t len);
static uint8_t calc_checksum_f(const uint8_t *payload, uint8_t payload_len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < payload_len; i++) {
        sum += payload[i];
        // printf("%d: payload[%d]: %02X, sum: %02X\n", payload_len, i, payload[i], sum);
    }
    return sum;
}
static void uart_cmd_receive_task(void *arg)
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            int offset = 0;  // 当前处理位置
            
            // 循环处理缓冲区中可能存在的多帧数据
            while (offset < len) {
                // 检查剩余字节是否足够一个最小帧（至少需要帧头2字节+长度2字节+命令1字节+校验和1字节=6字节）
                if (offset + 6 > len) {
                    if (offset < len) {
                        ESP_LOGW(TAG, "Incomplete frame, remaining bytes: %d", len - offset);
                    }
                    break;
                }
                
                // Check frame header
                if (data[offset] == 0xAA && data[offset + 1] == 0x55) {
                    // Get data length (包含命令码的长度)
                    uint16_t data_len = data[offset + 3] | (data[offset + 2] << 8);
                    
                    // 计算整个帧的长度：帧头(2) + 长度字段(2) + 数据 + 校验和(1)
                    uint16_t frame_len = 4 + data_len + 1;
                    
                    // 检查剩余数据是否足够一个完整帧
                    if (offset + frame_len > len) {
                        ESP_LOGW(TAG, "Incomplete frame, expected %d bytes, remaining %d bytes", frame_len, len - offset);
                        break;
                    }
                    
                    // Get command code
                    uint8_t cmd = data[offset + 4];
                    uint8_t calc_checksum = cmd;

                    // Parse data according to command type
                    if (cmd == CMD_BASE_ANGLE_CONTROL && data_len >= 3) {  // Base angle control
                        // Get data value (combine two bytes)
                        uint16_t value = (data[offset + 5] << 8) | data[offset + 6];
                        // Calculate checksum
                        calc_checksum += data[offset + 5] + data[offset + 6];
                        uint8_t received_checksum = data[offset + 7];

                        ESP_LOGI(TAG, "received angle value: %d", value);

                        // Checksum verification
                        if (calc_checksum == received_checksum) {
                            static int16_t last_diff_angle = 0;
                            int16_t diff_angle = value - 90;
                            printf("diff_angle: %d\n", diff_angle);
                            stepper_rotate_angle_with_accel(diff_angle - last_diff_angle, STEPPER_SPEED_ULTRA_FAST);
                            vTaskDelay(pdMS_TO_TICKS(100));
                            stepper_motor_power_off();
                            last_diff_angle = diff_angle;
                        } else {
                            ESP_LOGW(TAG, "Checksum verification failed");
                        }
                    } else if (cmd > 0x05 && data_len > 1) { 
                        uint8_t checksum = calc_checksum_f(data + offset + 4, data_len);
                        printf("CSI payload (%d bytes): ", data_len);
                        for (int i = 0; i < data_len; i++) {
                            printf("%02X ", data[offset + 4 + i]);
                        }
                        if (checksum == data[offset + data_len + 4]) {
                            app_csi_uart_cb(data + offset + 4, data_len);
                            printf("\n");
                        } else {
                            ESP_LOGW(TAG, "Checksum verification failed %02X != %02X", checksum, data[offset + data_len + 4]);
                        }
                    } else {
                        ESP_LOGW(TAG, "Invalid command or frame length");
                    }
                    
                    // 移动到下一帧
                    offset += frame_len;
                } else {
                    // 帧头不匹配，跳过一个字节继续查找
                    ESP_LOGW(TAG, "Invalid frame header at offset %d: 0x%02X 0x%02X", offset, data[offset], data[offset + 1]);
                    offset++;
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
esp_err_t control_serial_send_data(uint8_t *data, uint16_t data_len)
{
    uint8_t tx_buffer[data_len+5];
    
    // 帧头
    tx_buffer[0] = UART_FRAME_HEAD_H;  // 0xAA
    tx_buffer[1] = UART_FRAME_HEAD_L;  // 0x55
    
    // 数据长度 (命令码1字节 + 数据2字节 = 3字节)
    tx_buffer[2] = data_len>>8;
    tx_buffer[3] = data_len & 0xFF;
    // ESP_LOGI(TAG, "data_len: %d, %d, %d", data_len>>8,data_len && 0xFF,data_len);
    memcpy(tx_buffer + 4, data, data_len);
    uint8_t checksum = calc_checksum_f(data, data_len);
    tx_buffer[data_len+4] = checksum;
    
    // 通过UART发送
    // 打印发送的数据内容
    ESP_LOGI(TAG, "TX UART DATA (%d bytes): ", data_len + 5);
    for (int i = 0; i < data_len + 5; i++) {
        printf("%02X ", tx_buffer[i]);
    }
    printf("\n");
    int sent = uart_write_bytes(ECHO_UART_PORT_NUM, tx_buffer, sizeof(tx_buffer));
    
    if (sent == sizeof(tx_buffer)) {
        ESP_LOGV(TAG, "Data sent: data_len=%d", data_len);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send data: sent=%d", sent);
        return ESP_FAIL;
    }
}
