#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define UART_ECHO_TXD (GPIO_NUM_29)
#define UART_ECHO_RXD (GPIO_NUM_8)
#define UART_ECHO_RTS (-1)
#define UART_ECHO_CTS (-1)

#define ECHO_UART_PORT_NUM      (1)
#define ECHO_UART_BAUD_RATE     (115200)
#define UART_CMD_TASK_STACK_SIZE    (1024 * 3)
#define BUF_SIZE (1024)

#define UART_FRAME_HEAD_H       (0xAA)
#define UART_FRAME_HEAD_L       (0x55)
#define UART_FRAME_HEADER_LEN   (2)
#define UART_FRAME_MIN_LEN      (5) // Minimum: header + length + command + checksum

esp_err_t control_serial_init(void);

#ifdef __cplusplus
}
#endif

