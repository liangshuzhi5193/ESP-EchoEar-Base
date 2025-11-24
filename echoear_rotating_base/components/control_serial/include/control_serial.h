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

// 命令码定义
#define CMD_BASE_ANGLE_CONTROL              (0x01)  // 底座角度控制
#define CMD_BASE_ACTION_CONTROL             (0x02)  // 底座动作控制
#define CMD_MAGNETIC_SWITCH_EVENT           (0x03)  // 磁吸滑动开关事件
#define CMD_ACTION_COMPLETE                 (0x02)  // 特定动作执行完成

// 磁吸开关特殊命令码
#define MAG_SWITCH_CMD_RECALIBRATE          (0x0010)  // 重新校准磁吸开关
#define MAG_SWITCH_CALIB_FIRST_DONE         (0x0011)  // 第二个位置校准完成，请进行第三个动作
#define MAG_SWITCH_CALIB_COMPLETE           (0x0012)  // 校准完成

// 动作完成状态码
#define ACTION_COMPLETE_CODE                (0x0010)  // 动作完成代码 (16)

esp_err_t control_serial_init(void);
esp_err_t control_serial_send_magnetic_switch_event(uint16_t event);
esp_err_t control_serial_send_action_complete(void);
esp_err_t control_serial_send_magnetic_switch_calibration_step(uint16_t step_code);

#ifdef __cplusplus
}
#endif

