#pragma once

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

// 通用任务配置
#define MAGNETIC_SLIDE_SWITCH_TASK_STACK_SIZE    (1024 * 3)

// 根据传感器类型选择性编译宏定义
#ifdef CONFIG_SENSOR_LINEAR_HALL
// 线性霍尔传感器配置
#define HALL_SENSOR_ADC_CHANNEL             (ADC_CHANNEL_3)
#define HALL_SENSOR_ADC_ATTEN               ADC_ATTEN_DB_12
#define HALL_SENSOR_VOLTAGE_THRESHOLD       (25)   // mV - 电压变化阈值
#define HALL_SENSOR_INITIAL_STATE_THRESHOLD (1400) // mV - 初始状态判定阈值
#define HALL_SENSOR_SAMPLE_PERIOD_MS        (50)    // 采样周期

#elif defined(CONFIG_SENSOR_MAGNETOMETER)
// 地磁传感器I2C配置
#define I2C_MASTER_SCL_IO   (GPIO_NUM_3)            /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   (GPIO_NUM_2)            /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM      I2C_NUM_0               /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ  400000                  /*!< I2C master clock frequency */

#ifdef CONFIG_SENSOR_BMM150
    // BMM150磁力计特定配置
    #define MAGNETOMETER_I2C_ADDR           (0x10)      // BMM150 I2C地址
    #define MAGNETOMETER_CHIP_ID            (0x32)      // BMM150芯片ID

    // 滑动窗口配置
    #define MAG_WINDOW_SIZE                     (5)        // 滑动窗口大小
    #define MAG_SAMPLE_PERIOD_MS                (5)         // 采样周期(ms) - 越小响应越快
    #define MAG_STABLE_THRESHOLD                (5)         // 连续 N 次相同状态才认为稳定
    
    // 单击检测配置
    #define MAG_CLICK_TRANSITION_MAX_COUNT      (30)        // 单击下降最大持续次数（20次*5ms=100ms）
    #define MAG_CLICK_DROP_THRESHOLD            (60)       // 单击时的下降阈值（磁场值变化）

    // 状态阈值配置：中心值 ± 偏移量
    #define MAG_STATE_REMOVED_CENTER        (1230)      // 拿掉状态中心值
    #define MAG_STATE_REMOVED_OFFSET        (100)        // 拿掉状态偏移量
    #define MAG_STATE_REMOVED_MIN           (MAG_STATE_REMOVED_CENTER - MAG_STATE_REMOVED_OFFSET)
    #define MAG_STATE_REMOVED_MAX           (MAG_STATE_REMOVED_CENTER + MAG_STATE_REMOVED_OFFSET)
    
    #define MAG_STATE_UP_CENTER             (1619)      // 上面状态中心值
    #define MAG_STATE_UP_OFFSET             (100)        // 上面状态偏移量
    #define MAG_STATE_UP_MIN                (MAG_STATE_UP_CENTER - MAG_STATE_UP_OFFSET)
    #define MAG_STATE_UP_MAX                (MAG_STATE_UP_CENTER + MAG_STATE_UP_OFFSET)
    
    #define MAG_STATE_DOWN_CENTER           (1894)      // 下面状态中心值
    #define MAG_STATE_DOWN_OFFSET           (100)        // 下面状态偏移量
    #define MAG_STATE_DOWN_MIN              (MAG_STATE_DOWN_CENTER - MAG_STATE_DOWN_OFFSET)
    #define MAG_STATE_DOWN_MAX              (MAG_STATE_DOWN_CENTER + MAG_STATE_DOWN_OFFSET)
    
#elif defined(CONFIG_SENSOR_QMC6309)
    // QMC6309磁力计特定配置  
    #define MAGNETOMETER_I2C_ADDR           (0x7C)      // QMC6309 I2C地址
    #define MAGNETOMETER_CHIP_ID            (0x90)      // QMC6309芯片ID

    // 滑动窗口配置
    #define MAG_WINDOW_SIZE                     (5)        // 滑动窗口大小
    #define MAG_SAMPLE_PERIOD_MS                (2)         // 采样周期(ms) - 越小响应越快
    #define MAG_STABLE_THRESHOLD                (10)         // 连续 N 次相同状态才认为稳定
    
    // 单击检测配置
    #define MAG_CLICK_TRANSITION_MAX_COUNT      (20)        // 单击下降最大持续次数（20次*5ms=100ms）
    #define MAG_CLICK_DROP_THRESHOLD            (60)       // 单击时的下降阈值（磁场值变化）

    // 状态阈值配置：中心值 ± 偏移量
    #define MAG_STATE_REMOVED_CENTER        (1096)      // 拿掉状态中心值
    #define MAG_STATE_REMOVED_OFFSET        (100)        // 拿掉状态偏移量
    #define MAG_STATE_REMOVED_MIN           (MAG_STATE_REMOVED_CENTER - MAG_STATE_REMOVED_OFFSET)
    #define MAG_STATE_REMOVED_MAX           (MAG_STATE_REMOVED_CENTER + MAG_STATE_REMOVED_OFFSET)
    
    #define MAG_STATE_UP_CENTER             (1422)      // 上面状态中心值
    #define MAG_STATE_UP_OFFSET             (100)        // 上面状态偏移量
    #define MAG_STATE_UP_MIN                (MAG_STATE_UP_CENTER - MAG_STATE_UP_OFFSET)
    #define MAG_STATE_UP_MAX                (MAG_STATE_UP_CENTER + MAG_STATE_UP_OFFSET)
    
    #define MAG_STATE_DOWN_CENTER           (1813)      // 下面状态中心值
    #define MAG_STATE_DOWN_OFFSET           (100)        // 下面状态偏移量
    #define MAG_STATE_DOWN_MIN              (MAG_STATE_DOWN_CENTER - MAG_STATE_DOWN_OFFSET)
    #define MAG_STATE_DOWN_MAX              (MAG_STATE_DOWN_CENTER + MAG_STATE_DOWN_OFFSET)

#endif

#endif

typedef enum {
    MAGNETIC_SLIDE_SWITCH_EVENT_INIT = 0,               // 上电初始状态，未判断任何动作
    MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN = 1,         // 滑块从上往下拨
    MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_UP = 2,           // 滑块从下往上拨
    MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_UP = 3,     // 滑块从上面拿下
    MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_DOWN = 4,   // 滑块从下面拿下
    MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_UP = 5,      // 滑块从上面放上去
    MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_DOWN = 6,    // 滑块从下面放上去
    MAGNETIC_SLIDE_SWITCH_EVENT_SINGLE_CLICK = 7,       // 单击事件
} magnetic_slide_switch_event_t;

void magnetic_slide_switch_start(void);
magnetic_slide_switch_event_t magnetic_slide_switch_get_event(void);

#ifdef __cplusplus
}
#endif
