#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "bmm150.h"
#include "bmm150_defs.h"
#include "i2c_bus.h"
#include "esp_rom_sys.h"

#include "magnetic_slide_switch.h"
#include "control_serial.h"

static const char *TAG = "Magnetic Slide Switch";

// NVS 配置
#define NVS_NAMESPACE       "mag_calib"
#define NVS_KEY_REMOVED     "removed"
#define NVS_KEY_UP          "up"
#define NVS_KEY_DOWN        "down"
#define NVS_KEY_CALIBRATED  "calibrated"

static magnetic_slide_switch_event_t s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_INIT;

#if CONFIG_SENSOR_LINEAR_HALL
    static int adc_raw[2][10];
    static int voltage[2][10];
    static int s_last_hall_voltage = 0;
    static bool s_first_hall_reading = true;
#endif

#if CONFIG_SENSOR_BMM150
static struct bmm150_dev s_bmm150 = { 0 };
static uint8_t s_bmm150_addr = BMM150_DEFAULT_I2C_ADDRESS; /* BMM150默认I2C地址 */
#endif

#ifdef CONFIG_SENSOR_MAGNETOMETER
/*! I2C bus/device handles */
static i2c_bus_handle_t s_i2c_bus = NULL;
static i2c_bus_device_handle_t s_i2c_dev = NULL;
static uint8_t dev_addr;
#endif

#if CONFIG_SENSOR_LINEAR_HALL
/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void hall_sensor_read_task(void *arg)
{
    ESP_LOGI(TAG, "Hall sensor read task started");
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = HALL_SENSOR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, HALL_SENSOR_ADC_CHANNEL, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, HALL_SENSOR_ADC_CHANNEL, HALL_SENSOR_ADC_ATTEN, &adc1_cali_chan0_handle);

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, HALL_SENSOR_ADC_CHANNEL, &adc_raw[0][0]));
        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
            
            // 检查电压变化来判断磁吸开关事件
            if (!s_first_hall_reading) {  // 不是第一次读取
                int voltage_change = voltage[0][0] - s_last_hall_voltage;
                
                // 电压变小且变化量大于阈值，判定为滑块向下
                if (voltage_change < -HALL_SENSOR_VOLTAGE_THRESHOLD && s_slider_state != MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN) {
                    s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN;
                    ESP_LOGI(TAG, "Hall switch: SLIDE_DOWN - Voltage dropped by %d mV (from %d to %d)", 
                             -voltage_change, s_last_hall_voltage, voltage[0][0]);
                    control_serial_send_magnetic_switch_event(s_slider_state);
                }
                // 电压变大且变化量大于阈值，判定为滑块向上
                else if (voltage_change > HALL_SENSOR_VOLTAGE_THRESHOLD && s_slider_state == MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN) {
                    s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_UP;
                    ESP_LOGI(TAG, "Hall switch: SLIDE_UP - Voltage increased by %d mV (from %d to %d)", 
                             voltage_change, s_last_hall_voltage, voltage[0][0]);
                    control_serial_send_magnetic_switch_event(s_slider_state);
                }
            } else {
                s_first_hall_reading = false;
                
                // 根据第一次读取的电压值判断初始状态（初始状态不发送事件）
                if (voltage[0][0] < HALL_SENSOR_INITIAL_STATE_THRESHOLD) {
                    s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN;
                    ESP_LOGI(TAG, "First reading: Slider initially at DOWN position (voltage=%d < %d mV)", 
                             voltage[0][0], HALL_SENSOR_INITIAL_STATE_THRESHOLD);
                } else {
                    s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_UP;
                    ESP_LOGI(TAG, "First reading: Slider initially at UP position (voltage=%d >= %d mV)", 
                             voltage[0][0], HALL_SENSOR_INITIAL_STATE_THRESHOLD);
                }
            }
            
            // 更新上一次的电压值
            s_last_hall_voltage = voltage[0][0];
            
            // ESP_LOGI(TAG, "Hall sensor voltage: %d mV", voltage[0][0]);
        }
        vTaskDelay(pdMS_TO_TICKS(HALL_SENSOR_SAMPLE_PERIOD_MS));
    }
}
#endif

#ifdef CONFIG_SENSOR_BMM150
/*!
 * I2C read function for BMM150
 */
static BMM150_INTF_RET_TYPE bmm150_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    if (s_i2c_dev == NULL || device_addr != dev_addr) {
        if (s_i2c_dev) {
            i2c_bus_device_delete(&s_i2c_dev);
            s_i2c_dev = NULL;
        }
        s_i2c_dev = i2c_bus_device_create(s_i2c_bus, device_addr, 0);
        if (s_i2c_dev == NULL) {
            ESP_LOGE(TAG, "i2c_bus_device_create failed for 0x%02X", device_addr);
            return BMM150_E_COM_FAIL;
        }
        dev_addr = device_addr;
    }

    esp_err_t ret = i2c_bus_read_bytes(s_i2c_dev, reg_addr, (uint16_t)length, reg_data);
    return (ret == ESP_OK) ? BMM150_INTF_RET_SUCCESS : BMM150_E_COM_FAIL;
}

/*!
 * I2C write function for BMM150
 */
static BMM150_INTF_RET_TYPE bmm150_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    if (s_i2c_dev == NULL || device_addr != dev_addr) {
        if (s_i2c_dev) {
            i2c_bus_device_delete(&s_i2c_dev);
            s_i2c_dev = NULL;
        }
        s_i2c_dev = i2c_bus_device_create(s_i2c_bus, device_addr, 0);
        if (s_i2c_dev == NULL) {
            ESP_LOGE(TAG, "i2c_bus_device_create failed for 0x%02X", device_addr);
            return BMM150_E_COM_FAIL;
        }
        dev_addr = device_addr;
    }

    esp_err_t ret = i2c_bus_write_bytes(s_i2c_dev, reg_addr, (uint16_t)length, (uint8_t*)reg_data);
    return (ret == ESP_OK) ? BMM150_INTF_RET_SUCCESS : BMM150_E_COM_FAIL;
}

/*!
 * Delay function for BMM150
 */
static void bmm150_delay(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}

/*!
 * Interface initialization for BMM150
 */
int8_t bmm150_interface_init(struct bmm150_dev *dev)
{
    int8_t rslt = BMM150_OK;
    
    if (dev != NULL) {
        /* I2C setup */
        const i2c_config_t i2c_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ
        };
        
        s_i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &i2c_conf);
        if (s_i2c_bus == NULL) {
            ESP_LOGE(TAG, "I2C bus create failed");
            rslt = BMM150_E_COM_FAIL;
        } else {
            ESP_LOGI(TAG, "I2C interface initialized");
        }
    } else {
        rslt = BMM150_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * Deinitialize the interface
 */
void bmm150_coines_deinit(void)
{
    if (s_i2c_dev) {
        i2c_bus_device_delete(&s_i2c_dev);
        s_i2c_dev = NULL;
    }
    if (s_i2c_bus) {
        i2c_bus_delete(&s_i2c_bus);
        s_i2c_bus = NULL;
    }
    ESP_LOGI(TAG, "Interface deinitialized");
}


static int8_t bmm150_init_standalone(void)
{
    /* Try BMM150 default address first, then try other possible addresses */
    const uint8_t addr_candidates[] = { MAGNETOMETER_I2C_ADDR};
    int8_t rslt = BMM150_E_DEV_NOT_FOUND;

    for (size_t i = 0; i < sizeof(addr_candidates); ++i) {
        uint8_t addr = addr_candidates[i];
        /* 配置BMM150设备结构 */
        s_bmm150_addr = addr;
        s_bmm150.intf = BMM150_I2C_INTF;
        s_bmm150.read = bmm150_i2c_read;
        s_bmm150.write = bmm150_i2c_write;
        s_bmm150.delay_us = bmm150_delay;
        s_bmm150.intf_ptr = (void *)&s_bmm150_addr;

        /* Initialize BMM150 (reads CHIP_ID internally) */
        rslt = bmm150_init(&s_bmm150);
        ESP_LOGI(TAG, "Init at 0x%02X -> rslt=%d, chip_id=0x%02X (expect 0x32)", addr, rslt, s_bmm150.chip_id);

        if (s_bmm150.chip_id == MAGNETOMETER_CHIP_ID) {
            if (rslt != BMM150_OK) {
                /* 进行软复位 */
                (void)bmm150_soft_reset(&s_bmm150);
                s_bmm150.delay_us(BMM150_START_UP_TIME * 1000, s_bmm150.intf_ptr); /* 等待启动时间 */
            }

            /* 配置传感器设置：设置预设模式为常规模式 */
            struct bmm150_settings settings = { 0 };
            settings.pwr_mode = BMM150_POWERMODE_NORMAL;
            settings.preset_mode = BMM150_PRESETMODE_REGULAR;
            
            /* 设置预设模式 */
            rslt = bmm150_set_presetmode(&settings, &s_bmm150);
            if (rslt != BMM150_OK) {
                ESP_LOGE(TAG, "Set preset mode failed: %d", rslt);
                continue;
            }
            
            /* 设置功率模式 */
            rslt = bmm150_set_op_mode(&settings, &s_bmm150);
            if (rslt != BMM150_OK) {
                ESP_LOGE(TAG, "Set power mode failed: %d", rslt);
                continue;
            }
            
            ESP_LOGI(TAG, "BMM150 initialized successfully at address 0x%02X", addr);
            return BMM150_OK;
        }
    }

    return rslt;
}
#elif defined(CONFIG_SENSOR_QMC6309)

static esp_err_t qmc6309_interface_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    s_i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &i2c_conf);

    if (s_i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus create failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "I2C interface initialized");
    return ESP_OK;
}

/* -------------- 底层封装 -------------- */
static esp_err_t qmc_write_reg(uint8_t reg, uint8_t val)
{
    return i2c_bus_write_bytes(s_i2c_dev, reg, 1, &val);
}
static esp_err_t qmc_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_bus_read_bytes(s_i2c_dev, reg, len, data);
}

static esp_err_t qmc6309_init(void)
{
    // 创建I2C设备句柄
    s_i2c_dev = i2c_bus_device_create(s_i2c_bus, MAGNETOMETER_I2C_ADDR, 0);
    if (s_i2c_dev == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C device for QMC6309");
        return ESP_FAIL;
    }

    /* 软复位 */
    ESP_ERROR_CHECK(qmc_write_reg(0x0B, 0x80));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(qmc_write_reg(0x0B, 0x00));
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Continuous Mode, ±32 G, ODR=200 Hz, OSR=8/8 */
    ESP_ERROR_CHECK(qmc_write_reg(0x0B, 0x00));  // RNG=00
    ESP_ERROR_CHECK(qmc_write_reg(0x0A, 0x63));  // MODE=11, OSR=8
    
    ESP_LOGI(TAG, "QMC6309 initialized successfully");
    return ESP_OK;
}

/* -------------- 读取三轴 -------------- */
static esp_err_t qmc6309_read_raw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t status = 0;
    /* 等待 DRDY */
    do {
        ESP_ERROR_CHECK(qmc_read_reg(0x09, &status, 1));
    } while ((status & 0x01) == 0);
    uint8_t data[6];
    ESP_ERROR_CHECK(qmc_read_reg(0x01, data, 6));
    *x = (int16_t)(data[0] | (data[1] << 8));
    *y = (int16_t)(data[2] | (data[3] << 8));
    *z = (int16_t)(data[4] | (data[5] << 8));
    return ESP_OK;
}

static esp_err_t qmc6309_interface_deinit(void)
{
    if (s_i2c_dev) {
        i2c_bus_device_delete(&s_i2c_dev);
        s_i2c_dev = NULL;
    }
    if (s_i2c_bus) {
        i2c_bus_delete(&s_i2c_bus);
        s_i2c_bus = NULL;
    }
    ESP_LOGI(TAG, "QMC6309 interface deinitialized");
    return ESP_OK;
}
#endif

#if CONFIG_SENSOR_MAGNETOMETER

// 滑动窗口状态定义
typedef enum {
    MAG_POSITION_UNKNOWN = 0,
    MAG_POSITION_REMOVED,  // 拿掉
    MAG_POSITION_UP,       // 在上面
    MAG_POSITION_DOWN      // 在下面
} mag_position_t;

// 校准状态定义
typedef enum {
    CALIBRATION_IDLE = 0,           // 空闲状态
    CALIBRATION_FIRST_POSITION,     // 校准第一个位置（等待稳定）
    CALIBRATION_WAIT_SECOND,        // 等待移动到第二个位置
    CALIBRATION_SECOND_POSITION,    // 校准第二个位置（等待稳定）
    CALIBRATION_WAIT_THIRD,         // 等待移动到第三个位置
    CALIBRATION_THIRD_POSITION,     // 校准第三个位置（等待稳定）
    CALIBRATION_COMPLETED           // 校准完成
} calibration_state_t;

// 校准后的中心值（全局变量）
static int16_t s_calibrated_removed_center = MAG_STATE_REMOVED_CENTER;
static int16_t s_calibrated_up_center = MAG_STATE_UP_CENTER;
static int16_t s_calibrated_down_center = MAG_STATE_DOWN_CENTER;

// 重新校准标志（全局变量）
static volatile bool s_request_recalibration = false;

/**
 * @brief 保存校准数据到 NVS
 * 
 * @param removed_center REMOVED 状态中心值
 * @param up_center UP 状态中心值
 * @param down_center DOWN 状态中心值
 * @return esp_err_t 
 */
static esp_err_t save_calibration_to_nvs(int16_t removed_center, int16_t up_center, int16_t down_center)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    // 打开 NVS
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }
    
    // 保存三个中心值
    err = nvs_set_i16(nvs_handle, NVS_KEY_REMOVED, removed_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving removed_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_set_i16(nvs_handle, NVS_KEY_UP, up_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving up_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_set_i16(nvs_handle, NVS_KEY_DOWN, down_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving down_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // 设置校准标志
    err = nvs_set_u8(nvs_handle, NVS_KEY_CALIBRATED, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving calibrated flag: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // 提交更改
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Calibration data saved to NVS successfully");
    }
    
    // 关闭 NVS
    nvs_close(nvs_handle);
    
    return err;
}

/**
 * @brief 从 NVS 读取校准数据
 * 
 * @param removed_center 输出参数：REMOVED 状态中心值
 * @param up_center 输出参数：UP 状态中心值
 * @param down_center 输出参数：DOWN 状态中心值
 * @return esp_err_t ESP_OK 表示读取成功，其他表示失败或未校准
 */
static esp_err_t load_calibration_from_nvs(int16_t *removed_center, int16_t *up_center, int16_t *down_center)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    uint8_t calibrated = 0;
    
    // 打开 NVS
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No calibration data found, need to calibrate");
        } else {
            ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        }
        return err;
    }
    
    // 检查是否已校准
    err = nvs_get_u8(nvs_handle, NVS_KEY_CALIBRATED, &calibrated);
    if (err != ESP_OK || calibrated != 1) {
        ESP_LOGI(TAG, "Device not calibrated yet");
        nvs_close(nvs_handle);
        return ESP_ERR_NVS_NOT_FOUND;
    }
    
    // 读取三个中心值
    err = nvs_get_i16(nvs_handle, NVS_KEY_REMOVED, removed_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading removed_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_get_i16(nvs_handle, NVS_KEY_UP, up_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading up_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_get_i16(nvs_handle, NVS_KEY_DOWN, down_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading down_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // 关闭 NVS
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Calibration data loaded from NVS:");
    ESP_LOGI(TAG, "  REMOVED center: %d", *removed_center);
    ESP_LOGI(TAG, "  UP center: %d", *up_center);
    ESP_LOGI(TAG, "  DOWN center: %d", *down_center);
    
    return ESP_OK;
}

static void magnetometer_read_task(void *arg)
{
    ESP_LOGI(TAG, "Magnetometer read task started");
    
    // 滑动窗口
    static int16_t s_window[MAG_WINDOW_SIZE] = {0};
    static uint8_t s_window_index = 0;
    static uint8_t s_window_filled = 0;  // 窗口填充计数
    
    // 状态跟踪
    static mag_position_t s_current_position = MAG_POSITION_UNKNOWN;       // 当前稳定位置
    static mag_position_t s_motion_start_position = MAG_POSITION_UNKNOWN;  // 运动起始位置
    static mag_position_t s_candidate_position = MAG_POSITION_UNKNOWN;     // 候选新位置
    static bool s_is_in_motion = false;  // 是否正在运动中
    static uint8_t s_stable_count = 0;   // 稳定计数器
    
    // 单击检测 - 基于峰值检测
    static int16_t s_down_baseline = 0;        // DOWN状态的基准值
    static bool s_click_drop_detected = false;  // 检测到下降
    static int16_t s_click_max_drop = 0;       // 记录最大下降量
    static uint8_t s_click_duration = 0;       // 下降持续时间
    static uint8_t s_recover_count = 0;        // 恢复计数
    
    // 校准相关变量
    static calibration_state_t s_calibration_state = CALIBRATION_IDLE;
    static uint8_t s_calibration_stable_count = 0;
    static int16_t s_calibration_value = 0;
    static int16_t s_calibration_last_average = 0;
    static bool s_calibration_positions[3] = {false, false, false};  // 记录已校准的位置索引（0=REMOVED, 1=UP, 2=DOWN）
    static int16_t s_calibration_temp_values[3] = {0};  // 临时存储校准值
    static uint8_t s_calibration_settle_count = 0;  // 移动后沉淀计数器，确保数据真正稳定

#ifdef CONFIG_SENSOR_BMM150
    if (bmm150_interface_init(&s_bmm150) != BMM150_OK) {
        ESP_LOGE(TAG, "interface init failed");
        return;
    }
    
    int8_t rslt = bmm150_init_standalone();
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 init failed: %d", rslt);
        goto cleanup;
    }
#elif defined(CONFIG_SENSOR_QMC6309)
    if (qmc6309_interface_init() != ESP_OK) {
        ESP_LOGE(TAG, "QMC6309 interface init failed");
        return;
    }

    if (qmc6309_init() != ESP_OK) {
        ESP_LOGE(TAG, "QMC6309 init failed");
        goto cleanup;
    }
#endif
    
    // 尝试从 NVS 加载校准数据
    esp_err_t load_err = load_calibration_from_nvs(
        &s_calibrated_removed_center,
        &s_calibrated_up_center,
        &s_calibrated_down_center
    );
    
    if (load_err == ESP_OK) {
        // 校准数据加载成功，跳过校准流程
        ESP_LOGI(TAG, "========== Using Saved Calibration Data ==========");
        s_calibration_state = CALIBRATION_COMPLETED;
    } else {
        // 没有校准数据或加载失败，启动校准流程
        ESP_LOGI(TAG, "========== Magnetometer Calibration Started ==========");
        ESP_LOGI(TAG, "Please keep the slider in current position and wait for calibration...");
        s_calibration_state = CALIBRATION_FIRST_POSITION;
    }
    
    while (1) {
        // 检查是否需要重新校准
        if (s_request_recalibration && s_calibration_state == CALIBRATION_COMPLETED) {
            ESP_LOGI(TAG, "========== Re-calibration Requested ==========");
            ESP_LOGI(TAG, "Please keep the slider in current position and wait for calibration...");
            
            // 重置校准状态
            s_calibration_state = CALIBRATION_FIRST_POSITION;
            s_calibration_stable_count = 0;
            s_calibration_value = 0;
            s_calibration_last_average = 0;
            s_calibration_settle_count = 0;  // 重置沉淀计数器
            s_calibration_positions[0] = false;
            s_calibration_positions[1] = false;
            s_calibration_positions[2] = false;
            s_calibration_temp_values[0] = 0;
            s_calibration_temp_values[1] = 0;
            s_calibration_temp_values[2] = 0;
            
            // 重置窗口
            s_window_filled = 0;
            s_window_index = 0;
            
            // 清除请求标志
            s_request_recalibration = false;
        }
        
        int8_t ret = ESP_FAIL;
        int16_t s_mag_value = 0;
        
#ifdef CONFIG_SENSOR_BMM150
        struct bmm150_mag_data mag = { 0 };
        ret = bmm150_read_mag_data(&mag, &s_bmm150);

        // ESP_LOGI(TAG, "BMM150 mag value: %d, %d, %d", mag.x, mag.y, mag.z);
        // vTaskDelay(pdMS_TO_TICKS(200));

        s_mag_value = mag.z;
#elif defined(CONFIG_SENSOR_QMC6309)
        int16_t qmc_x, qmc_y, qmc_z;
        ret = qmc6309_read_raw(&qmc_x, &qmc_y, &qmc_z);
        s_mag_value = -qmc_z / 10;
#endif
        
#if 1
        if (ret == ESP_OK) {
            // 1. 更新滑动窗口
            s_window[s_window_index] = s_mag_value;
            s_window_index = (s_window_index + 1) % MAG_WINDOW_SIZE;
            if (s_window_filled < MAG_WINDOW_SIZE) {
                s_window_filled++;
            }
            
            // 2. 计算滑动窗口平均值（窗口填满后才开始判断）
            if (s_window_filled >= MAG_WINDOW_SIZE) {
                int32_t sum = 0;
                for (uint8_t i = 0; i < MAG_WINDOW_SIZE; i++) {
                    sum += s_window[i];
                }
                int16_t average = sum / MAG_WINDOW_SIZE;
                // printf("average: %d\n", average);
                
                // ========== 校准状态机 ==========
                if (s_calibration_state != CALIBRATION_COMPLETED) {
                    // 正在校准中，不执行正常的动作识别逻辑
                    
                    switch (s_calibration_state) {
                        case CALIBRATION_FIRST_POSITION:
                        {
                            // 检查数值稳定性（需要更严格）
                            if (s_calibration_last_average == 0) {
                                // 第一次读取，初始化
                                s_calibration_last_average = average;
                                s_calibration_stable_count = 0;
                            } else {
                                int16_t diff = abs(average - s_calibration_last_average);
                                
                                if (diff < 5) {  // 变化小于5，认为稳定（更严格）
                                    s_calibration_stable_count++;
                                    
                                    if (s_calibration_stable_count >= MAG_STABLE_THRESHOLD * 3) {  // 需要更长时间稳定
                                        // 稳定足够久，记录第一个位置
                                        s_calibration_value = average;
                                        
                                        // 判断最接近哪个预设值
                                        int16_t diff_removed = abs(average - MAG_STATE_REMOVED_CENTER);
                                        int16_t diff_up = abs(average - MAG_STATE_UP_CENTER);
                                        int16_t diff_down = abs(average - MAG_STATE_DOWN_CENTER);
                                        
                                        if (diff_removed < diff_up && diff_removed < diff_down) {
                                            s_calibration_temp_values[0] = average;
                                            s_calibration_positions[0] = true;
                                            ESP_LOGI(TAG, "First position calibrated as REMOVED: %d", average);
                                        } else if (diff_up < diff_down) {
                                            s_calibration_temp_values[1] = average;
                                            s_calibration_positions[1] = true;
                                            ESP_LOGI(TAG, "First position calibrated as UP: %d", average);
                                        } else {
                                            s_calibration_temp_values[2] = average;
                                            s_calibration_positions[2] = true;
                                            ESP_LOGI(TAG, "First position calibrated as DOWN: %d", average);
                                        }
                                        
                                        // 进入下一个状态
                                        s_calibration_state = CALIBRATION_WAIT_SECOND;
                                        s_calibration_stable_count = 0;
                                        s_calibration_settle_count = 0;  // 重置沉淀计数器
                                        ESP_LOGI(TAG, "Please move the slider to another position...");
                                    }
                                } else {
                                    // 数值还在变化，重置稳定计数
                                    s_calibration_stable_count = 0;
                                }
                                
                                s_calibration_last_average = average;
                            }
                            break;
                        }
                        
                        case CALIBRATION_WAIT_SECOND:
                        {
                            // 等待数据发生变化（说明用户移动了滑块）
                            int16_t change = abs(average - s_calibration_value);
                            
                            // 检测到移动后,先沉淀一段时间,确保数据稳定
                            if (s_calibration_settle_count > 0) {
                                // 已经检测到移动,正在沉淀期
                                int16_t settle_diff = abs(average - s_calibration_last_average);
                                
                                if (settle_diff < 10) {
                                    // 数据变化很小,认为正在稳定
                                    s_calibration_settle_count++;
                                    
                                    // 沉淀时间足够(约500ms),进入稳定性检查状态
                                    if (s_calibration_settle_count >= MAG_STABLE_THRESHOLD * 2) {
                                        ESP_LOGI(TAG, "Data settled at %d, starting position recording...", average);
                                        s_calibration_state = CALIBRATION_SECOND_POSITION;
                                        s_calibration_stable_count = 0;
                                        s_calibration_last_average = 0;  // 重置,强制第一次读取
                                        s_calibration_settle_count = 0;
                                    }
                                } else {
                                    // 数据还在大幅变化,重置沉淀计数(滑块还在移动中)
                                    s_calibration_settle_count = 1;
                                }
                                
                                s_calibration_last_average = average;
                            } else {
                                // 还未检测到移动,等待变化
                                if (change > 100) {
                                    ESP_LOGI(TAG, "Movement detected (change=%d), waiting for data to settle...", change);
                                    s_calibration_settle_count = 1;  // 开始沉淀期
                                    s_calibration_last_average = average;
                                }
                            }
                            break;
                        }
                        
                        case CALIBRATION_SECOND_POSITION:
                        {
                            // 检查数值稳定性（需要更严格）
                            if (s_calibration_last_average == 0) {
                                // 第一次读取，初始化
                                s_calibration_last_average = average;
                                s_calibration_stable_count = 0;
                            } else {
                                int16_t diff = abs(average - s_calibration_last_average);
                                
                                if (diff < 5) {  // 变化小于5，认为稳定（更严格）
                                    s_calibration_stable_count++;
                                    
                                    if (s_calibration_stable_count >= MAG_STABLE_THRESHOLD * 3) {  // 需要更长时间稳定
                                        // 稳定足够久，记录第二个位置
                                        s_calibration_value = average;
                                        
                                        // 判断最接近哪个未校准的预设值
                                        int16_t min_diff = 32767;
                                        int best_index = -1;
                                        
                                        if (!s_calibration_positions[0]) {
                                            int16_t diff = abs(average - MAG_STATE_REMOVED_CENTER);
                                            if (diff < min_diff) {
                                                min_diff = diff;
                                                best_index = 0;
                                            }
                                        }
                                        if (!s_calibration_positions[1]) {
                                            int16_t diff = abs(average - MAG_STATE_UP_CENTER);
                                            if (diff < min_diff) {
                                                min_diff = diff;
                                                best_index = 1;
                                            }
                                        }
                                        if (!s_calibration_positions[2]) {
                                            int16_t diff = abs(average - MAG_STATE_DOWN_CENTER);
                                            if (diff < min_diff) {
                                                min_diff = diff;
                                                best_index = 2;
                                            }
                                        }
                                        
                                    if (best_index >= 0) {
                                        s_calibration_temp_values[best_index] = average;
                                        s_calibration_positions[best_index] = true;
                                        const char *pos_names[] = {"REMOVED", "UP", "DOWN"};
                                        ESP_LOGI(TAG, "Second position calibrated as %s: %d", 
                                                 pos_names[best_index], average);
                                    }
                                    
                                    // 进入下一个状态
                                    s_calibration_state = CALIBRATION_WAIT_THIRD;
                                    s_calibration_stable_count = 0;
                                    s_calibration_settle_count = 0;  // 重置沉淀计数器
                                    ESP_LOGI(TAG, "Please move the slider to the last position...");
                                    
                                    // 发送 0x11 通知头部：第二个位置已完成，请提示用户进行第三个动作
                                    control_serial_send_magnetic_switch_calibration_step(MAG_SWITCH_CALIB_FIRST_DONE);
                                    }
                                } else {
                                    // 数值还在变化，重置稳定计数
                                    s_calibration_stable_count = 0;
                                }
                                
                                s_calibration_last_average = average;
                            }
                            break;
                        }
                        
                        case CALIBRATION_WAIT_THIRD:
                        {
                            // 等待数据发生变化
                            int16_t change = abs(average - s_calibration_value);
                            
                            // 检测到移动后,先沉淀一段时间,确保数据稳定
                            if (s_calibration_settle_count > 0) {
                                // 已经检测到移动,正在沉淀期
                                int16_t settle_diff = abs(average - s_calibration_last_average);
                                
                                if (settle_diff < 10) {
                                    // 数据变化很小,认为正在稳定
                                    s_calibration_settle_count++;
                                    
                                    // 沉淀时间足够(约500ms),进入稳定性检查状态
                                    if (s_calibration_settle_count >= MAG_STABLE_THRESHOLD * 2) {
                                        ESP_LOGI(TAG, "Data settled at %d, starting position recording...", average);
                                        s_calibration_state = CALIBRATION_THIRD_POSITION;
                                        s_calibration_stable_count = 0;
                                        s_calibration_last_average = 0;  // 重置,强制第一次读取
                                        s_calibration_settle_count = 0;
                                    }
                                } else {
                                    // 数据还在大幅变化,重置沉淀计数(滑块还在移动中)
                                    s_calibration_settle_count = 1;
                                }
                                
                                s_calibration_last_average = average;
                            } else {
                                // 还未检测到移动,等待变化
                                if (change > 100) {
                                    ESP_LOGI(TAG, "Movement detected (change=%d), waiting for data to settle...", change);
                                    s_calibration_settle_count = 1;  // 开始沉淀期
                                    s_calibration_last_average = average;
                                }
                            }
                            break;
                        }
                        
                        case CALIBRATION_THIRD_POSITION:
                        {
                            // 检查数值稳定性（需要更严格）
                            if (s_calibration_last_average == 0) {
                                // 第一次读取，初始化
                                s_calibration_last_average = average;
                                s_calibration_stable_count = 0;
                            } else {
                                int16_t diff = abs(average - s_calibration_last_average);
                                
                                if (diff < 5) {  // 变化小于5，认为稳定（更严格）
                                    s_calibration_stable_count++;
                                    
                                    if (s_calibration_stable_count >= MAG_STABLE_THRESHOLD * 3) {  // 需要更长时间稳定
                                        // 稳定足够久，记录第三个位置
                                        
                                    // 找到最后一个未校准的位置
                                    for (int i = 0; i < 3; i++) {
                                        if (!s_calibration_positions[i]) {
                                            s_calibration_temp_values[i] = average;
                                            s_calibration_positions[i] = true;
                                            const char *pos_names[] = {"REMOVED", "UP", "DOWN"};
                                            ESP_LOGI(TAG, "Third position calibrated as %s: %d", 
                                                     pos_names[i], average);
                                            break;
                                        }
                                    }
                                        
                                        // 校准完成，按照大小关系重新排序
                                        // REMOVED一定是最小值，UP是中间值，DOWN是最大值
                                        int16_t sorted_values[3];
                                        sorted_values[0] = s_calibration_temp_values[0];
                                        sorted_values[1] = s_calibration_temp_values[1];
                                        sorted_values[2] = s_calibration_temp_values[2];
                                        
                                        // 简单冒泡排序
                                        for (int i = 0; i < 2; i++) {
                                            for (int j = 0; j < 2 - i; j++) {
                                                if (sorted_values[j] > sorted_values[j + 1]) {
                                                    int16_t temp = sorted_values[j];
                                                    sorted_values[j] = sorted_values[j + 1];
                                                    sorted_values[j + 1] = temp;
                                                }
                                            }
                                        }
                                        
                                        // 按照大小关系赋值：最小=REMOVED，中间=UP，最大=DOWN
                                        s_calibrated_removed_center = sorted_values[0];
                                        s_calibrated_up_center = sorted_values[1];
                                        s_calibrated_down_center = sorted_values[2];
                                        
                                        s_calibration_state = CALIBRATION_COMPLETED;
                                        
                                        ESP_LOGI(TAG, "========== Calibration Completed ==========");
                                        ESP_LOGI(TAG, "Calibrated values (sorted by magnitude):");
                                        ESP_LOGI(TAG, "  REMOVED center: %d (default: %d)", 
                                                 s_calibrated_removed_center, MAG_STATE_REMOVED_CENTER);
                                        ESP_LOGI(TAG, "  UP center: %d (default: %d)", 
                                                 s_calibrated_up_center, MAG_STATE_UP_CENTER);
                                        ESP_LOGI(TAG, "  DOWN center: %d (default: %d)", 
                                                 s_calibrated_down_center, MAG_STATE_DOWN_CENTER);
                                        
                                        // 保存校准数据到 NVS
                                        esp_err_t save_err = save_calibration_to_nvs(
                                            s_calibrated_removed_center,
                                            s_calibrated_up_center,
                                            s_calibrated_down_center
                                        );
                                        
                                        if (save_err == ESP_OK) {
                                            ESP_LOGI(TAG, "Calibration data saved to Flash successfully");
                                        } else {
                                            ESP_LOGW(TAG, "Failed to save calibration data to Flash: %s", 
                                                     esp_err_to_name(save_err));
                                        }
                                        
                                        ESP_LOGI(TAG, "Normal operation started.");
                                        
                                        // 发送 0x12 通知头部：校准完成
                                        control_serial_send_magnetic_switch_calibration_step(MAG_SWITCH_CALIB_COMPLETE);
                                    }
                                } else {
                                    // 数值还在变化，重置稳定计数
                                    s_calibration_stable_count = 0;
                                }
                                
                                s_calibration_last_average = average;
                            }
                            break;
                        }
                        
                        default:
                            break;
                    }
                    
                    // 校准期间不执行后续的动作识别逻辑
                    vTaskDelay(pdMS_TO_TICKS(MAG_SAMPLE_PERIOD_MS));
                    continue;
                }
                
                // ========== 正常运行模式（校准完成后） ==========
                // printf("average: %d\n", average);
                
                // 3. 根据平均值判断当前位置状态（使用校准后的值）
                mag_position_t detected_position = MAG_POSITION_UNKNOWN;
                
                // 使用校准后的中心值计算状态范围
                int16_t calibrated_removed_min = s_calibrated_removed_center - MAG_STATE_REMOVED_OFFSET;
                int16_t calibrated_removed_max = s_calibrated_removed_center + MAG_STATE_REMOVED_OFFSET;
                int16_t calibrated_up_min = s_calibrated_up_center - MAG_STATE_UP_OFFSET;
                int16_t calibrated_up_max = s_calibrated_up_center + MAG_STATE_UP_OFFSET;
                int16_t calibrated_down_min = s_calibrated_down_center - MAG_STATE_DOWN_OFFSET;
                int16_t calibrated_down_max = s_calibrated_down_center + MAG_STATE_DOWN_OFFSET;
                
                if (average >= calibrated_removed_min && average <= calibrated_removed_max) {
                    detected_position = MAG_POSITION_REMOVED;  // 拿掉
                } else if (average >= calibrated_up_min && average <= calibrated_up_max) {
                    detected_position = MAG_POSITION_UP;       // 上面
                } else if (average >= calibrated_down_min && average <= calibrated_down_max) {
                    detected_position = MAG_POSITION_DOWN;     // 下面
                }
                
                // 4. 单击检测 - 基于峰值检测（不使用平均值）
                if (s_current_position == MAG_POSITION_DOWN) {
                    // 在 DOWN 状态下，监测磁场值的快速下降和恢复
                    
                    // 更新基准值（使用平滑更新，避免噪声影响）
                    if (s_down_baseline == 0) {
                        s_down_baseline = s_mag_value;  // 首次初始化
                    } else if (!s_click_drop_detected) {
                        // 未检测到下降时，缓慢跟踪基准值
                        s_down_baseline = (s_down_baseline * 9 + s_mag_value) / 10;
                    }
                    
                    // 检测下降（实时值相对基准值）
                    int16_t drop_value = s_down_baseline - s_mag_value;
                    
                    if (!s_click_drop_detected && drop_value >= MAG_CLICK_DROP_THRESHOLD) {
                        // 首次检测到下降，进入单击检测模式
                        s_click_drop_detected = true;
                        s_click_max_drop = drop_value;
                        s_click_duration = 1;
                        s_recover_count = 0;
                        // printf("Click started: baseline=%d, current=%d, drop=%d\n", 
                        //        s_down_baseline, s_mag_value, drop_value);
                    } else if (s_click_drop_detected) {
                        // 已进入单击检测模式
                        s_click_duration++;
                        
                        // 更新最大下降量
                        if (drop_value > s_click_max_drop) {
                            s_click_max_drop = drop_value;
                        }
                        
                        // 检测恢复：当前下降值小于最大下降的40%，认为开始恢复
                        if (drop_value < (s_click_max_drop * 4 / 10)) {
                            s_recover_count++;
                            
                            // 恢复稳定，触发单击事件
                            if (s_recover_count >= MAG_STABLE_THRESHOLD) {
                                s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SINGLE_CLICK;
                                ESP_LOGI(TAG, "SINGLE_CLICK detected (max_drop=%d, duration=%d)", 
                                         s_click_max_drop, s_click_duration);
                                
                                // 发送事件通知
                                control_serial_send_magnetic_switch_event(s_slider_state);
                                
                                // ESP_LOGI(TAG, "--------------------------------\n");
                                
                                // 重置单击检测
                                s_click_drop_detected = false;
                                s_click_max_drop = 0;
                                s_click_duration = 0;
                                s_recover_count = 0;
                                s_down_baseline = s_mag_value;
                            }
                        } else {
                            // 还未恢复，重置恢复计数
                            s_recover_count = 0;
                        }
                        
                        // 超时检测：持续时间过长（>200ms），重置
                        if (s_click_duration > MAG_CLICK_TRANSITION_MAX_COUNT * 2) {
                            // printf("Click timeout: duration=%d, max_drop=%d\n", 
                            //        s_click_duration, s_click_max_drop);
                            s_click_drop_detected = false;
                            s_click_max_drop = 0;
                            s_click_duration = 0;
                            s_recover_count = 0;
                            s_down_baseline = s_mag_value;
                        }
                    }
                } else {
                    // 不在 DOWN 状态，重置单击检测
                    s_click_drop_detected = false;
                    s_click_max_drop = 0;
                    s_click_duration = 0;
                    s_recover_count = 0;
                    s_down_baseline = 0;
                }
                
                // 5. 稳定性检测和滑动事件触发
                if (detected_position == MAG_POSITION_UNKNOWN) {
                    // 处于过渡区域（不在任何定义的状态范围内）
                    if (!s_is_in_motion && s_current_position != MAG_POSITION_UNKNOWN) {
                        // 刚离开稳定状态，记录运动起始位置
                        s_motion_start_position = s_current_position;
                        s_is_in_motion = true;
                    }
                    
                    // 重置候选状态和稳定计数
                    s_candidate_position = MAG_POSITION_UNKNOWN;
                    s_stable_count = 0;
                }
                else {
                    // 检测到有效状态（REMOVED/UP/DOWN）
                    if (detected_position == s_candidate_position) {
                        // 候选状态保持不变，累积稳定计数
                        s_stable_count++;
                        
                        // 检查是否达到稳定阈值
                        if (s_stable_count >= MAG_STABLE_THRESHOLD && 
                            detected_position != s_current_position) {
                            // 新状态已稳定，可以触发事件
                            
                            // 只在运动过程中触发事件（避免初始化时误触发）
                            if (s_is_in_motion && s_motion_start_position != MAG_POSITION_UNKNOWN) {
                                magnetic_slide_switch_event_t event = MAGNETIC_SLIDE_SWITCH_EVENT_INIT;
                                
                                // 根据起始位置和目标位置判断事件
                                if (s_motion_start_position == MAG_POSITION_UP && 
                                    detected_position == MAG_POSITION_DOWN) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN;
                                    ESP_LOGI(TAG, "SLIDE_DOWN");
                                }
                                else if (s_motion_start_position == MAG_POSITION_DOWN && 
                                         detected_position == MAG_POSITION_UP) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_UP;
                                    ESP_LOGI(TAG, "SLIDE_UP");
                                }
                                else if (s_motion_start_position == MAG_POSITION_UP && 
                                         detected_position == MAG_POSITION_REMOVED) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_UP;
                                    ESP_LOGI(TAG, "REMOVE_FROM_UP");
                                }
                                else if (s_motion_start_position == MAG_POSITION_DOWN && 
                                         detected_position == MAG_POSITION_REMOVED) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_DOWN;
                                    ESP_LOGI(TAG, "REMOVE_FROM_DOWN");
                                }
                                else if (s_motion_start_position == MAG_POSITION_REMOVED && 
                                         detected_position == MAG_POSITION_UP) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_UP;
                                    ESP_LOGI(TAG, "PLACE_FROM_UP");
                                }
                                else if (s_motion_start_position == MAG_POSITION_REMOVED && 
                                         detected_position == MAG_POSITION_DOWN) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_DOWN;
                                    ESP_LOGI(TAG, "PLACE_FROM_DOWN");
                                }
                                
                                // 更新状态并发送事件
                                if (event != MAGNETIC_SLIDE_SWITCH_EVENT_INIT) {
                                    s_slider_state = event;
                                    control_serial_send_magnetic_switch_event(event);
                                    
                                    // 只在触发事件后才更新运动起始位置
                                    // 这样可以正确处理连续的动作序列
                                    s_motion_start_position = detected_position;
                                }
                                
                                // ESP_LOGI(TAG, "--------------------------------\n");
                            } else {
                                // 初始化或没有触发事件时，也更新起始位置
                                // 这是第一次检测到位置，或者状态未变化
                                if (s_current_position == MAG_POSITION_UNKNOWN) {
                                    s_motion_start_position = detected_position;
                                }
                            }
                            
                            // 更新当前位置，清除运动标志
                            s_current_position = detected_position;
                            s_is_in_motion = false;
                        }
                    }
                    else {
                        // 检测到新的候选状态，重新开始稳定性验证
                        s_candidate_position = detected_position;
                        s_stable_count = 1;
                    }
                }
            }
        } else {
            ESP_LOGE(TAG, "read_mag_data failed: %d", ret);
        }
        
        vTaskDelay(pdMS_TO_TICKS(MAG_SAMPLE_PERIOD_MS));
#endif
    }

cleanup:
#ifdef CONFIG_SENSOR_BMM150
    bmm150_coines_deinit();
#elif defined(CONFIG_SENSOR_QMC6309)
    qmc6309_interface_deinit();
#endif
    ESP_LOGI(TAG, "Magnetometer read task finished.");
    vTaskDelete(NULL);
}
#endif

void magnetic_slide_switch_start(void)
{
#ifdef CONFIG_SENSOR_LINEAR_HALL
    xTaskCreate(hall_sensor_read_task, "hall_sensor_read_task", MAGNETIC_SLIDE_SWITCH_TASK_STACK_SIZE, NULL, 2, NULL);
#elif defined(CONFIG_SENSOR_MAGNETOMETER)
    xTaskCreate(magnetometer_read_task, "magnetometer_read_task", MAGNETIC_SLIDE_SWITCH_TASK_STACK_SIZE, NULL, 2, NULL);
#endif
}

magnetic_slide_switch_event_t magnetic_slide_switch_get_event(void)
{
    return s_slider_state;
}

/**
 * @brief 触发重新校准（无需重启，立即生效）
 * 
 * 此函数设置重新校准标志，magnetometer_read_task 将在下一次循环中
 * 自动重新进入校准模式，校准完成后会覆盖之前保存的校准数据。
 */
void magnetic_slide_switch_start_recalibration(void)
{
    ESP_LOGI(TAG, "Recalibration request received");
    s_request_recalibration = true;
}
