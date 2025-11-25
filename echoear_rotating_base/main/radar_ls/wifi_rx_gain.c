
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include "esp_timer.h"

// #include "esp_radar.h"
 
// #if WIFI_CSI_PHY_GAIN_ENABLE
#if 1
#define FIX_GAIN_BUFF_SIZE          50
#define FIX_GAIN_OUTLIER_THRESHOLD  8

typedef struct
{
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
    unsigned : 32; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 ||CONFIG_IDF_TARGET_ESP32C6|| CONFIG_IDF_TARGET_ESP32C61
    unsigned : 16; /**< reserved */
    unsigned fft_gain : 8;
    unsigned agc_gain : 8;
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
      signed : 8;  /**< reserved */
    unsigned : 24; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5|| CONFIG_IDF_TARGET_ESP32C61
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
} wifi_pkt_rx_ctrl_phy_t;


typedef struct {
    uint32_t timestamp;
    int8_t rssi;
    bool force_en;

    uint32_t count;
    uint32_t baseline_count;

    uint8_t agc_gain_buff[FIX_GAIN_BUFF_SIZE];
    int8_t fft_gain_buff[FIX_GAIN_BUFF_SIZE];
} rx_gain_record_t;

extern void phy_fft_scale_force(bool force_en, uint8_t force_value);
extern void phy_force_rx_gain(bool force_en, int force_value);

static const char *TAG = "wifi_rx_gain";

static rx_gain_record_t *g_rx_gain_record = NULL;


bool esp_radar_auto_rx_gain_status()
{
    if (!g_rx_gain_record) {
        g_rx_gain_record = calloc(1, sizeof(rx_gain_record_t));
    }

    return g_rx_gain_record->force_en;
}

// esp_err_t esp_radar_get_rx_gain(uint8_t* agc_gain, int8_t *fft_gain)
// {
//     *agc_gain = g_rx_gain_record->agc_gain_buff[g_rx_gain_record->count % FIX_GAIN_BUFF_SIZE];
//     *fft_gain = g_rx_gain_record->fft_gain_buff[g_rx_gain_record->count % FIX_GAIN_BUFF_SIZE];

//     return ESP_OK;
// }

static int cmp_int8(const void *a, const void *b)
{
    return *(int8_t *)a > *(int8_t *)b ? 1 : -1;
}

static int8_t get_median(int8_t *data, uint32_t size)
{
    int8_t *tmp = malloc(sizeof(int8_t) * size);
    memcpy(tmp, data, sizeof(int8_t) * size);

    qsort(tmp, size, sizeof(int8_t), cmp_int8);

    return tmp[size / 2];
}

esp_err_t esp_radar_get_rx_gain_baseline(uint8_t* agc_gain, int8_t *fft_gain)
{
    if (!g_rx_gain_record) {
        g_rx_gain_record = calloc(1, sizeof(rx_gain_record_t));
    }

    *agc_gain = get_median((int8_t *)g_rx_gain_record->agc_gain_buff, FIX_GAIN_BUFF_SIZE);
    // *fft_gain = get_median(g_rx_gain_record->fft_gain_buff, FIX_GAIN_BUFF_SIZE);

    for (int i = 0; i < FIX_GAIN_BUFF_SIZE; i++) {
        if (*agc_gain == g_rx_gain_record->agc_gain_buff[i]) {
            *fft_gain = g_rx_gain_record->fft_gain_buff[i];
            break;
        }
    }

    // ESP_LOGE(TAG, "agc_gain: %d, fft_gain: %d", *agc_gain, *fft_gain);
    ESP_LOGW(TAG, "agc_gain: %d, fft_gain: %d", *agc_gain, *fft_gain);

    return ESP_FAIL;
}

esp_err_t esp_radar_record_rx_gain(uint8_t agc_gain, int8_t fft_gain)
{
    if (!g_rx_gain_record) {
        g_rx_gain_record = calloc(1, sizeof(rx_gain_record_t));
    }

    g_rx_gain_record->timestamp = esp_log_timestamp();
    g_rx_gain_record->count++;
    g_rx_gain_record->baseline_count++;

    uint32_t index = g_rx_gain_record->count % FIX_GAIN_BUFF_SIZE;
    g_rx_gain_record->agc_gain_buff[index] = agc_gain;
    g_rx_gain_record->fft_gain_buff[index] = fft_gain;

    // ESP_LOGD(TAG, "count: %d, agc_gain: %d, fft_gain: %d", g_rx_gain_record->count, rx_ctrl_gain->agc_gain, rx_ctrl_gain->fft_gain);

    return ESP_OK;
}

esp_err_t esp_radar_set_rx_force_gain(uint8_t agc_gain, int8_t fft_gain)
{
    if (!g_rx_gain_record) {
        g_rx_gain_record = calloc(1, sizeof(rx_gain_record_t));
    }

    if (agc_gain == 0 && fft_gain == 0) {
        phy_force_rx_gain(false, 0);
        phy_fft_scale_force(false, 0);

        g_rx_gain_record->force_en = false;
    } else {
        if (agc_gain <= 25) {
            ESP_LOGE(TAG, "Fixed rx gain failed, 'rx_gain <= 25' will prevent wifi packets from being sent out properly");
            return ESP_ERR_INVALID_STATE;
        }

        g_rx_gain_record->force_en = true;
        phy_force_rx_gain(true, agc_gain);
        phy_fft_scale_force(true, (uint8_t)fft_gain);
    }

    return ESP_OK;
}

void esp_radar_reset_rx_gain_baseline(void)
{
    if (!g_rx_gain_record) {
        return;
    }

    g_rx_gain_record->baseline_count = 0;
}

esp_err_t esp_radar_get_gain_compensation(float *compensate_gain, uint8_t agc_gain, int8_t fft_gain)
{
    static uint8_t s_agc_gain_baseline = 0;
    static int8_t s_fft_gain_baseline  = 0;

    if (!g_rx_gain_record) {
        return ESP_ERR_INVALID_STATE;
    }

    if (g_rx_gain_record->baseline_count < FIX_GAIN_BUFF_SIZE) {
        s_agc_gain_baseline = 0;
        s_fft_gain_baseline = 0;
        return ESP_ERR_INVALID_STATE;
    }

    if (s_agc_gain_baseline == 0 && s_fft_gain_baseline == 0) {
        if (esp_radar_get_rx_gain_baseline(&s_agc_gain_baseline, &s_fft_gain_baseline) != ESP_OK) {
            return ESP_ERR_INVALID_STATE;
        }
    }

    float compensate_factor = powf(10.0, ((agc_gain - s_agc_gain_baseline) + (fft_gain - s_fft_gain_baseline) / 4.0) / -20.0);
    *compensate_gain = compensate_factor;

    return ESP_OK;
}

esp_err_t esp_radar_compensate_rx_gain(int8_t *data, uint16_t size, float *compensate_gain, uint8_t agc_gain, int8_t fft_gain)
{
    float compensate_factor =0;
    esp_err_t err = esp_radar_get_gain_compensation(&compensate_factor, agc_gain, fft_gain);
    if (err!=ESP_OK) return err;
    
    *compensate_gain = compensate_factor;
    for (int i = 0; i < size; i++) {
        data[i] = (int8_t)(data[i] * compensate_factor);
    }
    return ESP_OK;
}

esp_err_t esp_radar_get_rx_gain(wifi_csi_info_t *info, uint8_t* agc_gain, int8_t* fft_gain)
{
    wifi_pkt_rx_ctrl_phy_t *phy_info = (wifi_pkt_rx_ctrl_phy_t *)info;
    *agc_gain = phy_info->agc_gain;
    *fft_gain = (int8_t)phy_info->fft_gain;
    return ESP_OK;
}

#endif