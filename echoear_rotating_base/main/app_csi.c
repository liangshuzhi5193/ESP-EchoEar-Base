/* Get Start Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_radar.h"
// #include "led_strip.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "driver/uart.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "ping/ping_sock.h"
#include "control_serial.h"
#include "protocol_examples_common.h"
#define CONFIG_LESS_INTERFERENCE_CHANNEL   11
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    #define CONFIG_WIFI_BAND_MODE   WIFI_BAND_MODE_2G_ONLY
    #define CONFIG_WIFI_2G_BANDWIDTHS           WIFI_BW_HT40
    #define CONFIG_WIFI_5G_BANDWIDTHS           WIFI_BW_HT40
    #define CONFIG_WIFI_2G_PROTOCOL             WIFI_PROTOCOL_11N
    #define CONFIG_WIFI_5G_PROTOCOL             WIFI_PROTOCOL_11N
    #define CONFIG_ESP_NOW_PHYMODE           WIFI_PHY_MODE_HT40
#else
    #define CONFIG_WIFI_BANDWIDTH           WIFI_BW_HT20
#endif
#define CONFIG_ESP_NOW_RATE             WIFI_PHY_RATE_MCS0_LGI
#define CONFIG_WS2812_LED_GPIO              27
#define CONFIG_WS2812_LED_NUMBERS           1
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    #define CSI_FORCE_LLTF                      0   
#endif
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA


static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t CONFIG_CSI_ECHOEAR_MAC[] = {0x90, 0xe5, 0xb1, 0xa8, 0xcb, 0x4c};
static const char *TAG = "csi_recv";


// 全局 Wi-Fi 配置（UART 指令中下发的 SSID / 密码）
char g_wifi_ssid[33] = {0};   // 最大 32 字节 + 结束符
char g_wifi_pwd[65]  = {0};   // 最大 64 字节 + 结束符

// 全局 CSI 模式 / 信道 / 灵敏度参数（通过串口指令配置）
uint8_t g_csi_mode             = 0;    // 0: 未配置，0x01: 自发自收，0x02: 路由器模式
uint8_t g_csi_channel          = CONFIG_LESS_INTERFERENCE_CHANNEL;
float   g_csi_move_sensitivity = 0.20f;

// 全局 CSI 实时/统计/事件配置（通过串口查询命令配置）
uint16_t g_realtime_time    = 0;    // 实时窗口时间（单位自行约定）
bool     g_realtime_enable  = false;
bool     g_csi_enevt_enable = false;
bool     g_csi_data_enable = false;

typedef struct  {
    bool threshold_calibrate;               /**< Self-calibration acquisition, the most suitable threshold, calibration is to ensure that no one is in the room */
    uint8_t threshold_calibrate_timeout;    /**< Calibration timeout, the unit is second */
    float someone_timeout;                  /**< The unit is second, how long no one moves in the room is marked as no one */
    float move_threshold;                   /**< Use to determine whether someone is moving */
    uint8_t filter_window;                  /**< window without filtering outliers */
    uint8_t filter_count;                   /**< Detect multiple times within the outlier window greater than move_threshold to mark as someone moving */
} radar_detect_config_t;

// 将检测配置改为可修改的全局变量
static radar_detect_config_t g_detect_config      = {
    .someone_timeout = 3 * 60,
    .move_threshold  = 0.002,
    .filter_window   = 5,
    .filter_count    = 2,
    .threshold_calibrate_timeout = 60,
};

// CSI数据打印开关，默认关闭
static bool g_print_csi_data = false;


#define RADAR_BUFF_MAX_LEN                  75
static struct console_input_config {
    float predict_move_sensitivity;
    float predict_touch_sensitivity;
    uint32_t predict_buff_size;
    uint32_t predict_outliers_number;
} g_console_input_config = {
    .predict_move_sensitivity  = 0.50,
    .predict_touch_sensitivity  = 0.20,
    .predict_buff_size         = 5,
    .predict_outliers_number   = 1,
};

typedef struct
{
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
    unsigned : 32; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 ||CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    unsigned : 16; /**< reserved */
    unsigned fft_gain : 8;
    unsigned agc_gain : 8;
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
      signed : 8;  /**< reserved */
    unsigned : 24; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
} wifi_pkt_rx_ctrl_phy_t;


static void wifi_init_mode1()
{
    // esp_wifi_enable_rx_stbc(1);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    ESP_ERROR_CHECK(esp_wifi_start());
    
    esp_wifi_set_band_mode(CONFIG_WIFI_BAND_MODE);
    wifi_protocols_t protocols = {
        .ghz_2g = CONFIG_WIFI_2G_PROTOCOL,
        // .ghz_5g = CONFIG_WIFI_5G_PROTOCOL
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols));


    wifi_bandwidths_t bandwidth = {
        .ghz_2g = CONFIG_WIFI_2G_BANDWIDTHS,
        // .ghz_5g = CONFIG_WIFI_5G_BANDWIDTHS
    };
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth));

#else
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, CONFIG_WIFI_BANDWIDTH));
    ESP_ERROR_CHECK(esp_wifi_start());
#endif

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3 
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA, CONFIG_ESP_NOW_RATE));
#endif
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6  || CONFIG_IDF_TARGET_ESP32C61
    if ((CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_2G_ONLY && CONFIG_WIFI_2G_BANDWIDTHS == WIFI_BW_HT20) 
     || (CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_5G_ONLY && CONFIG_WIFI_5G_BANDWIDTHS == WIFI_BW_HT20))
        ESP_ERROR_CHECK(esp_wifi_set_channel(g_csi_channel, WIFI_SECOND_CHAN_NONE));
    else
        ESP_ERROR_CHECK(esp_wifi_set_channel(g_csi_channel, WIFI_SECOND_CHAN_BELOW));
#else
    if (CONFIG_WIFI_BANDWIDTH == WIFI_BW_HT20)
        ESP_ERROR_CHECK(esp_wifi_set_channel(g_csi_channel, WIFI_SECOND_CHAN_NONE));
    else
        ESP_ERROR_CHECK(esp_wifi_set_channel(g_csi_channel, WIFI_SECOND_CHAN_BELOW));
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
}
#if CONFIG_IDF_TARGET_ESP32C5  || CONFIG_IDF_TARGET_ESP32C61
static void wifi_esp_now_init(esp_now_peer_info_t peer) 
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE, 
        .rate = CONFIG_ESP_NOW_RATE,//  WIFI_PHY_RATE_MCS0_LGI,    
        .ersu = false,                     
        .dcm = false                       
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr,&rate_config));

}
#endif




static void ws2812_led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    // if (led_strip == NULL) {
    //     ESP_LOGW(TAG, "LED Strip not initialized");
    //     return;
    // }
    
    // ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, b));
    // ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

static void radar_cb(const wifi_radar_info_t *info, void *ctx)
{
    static float *s_buff_jitter  = NULL;
    static uint32_t s_buff_count = 0;
    uint32_t buff_max_size       = g_console_input_config.predict_buff_size;
    uint32_t buff_outliers_num   = g_console_input_config.predict_outliers_number;
    uint32_t move_count          = 0;
    uint32_t touch_count         = 0;
    bool human_status            = false;
    bool touch_status            = false;

    if (!s_buff_jitter) {
        s_buff_jitter = calloc(RADAR_BUFF_MAX_LEN, sizeof(float));
    }

    s_buff_jitter[s_buff_count % RADAR_BUFF_MAX_LEN] = info->waveform_jitter;
    s_buff_count++;

    if (s_buff_count < buff_max_size) {
        return;
    }
    // static uint32_t s_last_call_time = 0;
    // uint32_t now = esp_log_timestamp();
    // if (s_last_call_time != 0) {
    //     ESP_LOGI(TAG, "radar_cb interval: %lu ms", now - s_last_call_time);
    // }
    // s_last_call_time = now;

    extern float median(const float *a, size_t len);

    float jitter_midean = median(s_buff_jitter, RADAR_BUFF_MAX_LEN);

    for (int i = 0; i < buff_max_size; i++) {
        uint32_t index = (s_buff_count - 1 - i) % RADAR_BUFF_MAX_LEN;
        if ((s_buff_jitter[index] * g_console_input_config.predict_move_sensitivity > jitter_midean && s_buff_jitter[index] > 0.0002)) {
            move_count++;
        }
        if ((s_buff_jitter[index] * g_console_input_config.predict_touch_sensitivity > jitter_midean && s_buff_jitter[index] > 0.0002)) {
            touch_count++;
        }
    }

    if (move_count >= buff_outliers_num) {
        human_status = true;
    }
    if (touch_count >= buff_outliers_num) {
        touch_status = true;
    }

    static uint32_t s_count = 0;

    if (!s_count) {
        ESP_LOGI(TAG, "================ RADAR RECV ================");
        ESP_LOGI(TAG, "type,sequence,timestamp,waveform_jitter,jitter_midean,move_threshold,move_status,move_sensitivity,touch_status,touch_sensitivity,touch_threshold");
    }

    char timestamp_str[32] = {0};
    sprintf(timestamp_str, "%lu", esp_log_timestamp());

    if (ctx) {
        strncpy(timestamp_str, (char *)ctx, 31);
    }

    static uint32_t s_last_move_time    = 0;

    printf("RADAR_DADA,%ld,%s,%d,%d,%d,%d,%.6f,%.6f,%.6f,%d,%.6f,%d,%.6f,%.6f\n",
           s_count++, timestamp_str,0,0,0,0,
           info->waveform_jitter, jitter_midean, jitter_midean / g_console_input_config.predict_move_sensitivity, human_status,g_console_input_config.predict_move_sensitivity, touch_status, g_console_input_config.predict_touch_sensitivity,jitter_midean / g_console_input_config.predict_touch_sensitivity);
    if (g_csi_data_enable) {
        uint8_t data_record[3] = {0};
        data_record[0] = 0x0a;
        data_record[1] = ((int16_t)(info->waveform_jitter *1000*100))>>8;
        data_record[2] = ((int16_t)(info->waveform_jitter *1000*100))&0xFF;
        if (g_csi_data_enable) {
            control_serial_send_data(data_record, 3);
        }
    }
    static uint8_t event_record[3] = {0};
    event_record[0] = 0x07;
    event_record[1] = (uint8_t)touch_status | ((uint8_t)human_status<<1);


    if (event_record[1] != event_record[2]) {
        control_serial_send_data(event_record, 2);
        ESP_LOGI(TAG, "event_record[1]: %d", event_record[1]);
    }
    event_record[2] = event_record[1];
    // if (human_status) {
    //     // s_last_move_time = esp_log_timestamp();
    //     control_serial_send_data(0x07, 0x01);
    // } else {

    // }
    // else if (esp_log_timestamp() - s_last_move_time > 3 * 1000) {

    // }

}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{

    // ESP_LOGI(TAG, "<%s>", __func__);
    
    if (!info || !info->buf) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }
    
    // 过滤不关心的 MAC：
    // - 自发自收模式（0x01）：只保留来自 CONFIG_CSI_SEND_MAC 的数据
    // - 路由器模式（0x02）：只保留来自 AP BSSID (ctx) 的数据
    if (((g_csi_mode == 0x01 && memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6) != 0) && (g_csi_mode == 0x01 && memcmp(info->mac, CONFIG_CSI_ECHOEAR_MAC, 6) != 0))||
        (g_csi_mode == 0x02 && ctx != NULL && memcmp(info->mac, ctx, 6) != 0)) {
        return;
    }
    wifi_csi_cb(ctx, info);
    wifi_pkt_rx_ctrl_phy_t *phy_info = (wifi_pkt_rx_ctrl_phy_t *)info;
    static int s_count = 0;
    uint32_t rx_id = *(uint32_t *)(info->payload + 15);
    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
if (g_print_csi_data) {
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,seq,mac,rssi,rate,noise_floor,fft_gain,agc_gain,channel,local_timestamp,sig_len,rx_state,len,first_word,data\n");
    }

    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
            rx_id, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
            rx_ctrl->noise_floor, phy_info->fft_gain, phy_info->agc_gain,rx_ctrl->channel,
            rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->rx_state);
#else
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,id,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,len,first_word,data\n");
    }

    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            rx_id, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
            rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
            rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
            rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
            rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->rx_state);

#endif
#if (CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61) && CSI_FORCE_LLTF
    ets_printf(",%d,%d,\"[%d", (info->len-2)/2, info->first_word_invalid, (int16_t)(((int16_t)info->buf[1]) << 12)>>4 | (uint8_t)info->buf[0]);
    for (int i = 2; i < (info->len-2); i+=2) {
        ets_printf(",%d", (int16_t)(((int16_t)info->buf[i+1]) << 12)>>4 | (uint8_t)info->buf[i]);
    }
#else
    ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, info->buf[0]);
    for (int i = 1; i < info->len; i++) {
        ets_printf(",%d", info->buf[i]);
    }
#endif
    ets_printf("]\"\n");
    s_count++;
    }
}

static void wifi_csi_init()
{
    esp_radar_start();
    wifi_radar_config_t radar_config = WIFI_RADAR_CONFIG_DEFAULT();
    radar_config.wifi_radar_cb = radar_cb;
    esp_radar_set_config(&radar_config);

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    /**< default config */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    wifi_csi_config_t csi_config = {
        .enable                   = true,                           
        .acquire_csi_legacy       = false,               
        .acquire_csi_force_lltf   = CSI_FORCE_LLTF,           
        .acquire_csi_ht20         = true,                  
        .acquire_csi_ht40         = true,                  
        .acquire_csi_vht          = false,                  
        .acquire_csi_su           = false,                   
        .acquire_csi_mu           = false,                   
        .acquire_csi_dcm          = false,                  
        .acquire_csi_beamformed   = false,           
        .acquire_csi_he_stbc_mode = 2, 
        .val_scale_cfg            = 0,                    
        .dump_ack_en              = false, 
        .lltf_bit_mode            = 0,
        .reserved                 = false                         
    };
#elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config_t csi_config = {
        .enable                 = true,                           
        .acquire_csi_legacy     = false,                        
        .acquire_csi_ht20       = true,                  
        .acquire_csi_ht40       = true,                                   
        .acquire_csi_su         = false,                   
        .acquire_csi_mu         = false,                   
        .acquire_csi_dcm        = false,                  
        .acquire_csi_beamformed = false,           
        .acquire_csi_he_stbc    = 2,  
        .val_scale_cfg          = false,                    
        .dump_ack_en            = false,                      
        .reserved               = false                         
    };
#else
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = false,
    };
#endif

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    if (g_csi_mode == 0x02) {
        static wifi_ap_record_t s_ap_info = {0};
        ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&s_ap_info));
        ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, s_ap_info.bssid));
    }else{
        ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    }
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));


}
void app_csi_uart_cb(uint8_t *data, size_t len)
{
    uint32_t cmd = data[0];
    switch (cmd) {
        case 0x06:
            uint8_t type = data[1];
            if (type == 0x01) { //指令
                g_csi_mode = data[2];
                ESP_LOGI(TAG, "g_csi_mode: %d", g_csi_mode);
                if (g_csi_mode == 0x01) {//自发自收
                    g_csi_channel = data[3];
                    g_console_input_config.predict_move_sensitivity = (float)data[4]/100.0;
                    g_console_input_config.predict_touch_sensitivity = (float)data[5]/100.0;
                    ESP_LOGI(TAG, "g_csi_channel: %d, predict_move_sensitivity: %f, predict_touch_sensitivity: %f", g_csi_channel, g_console_input_config.predict_move_sensitivity, g_console_input_config.predict_touch_sensitivity);
                } else if (g_csi_mode == 0x02) {//路由器模式
                    size_t ssid_len = 0, pwd_len = 0;
                    uint8_t *p = data + 3; // 跳过 [0]=0x06,[1]=type,[2]=g_csi_mode
                    size_t max_scan = len > 3 ? len - 3 : 0;
                    size_t sep_pos = 0;
                    for (size_t i = 0; i < max_scan; ++i) {
                        if (p[i] == 0x1F) {
                            sep_pos = i;
                            break;
                        }
                    }
                    if (sep_pos > 0 && sep_pos < max_scan - 1) {
                        ssid_len = sep_pos;
                        pwd_len = max_scan - (sep_pos + 1);
                        size_t copy_len_ssid = ssid_len < 32 ? ssid_len : 32;
                        size_t copy_len_pwd  = pwd_len  < 64 ? pwd_len  : 64;

                        // 写入全局 SSID / PWD 变量
                        memset(g_wifi_ssid, 0, sizeof(g_wifi_ssid));
                        memset(g_wifi_pwd,  0, sizeof(g_wifi_pwd));
                        memcpy(g_wifi_ssid, p, copy_len_ssid);
                        memcpy(g_wifi_pwd,  p + sep_pos + 1, copy_len_pwd);
                        g_wifi_ssid[copy_len_ssid] = '\0';
                        g_wifi_pwd[copy_len_pwd]   = '\0';

                        // 打印全局变量内容
                        ESP_LOGI(TAG, "Received SSID: [%s]", g_wifi_ssid);
                        ESP_LOGI(TAG, "Received PASS: [%s]", g_wifi_pwd);
                    } else {
                        ESP_LOGW(TAG, "SSID/PASS Parse Error: No separator 0x1F found or format error.");
                    }
                }else if(data[2] == 0x00) {

                }

            } else if (type == 0x02) { //查询
                if(data[2] == 0x01) { // 实时信息
                    g_realtime_time = (uint16_t)data[3]<<8 | data[4];
                    if(g_realtime_time > 0) {
                        g_realtime_enable = true;
                    } else {
                        g_realtime_enable = false;
                    }
                } else if(data[2] == 0x02) {//获取统计信息
                    // csi_summary_report();|
                    ESP_LOGI(TAG, "CSI Summary Report");
                }else if(data[2] == 0x03) {//获取事件信息
                    g_csi_enevt_enable = (bool)data[3];
                }else if(data[2] == 0x04) {//获取CSI数据
                    g_csi_data_enable = (bool)data[3];
                }
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd);
            break;
    }
}
#define CONFIG_SEND_FREQUENCY      100
static esp_err_t wifi_ping_router_start()
{
    static esp_ping_handle_t ping_handle = NULL;

    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ping_config.count             = 0;
    ping_config.interval_ms       = 1000 / CONFIG_SEND_FREQUENCY;
    ping_config.task_stack_size   = 3072;
    ping_config.data_size         = 1;

    esp_netif_ip_info_t local_ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &local_ip);
    ESP_LOGI(TAG, "got ip:" IPSTR ", gw: " IPSTR, IP2STR(&local_ip.ip), IP2STR(&local_ip.gw));
    ping_config.target_addr.u_addr.ip4.addr = ip4_addr_get_u32(&local_ip.gw);
    ping_config.target_addr.type = ESP_IPADDR_TYPE_V4;

    esp_ping_callbacks_t cbs = { 0 };
    esp_ping_new_session(&ping_config, &cbs, &ping_handle);
    esp_ping_start(ping_handle);

    return ESP_OK;
}
// WiFi connected initialization task
static void wifi_connected_init_task(void *pvParameters)
{
    // Wait for IP address
    ESP_LOGI(TAG, "Waiting for IP address...");
    int retry = 0;
    while (retry < 30) {
        esp_netif_ip_info_t ip_info;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            if (ip_info.ip.addr != 0) {
                ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&ip_info.ip));
                
                // Initialize CSI
                wifi_csi_init();
                
                // Start ping
                wifi_ping_router_start();
                
                ESP_LOGI(TAG, "WiFi connected successfully, CSI and Ping started");
                vTaskDelete(NULL);
                return;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry++;
    }
    
    ESP_LOGW(TAG, "Waiting for IP address timeout");
    vTaskDelete(NULL);
}
// WiFi configuration command handler
static int wifi_init_mode2()
{
    // Configure WiFi
    wifi_config_t wifi_config = {0};
    memcpy(wifi_config.sta.ssid, g_wifi_ssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, g_wifi_pwd, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    // Disconnect current connection (if any)
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Set new configuration and connect
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        printf("WiFi connect failed, error code: 0x%x\n", ret);
        return 1;
    }
    printf("WiFi connect request sent, waiting for result...\n");
    
    // Create task to initialize CSI and Ping after connection
    xTaskCreate(wifi_connected_init_task, "wifi_init", 4096, NULL, 5, NULL);
    
    return 0;
}
void app_csi_task(void *pvParameter)
{
    /**
     * @breif Initialize NVS
     */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    while(g_csi_mode ==0)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if (g_csi_mode == 0x01) {
        wifi_init_mode1();

    #if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
        esp_now_peer_info_t peer = {
            .channel   = g_csi_channel,
            .ifidx     = WIFI_IF_STA,    
            .encrypt   = false,   
            .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        };

        wifi_esp_now_init(peer);
    #endif
        wifi_csi_init();
    }else if (g_csi_mode == 0x02) {
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(esp_netif_init());
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        
        esp_netif_create_default_wifi_sta();
        
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "WiFi initialized. Use console command to configure and connect.");
        ESP_LOGI(TAG, "Enter: wifi_config <SSID> <password>");
        wifi_init_mode2();
    }
    vTaskDelete(NULL);
}
