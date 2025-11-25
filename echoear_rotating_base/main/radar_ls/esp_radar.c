// ESP雷达系统核心实现文件
// 用于基于WiFi CSI的人体活动检测和运动感知

#include "freertos/FreeRTOS.h"        // FreeRTOS实时操作系统
#include "freertos/task.h"            // FreeRTOS任务管理
#include "freertos/event_groups.h"    // FreeRTOS事件组
#include "freertos/queue.h"           // FreeRTOS队列

#include "esp_mac.h"                  // ESP MAC地址相关
#include "esp_system.h"               // ESP系统相关
#include "esp_wifi.h"                 // ESP WiFi接口
#include "esp_log.h"                  // ESP日志系统

#include "esp_radar.h"               // 雷达功能头文件
#include "utils.h"                   // 工具函数头文件

// 事件位定义
#define CSI_HANDLE_EXIT_BIT         BIT0    // CSI数据处理任务退出标志位
#define CSI_COMBINED_EXIT_BIT       BIT1    // CSI数据组合任务退出标志位

// 异常值检测阈值
#define CSI_OUTLIERS_THRESHOLD      8       // CSI异常值检测阈值

// 子载波配置参数
#define SUB_CARRIER_TABLE_SIZE      13      // 子载波表大小
#define CSI_SUBCARRIER_OFFSET       1       // CSI子载波偏移量

// 雷达处理参数
#define SUB_CARRIER_STEP_SIZE       5       // 子载波步长
#define RADAR_MOVE_BUFFER_SIZE      4       // 雷达移动缓冲区大小
#define RADAR_SUBCARRIER_LEN        ((52 - CSI_SUBCARRIER_OFFSET)/ SUB_CARRIER_STEP_SIZE)  // 雷达子载波长度
// #define FILTER_OUTLIER                      // 启用异常值过滤

// MAC地址判断宏
#define ADDR_IS_FILTER(addr)        (((addr)[0] & (addr)[1] & (addr)[2] & (addr)[3] & (addr)[4] & (addr)[5]) != 0xFF)  // 判断是否为过滤地址
#define ADDR_IS_EMPTY(addr)         (((addr)[0] | (addr)[1] | (addr)[2] | (addr)[3] | (addr)[4] | (addr)[5]) == 0x0)   // 判断是否为空地址

// WiFi数据包接收控制信息结构（带增益信息）
// 支持ESP32/ESP32S2/ESP32S3/ESP32C3系列芝片
typedef struct {
    signed rssi:8;                /**< 接收信号强度指示器(RSSI)，单位: dBm */
    unsigned rate:5;              /**< 数据包的PHY速率编码，仅对非 HT(11bg)数据包有效 */
    unsigned :1;                  /**< 保留位 */
    unsigned sig_mode:2;          /**< 信号模式: 0=非HT(11bg); 1=HT(11n); 3=VHT(11ac) */
    unsigned :16;                 /**< 保留位 */
    unsigned mcs:7;               /**< 调制编码方案，对HT数据包显示调制方式，范围 0-76 */
    unsigned cwb:1;               /**< 信道带宽: 0=20MHz; 1=40MHz */
    unsigned :16;                 /**< 保留位 */
    unsigned smoothing:1;         /**< 保留位 */
    unsigned not_sounding:1;      /**< 保留位 */
    unsigned :1;                  /**< 保留位 */
    unsigned aggregation:1;       /**< 聚合标志: 0=MPDU数据包; 1=AMPDU数据包 */
    unsigned stbc:2;              /**< 空时块编码(STBC): 0=非STBC; 1=STBC数据包 */
    unsigned fec_coding:1;        /**< 11n数据包LDPC编码标志 */
    unsigned sgi:1;               /**< 短保护间隔(SGI): 0=长保护间隔; 1=短保护间隔 */
#if CONFIG_IDF_TARGET_ESP32
    signed noise_floor:8;         /**< noise floor of Radio Frequency Module(RF). unit: dBm*/
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 ||CONFIG_IDF_TARGET_ESP32C6|| CONFIG_IDF_TARGET_ESP32C61
    unsigned :8;                  /**< reserved */
#endif
    unsigned ampdu_cnt:8;         /**< ampdu cnt */
    unsigned channel:4;           /**< primary channel on which this packet is received */
    unsigned secondary_channel:4; /**< secondary channel on which this packet is received. 0: none; 1: above; 2: below */
    unsigned :8;                  /**< reserved */
    unsigned timestamp:32;        /**< timestamp. The local time when this packet is received. It is precise only if modem sleep or light sleep is not enabled. unit: microsecond */
    unsigned :32;                 /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
    unsigned :32;                 /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 ||CONFIG_IDF_TARGET_ESP32C6|| CONFIG_IDF_TARGET_ESP32C61
    signed noise_floor:8;         /**< noise floor of Radio Frequency Module(RF). unit: dBm*/
    unsigned :8;                 /**< reserved */
    signed fft_gain: 8;
    unsigned agc_gain: 8;
    int32_t compensate_gain: 32;                 /**<  */
#endif
    unsigned :31;                 /**< reserved */
    unsigned ant:1;               /**< antenna number from which this packet is received. 0: WiFi antenna 0; 1: WiFi antenna 1 */
#if CONFIG_IDF_TARGET_ESP32S2
    signed noise_floor:8;         /**< noise floor of Radio Frequency Module(RF). unit: dBm*/
    unsigned :24;                 /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 ||CONFIG_IDF_TARGET_ESP32C6|| CONFIG_IDF_TARGET_ESP32C61
    unsigned :32;                 /**< reserved */
    unsigned :32;                 /**< reserved */
    unsigned :32;                 /**< reserved */
#endif
    unsigned sig_len:12;          /**< 数据包长度（包含帧校验序列FCS） */
    unsigned :12;                 /**< 保留位 */
    unsigned rx_state:8;          /**< 数据包状态: 0=无错误; 其他=错误编号 */
} wifi_pkt_rx_ctrl_gain_t;           // WiFi数据包接收控制信息（带增益）

// WiFi信号模式枚举
typedef enum {
    WIFI_SIGNAL_MODE_NON_HT,  /**< 802.11b/g模式 */
    WIFI_SIGNAL_MODE_HT,      /**< 802.11n模式 */
    WIFI_SIGNAL_MODE_VHT,     /**< 802.11ac模式 */
} wifi_signal_mode_t;

// WiFi信道带宽枚举
typedef enum {
    WIFI_CHANNEL_BANDWIDTH_20MHZ = 0,    // 20MHz带宽
    WIFI_CHANNEL_BANDWIDTH_40MHZ = 1,    // 40MHz带宽
} wifi_channel_bandwidth_t;

// WiFi单用户SIGA2结构（802.11ac）
typedef struct {
    uint16_t txop : 7;                      // 传输机会时间
    uint16_t coding : 1;                    // 编码方式
    uint16_t ldpc_extra_symbol_segment : 1; // LDPC额外符号段
    uint16_t stbc : 1;                      // 空时块编码
    uint16_t beamformed : 1;                // 波束成形标志
    uint16_t pre_fec_padding_factor : 2;    // FEC前填充因子
    uint16_t pe_disambiguity : 1;           // 包扩展消歧义
    uint16_t rsvd : 1;                      // 保留位
    uint16_t doppler : 1;                   // 多普勒标志
} __attribute__((packed)) esp_wifi_su_siga2_t;

// WiFi单用户SIGA1结构（802.11ax HE）
typedef struct {
    uint32_t format : 1;                        // 格式标志
    uint32_t beam_change : 1;                   // 波束变化标志
    uint32_t ul_dl : 1;                         // 上行/下行标志
    uint32_t he_mcs : 4;                        // HE调制编码方案
    uint32_t dcm : 1;                           // 双载波调制
    uint32_t bss_color : 6;                     // BSS颜色
    uint32_t rsvd : 1;                          // 保留位
    uint32_t spatial_reuse : 4;                 // 空间复用
    uint32_t bw : 2;                            // 带宽
    uint32_t gi_and_he_ltf_size : 2;            // 保护间隔和HE-LTF大小
    uint32_t nsts_and_midamble_periodicity : 3; // 空时流数和中间序列周期
    uint32_t rsvd1 : 6;                         // 保留位1
} esp_wifi_su_siga1_t;

// 子载波范围结构
typedef struct {
    uint16_t start;    // 起始子载波索引
    uint16_t stop;     // 结束子载波索引
} sub_carrier_range_t;

// CSI子载波表结构，定义不同信号模式下的子载波分布
typedef struct {
    wifi_second_chan_t second;              // 辅助信道类型
    wifi_signal_mode_t signal_mode;         // 信号模式（非HT/HT/VHT）
    wifi_channel_bandwidth_t channel_bandwidth; // 信道带宽
    bool stbc;                              /**< 空时块编码(STBC): 0=非STBC; 1=STBC数据包 */
    size_t total_bytes;                     // 总字节数
    size_t valid_bytes;                     // 有效字节数
    uint8_t llft_bytes;                     // LLTF字节数
    uint8_t ht_lft_bytes;                   // HT-LTF字节数
    uint8_t stbc_ht_lft_bytes;              // STBC HT-LTF字节数
    union  {
        struct {
            sub_carrier_range_t llft[2];        // LLTF子载波范围（最多2个范围）
            sub_carrier_range_t ht_lft[4];      // HT-LTF子载波范围（最多4个范围）
            sub_carrier_range_t stbc_ht_lft[4]; // STBC HT-LTF子载波范围（最多4个范围）
        };
        sub_carrier_range_t sub_carrier[10];    // 所有子载波范围数组（最多10个）
    };
} csi_sub_carrier_table_t;

static const csi_sub_carrier_table_t sub_carrier_table[SUB_CARRIER_TABLE_SIZE] = {
    /**< ------------------- secondary channel : none ------------------- **/
    /**< non HT, 20 MHz, non STBC, LLTF: 0~31, -32~-1, 128 */
    {
        .second            = WIFI_SECOND_CHAN_NONE,
        .signal_mode       = WIFI_SIGNAL_MODE_NON_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 128,
        .valid_bytes       = 104,
        .llft_bytes        = 104,
        .llft = {{2, 54}, {76, 128}},
    },
    /**< HT,     20 MHz, non STBC, LLTF: 0~31, -32~-1, HT-LFT: 0~31, -32~-1, 256 */
    {
        .second            = WIFI_SECOND_CHAN_NONE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 256,
        .valid_bytes       = 216,
        .llft_bytes        = 104,
        .ht_lft_bytes      = 112,
        .llft = {{2, 54}, {76, 128}},
        .ht_lft = {{130, 186}, {200, 256}}
    },
    /**< HT,     20 MHz,     STBC, LLTF: 0~31, -32~-1, HT-LFT: 0~31, -32~-1, STBC-HT-LTF: 0~31, -32~-1, 384 */
    {
        .second            = WIFI_SECOND_CHAN_NONE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = true,
        .total_bytes       = 384,
        .valid_bytes       = 328,
        .llft_bytes        = 104,
        .ht_lft_bytes      = 112,
        .stbc_ht_lft_bytes = 112,
        .llft = {{2, 54}, {76, 128}},              // 1, 11, 0
        .ht_lft = {{130, 186}, {200, 256}},        // 1, 7, 0
        .stbc_ht_lft = {{258, 314}, {328, 384}}    // 1, 7, 0
    },

    /**< ------------------- secondary channel : below ------------------- **/
    /**< non HT, 20 MHz, non STBC, LLTF: 0~63, 128 */
    {
        .second            = WIFI_SECOND_CHAN_BELOW,
        .signal_mode       = WIFI_SIGNAL_MODE_NON_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 128,
        .valid_bytes       = 104,
        .llft_bytes        = 104,
        .llft = {{12, 64}, {66, 118}},         // 6, 1, 5
    },
    /**< HT,     20 MHz, non STBC, LLTF: 0~63, HT-LFT: 0~63, 256 */
    {
        .second            = WIFI_SECOND_CHAN_BELOW,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 256,
        .valid_bytes       = 216,
        .llft_bytes        = 104,
        .ht_lft_bytes      = 112,
        .llft   = {{12, 64}, {66, 118}},      // 6, 1, 5
        .ht_lft = {{132, 188}, {190, 246}},   // 2, 1, 5
    },
    /**< HT,     20 MHz,     STBC, LLTF: 0~63, HT-LFT: 0~62, STBC-HT-LTF: 0~62, 380 */
    {
        .second            = WIFI_SECOND_CHAN_BELOW,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = true,
        .total_bytes       = 380,
        .valid_bytes       = 328,
        .llft_bytes        = 104,
        .ht_lft_bytes      = 112,
        .stbc_ht_lft_bytes = 112,
        .llft   = {{12, 64}, {66, 118}},           // 6, 1, 5   {{12,64}, {66,118}},
        .ht_lft = {{132, 188}, {190, 246}},        // 2, 1, 5   {{132,188}, {190,246}}
        .stbc_ht_lft = {{256, 312}, {314, 370}},   // 0, 1, 5   {{256,312}, {314,370}}
    },

    /**< HT,     40 MHz, non STBC, LLTF: 0~63, HT-LFT: 0~63, -64~-1, 384 */
    {
        .second            = WIFI_SECOND_CHAN_BELOW,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = false,
        .total_bytes       = 384,
        .valid_bytes       = 328,
        .llft_bytes        = 104,
        .ht_lft_bytes      = 224,
        .llft   = {{12, 64}, {66, 118}},                           // 6, 1, 5
        .ht_lft = {{132, 188}, {190, 246}, {268, 324}, {326, 382}} // 2, 1, 5   6, 1, 1
    },
    /**< HT,     40 MHz,     STBC, LLTF: 0~63, HT-LFT: 0~60, -60~-1, STBC-HT-LTF: 0~60, -60~-1, 612 */
    {
        .second            = WIFI_SECOND_CHAN_BELOW,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = true,
        .total_bytes       = 612,
        .valid_bytes       = 552,
        .llft_bytes        = 104,
        .ht_lft_bytes      = 224,
        .stbc_ht_lft_bytes = 224,
        .llft   = {{12, 64}, {66, 118}},                                // 6, 1, 5
        .ht_lft = {{132, 188}, {190, 246}, {254, 310}, {312, 368}},     // 2, 1, 2   2, 1, 1
        .stbc_ht_lft = {{374, 430}, {432, 488}, {496, 552}, {554, 610}} // 2, 1, 2   2, 1, 1
    },

    /**< ------------------- secondary channel : above ------------------- **/
    /**< non HT, 20 MHz, non STBC, LLTF: -64~-1, 128 */
    {
        .second            = WIFI_SECOND_CHAN_ABOVE,
        .signal_mode       = WIFI_SIGNAL_MODE_NON_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 128,
        .valid_bytes       = 104,
        .llft_bytes        = 104,
        .llft = {{12, 64}, {66, 118}},
    },
    /**< HT,     20 MHz, non STBC, LLTF: -64~-1, HT-LFT: -64~-1, 256 */
    {
        .second            = WIFI_SECOND_CHAN_ABOVE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 256,
        .valid_bytes       = 216,
        .llft_bytes        = 104,
        .ht_lft_bytes      = 112,
        .llft   = {{12, 64}, {66, 118}},   // 6, 1, 5
        .ht_lft = {{136, 192}, {194, 250}} // 4, 1, 3
    },
    /**< HT,     20 MHz,     STBC, LLTF: -64~-1, HT-LFT: -62~-1, STBC-HT-LTF: -62~-1, 376 */
    {
        .second            = WIFI_SECOND_CHAN_ABOVE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = true,
        .total_bytes       = 376,
        .valid_bytes       = 328,
        .llft_bytes        = 104,
        .ht_lft_bytes      = 112,
        .stbc_ht_lft_bytes = 112,
        .llft   = {{12, 64}, {66, 118}},         // 6, 1, 5
        .ht_lft = {{132, 188}, {190, 246}},      // 2, 1, 3
        .stbc_ht_lft = {{256, 312}, {314, 370}}   // 2, 1, 3
    },
    /**< HT,     40 MHz, non STBC, LLTF: -64~-1, HT-LFT: 0~63, -64~-1, 384 */ // TODO
    {
        .second            = WIFI_SECOND_CHAN_ABOVE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = false,
        .total_bytes       = 384,
        .valid_bytes       = 328,
        .llft_bytes        = 104,
        .ht_lft_bytes      = 224,
        .llft   = {{12, 64}, {66, 118}},                           // 6, 1, 5
        .ht_lft = {{136, 192}, {194, 250}, {260, 316}, {318, 374}} // 2, 1, 3,    6, 1, 1
    },
    /**< HT,     40 MHz,     STBC, LLTF: -64~-1, HT-LFT: 0~60, -60~-1, STBC-HT-LTF: 0~60, -60~-1, 612 */ // TODO
    {
        .second            = WIFI_SECOND_CHAN_ABOVE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = true,
        .total_bytes       = 612,
        .valid_bytes       = 552,
        .llft_bytes        = 104,
        .ht_lft_bytes      = 224,
        .stbc_ht_lft_bytes = 224,
        .llft   = {{12, 64}, {66, 118}},                                // 6, 1, 5
        .ht_lft = {{132, 188}, {190, 246}, {254, 310}, {312, 368}},     // 2, 1, 2   2, 1, 1
        .stbc_ht_lft = {{374, 430}, {432, 488}, {496, 552}, {554, 610}} // 2, 1, 2   2, 1, 1
    },
};

// ===== 全局变量定义 =====
static const char *TAG                          = "esp_radar";         // 日志标签
static EventGroupHandle_t g_csi_task_exit_group = NULL;                 // CSI任务退出事件组
static QueueHandle_t g_csi_info_queue           = NULL;                 // CSI信息队列
static QueueHandle_t g_csi_data_queue           = NULL;                 // CSI数据队列
static bool g_wifi_radar_init_flag              = false;                // WiFi雷达初始化标志
static wifi_radar_config_t *g_wifi_radar_config = NULL;                 // WiFi雷达配置指针
static bool g_wifi_radar_run_flag               = false;                // WiFi雷达运行标志
static uint32_t g_wifi_radar_buff_size          = 0;                    // WiFi雷达缓冲区大小
static uint32_t g_wifi_radar_handle_window      = 0;                    // WiFi雷达处理窗口大小
static bool g_csi_filter_dmac_flag              = false;                // CSI目标MAC过滤标志

// ===== 内存管理宏定义 =====
/**
 * @brief 动态内存分配函数
 * 优先使用SPIRAM（如果可用），否则使用默认堆
 */
#ifdef CONFIG_SPIRAM_SUPPORT
#define MALLOC_CAP_INDICATE MALLOC_CAP_SPIRAM      // 使用SPIRAM
#else
#define MALLOC_CAP_INDICATE MALLOC_CAP_DEFAULT     // 使用默认堆
#endif

// 带重试机制的内存分配完，分配失败时会等待并重试
#define RADAR_MALLOC_RETRY(size) ({                                                                                               \
        void *ptr = NULL;                                                                                                         \
        while (size > 0 && !(ptr = heap_caps_malloc(size, MALLOC_CAP_INDICATE))) {                                                \
            ESP_LOGW(TAG, "<ESP_ERR_NO_MEM> Realloc size: %d, ptr: %p, heap free: %d", (int)size, ptr, esp_get_free_heap_size()); \
            vTaskDelay(pdMS_TO_TICKS(100));                                                                                       \
        }                                                                                                                         \
        ptr;                                                                                                                      \
    })

// 安全释放内存宏，释放后将指针置为NULL
#define RADAR_FREE(ptr) do {    \
        if (ptr) {           \
            free(ptr);       \
            ptr = NULL;      \
        }                    \
    } while(0)

// 计算复数幅度的宏（欧几里得距离）
#define my_hypotf(a, b) sqrtf(a * a + b * b)

// CSI数据缓冲区结构
typedef struct {
    uint32_t begin;                                     // 缓冲区起始索引
    uint32_t end;                                       // 缓冲区结束索引
    float (*lltf_amplitude)[RADAR_SUBCARRIER_LEN];     // LLTF幅度数据数组指针
    uint32_t *timestamp;                               // 时间戳数组指针
} csi_data_buff_t;

// CSI数据缓冲区索引结构
typedef struct {
    union  {
        struct {
            uint8_t begin;      // 窗口起始位置
            uint8_t end;        // 窗口结束位置
            uint8_t window;     // 窗口大小
            uint8_t reserved;   // 保留字节
        };
        uint32_t data;          // 作为32位整数访问
    };
} csi_data_buff_index_t;

static csi_data_buff_t *g_csi_data_buff = NULL;    // 全局CSI数据缓冲区指针

// CSI缓冲区索引计算宏（环形缓冲区）
#define CSI_BUFF_INDEX(x) ((x) % g_wifi_radar_buff_size)

// 雷达校准状态枚举
typedef enum {
    RADAR_CALIBRATE_NO,         // 未校准
    RADAR_CALIBRATE_PROGRESS,   // 正在校准
    RADAR_CALIBRATE_COMPLETE,   // 校准完成
} radar_calibrate_status_t;

// 雷达校准相关常数
#define CSI_CORR_NUM        10      // CSI相关数据数量
#define CSI_CORR_THRESHOLD  0.998   // CSI相关系数阈值
#define RADAR_BUFF_NUM      3       // 雷达缓冲区数量

// 雷达校准数据结构
typedef struct {
    radar_calibrate_status_t calibrate_status;      // 校准状态
    float *data[CSI_CORR_NUM];                      // 校准数据数组指针
    // float data[CSI_CORR_NUM][RADAR_SUBCARRIER_LEN];  // 原始数据数组（已注释）
    size_t buff_size;                               // 缓冲区大小
    float waveform_jitter_buff[RADAR_BUFF_NUM];     // 波形抖动缓冲区
    size_t data_num;                                // 数据数量
    float none_corr_sum;                            // 非相关性总和
    float none_corr_count;                          // 非相关性计数
    float none_corr;                                // 当前非相关性
    float static_corr;                              // 静态相关性
} radar_calibrate_t;

static radar_calibrate_t *g_radar_calibrate = NULL;        // 全局雷达校准数据指针
static uint32_t g_pca_subcarrier_buff_num = 0;              // PCA子载波缓冲区计数
static wifi_pkt_rx_ctrl_gain_t g_wifi_pkt_rx_ctrl = {0};   // WiFi数据包接收控制信息

/**
 * @brief 开始雷达校准训练
 * 初始化校准数据结构，开始收集静态环境的基准数据
 * @return ESP_OK 成功
 */
esp_err_t esp_radar_train_start()
{
    // 如果校准结构未初始化，则分配内存并初始化
    if (!g_radar_calibrate) {
        g_radar_calibrate = RADAR_MALLOC_RETRY(sizeof(radar_calibrate_t));
        memset(g_radar_calibrate, 0, sizeof(radar_calibrate_t));
        g_radar_calibrate->data_num = 0;
    }

    // 设置校准状态为“正在进行”
    g_radar_calibrate->calibrate_status = RADAR_CALIBRATE_PROGRESS;
    g_radar_calibrate->none_corr        = 1;    // 初始非相关性为1
    g_radar_calibrate->static_corr      = 1;    // 初始静态相关性为1
    g_radar_calibrate->buff_size        = 0;    // 重置缓冲区大小
    g_pca_subcarrier_buff_num           = 0;    // 重置PCA缓冲区计数

    return ESP_OK;
}

/**
 * @brief 移除雷达校准数据
 * 清空所有已收集的校准数据，但保留校准结构
 * @return ESP_OK 成功
 */
esp_err_t esp_radar_train_remove()
{
    if (g_radar_calibrate) {
        g_radar_calibrate->data_num = 0;            // 重置数据数量
        g_radar_calibrate->none_corr_sum    = 0;    // 重置非相关性总和
        g_radar_calibrate->none_corr_count  = 0;    // 重置非相关性计数

        // 释放所有已分配的数据内存
        for (int i = 0; i < CSI_CORR_NUM && g_radar_calibrate->data[i]; ++i) {
            RADAR_FREE(g_radar_calibrate->data[i]);
        }
    }

    return ESP_OK;
}

/**
 * @brief 停止雷达校准训练
 * 结束校准过程，计算并返回检测阈值
 * @param wander_threshold 返回漂移检测阈值
 * @param jitter_threshold 返回抖动检测阈值
 * @return ESP_OK 成功
 * @return ESP_ERR_NOT_SUPPORTED 校准未初始化
 * @return ESP_ERR_INVALID_STATE 没有收集到有效数据
 */
esp_err_t esp_radar_train_stop(float *wander_threshold, float *jitter_threshold)
{
    // 检查校准结构是否存在
    if (!g_radar_calibrate) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    // 检查是否有有效数据
    if (!g_radar_calibrate->data_num) {
        return ESP_ERR_INVALID_STATE;
    }

    // 设置校准状态为“完成”
    g_radar_calibrate->calibrate_status = RADAR_CALIBRATE_COMPLETE;
    // 计算漂移阈值：1 - 平均非相关性
    *wander_threshold = 1 - (g_radar_calibrate->none_corr_sum / g_radar_calibrate->none_corr_count);
    // 计算抖动阈值：1 - 静态相关性
    *jitter_threshold = 1 - g_radar_calibrate->static_corr;

    return ESP_OK;
}

#ifdef RADAR_DEBUG
/**
 * @brief 调试函数：打印浮点数数组
 * @param tag 标签字符串
 * @param data 浮点数数组指针
 * @param size 数组大小
 */
void print_float_list(const char *tag, float *data, size_t size)
{
    printf("%s, [", tag);  // 打印标签和开始括号

    // 遍历数组，打印每个元素
    for (int i = 0; i < size; ++i) {
        printf("%0.2f, ", data[i]);  // 保疙2位小数
    }

    printf("]\r\n");  // 打印结束括号和换行
}
#endif

/**
 * @brief CSI数据处理任务
 * 雷达系统的核心数据处理任务，负责：
 * 1. 接收组合好的CSI数据包
 * 2. 执行PCA主成分分析
 * 3. 计算相关性分析（抖动和漂移）
 * 4. 进行雷达校准和训练
 * 5. 输出雷达检测结果
 * @param arg 任务参数（未使用）
 */
static void csi_data_handle_task(void *arg)
{
    // 初始化PCA子载波缓冲区数组（滑动窗口）
    float *pca_subcarrier_buff[RADAR_MOVE_BUFFER_SIZE] = {0};

    // 为每个缓冲区分配内存
    for (int i = 0; i < RADAR_MOVE_BUFFER_SIZE; ++i) {
        pca_subcarrier_buff[i] = RADAR_MALLOC_RETRY(RADAR_SUBCARRIER_LEN * sizeof(float));
    }

    csi_data_buff_index_t buff_index = {0};  // 数据缓冲区索引

    esp_err_t ret = ESP_OK;  // 返回状态

    // 主循环：持续从队列接收数据包索引
    while (xQueueReceive(g_csi_data_queue, &buff_index, portMAX_DELAY) && g_wifi_radar_run_flag) {
        // 初始化雷达信息结构（默认无运动）
        wifi_radar_info_t radar_info = {
            .waveform_jitter = 1,  // 波形抖动初始值
            .waveform_wander = 1,  // 波形漂移初始值
        };

        // 获取时间窗口的起始和结束时间戳
        uint32_t time_start = g_csi_data_buff->timestamp[buff_index.begin];
        uint32_t time_end   = g_csi_data_buff->timestamp[buff_index.end];
        int32_t time_spent  = time_end - time_start;  // 计算时间窗口耗时

        // 注释：自动接收增益控制相关代码（当前未启用）
        // 如果启用自动增益控制，会在增益调整时重置校准数据
        // extern bool esp_radar_auto_rx_gain_status();
        // if (esp_radar_auto_rx_gain_status()) {
        //     if (g_radar_calibrate && g_radar_calibrate->calibrate_status == RADAR_CALIBRATE_PROGRESS) {
        //         esp_radar_train_remove();
        //     }
        //     g_pca_subcarrier_buff_num = 0;
        //     continue;
        // }

        // 检查数据窗口大小是否足够（至少为处理窗口的1/3）
        if (buff_index.window < g_wifi_radar_handle_window / 3) {
            ESP_LOGW(TAG, "buff_index.window: %d, time_spent: %d, g_wifi_radar_handle_window: %d", buff_index.window, time_spent, g_wifi_radar_handle_window);
            continue;  // 数据不足，跳过本次处理
        }

        uint32_t timestamp_start = esp_log_timestamp();  // 记录处理开始时间

        // 获取当前的PCA缓冲区指针（环形缓冲区）
        float *pca_subcarrier_buff_current = pca_subcarrier_buff[g_pca_subcarrier_buff_num % RADAR_MOVE_BUFFER_SIZE];
        g_pca_subcarrier_buff_num++;  // 增加PCA缓冲区计数

        // 准备CSI数据指针（处理环形缓冲区的跨界情况）
        float (*csi_data_0)[RADAR_SUBCARRIER_LEN] = g_csi_data_buff->lltf_amplitude + buff_index.begin;
        float (*csi_data_1)[RADAR_SUBCARRIER_LEN] = g_csi_data_buff->lltf_amplitude;
        // 计算数据段的长度（处理缓冲区跨界的情况）
        uint8_t csi_data_0_len = buff_index.begin <= buff_index.end ? buff_index.window : g_wifi_radar_buff_size - buff_index.begin;
        uint8_t csi_data_1_len = buff_index.window - csi_data_0_len;

        // 注释：调试打印CSI数据（当前未启用）
        // print_float_list("csi_data_0", csi_data_0, RADAR_SUBCARRIER_LEN);
        
        // 执行PCA主成分分析，将CSI数据降维为一维特征向量
        ret = pca(RADAR_SUBCARRIER_LEN, csi_data_0_len, csi_data_0,
                  csi_data_1_len, csi_data_1, pca_subcarrier_buff_current);
        if (ret != ESP_OK) {
            // PCA计算失败，回退计数并跳过本次处理
            g_pca_subcarrier_buff_num--;
            continue;
        }
        float_t corr_temp[RADAR_MOVE_BUFFER_SIZE] = {0};  // 临时相关系数数组

        // 检查是否有足够的历史数据进行相关性分析
        if (g_pca_subcarrier_buff_num < RADAR_MOVE_BUFFER_SIZE) {
            goto EXIT;  // 数据不足，跳转到退出处理
        }

        // 初始化波形抖动为0（将计算最大相关性）
        radar_info.waveform_jitter = 0.0;

        // 计算当前数据与历史数据的相关性（短期变化检测）
        for (int i = 0; i < RADAR_MOVE_BUFFER_SIZE - 1; ++i) {
            // 计算与第i+1个历史数据的相关系数
            corr_temp[i] = fabs(corr(pca_subcarrier_buff_current,
                                     pca_subcarrier_buff[(g_pca_subcarrier_buff_num - 2 - i) % RADAR_MOVE_BUFFER_SIZE],
                                     RADAR_SUBCARRIER_LEN));

            // 找到最大相关性（最相似的历史数据）
            if (radar_info.waveform_jitter < corr_temp[i]) {
                radar_info.waveform_jitter = corr_temp[i];
                // ESP_LOGW(TAG, "i: %d, static_corr: %f", i, radar_info.waveform_jitter);
            }
        }

        // 注释：调试输出相关性信息（当前未启用）
        // ESP_LOGW(TAG, "[%s,%d]: wander: %f, jitter: %f", __func__, __LINE__, radar_info.waveform_wander, radar_info.waveform_jitter);
        
        // 如果有校准数据，计算与基准环境的相关性（长期变化检测）
        if (g_radar_calibrate) {
            radar_info.waveform_wander = 0.0;  // 初始化波形漂移为0

            // 遍历所有校准数据，计算与当前数据的相关性
            for (int i = 0;  i < g_radar_calibrate->data_num && i < CSI_CORR_NUM; ++i) {
                float_t corr_temp = fabs(corr(g_radar_calibrate->data[i], pca_subcarrier_buff_current, RADAR_SUBCARRIER_LEN));

                // 找到与校准数据的最大相关性
                if (radar_info.waveform_wander < corr_temp) {
                    radar_info.waveform_wander = corr_temp;
                }
                // ESP_LOGW(TAG, "num: %d, i: %d, corr: %f/%f", g_radar_calibrate->data_num, i, radar_info.waveform_wander, corr_temp);
            }

            // 如果正在进行校准，执行校准数据收集和处理
            if (g_radar_calibrate->calibrate_status == RADAR_CALIBRATE_PROGRESS) {
#define RADAR_OUTLIERS_THRESHOLD  0.005     // 雷达异常值检测阈值
                static float s_waveform_wander_last = 0;  // 上次的波形漂移值

                // 获取抖动缓冲区中的三个位置指针（环形缓冲区）
                float *jitter_buff_first  = g_radar_calibrate->waveform_jitter_buff + (g_radar_calibrate->buff_size - 2) % RADAR_BUFF_NUM;
                float *jitter_buff_second = g_radar_calibrate->waveform_jitter_buff + (g_radar_calibrate->buff_size - 1) % RADAR_BUFF_NUM;
                float *jitter_buff_third  = g_radar_calibrate->waveform_jitter_buff + g_radar_calibrate->buff_size % RADAR_BUFF_NUM;
                *jitter_buff_third  = radar_info.waveform_jitter;  // 存储当前抖动值

                // 增加缓冲区计数，检查是否有足够的数据进行异常值检测
                if (++g_radar_calibrate->buff_size < RADAR_BUFF_NUM) {
                    goto EXIT;  // 数据不足，跳过异常值检测
                }

                // 异常值检测：如果中间值明显低于相邻值，则认为是异常值
                if (*jitter_buff_first - *jitter_buff_second > RADAR_OUTLIERS_THRESHOLD
                        && *jitter_buff_third - *jitter_buff_second > RADAR_OUTLIERS_THRESHOLD) {
                    ESP_LOGW(TAG, "RADAR_CALIBRATE_PROGRESS, outlier: %f, %f, %f", *jitter_buff_first, *jitter_buff_second, *jitter_buff_third);
                    
                    goto EXIT;  // 检测到异常值，跳过本次处理
                }

                // 更新静态相关性（取最小值作为基线）
                if (g_radar_calibrate->static_corr > radar_info.waveform_jitter) {
                    g_radar_calibrate->static_corr = *jitter_buff_second;  // 使用中间值更新
                }

                // 注释：调试输出相关性信息（当前未启用）
                ESP_LOGW(TAG, "[%s,%d]: wander: %f, jitter: %f", __func__, __LINE__, radar_info.waveform_wander, radar_info.waveform_jitter);

                // 校准数据收集逻辑：根据相关性决定是否保存数据
                if (s_waveform_wander_last < CSI_CORR_THRESHOLD) {
                    // 相关性较低，认为是好的校准数据，保存上一个数据
                    float *pca_subcarrier_buff_last = pca_subcarrier_buff[(g_pca_subcarrier_buff_num - 2) % RADAR_MOVE_BUFFER_SIZE];
                    uint32_t index = g_radar_calibrate->data_num % CSI_CORR_NUM;  // 环形索引
                    g_radar_calibrate->data[index] = RADAR_MALLOC_RETRY(RADAR_SUBCARRIER_LEN * sizeof(float));

                    // 复制数据到校准数组
                    memcpy(g_radar_calibrate->data[index],
                           pca_subcarrier_buff_last, RADAR_SUBCARRIER_LEN * sizeof(float));
                    ESP_LOGI(TAG, "Add record num: %d, corr: %f", g_radar_calibrate->data_num, s_waveform_wander_last);

                    g_radar_calibrate->data_num++;          // 增加数据计数
                    g_radar_calibrate->none_corr = 1;       // 设置非相关性为1
                    radar_info.waveform_wander = 1;         // 设置输出为无运动
                } else {
                    // 相关性较高，用于统计平均非相关性
                    g_radar_calibrate->none_corr = s_waveform_wander_last;
                    if (s_waveform_wander_last < 0.99999) {  // 防止数值溢出
                        g_radar_calibrate->none_corr_sum += s_waveform_wander_last;  // 累加相关性
                        g_radar_calibrate->none_corr_count++;                       // 增加计数
                    }

                    // 注释：调试输出平均非相关性（当前低级别）
                    ESP_LOGI(TAG, "[%s, %d] none_corr_sum: %f, none_corr_count: %f, avg: %f", 
                        g_radar_calibrate->none_corr_sum, g_radar_calibrate->none_corr_count,
                        g_radar_calibrate->none_corr_sum / g_radar_calibrate->none_corr_count);
                }

                s_waveform_wander_last = radar_info.waveform_wander;  // 保存当前漂移值
            }
        }
        // 注释：调试输出最终相关性信息（当前未启用）
        // ESP_LOGW(TAG, "[%s,%d]: wander: %f, jitter: %f", __func__, __LINE__, radar_info.waveform_wander, radar_info.waveform_jitter);
EXIT:
        // 转换相关性为运动检测值（1-相关性 = 运动强度）
        radar_info.waveform_wander = 1 - radar_info.waveform_wander;  // 漂移检测值
        radar_info.waveform_jitter = 1 - radar_info.waveform_jitter;  // 抖动检测值

        // 输出雷达检测结果和系统状态信息
        ESP_LOGV(TAG, "time: %u/%u, free_heap: %d, num: %d, rssi: %d, gain: %u/%d/%0.3f, wander: %f, jitter: %f, window: %d, begin: %d, end: %d, freq: %dHz",
                 time_spent,                                        // 数据窗口时间跨度
                 esp_log_timestamp() - timestamp_start,            // 处理耗时
                 esp_get_free_heap_size(),                         // 剩余堆内存
                 g_radar_calibrate ? g_radar_calibrate->data_num : 0,  // 校准数据数量
                 g_wifi_pkt_rx_ctrl.rssi,                          // 接收信号强度
                 g_wifi_pkt_rx_ctrl.agc_gain,                      // AGC增益
                 g_wifi_pkt_rx_ctrl.fft_gain,                      // FFT增益
                 g_wifi_pkt_rx_ctrl.compensate_gain / 1000.0,      // 补偿增益
                 radar_info.waveform_wander,                       // 漂移检测值
                 radar_info.waveform_jitter,                       // 抖动检测值
                 buff_index.window,                                // 数据窗口大小
                 buff_index.begin,                                 // 窗口起始位置
                 buff_index.end,                                   // 窗口结束位置
                 buff_index.window * 1000 / time_spent);           // 采样频率
        
        // 如果配置了回调函数，调用用户回调函数返回结果
        if (g_wifi_radar_config->wifi_radar_cb) {
            // ESP_LOGI(TAG, "call wifi_radar_cb");
            g_wifi_radar_config->wifi_radar_cb(&radar_info, g_wifi_radar_config->wifi_radar_cb_ctx);
        }
    }  // 主循环结束

    // 任务退出清理工作
    ESP_LOGW(TAG, "csi_data_handle_task exit");  // 记录任务退出

    xEventGroupSetBits(g_csi_task_exit_group, CSI_HANDLE_EXIT_BIT);  // 设置退出标志位

    // 释放所有PCA缓冲区内存
    for (int i = 0; i < RADAR_MOVE_BUFFER_SIZE; ++i) {
        if (pca_subcarrier_buff[i]) {
            RADAR_FREE(pca_subcarrier_buff[i]);
        }
    }

    g_pca_subcarrier_buff_num = 0;  // 重置计数器

    vTaskDelete(NULL);  // 删除当前任务
}

/**
 * @brief CSI数据组合任务
 * 将从队列接收的过滤后CSI数据组合成数据包，主要功能：
 * 1. 从队列接收过滤后的CSI数据
 * 2. 执行异常值过滤（可选）
 * 3. 转换复数数据为幅度数据
 * 4. 管理时间窗口和缓冲区
 * 5. 将组合好的数据包发送给处理任务
 * @param arg 任务参数（未使用）
 */
static void csi_data_combine_task(void *arg)
{
    wifi_csi_filtered_info_t *filtered_info = NULL;  // 过滤后的CSI数据指针
    uint32_t last_time = 0;                          // 上次数据的时间戳

    // 主循环：持续从队列接收CSI数据
    while (xQueueReceive(g_csi_info_queue, &filtered_info, portMAX_DELAY) && g_wifi_radar_run_flag) {
        // 注释：性能监测代码（当前未启用）
        // unsigned int start_b = dsp_get_cpu_cycle_count();
        // uint32_t timestamp_start = filtered_info->rx_ctrl.timestamp / 1000;

#ifdef  FILTER_OUTLIER  // 如果启用异常值过滤
#define CSI_DATA_BUFF_SIZE 4    // 异常值检测缓冲区大小

        // 静态异常值检测缓冲区（4个样本的滑动窗口）
        static float_t csi_data_buff[CSI_DATA_BUFF_SIZE][RADAR_SUBCARRIER_LEN] = {0};
        static uint32_t csi_data_buff_num = 0;  // 缓冲区计数器

        // 获取缓冲区中四个位置的指针（环形缓冲区）
        float *csi_data_buff_first  = csi_data_buff[(csi_data_buff_num - 3) % CSI_DATA_BUFF_SIZE];  // 第一个样本
        float *csi_data_buff_second = csi_data_buff[(csi_data_buff_num - 2) % CSI_DATA_BUFF_SIZE];  // 第二个样本
        float *csi_data_buff_third  = csi_data_buff[(csi_data_buff_num - 1) % CSI_DATA_BUFF_SIZE];  // 第三个样本
        float *csi_data_buff_fourth = csi_data_buff[csi_data_buff_num % CSI_DATA_BUFF_SIZE];        // 当前样本

        // 转换复数CSI数据为幅度数据（欧几里得距离）
        for (int i = CSI_SUBCARRIER_OFFSET;  i < CSI_SUBCARRIER_OFFSET + RADAR_SUBCARRIER_LEN; i++) {
            // 计算复数幅度：sqrt(real^2 + imag^2)
            csi_data_buff_fourth[i - CSI_SUBCARRIER_OFFSET] = my_hypotf(
                filtered_info->valid_data[i * SUB_CARRIER_STEP_SIZE * 2],      // 实部
                filtered_info->valid_data[i * SUB_CARRIER_STEP_SIZE * 2 + 1]   // 虚部
            );
        }

        // 检查是否有足够的数据进行异常值检测
        if (++csi_data_buff_num < CSI_DATA_BUFF_SIZE) {
            RADAR_FREE(filtered_info);  // 释放当前数据
            ESP_LOGW(TAG, "检查是否有足够的数据进行异常值检测 csi_data_buff_num: %d", csi_data_buff_num);
            continue;                   // 数据不足，等待更多数据
        }

        uint32_t outliers_count = 0;  // 异常值计数器

        // 遍历所有子载波，检测异常值
        for (int i = 0;  i < RADAR_SUBCARRIER_LEN; ++i) {
            // 计算第二个样本与其他样本的差值
            float diff_0 = csi_data_buff_second[i] - csi_data_buff_first[i];   // 与第一个样本的差值
            float diff_1 = csi_data_buff_second[i] - csi_data_buff_third[i];   // 与第三个样本的差值
            float diff_2 = csi_data_buff_second[i] - csi_data_buff_fourth[i];  // 与第四个样本的差值

            // 异常值检浌逻辑：如果第二个样本与相邻样本差异较大，则认为是异常值
            if ((diff_0 > CSI_OUTLIERS_THRESHOLD && diff_1 > CSI_OUTLIERS_THRESHOLD)
                    || (diff_0 < -CSI_OUTLIERS_THRESHOLD && diff_1 < -CSI_OUTLIERS_THRESHOLD)
                    || (diff_0 > CSI_OUTLIERS_THRESHOLD && diff_2 > CSI_OUTLIERS_THRESHOLD)) {
                outliers_count++;  // 增加异常值计数
                // ESP_LOGW(TAG, "检测异常值 data: %f, diff: %f, %f", csi_data_buff_second[i], diff_0, diff_1);
            }
        }

        // 判断是否异常值过多（超过一半子载波）
        if (outliers_count >= RADAR_SUBCARRIER_LEN / 2) {
            // 异常值过多，丢弃第二个样本，将后面的数据前移
            memcpy(csi_data_buff_second, csi_data_buff_third, RADAR_SUBCARRIER_LEN * sizeof(float));
            memcpy(csi_data_buff_third, csi_data_buff_fourth, RADAR_SUBCARRIER_LEN * sizeof(float));
            csi_data_buff_num--;            // 回退计数器
            RADAR_FREE(filtered_info);      // 释放当前数据
            continue;                       // 重新处理
        }

#endif  // FILTER_OUTLIER 结束
        // ESP_LOGI(TAG, "初始化缓冲区索引结构");
        // 初始化缓冲区索引结构
        csi_data_buff_index_t buff_index = {
            .begin  = g_csi_data_buff->begin % g_wifi_radar_buff_size,   // 窗口起始位置（环形索引）
            .end    = g_csi_data_buff->end % g_wifi_radar_buff_size,     // 窗口结束位置（环形索引）
            .window = g_csi_data_buff->end - g_csi_data_buff->begin,     // 窗口大小
        };
        // ESP_LOGI(TAG, "begin: %d, end: %d, window: %d", buff_index.begin, buff_index.end, buff_index.window);
        // 记录当前数据的时间戳（转换为毫秒）
        g_csi_data_buff->timestamp[buff_index.end] = filtered_info->rx_ctrl.timestamp / 1000;

        // 计算时间相关参数
        int32_t spent_time = g_csi_data_buff->timestamp[buff_index.end] - g_csi_data_buff->timestamp[buff_index.begin];  // 窗口时间跨度
        int32_t time_tamp  = g_csi_data_buff->timestamp[buff_index.end] - last_time;                                    // 与上次数据的时间间隔
        float *csi_data    = g_csi_data_buff->lltf_amplitude[buff_index.end];                                          // 当前数据存储位置

#ifdef FILTER_OUTLIER
        // 如果启用了异常值过滤，使用经过过滤的第二个样本
        memcpy(csi_data, csi_data_buff_second, RADAR_SUBCARRIER_LEN * sizeof(float));
#else
        // 如果未启用异常值过滤，直接转换复数数据为幅度数据
        for (int i = CSI_SUBCARRIER_OFFSET; i < CSI_SUBCARRIER_OFFSET + RADAR_SUBCARRIER_LEN; i++) {
            // 计算复数幅度：sqrt(real^2 + imag^2)
            csi_data[i - CSI_SUBCARRIER_OFFSET] = my_hypotf(
                filtered_info->valid_data[i * SUB_CARRIER_STEP_SIZE * 2],      // 实部
                filtered_info->valid_data[i * SUB_CARRIER_STEP_SIZE * 2 + 1]   // 虚部
            );
        }

#endif  // FILTER_OUTLIER

#if 0  // 注释：时间间隔检查代码（当前未启用）
        // 调试输出缓冲区信息
        ESP_LOGW(TAG, "begin: %d, end: %d, window: %d", buff_index.begin, buff_index.end, buff_index.window);
        // 检查时间间隔是否过短
        if (time_tamp <= g_wifi_radar_config->csi_recv_interval / 4) {
            ESP_LOGW(TAG, "csi_recv_interval: %d, end: %d, last: %d", time_tamp, g_csi_data_buff->timestamp[buff_index.end], last_time);
            RADAR_FREE(filtered_info);
            continue;
        }
#endif  // 时间间隔检查代码结束
        // ESP_LOGI(TAG, "检查时间间隔是否异常（负值或过大）");
        // 检查时间间隔是否异常（负值或过大）
        if (time_tamp < 0 || time_tamp > g_wifi_radar_config->csi_handle_time / 2) {
            ESP_LOGW(TAG, "time_tamp: %d, end: %d, last: %d, num: %d",
                     time_tamp,                                    // 时间间隔
                     g_csi_data_buff->timestamp[buff_index.end],   // 当前时间戳
                     last_time,                                    // 上次时间戳
                     g_csi_data_buff->end - g_csi_data_buff->begin); // 缓冲区数据数量

            // 如果窗口大小足够，尝试发送现有数据
            if (buff_index.window > g_wifi_radar_handle_window / 3) {
                buff_index.window--;  // 减少窗口大小（排除当前异常数据）
                buff_index.end = (g_csi_data_buff->end - 1) % g_wifi_radar_buff_size;  // 调整结束位置
                ESP_LOGI(TAG, "尝试发送数据包索引到处理队列 buff_index.end: %d", buff_index.end);
                // 尝试发送数据包索引到处理队列
                if (!g_csi_data_queue || xQueueSend(g_csi_data_queue, &buff_index, portMAX_DELAY) == pdFALSE) {
                    ESP_LOGW(TAG, "The buffer is full");  // 队列已满
                }
            }else{
                ESP_LOGI(TAG, "窗口大小不足，不发送数据包索引到处理队列");
            }

            g_csi_data_buff->begin = g_csi_data_buff->end;  // 重置窗口起始位置
            goto FREE_MEM;  // 跳转到内存释放
        }

        // 检查是否需要发送数据包（时间超时或窗口已满）
        if (spent_time >= g_wifi_radar_config->csi_handle_time * 2 || buff_index.window >= g_wifi_radar_handle_window) {
            // 检查窗口大小是否足够进行处理
            if (buff_index.window < g_wifi_radar_handle_window / 3) {
                // g_csi_data_buff->begin = g_csi_data_buff->end;  // 窗口太小，重置窗口
                ESP_LOGW(TAG, "buff_index.window: %d, spent_time: %d, csi_handle_time: %d, g_wifi_radar_handle_window: %d", 
                         buff_index.window,                       // 当前窗口大小
                         spent_time,                              // 时间跨度
                         g_wifi_radar_config->csi_handle_time,    // 配置的处理时间
                         g_wifi_radar_handle_window);             // 配置的处理窗口
                
                goto FREE_MEM;  // 跳转到内存释放
            }
            // ESP_LOGI(TAG, "发送数据包索引到处理队列 buff_index.end: %d", buff_index.end);
            // 发送数据包索引到处理队列
            if (!g_csi_data_queue || xQueueSend(g_csi_data_queue, &buff_index, portMAX_DELAY) == pdFALSE) {
                ESP_LOGW(TAG, "The buffer is full");  // 队列已满警告
            }

            // 滑动窗口：将起始位置前移一半窗口大小（重叠处理）
            g_csi_data_buff->begin += (buff_index.window / 2);
        }

FREE_MEM:  // 内存释放标签
        RADAR_FREE(filtered_info);  // 释放当前处理的CSI数据
        last_time = g_csi_data_buff->timestamp[buff_index.end];  // 更新上次时间戳
        g_csi_data_buff->end++;  // 增加结束位置索引
    }  // 主循环结束

    // 任务退出清理工作
    ESP_LOGW(TAG, "csi_data_combine_task exit");  // 记录任务退出
    xEventGroupSetBits(g_csi_task_exit_group, CSI_COMBINED_EXIT_BIT);  // 设置退出标志位
    vTaskDelete(NULL);  // 删除当前任务
}

/**
 * @brief 推送CSI数据到处理队列
 * 将过滤后的CSI数据放入队列等待处理
 * @param info 过滤后的CSI数据指针
 * @return ESP_OK 成功
 */
esp_err_t csi_data_push(wifi_csi_filtered_info_t *info)
{
    // 尝试将数据放入队列，如果队列满则丢弃数据
    if (!g_csi_info_queue || xQueueSend(g_csi_info_queue, &info, portMAX_DELAY) == pdFALSE) {
        ESP_LOGW(TAG, "g_csi_info_queue, The buffer is full");  // 队列已满警告
        free(info);  // 释放数据内存
    }

    return ESP_OK;
}

/**
 * @brief WiFi CSI数据接收回调函数
 * ESP-IDF WiFi驱动层调用的回调函数，用于接收原始CSI数据
 * 对数据进行预处理和过滤，然后传递给处理任务
 * @param ctx 上下文指针（未使用）
 * @param info 原始CSI数据信息指针
 */
void wifi_csi_cb(void *ctx, wifi_csi_info_t *info)
{
    // 注释：调试输出Mac地址和过滤信息（当前未启用）
    // ESP_LOGW(TAG, "wifi_csi_cb, mac: " MACSTR ", filter_mac: " MACSTR", len: %d, filter_len: %d",
    //          MAC2STR(info->mac), MAC2STR(g_wifi_radar_config->filter_mac), info->len, g_wifi_radar_config->filter_len);
    
    // 注释：调试输出目标MAC地址过滤信息（当前未启用）
    // ESP_LOGW(TAG, "filter_dmac: " MACSTR ", dmac: " MACSTR ", filter_dmac_flag: %d",
    //         MAC2STR(g_wifi_radar_config->filter_dmac), MAC2STR((uint8_t *)info->dmac), g_csi_filter_dmac_flag);

    // 检查输入参数的有效性
    if (!info || !info->buf) {
        ESP_LOGD(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));  // 参数无效
        return;
    }

    // MAC地址过滤：检查源MAC地址是否匹配过滤条件
//     if ((ADDR_IS_FILTER(g_wifi_radar_config->filter_mac) && memcmp(info->mac, g_wifi_radar_config->filter_mac, 6))
// #if WIFI_CSI_SEND_NULL_DATA_ENABLE
//         || (ADDR_IS_EMPTY(g_wifi_radar_config->filter_mac) && info->payload_len != 14)  // NULL数据包检查
// #endif
//         ) {
//         ESP_LOGD(TAG, "mac: " MACSTR ", filter_mac: " MACSTR, MAC2STR(info->mac), MAC2STR(g_wifi_radar_config->filter_mac));
//         return;  // 源MAC不匹配，丢弃数据
//     }

//     // 目标MAC地址过滤（如果启用）
//     if (g_csi_filter_dmac_flag && memcmp(info->mac + 6, g_wifi_radar_config->filter_dmac, 6)) {
//         // ESP_LOGD(TAG, "dmac: " MACSTR ", filter_dmac: " MACSTR, MAC2STR(info->dmac), MAC2STR(g_wifi_radar_config->filter_dmac));
//         return;  // 目标MAC不匹配，丢弃数据
//     }

// #if 0  // 时间间隔过滤（防止数据过于频繁）
//     static uint32_t s_last_timestamp = 0;  // 上次接收的时间戳

//     // 检查与上次数据的时间间隔是否足够（防止数据过于频繁）
//     if (info->rx_ctrl.timestamp - s_last_timestamp <= g_wifi_radar_config->csi_recv_interval * 1000 / 5) {
//         return;  // 时间间隔过短，丢弃数据
//     }

//     s_last_timestamp = info->rx_ctrl.timestamp;  // 更新时间戳
// #endif  // 时间间隔过滤结束

    // 注释：调试输出CSI数据详细信息（当前未启用）
    // ESP_LOGW(TAG, "rssi: %d, rate: %d, sig_mode: %d, mcs: %d, cwb: %d, stbc: %d, len: %d, mac: " MACSTR,
    //          info->rx_ctrl.rssi, info->rx_ctrl.rate, info->rx_ctrl.sig_mode, info->rx_ctrl.mcs,
    //          info->rx_ctrl.cwb, info->rx_ctrl.stbc, info->len, MAC2STR(info->mac));

    // 调试输出：显示接收到的CSI数据基本信息
    ESP_LOGV(TAG, "wifi_csi_cb, mac: " MACSTR ", len: %d, filter_len: %d",
             MAC2STR(info->mac),                 // 源MAC地址
             info->len,                          // 数据长度
             g_wifi_radar_config->filter_len);   // 过滤器长度

    wifi_csi_filtered_info_t *filtered_info = NULL;  // 过滤后的CSI数据指针

    // 遍历子载波表，查找匹配当前信号参数的配置
    for (int i = 0; i < SUB_CARRIER_TABLE_SIZE; ++i) {
        // 检查信号参数是否匹配：辅助信道、信号模式、带宽、STBC
        // if (info->rx_ctrl.secondary_channel != sub_carrier_table[i].second      // 辅助信道不匹配
        //         || info->rx_ctrl.sig_mode != sub_carrier_table[i].signal_mode   // 信号模式不匹配
        //         || info->rx_ctrl.cwb != sub_carrier_table[i].channel_bandwidth  // 信道带宽不匹配
        //         || info->rx_ctrl.stbc != sub_carrier_table[i].stbc) {           // STBC不匹配
        //     continue;  // 参数不匹配，继续查找
        // }

        // 找到匹配的子载波表条目
        const csi_sub_carrier_table_t *sub_carrier_index =  sub_carrier_table + i;
        // wifi_csi_config_t *csi_config = &g_wifi_radar_config->csi_config;  // CSI配置指针
        
        // 计算有效数据长度（根据启用的子载波类型）
        uint32_t valid_len = 234 ;//((csi_config->lltf_en) ? sub_carrier_index->llft_bytes : 0)        // LLTF字节数
                            //  + ((csi_config->htltf_en) ? sub_carrier_index->ht_lft_bytes : 0)   // HT-LTF字节数
                            //  + ((csi_config->stbc_htltf2_en) ? sub_carrier_index->stbc_ht_lft_bytes : 0);  // STBC HT-LTF字节数

        // 分配过滤后数据结构的内存（结构体 + 数据空间）
        filtered_info = RADAR_MALLOC_RETRY(sizeof(wifi_csi_filtered_info_t) + valid_len);

        // 初始化过滤后数据结构
        filtered_info->rx_ctrl = info->rx_ctrl;                 // 复制接收控制信息
        filtered_info->raw_data = info->buf;                    // 原始数据指针
        filtered_info->raw_len  = info->len;                    // 原始数据长度
        filtered_info->valid_len = 0;                           // 初始化有效数据长度
        filtered_info->valid_llft_len = 0;                      // 初始化LLTF数据长度
        filtered_info->valid_ht_lft_len = 0;                    // 初始化HT-LTF数据长度
        filtered_info->valid_stbc_ht_lft_len = 0;               // 初始化STBC HT-LTF数据长度
        memcpy(filtered_info->mac, info->mac, 6);               // 复制MAC地址

        // // 提取LLTF数据（如果启用）
        // if (csi_config->lltf_en) {
        //     // 遍历LLTF子载波范围，提取数据
        //     for (uint32_t i = 0; filtered_info->valid_llft_len < sub_carrier_index->llft_bytes; i++) {
        //         uint32_t size = sub_carrier_index->llft[i].stop - sub_carrier_index->llft[i].start;  // 计算范围大小
        //         // 复制LLTF数据到过滤后的数据缓冲区
        //         memcpy(filtered_info->valid_data + filtered_info->valid_len, 
        //                info->buf + sub_carrier_index->llft[i].start, size);
        //         filtered_info->valid_llft_len += size;  // 增加LLTF数据长度
        //         filtered_info->valid_len      += size;  // 增加总数据长度
        //     }
        // }

        // 提取HT-LTF数据（如果启用）
        if (1) {
            // 遍历HT-LTF子载波范围，提取数据
            for (uint32_t i = 0; filtered_info->valid_ht_lft_len < 234; i++) {
                uint32_t size = 234;  // 计算范围大小
                // 复制HT-LTF数据到过滤后的数据缓冲区
                memcpy(filtered_info->valid_data + filtered_info->valid_len, 
                       info->buf, size);
                filtered_info->valid_ht_lft_len += size;  // 增加HT-LTF数据长度
                filtered_info->valid_len        += size;  // 增加总数据长度
            }
        }

        // // 提取STBC HT-LTF数据（如果启用）
        // if (csi_config->stbc_htltf2_en) {
        //     // 遍历STBC HT-LTF子载波范围，提取数据
        //     for (uint32_t i = 0; filtered_info->valid_stbc_ht_lft_len < sub_carrier_index->stbc_ht_lft_bytes; i++) {
        //         uint32_t size = sub_carrier_index->stbc_ht_lft[i].stop - sub_carrier_index->stbc_ht_lft[i].start;  // 计算范围大小
        //         // 复制STBC HT-LTF数据到过滤后的数据缓冲区
        //         memcpy(filtered_info->valid_data + filtered_info->valid_len, 
        //                info->buf + sub_carrier_index->stbc_ht_lft[i].start, size);
        //         filtered_info->valid_stbc_ht_lft_len += size;  // 增加STBC HT-LTF数据长度
        //         filtered_info->valid_len             += size;  // 增加总数据长度
        //     }
        // }

        // 调试输出：显示提取的数据长度信息
        ESP_LOGV(TAG, "len: %d, valid_len: %d, valid_llft_len: %d, valid_ht_lft_len: %d, valid_stbc_ht_lft_len: %d",
                 valid_len,                             // 预期数据长度
                 filtered_info->valid_len,              // 实际提取的总数据长度
                 filtered_info->valid_llft_len,         // LLTF数据长度
                 filtered_info->valid_ht_lft_len,       // HT-LTF数据长度
                 filtered_info->valid_stbc_ht_lft_len); // STBC HT-LTF数据长度
        break;  // 找到匹配项，退出循环
    }  // 子载波表遍历结束

    // 检查是否成功分配和提取数据
    if (!filtered_info) {
        // 未找到匹配的子载波配置，输出详细信息并丢弃数据
        ESP_LOGW(TAG, "value fail, len: %d, secondary_channel: %d",
                 info->len,                        // 数据长度
                 info->rx_ctrl.second);  // 辅助信道
        return;  // 退出函数
    }

    // 注释：调试输出过滤后数据长度（当前未启用）
    // ESP_LOGW(TAG, "[%d] filtered_info->valid_len: %d", __LINE__, filtered_info->valid_len);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)  // ESP-IDF 5.0+支持目标MAC地址
    memcpy(filtered_info->dmac, info->dmac, 6);  // 复制目标MAC地址
#endif

    float compensate_gain = 0;  // 增益补偿值
#if WIFI_CSI_PHY_GAIN_ENABLE  // 如果启用了PHY增益功能
    // 获取增益控制信息
    wifi_pkt_rx_ctrl_gain_t *rx_ctrl_gain = (wifi_pkt_rx_ctrl_gain_t *)(&info->rx_ctrl);
    filtered_info->agc_gain = rx_ctrl_gain->agc_gain;  // 保存AGC增益
    filtered_info->fft_gain = rx_ctrl_gain->fft_gain;  // 保存FFT增益

    // 更新雷达系统的增益信息
    extern esp_err_t esp_radar_record_rx_gain(uint8_t agc_gain, int8_t fft_gain);
    static uint8_t gain_count =0;
    if (gain_count < 50) {
        esp_radar_record_rx_gain(rx_ctrl_gain->agc_gain, rx_ctrl_gain->fft_gain);
        gain_count++;
    }

    // 如果启用了增益补偿，对CSI数据进行补偿处理
    if (g_wifi_radar_config->wifi_radar_compensate_en) {
        extern esp_err_t esp_radar_compensate_rx_gain(int8_t *data, uint16_t size, float *compensate_gain, uint8_t agc_gain, int8_t fft_gain);
        esp_radar_compensate_rx_gain(filtered_info->valid_data, filtered_info->valid_len, 
                                   &compensate_gain, rx_ctrl_gain->agc_gain, rx_ctrl_gain->fft_gain);
    }
#endif  // WIFI_CSI_PHY_GAIN_ENABLE

    // 保存接收控制信息到全局变量（用于日志输出）
    memcpy(&g_wifi_pkt_rx_ctrl, &info->rx_ctrl, sizeof(wifi_pkt_rx_ctrl_gain_t));
    g_wifi_pkt_rx_ctrl.compensate_gain = (int32_t)(compensate_gain * 1000);  // 保存补偿增益（放大1000倍）

    // 如果配置了过滤后数据回调函数，先调用用户回调
    if (g_wifi_radar_config->wifi_csi_filtered_cb) {
        g_wifi_radar_config->wifi_csi_filtered_cb(filtered_info, g_wifi_radar_config->wifi_csi_cb_ctx);
    }

    // 尝试将过滤后的数据发送到处理队列
    if (!g_wifi_radar_run_flag || !g_csi_info_queue || xQueueSend(g_csi_info_queue, &filtered_info, 0) == pdFALSE) {
        ESP_LOGW(TAG, "The buffer is full");  // 系统停止或队列已满
        RADAR_FREE(filtered_info);            // 释放内存
    }
    // 注意：如果数据成功发送到队列，内存将由接收方释放
}

/**
 * @brief 设置雷达配置参数
 * 更新雷达系统的配置参数，如果系统正在运行会重新初始化缓冲区
 * @param config 新的雷达配置指针
 * @return ESP_OK 成功
 * @return ESP_ERR_INVALID_ARG 参数无效
 */
esp_err_t esp_radar_set_config(const wifi_radar_config_t *config)
{
    // // 检查参数有效性
    // if (!config || !g_wifi_radar_config) {
    //     return ESP_ERR_INVALID_ARG;
    // }

    // // 检查CSI配置是否发生变化，如果有变化则更新
    // // if (memcmp(&g_wifi_radar_config->csi_config, &config->csi_config, sizeof(wifi_csi_config_t))) {
    // //     ESP_ERROR_CHECK(esp_wifi_set_csi_config(&config->csi_config));
    // // }

    // // 检查目标MAC地址过滤功能支持
    // if (ADDR_IS_FILTER(config->filter_dmac)) {
    //     // 通过分配临时结构来检查ESP-IDF版本支持
    //     wifi_csi_info_t *info = RADAR_MALLOC_RETRY(sizeof(wifi_csi_info_t));
    //     if ((uint32_t)(&info->first_word_invalid) - (uint32_t)info->mac == 12) {
    //         g_csi_filter_dmac_flag = true;  // ESP-IDF 5.0+支持目标MAC过滤
    //     } else {
    //         ESP_LOGW(TAG, "filter_dmac is not supported in esp-idf version < 5.0.0");
    //     }
    //     RADAR_FREE(info);
    // } else {
    //     g_csi_filter_dmac_flag = false;  // 禁用目标MAC过滤
    // }

    // // 检查时间相关参数是否发生变化，如有变化需重新初始化缓冲区
    // if (g_wifi_radar_config->csi_handle_time!= config->csi_handle_time
    //         || g_wifi_radar_config->csi_recv_interval != config->csi_recv_interval) {
    //     if (g_wifi_radar_run_flag) {
    //         // 暂时停止CSI接收
    //         ESP_ERROR_CHECK(esp_wifi_set_csi(false));
    //         vTaskDelay(100 / portTICK_PERIOD_MS);  // 等待系统稳定

    //         // 释放旧的缓冲区内存
    //         RADAR_FREE(g_csi_data_buff->lltf_amplitude);
    //         RADAR_FREE(g_csi_data_buff->timestamp);

    //         // 根据新参数重新计算缓冲区大小
    //         g_wifi_radar_handle_window = (config->csi_handle_time / config->csi_recv_interval) * 2;
    //         g_wifi_radar_buff_size     = (config->csi_handle_time / config->csi_recv_interval) * 2 + 10;
    
    //         // 重新分配缓冲区内存
    //         g_csi_data_buff->lltf_amplitude = RADAR_MALLOC_RETRY(g_wifi_radar_buff_size * RADAR_SUBCARRIER_LEN * sizeof(float));
    //         g_csi_data_buff->timestamp      = RADAR_MALLOC_RETRY(g_wifi_radar_buff_size * sizeof(uint32_t));

    //         // 输出新的缓冲区配置信息
    //         ESP_LOGI(TAG, "[%s, %d] csi_recv_interval: %d, csi_handle_time: %d, csi_handle_window: %d, csi_handle_buffer: %d",
    //                     __func__, __LINE__,
    //                 config->csi_recv_interval,      // CSI接收间隔
    //                 config->csi_handle_time,        // CSI处理时间
    //                 g_wifi_radar_handle_window,     // 处理窗口大小
    //                 g_wifi_radar_buff_size);        // 缓冲区大小

    //         // 重新启动CSI接收
    //         ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    //     }
    // }

    // 复制新配置到全局配置结构
    // memcpy(g_wifi_radar_config, config, sizeof(wifi_radar_config_t));
    g_wifi_radar_config->wifi_radar_cb = config->wifi_radar_cb;
    ESP_LOGI(TAG, "g_wifi_radar_config wifi_radar_cb: %p", g_wifi_radar_config->wifi_radar_cb);
//     // 输出增益补偿状态
//     ESP_LOGW(TAG, "g_wifi_radar_config compensate_enable: %d", 
//                 g_wifi_radar_config->wifi_radar_compensate_en);

// #if WIFI_CSI_PHY_GAIN_ENABLE  // 如果启用了PHY增益功能
//     // 重置接收增益基线
//     extern void esp_radar_reset_rx_gain_baseline();
//     esp_radar_reset_rx_gain_baseline();
// #endif

    return ESP_OK;
}

/**
 * @brief 获取当前雷达配置参数
 * 返回当前雷达系统的配置参数
 * @param config 用于存放配置的结构体指针
 * @return ESP_OK 成功
 * @return ESP_ERR_INVALID_ARG 参数无效
 */
esp_err_t esp_radar_get_config(wifi_radar_config_t *config)
{
    // 检查参数有效性
    if (!config || !g_wifi_radar_config) {
        return ESP_ERR_INVALID_ARG;
    }

    // 复制当前配置到输出参数
    memcpy(config, g_wifi_radar_config, sizeof(wifi_radar_config_t));

    return ESP_OK;
}

/**
 * @brief 启动雷达系统
 * 初始化并启动雷达检测系统，包括创建任务和分配缓冲区
 * @return ESP_OK 成功
 */
esp_err_t esp_radar_start()
{
    esp_radar_init();
    // 检查系统是否已经在运行
    if (g_wifi_radar_run_flag) {
        return ESP_OK;  // 已经运行，直接返回
    }

    g_wifi_radar_run_flag = true;  // 设置运行标志

    // 分配并初始化CSI数据缓冲区结构
    g_csi_data_buff = RADAR_MALLOC_RETRY(sizeof(csi_data_buff_t));
    memset(g_csi_data_buff, 0, sizeof(csi_data_buff_t));

    // 创建任务间通信队列
    g_csi_info_queue = xQueueCreate(5, sizeof(void *));                         // CSI信息队列（5个元素）
    g_csi_data_queue = xQueueCreate(1, sizeof(csi_data_buff_index_t));          // CSI数据队列（1个元素）

    // 根据配置计算缓冲区参数
    g_wifi_radar_handle_window = (g_wifi_radar_config->csi_handle_time / g_wifi_radar_config->csi_recv_interval) * 2;  // 处理窗口大小
    g_wifi_radar_buff_size = (g_wifi_radar_config->csi_handle_time / g_wifi_radar_config->csi_recv_interval) * 2 +20;  // 缓冲区大小

    // 输出缓冲区配置信息
    ESP_LOGI(TAG, "[%s, %d] csi_recv_interval: %d, csi_handle_time: %d, csi_handle_window: %d, csi_handle_buffer: %d",
             __func__, __LINE__,
             g_wifi_radar_config->csi_recv_interval,  // CSI接收间隔
             g_wifi_radar_config->csi_handle_time,    // CSI处理时间
             g_wifi_radar_handle_window,              // 处理窗口大小
             g_wifi_radar_buff_size);                 // 缓冲区大小

    // 分配实际的数据存储空间
    g_csi_data_buff->lltf_amplitude = RADAR_MALLOC_RETRY(g_wifi_radar_buff_size * RADAR_SUBCARRIER_LEN * sizeof(float));  // 幅度数据空间
    g_csi_data_buff->timestamp      = RADAR_MALLOC_RETRY(g_wifi_radar_buff_size * sizeof(uint32_t));                     // 时间戳数据空间

    // 创建两个核心处理任务
    xTaskCreate(csi_data_handle_task, "csi_handle", 3 * 1024, NULL, g_wifi_radar_config->csi_handle_priority, NULL);    // CSI数据处理任务
    xTaskCreate(csi_data_combine_task, "csi_combine", 3 * 1024, NULL, g_wifi_radar_config->csi_combine_priority, NULL); // CSI数据组合任务

    return ESP_OK;
}

/**
 * @brief 停止雷达系统
 * 停止雷达检测系统，清理资源并等待任务退出
 * @return ESP_OK 成功
 */
esp_err_t esp_radar_stop()
{
    // 创建任务退出事件组
    g_csi_task_exit_group  = xEventGroupCreate();
    g_wifi_radar_run_flag = false;  // 设置停止标志
    
    // 准备退出信号
    csi_data_buff_index_t buff_index = {0};
    wifi_csi_info_t *info = NULL;

    // 发送退出信号给两个任务
    xQueueSend(g_csi_info_queue, &info, 0);        // 通知组合任务退出
    xQueueSend(g_csi_data_queue, &buff_index, 0);  // 通知处理任务退出

    // 等待两个任务都退出
    xEventGroupWaitBits(g_csi_task_exit_group, CSI_HANDLE_EXIT_BIT | CSI_COMBINED_EXIT_BIT,
                        pdTRUE, pdTRUE, portMAX_DELAY);

    // 清理CSI信息队列中的剩余数据
    if (g_csi_info_queue) {
        while (xQueueReceive(g_csi_info_queue, &info, 0) && info) {
            RADAR_FREE(info->buf);  // 释放数据缓冲区
            RADAR_FREE(info);       // 释放信息结构
        }
    }

    // 清理CSI数据队列和缓冲区
    if (g_csi_data_queue) {
        // 清空数据队列
        while (xQueueReceive(g_csi_data_queue, &buff_index, 0)) ;

        // 释放主数据缓冲区
        if (g_csi_data_buff) {
            RADAR_FREE(g_csi_data_buff->lltf_amplitude);  // 释放幅度数据空间
            RADAR_FREE(g_csi_data_buff->timestamp);       // 释放时间戳数据空间
            RADAR_FREE(g_csi_data_buff);                  // 释放缓冲区结构
            g_csi_data_buff = NULL;
        }
    }

    // 清理事件组
    vEventGroupDelete(g_csi_task_exit_group);
    g_csi_task_exit_group = NULL;

    return ESP_OK;
}

/**
 * @brief 初始化雷达系统
 * 初始化ESP雷达系统，包括分配配置空间、设置默认参数和注册CSI回调
 * @return ESP_OK 成功
 */
esp_err_t esp_radar_init()
{
    // 检查是否已经初始化
    if (g_wifi_radar_init_flag) {
        return ESP_OK;  // 已初始化，直接返回
    }

    // 输出版本信息
    ESP_LOGI(TAG, "CSI_RADAR Version: 0.2.1, Compile time: %s - %s", __DATE__, __TIME__);

    // 分配配置结构内存
    g_wifi_radar_config    = RADAR_MALLOC_RETRY(sizeof(wifi_radar_config_t));
    g_wifi_radar_init_flag = true;  // 设置初始化标志

    // 初始化为默认配置
    wifi_radar_config_t radar_config_default = WIFI_RADAR_CONFIG_DEFAULT();
    memcpy(g_wifi_radar_config, &radar_config_default, sizeof(wifi_radar_config_t));

    // 配置ESP-IDF WiFi CSI功能
    // ESP_ERROR_CHECK(esp_wifi_set_csi(true));                                        // 启用CSI功能
    // ESP_ERROR_CHECK(esp_wifi_set_csi_config(&g_wifi_radar_config->csi_config));    // 设置CSI配置
    // ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_cb, NULL));                    // 注册CSI接收回调函数

    return ESP_OK;
}

/**
 * @brief 去初始化雷达系统
 * 清理雷达系统资源，关闭CSI功能
 * @return ESP_OK 成功
 */
esp_err_t esp_radar_deinit()
{
    g_wifi_radar_init_flag = false;  // 清除初始化标志

    ESP_ERROR_CHECK(esp_wifi_set_csi(false));  // 关闭CSI功能
    RADAR_FREE(g_wifi_radar_config);           // 释放配置结构内存

    return ESP_OK;
}
// 文件结束