/**
 * @file audio_config.h
 * @brief ESP-NOW 无线对讲机 — 收发共用参数与数据包格式
 *
 * 修改 MAC 地址、引脚、采样率时只需改本文件。
 */
#pragma once

#include <stdint.h>

// ===================== 网络 / 射频 =====================
/** WiFi 固定信道，收发必须一致 */
#define WIFI_CHANNEL 1

/** 是否启用 WiFi Long Range（收发必须同为 true 或同为 false） */
#define WIFI_USE_LR_PROTOCOL 1

// ===================== 音频 =====================
/** 采样率 (Hz)，与 I2S 一致 */
#define SAMPLE_RATE 32000

/** 每包 PCM 单声道采样点数（8ms @ 32kHz） */
#define SAMPLES_PER_PACKET 256

/** 每包 ADPCM 净荷字节数（2 个采样压缩为 1 字节） */
#define ADPCM_PAYLOAD_BYTES (SAMPLES_PER_PACKET / 2)

// ===================== 缓冲策略 =====================
/**
 * 接收端开始播放前需要攒够的包数（抖动缓冲 / 预缓冲）
 * 256 采样 @ 32kHz ≈ 8ms/包，4 包 ≈ 32ms 预缓冲
 */
#define RX_PREFILL_PACKETS 4

/** 接收队列容量（包数），满时丢弃最旧一包再入队 */
#define RX_QUEUE_DEPTH 16

/** 发送端待发送队列深度，满时丢弃最旧编码包 */
#define TX_QUEUE_DEPTH 6

/**
 * 接收端播放任务节拍 (ms)，应等于 SAMPLES_PER_PACKET * 1000 / SAMPLE_RATE
 */
#define RX_PLAYOUT_PERIOD_MS ((SAMPLES_PER_PACKET * 1000) / SAMPLE_RATE)

/** 连续多少次取不到包触发重新预缓冲（欠载防抖） */
#define RX_UNDERRUN_THRESHOLD 8

/** 队列深度超过此值时触发积压修剪 */
#define RX_QUEUE_TRIM_ABOVE 12

/** 积压修剪目标深度 */
#define RX_QUEUE_TARGET 8

/** 两次修剪之间的最小间隔 (ms) */
#define RX_TRIM_MIN_INTERVAL_MS 100

/** 单次最多丢弃的包数 */
#define RX_MAX_TRIM_DROP_PER_TICK 2

// ===================== 接收端低通滤波 =====================
/** 一阶低通滤波器移位位数（0 表示禁用） */
#define RX_LPF_SHIFT 2

// ===================== 统计打印间隔 =====================
/** 统计信息打印间隔 (ms)，0 表示禁用 */
#define STATS_PRINT_INTERVAL_MS 5000

// ===================== I2S DAC 配置 =====================
/** I2S DAC DMA 缓冲区数量 */
#define I2S_DAC_DMA_BUF_COUNT 4

/** I2S DAC DMA 缓冲区长度 */
#define I2S_DAC_DMA_BUF_LEN 256

// ===================== 数据包 =====================
/**
 * 空中传输的一帧结构（packed）
 * - seq: 递增序号，用于检测丢包并在接收端重置 ADPCM 状态
 * - adpcm: 128 字节 IMA ADPCM 数据
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t seq;
    uint8_t adpcm[ADPCM_PAYLOAD_BYTES];
} audio_packet_t;
#pragma pack(pop)

#define AUDIO_PACKET_SIZE (sizeof(audio_packet_t))

// ===================== 引脚配置 =====================
// --- I2S 麦克风 ---
#define I2S_MIC_NUM I2S_NUM_0
#define I2S_MIC_BCK 20
#define I2S_MIC_WS 3
#define I2S_MIC_SD 6

// --- I2S DAC ---
#define I2S_DAC_NUM I2S_NUM_1
#define I2S_DAC_BCK 7
#define I2S_DAC_WS 10
#define I2S_DAC_DIN 16
#define I2S_DAC_MCK 8

// --- 麦克风 I2S 位深配置 ---
/** 是否使用 32bit I2S（ICS-43434 / INMP441 等需要） */
#define MIC_I2S_USE_32BIT 1

/** 32bit 数据右移位数（按麦克风数据手册微调，常见 14~16） */
#define MIC_I2S_SHIFT_BITS 16

// --- 发送前预增益（浮点，1.0 = 不变） ---
#define TX_PRE_GAIN 3.0f

// --- 接收后增益（定点：实际倍数 = RX_GAIN_MUL / RX_GAIN_DIV） ---
#define RX_GAIN_MUL 3
#define RX_GAIN_DIV 2

// ===================== GPS 配置 =====================
/** GPS模块TX连接的ESP32引脚 (GPS模块RX) */
#define GPS_TX_PIN 18

/** GPS模块RX连接的ESP32引脚 (GPS模块TX) */
#define GPS_RX_PIN 17

/** GPS位置广播间隔 (毫秒) */
#define GPS_BROADCAST_INTERVAL_MS 3000

/** GPS位置显示队列深度 (最多保存2条) */
#define GPS_DISPLAY_QUEUE_SIZE 2

// ===================== 电池电压检测 =====================
/** 电池电压ADC采样引脚 */
#define BATTERY_ADC_PIN 4

/** ADC采样次数（多次采样取平均，提高精度） */
#define BATTERY_ADC_SAMPLES 16

/** 电池分压比 = (R1 + R2) / R2，例如 100k + 100k = 2.0 */
#define BATTERY_DIVIDER_RATIO 2.0f

/** ADC参考电压 (ESP32-S3 默认 3.3V) */
#define ADC_REF_VOLTAGE 3.3f

/** ADC分辨率 (12bit = 4095) */
#define ADC_RESOLUTION 4095

/** 锂电池满充电压 */
#define BATTERY_VOLTAGE_MAX 4.2f

/** 锂电池截止电压 */
#define BATTERY_VOLTAGE_MIN 3.0f

/** 电池电量显示刷新间隔 (毫秒) */
#define BATTERY_UPDATE_INTERVAL_MS 5000
