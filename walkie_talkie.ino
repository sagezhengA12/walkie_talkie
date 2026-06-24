/**
 * =============================================================================
 *  ESP-NOW 无线对讲机 (walkie_talkie.ino)
 * =============================================================================
 *
 * 功能概述：
 *   通过 GPIO5 按键控制发射/接收状态：
 *   - 按键按下 (GPIO5 低电平) → 发射模式：麦克风采集 → ADPCM 编码 → ESP-NOW 发送
 *   - 按键松开 (GPIO5 高电平) → 接收模式：ESP-NOW 接收 → ADPCM 解码 → DAC 播放
 *
 * 功放静音控制：
 *   - GPIO19 高电平 → 功放静音（发射时或无信号时）
 *   - GPIO19 低电平 → 功放开启（接收且有信号时）
 *   目的：
 *     1. 发射时避免扬声器播放本地声音，防止啸叫和回声
 *     2. 无信号时关断功放，节省功耗并避免播放噪声
 *
 * 无信号自动静音逻辑：
 *   - 切换到接收模式后，功放初始为静音状态
 *   - 连续收到 2 个有效数据包后，才开启功放
 *   - 超过 500ms 未收到有效信号，自动关断功放
 *
 * LED 指示灯（GPIO38）：
 *   - 发射时 GPIO38 = 低电平 → 红灯亮
 *   - 接收到有效信号时 GPIO38 = 高电平 → 绿灯亮
 *   - 空闲时 GPIO38 = 高阻态 → LED 全灭
 *

 *
 * 工作流程：
 *   1. 初始化时同时配置 I2S 麦克风输入和 DAC 输出
 *   2. 创建三个 FreeRTOS 任务：
 *      - micTask: 采集麦克风音频（仅在发射模式运行）
 *      - sendTask: 发送 ADPCM 数据包（仅在发射模式运行）
 *      - playoutTask: 播放接收到的音频（仅在接收模式运行）
 *   3. 按键检测任务监控 GPIO5 状态，切换工作模式并控制功放静音
 *
 * 配置：见 audio_config.h
 * =============================================================================
 */

#include <driver/i2s.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <string.h>

#include <Wire.h>
#include <U8g2lib.h>

#include "audio_config.h"
#include "adpcm.h"
#include "gps.h"

// =============================================================================
// 用户配置
// =============================================================================
// 对讲机模式：设为 true 表示本机是主机（先按下按键发射），false 表示从机
#define IS_MASTER_DEVICE true

// 对方设备的 MAC 地址
// 使用广播地址 0xFF:0xFF:0xFF:0xFF:0xFF:0xFF 可以让所有设备收到，无需硬编码对方MAC
// 如需点对点通信，可改为对方实际MAC地址
uint8_t peerMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 按键配置
#define PTT_BUTTON_PIN 5  // GPIO5 作为按键输入（按下为低电平）
#define BUTTON_DEBOUNCE_MS 50  // 按键消抖时间

// 功放静音控制
#define AMP_MUTE_PIN 19  // GPIO19 控制功放静音（高电平=静音，低电平=开启）

// 无信号自动静音配置
#define SIGNAL_LOST_TIMEOUT_MS 500  // 无信号超时时间（毫秒），超过此时间未收到包则关断功放
#define AMP_TURN_ON_DELAY_PACKETS 2  // 重新开启功放前需要连续收到的包数（避免频繁开关）

// LED 指示灯控制
#define LED_PIN 38  // GPIO38 双色LED指示
// 发射时 GPIO38=LOW（红灯亮），接收有效信号时 GPIO38=HIGH（绿灯亮），空闲时高阻态（全灭）

// OLED 显示屏配置（SSD1306 128x64 I2C）
#define OLED_SDA_PIN 1
#define OLED_SCL_PIN 2
#define OLED_SCREEN_WIDTH 128
#define OLED_SCREEN_HEIGHT 64
#define OLED_RESET -1  // 共用 ESP32 复位脚则为 -1
#define SCREEN_ADDRESS 0x3C  // SSD1306 I2C 地址

// =============================================================================
// 工作模式枚举
// =============================================================================
typedef enum {
    MODE_RECEIVE = 0,  // 接收模式（默认）
    MODE_TRANSMIT,     // 发射模式
} WorkMode;

static volatile WorkMode currentMode = MODE_RECEIVE;
static volatile WorkMode lastMode = MODE_RECEIVE;

// =============================================================================
// 全局对象 - 发射相关
// =============================================================================
static QueueHandle_t sendQueue = nullptr;
static SemaphoreHandle_t txDoneSem = nullptr;
static adpcm_state_t encoderState;
static volatile uint8_t txSeq = 0;

// 发射任务句柄（用于挂起/恢复）
static TaskHandle_t micTaskHandle = nullptr;
static TaskHandle_t sendTaskHandle = nullptr;

// =============================================================================
// 全局对象 - 接收相关
// =============================================================================
static QueueHandle_t audioQueue = nullptr;
static adpcm_state_t decoderState;

// 接收任务句柄
static TaskHandle_t playoutTaskHandle = nullptr;

// 播放状态机
typedef enum {
    PLAY_STATE_PREFILL = 0,
    PLAY_STATE_RUNNING,
} play_state_t;

static play_state_t playState = PLAY_STATE_PREFILL;
static uint8_t expectedSeq = 0;
static bool seqInitialized = false;
static uint8_t lastPlayedSeq = 0;
static bool hasPlayedSeq = false;
static uint8_t emptyStreak = 0;

// 无信号自动静音相关变量
static bool ampCurrentlyMuted = false;      // 当前功放静音状态
static uint32_t lastValidPacketTime = 0;    // 上次收到有效包的时间
static uint8_t consecutiveValidPackets = 0; // 连续收到的有效包计数

// LED 指示状态
typedef enum {
    LED_OFF = 0,       // 高阻态，LED 全灭
    LED_TX,            // 高电平，发射指示灯亮
    LED_RX,            // 低电平，接收指示灯亮
} LedState;
static volatile LedState ledState = LED_OFF;

// =============================================================================
// GPS 相关变量
// =============================================================================
// GPS位置显示队列 (保存接收到的对方位置，最多2条)
static gps_data_t remoteGpsDisplay[GPS_DISPLAY_QUEUE_SIZE];
static uint8_t remoteGpsIndex = 0;
static uint8_t remoteGpsCount = 0;
static volatile uint32_t g_gpsTxCount = 0;  // GPS发送包计数
static volatile uint32_t g_gpsRxCount = 0;  // GPS接收包计数

// GPS广播相关
static gps_packet_t gpsBroadcastPkt;
static bool hasGpsDataToSend = false;
static volatile uint32_t g_gpsLastSendTime = 0;  // GPS上次发送时间戳
static SemaphoreHandle_t gpsTxDoneSem = nullptr;    // GPS发送完成信号量（独立于音频txDoneSem）

// GPS任务句柄
static TaskHandle_t gpsTaskHandle = nullptr;

// =============================================================================
// 电池电压检测相关变量
// =============================================================================
static float batteryVoltage = 0.0f;       // 当前电池电压
static uint8_t batteryPercentage = 0;     // 电池电量百分比
static uint32_t lastBatteryUpdate = 0;    // 上次更新电池电量时间

// OLED 显示相关（U8g2 支持中文）
// 使用 u8g2_font_unifont_t_chinese2 字体支持中文
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RESET, OLED_SCL_PIN, OLED_SDA_PIN);
static SemaphoreHandle_t i2cMutex = nullptr;  // I2C/OLED 互斥锁，防止多任务并发访问

typedef enum {
    DISPLAY_OFF = 0,      // 关闭显示（上电初始状态）
    DISPLAY_TX,           // 发射中
    DISPLAY_STANDBY,      // 待机中
    DISPLAY_RX,           // 接收中（有有效信号）
} DisplayState;
static volatile DisplayState displayState = DISPLAY_OFF;
static bool displayEnabled = false;  // 显示是否已启用
static int8_t lastRssi = 0;          // 上次接收的信号强度



#if RX_LPF_SHIFT > 0
static int32_t lpfState = 0;
#endif

// =============================================================================
// 统计信息
// =============================================================================
typedef struct {
    // 发射统计
    uint32_t txEncoded;
    uint32_t txEnqueued;
    uint32_t txSendCalls;
    uint32_t txSendCbOk;
    uint32_t txSendCbFail;
    // 接收统计
    uint32_t rxRecvOk;
    uint32_t rxPlayed;
    uint32_t rxEmptySlots;
    uint32_t rxUnderrunEvents;
} WalkieStats;

static volatile WalkieStats g_stats = {};
static volatile WalkieStats g_lastPrint = {};

// =============================================================================
// WiFi / ESP-NOW 初始化（延迟启动 + 间歇扫描，减少GNSS干扰）
// =============================================================================
static bool wifiInitialized = false;  // WiFi是否已初始化

static void setupWifiRadio() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // 关闭省电模式，射频持续全功率工作，最大化传输距离
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    // 设置最大发射功率 20dBm (80 = 20dBm)
    esp_wifi_set_max_tx_power(80);
    
#if WIFI_USE_LR_PROTOCOL
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#endif
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    
    wifiInitialized = true;
    Serial.println("[WALKIE] WiFi已启动，发射功率20dBm(最大)，省电关闭");
}

static void setupEspNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("[WALKIE] ESP-NOW初始化失败");
        return;
    }
    
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerMac, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[WALKIE] ESP-NOW添加对等节点失败");
        return;
    }
    
    Serial.println("[WALKIE] ESP-NOW已启动");
}

// 标记当前正在发送的是GPS包还是音频包
static volatile bool sendingGpsPacket = false;

// ESP-NOW 发送完成回调
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    (void)info;
    BaseType_t wake = pdFALSE;
    
    if (sendingGpsPacket) {
        // GPS包发送完成，释放GPS信号量
        sendingGpsPacket = false;
        if (gpsTxDoneSem != nullptr) {
            xSemaphoreGiveFromISR(gpsTxDoneSem, &wake);
        }
    } else {
        // 音频包发送完成
        if (status == ESP_NOW_SEND_SUCCESS) {
            g_stats.txSendCbOk++;
        } else {
            g_stats.txSendCbFail++;
        }
        if (txDoneSem != nullptr) {
            xSemaphoreGiveFromISR(txDoneSem, &wake);
        }
    }
    if (wake == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// ESP-NOW 接收回调
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    (void)info;
    
    // 检查是否是GPS数据包
    if (len == GPS_PACKET_SIZE && data[0] == GPS_PACKET_TYPE) {
        gps_data_t parsedGps;
        if (gpsParsePacket(data, len, &parsedGps)) {
            // 保存到显示队列，超过2条就丢弃最旧的
            if (remoteGpsCount >= GPS_DISPLAY_QUEUE_SIZE) {
                // 丢弃最旧的，移动数据
                for (uint8_t i = 0; i < GPS_DISPLAY_QUEUE_SIZE - 1; i++) {
                    remoteGpsDisplay[i] = remoteGpsDisplay[i + 1];
                }
                remoteGpsIndex = (remoteGpsIndex + GPS_DISPLAY_QUEUE_SIZE - 1) % GPS_DISPLAY_QUEUE_SIZE;
            } else {
                remoteGpsCount++;
            }
            // 写入最新数据
            remoteGpsDisplay[(remoteGpsIndex + remoteGpsCount - 1) % GPS_DISPLAY_QUEUE_SIZE] = parsedGps;
            g_gpsRxCount++;
            Serial.printf("[GPS] 收到对方位置: %.6f, %.6f, UTC %02d:%02d:%02d\n",
                          parsedGps.latitude, parsedGps.longitude,
                          parsedGps.utc_hour, parsedGps.utc_minute, parsedGps.utc_second);
        }
        return;  // GPS包不作为音频处理
    }
    
    // 只有在接收模式才处理接收到的数据
    if (currentMode != MODE_RECEIVE || audioQueue == nullptr) {
        return;
    }
    if (len != (int)AUDIO_PACKET_SIZE) {
        return;
    }
    
    // 保存信号强度 (RSSI)
    if (info && info->rx_ctrl) {
        lastRssi = info->rx_ctrl->rssi;
    }
    
    static audio_packet_t isrCopy;
    memcpy(&isrCopy, data, AUDIO_PACKET_SIZE);
    
    UBaseType_t depth = uxQueueMessagesWaiting(audioQueue);
    if (depth >= RX_QUEUE_DEPTH) {
        audio_packet_t old;
        xQueueReceiveFromISR(audioQueue, &old, nullptr);
    }
    BaseType_t wake = pdFALSE;
    if (xQueueSendFromISR(audioQueue, &isrCopy, &wake) == pdTRUE) {
        g_stats.rxRecvOk++;
    }
    if (wake == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// =============================================================================
// I2S 初始化
// =============================================================================
void setupI2SMic() {
    i2s_config_t mic_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
#if MIC_I2S_USE_32BIT
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
#else
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
#endif
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = SAMPLES_PER_PACKET,
    };
    i2s_pin_config_t mic_pin_config = {
        .bck_io_num = I2S_MIC_BCK,
        .ws_io_num = I2S_MIC_WS,
        .data_out_num = -1,
        .data_in_num = I2S_MIC_SD,
    };
    i2s_driver_install(I2S_MIC_NUM, &mic_config, 0, nullptr);
    i2s_set_pin(I2S_MIC_NUM, &mic_pin_config);
}

void setupI2SDac() {
    i2s_config_t dac_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = I2S_DAC_DMA_BUF_COUNT,
        .dma_buf_len = I2S_DAC_DMA_BUF_LEN,
        .use_apll = true,
        .fixed_mclk = 384 * SAMPLE_RATE,
    };
    i2s_pin_config_t dac_pin_config = {
        .mck_io_num = I2S_DAC_MCK,
        .bck_io_num = I2S_DAC_BCK,
        .ws_io_num = I2S_DAC_WS,
        .data_out_num = I2S_DAC_DIN,
        .data_in_num = -1,
    };
    i2s_driver_install(I2S_DAC_NUM, &dac_config, 0, nullptr);
    i2s_set_pin(I2S_DAC_NUM, &dac_pin_config);
}

// =============================================================================
// 音频处理辅助函数
// =============================================================================
static int16_t softClip(int32_t s) {
    const int32_t t = 28000;
    if (s > t) {
        return (int16_t)(t + (s - t) / 4);
    }
    if (s < -t) {
        return (int16_t)(-t + (s + t) / 4);
    }
    return (int16_t)s;
}

#if MIC_I2S_USE_32BIT
static void readMicPcm(int16_t *pcm) {
    static int32_t raw32[SAMPLES_PER_PACKET];
    size_t bytesRead = 0;
    i2s_read(I2S_MIC_NUM, raw32, sizeof(raw32), &bytesRead, portMAX_DELAY);
    int count = bytesRead / (int)sizeof(int32_t);
    if (count > SAMPLES_PER_PACKET) {
        count = SAMPLES_PER_PACKET;
    }
    for (int i = 0; i < count; i++) {
        pcm[i] = softClip(raw32[i] >> MIC_I2S_SHIFT_BITS);
    }
    for (int i = count; i < SAMPLES_PER_PACKET; i++) {
        pcm[i] = 0;
    }
}
#else
static void readMicPcm(int16_t *pcm) {
    size_t bytesRead = 0;
    i2s_read(I2S_MIC_NUM, pcm, SAMPLES_PER_PACKET * sizeof(int16_t), &bytesRead, portMAX_DELAY);
}
#endif

#if RX_LPF_SHIFT > 0
static int16_t applyLowPass(int16_t x) {
    lpfState += ((int32_t)x - lpfState) >> RX_LPF_SHIFT;
    return (int16_t)lpfState;
}
#endif

static void writeStereoToI2S(const int16_t *mono) {
    static int16_t stereo[2][SAMPLES_PER_PACKET * 2];
    static uint8_t bufIdx = 0;
    int16_t *out = stereo[bufIdx];
    bufIdx ^= 1;
    for (int i = 0; i < SAMPLES_PER_PACKET; i++) {
        int32_t s = mono[i];
#if RX_LPF_SHIFT > 0
        s = applyLowPass((int16_t)s);
#endif
        s = (s * RX_GAIN_MUL) / RX_GAIN_DIV;
        int16_t v = softClip(s);
        out[2 * i] = v;
        out[2 * i + 1] = v;
    }
    size_t written = 0;
    i2s_write(I2S_DAC_NUM, out, SAMPLES_PER_PACKET * 2 * sizeof(int16_t), &written, portMAX_DELAY);
}

static void writeSilenceToI2S() {
    static int16_t silence[SAMPLES_PER_PACKET * 2];
    memset(silence, 0, sizeof(silence));
    size_t written = 0;
    i2s_write(I2S_DAC_NUM, silence, sizeof(silence), &written, portMAX_DELAY);
}

// =============================================================================
// 发射任务
// =============================================================================
void micTask(void *arg) {
    (void)arg;
    static int16_t pcm[SAMPLES_PER_PACKET];
    audio_packet_t packet;
    
    for (;;) {
        // 检查当前模式，如果不是发射模式则挂起自己
        if (currentMode != MODE_TRANSMIT) {
            vTaskSuspend(nullptr);
            continue;
        }
        
        readMicPcm(pcm);
        for (int i = 0; i < SAMPLES_PER_PACKET; i++) {
            pcm[i] = softClip((int32_t)(pcm[i] * TX_PRE_GAIN));
        }
        adpcm_encode(pcm, packet.adpcm, SAMPLES_PER_PACKET, &encoderState);
        packet.seq = txSeq++;
        g_stats.txEncoded++;
        
        xQueueSend(sendQueue, &packet, portMAX_DELAY);
        g_stats.txEnqueued++;
    }
}

void sendTask(void *arg) {
    (void)arg;
    audio_packet_t packet;
    
    for (;;) {
        if (currentMode != MODE_TRANSMIT) {
            vTaskSuspend(nullptr);
            continue;
        }
        
        // WiFi未初始化时（前5秒），不发送，直接丢弃队列中的包
        if (!wifiInitialized) {
            while (xQueueReceive(sendQueue, &packet, 0) == pdTRUE) {
                // 丢弃音频包
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        xSemaphoreTake(txDoneSem, portMAX_DELAY);
        
        // 只发送音频包，GPS包由gpsTask独立发送
        if (xQueueReceive(sendQueue, &packet, 0) == pdTRUE) {
            g_stats.txSendCalls++;
            if (esp_now_send(peerMac, (uint8_t *)&packet, sizeof(packet)) != ESP_OK) {
                xSemaphoreGive(txDoneSem);
            }
        } else {
            xSemaphoreGive(txDoneSem);
        }
    }
}

// =============================================================================
// 接收任务
// =============================================================================
static void resetDecoderOnly() {
    adpcm_state_reset(&decoderState);
    seqInitialized = false;
    hasPlayedSeq = false;
#if RX_LPF_SHIFT > 0
    lpfState = 0;
#endif
}

static void flushAudioQueue() {
    audio_packet_t tmp;
    while (xQueueReceive(audioQueue, &tmp, 0) == pdTRUE) {
    }
}

static bool acceptPacketSeq(uint8_t seq) {
    if (hasPlayedSeq && seq == lastPlayedSeq) {
        return false;
    }
    if (seqInitialized && seq != expectedSeq) {
        uint8_t ahead = (uint8_t)(seq - expectedSeq);
        if (ahead > 127) {
            return false;
        }
        adpcm_state_reset(&decoderState);
    }
    seqInitialized = true;
    expectedSeq = (uint8_t)(seq + 1);
    return true;
}

void playoutTask(void *arg) {
    (void)arg;
    static int16_t decoded[SAMPLES_PER_PACKET];
    audio_packet_t packet;
    
    TickType_t nextWake = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(RX_PLAYOUT_PERIOD_MS);
    
    for (;;) {
        // 检查当前模式，如果不是接收模式则挂起自己
        if (currentMode != MODE_RECEIVE) {
            vTaskSuspend(nullptr);
            continue;
        }
        
        // 预缓冲阶段
        if (playState == PLAY_STATE_PREFILL) {
            if (uxQueueMessagesWaiting(audioQueue) >= RX_PREFILL_PACKETS) {
                playState = PLAY_STATE_RUNNING;
                emptyStreak = 0;
                nextWake = xTaskGetTickCount();
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            // 检查信号是否丢失（预缓冲阶段也检查）
            checkSignalLost();
            continue;
        }
        
        // 正常播放
        vTaskDelayUntil(&nextWake, periodTicks);
        
        // 检查信号是否丢失
        checkSignalLost();
        
        if (xQueueReceive(audioQueue, &packet, 0) != pdTRUE) {
            writeSilenceToI2S();
            g_stats.rxEmptySlots++;
            emptyStreak++;
            if (emptyStreak >= RX_UNDERRUN_THRESHOLD) {
                playState = PLAY_STATE_PREFILL;
                emptyStreak = 0;
                g_stats.rxUnderrunEvents++;
                resetDecoderOnly();
                flushAudioQueue();
            }
            continue;
        }
        
        emptyStreak = 0;
        if (!acceptPacketSeq(packet.seq)) {
            writeSilenceToI2S();
            g_stats.rxEmptySlots++;
            continue;
        }
        
        // 收到有效包，更新功放状态
        updateAmpStateOnValidPacket();
        
        // 如果功放当前是静音状态，不解码播放（节省CPU）
        if (ampCurrentlyMuted) {
            // 只更新状态，不实际播放
            lastPlayedSeq = packet.seq;
            hasPlayedSeq = true;
            g_stats.rxPlayed++;
            continue;
        }
        
        adpcm_decode(packet.adpcm, decoded, SAMPLES_PER_PACKET, &decoderState);
        writeStereoToI2S(decoded);
        
        lastPlayedSeq = packet.seq;
        hasPlayedSeq = true;
        g_stats.rxPlayed++;
    }
}

// =============================================================================
// GPS 任务 - 更新GPS数据和处理位置广播
// =============================================================================
void gpsTask(void *arg) {
    (void)arg;
    gps_data_t* localGps;
    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));  // 每100ms更新一次GPS
        
        // 更新GPS数据
        gpsUpdate();
        
        // 检查本地GPS是否有效
        localGps = gpsGetData();
        if (gpsIsValid()) {
            // 准备GPS广播包
            gpsCreatePacket(localGps, &gpsBroadcastPkt);
            hasGpsDataToSend = true;
        } else {
            hasGpsDataToSend = false;
        }
        
        // 在接收模式下，独立发送GPS包（不干扰音频发送链路）
        // WiFi未初始化时（前5秒）不发送GPS包
        if (wifiInitialized && currentMode == MODE_RECEIVE && hasGpsDataToSend) {
            uint32_t now = millis();
            if (now - g_gpsLastSendTime >= GPS_BROADCAST_INTERVAL_MS) {
                // 等待上一包GPS发送完成
                if (xSemaphoreTake(gpsTxDoneSem, pdMS_TO_TICKS(100)) == pdTRUE) {
                    g_gpsLastSendTime = now;
                    sendingGpsPacket = true;  // 标记为GPS包，回调中释放gpsTxDoneSem
                    esp_now_send(peerMac, (uint8_t *)&gpsBroadcastPkt, sizeof(gpsBroadcastPkt));
                    g_gpsTxCount++;
                    Serial.printf("[GPS] 发送位置: %.6f, %.6f\n", 
                                  gpsBroadcastPkt.latitude, gpsBroadcastPkt.longitude);
                }
            }
        }
        
        // 在发射模式下，每3秒在串口打印一次GPS状态
        if (currentMode == MODE_TRANSMIT && hasGpsDataToSend) {
            uint32_t now = millis();
            static uint32_t lastPrint = 0;
            if (now - lastPrint >= 3000) {
                lastPrint = now;
                char latStr[32], lonStr[32];
                formatLatitude(localGps->latitude, 'N', latStr, sizeof(latStr));
                formatLongitude(localGps->longitude, 'E', lonStr, sizeof(lonStr));
                uint8_t bjHour = utcToBeijing(localGps->utc_hour);
                Serial.printf("[GPS] 本地位置: %s %s, 高度: %.1fm, 北京时间: %02d:%02d:%02d, 卫星: %d\n",
                              latStr, lonStr, localGps->altitude,
                              bjHour, localGps->utc_minute, localGps->utc_second,
                              localGps->satellites);
            }
        }
    }
}

// =============================================================================
// 功放静音控制函数
// =============================================================================
static void setupAmpMute() {
    pinMode(AMP_MUTE_PIN, OUTPUT);
    // 初始状态：静音（等待有效信号后再开启）
    digitalWrite(AMP_MUTE_PIN, HIGH);
    ampCurrentlyMuted = true;
}

static void setAmpMute(bool mute) {
    digitalWrite(AMP_MUTE_PIN, mute ? HIGH : LOW);
    ampCurrentlyMuted = mute;
}

/**
 * 检查是否应该开启功放
 * 在接收模式下，收到有效信号时调用
 */
static void updateAmpStateOnValidPacket() {
    if (currentMode != MODE_RECEIVE) {
        return;
    }
    
    lastValidPacketTime = millis();
    consecutiveValidPackets++;
    
    // 如果当前是静音状态，且连续收到足够数量的包，则开启功放
    if (ampCurrentlyMuted && consecutiveValidPackets >= AMP_TURN_ON_DELAY_PACKETS) {
        setAmpMute(false);
        setLed(LED_RX);  // 收到有效信号，LED 指示接收
        updateDisplay(DISPLAY_RX);  // 显示"接收中"
        // Serial.println("[WALKIE] 检测到信号，开启功放");
    }
}

/**
 * 检查信号是否丢失，如果超时未收到包则静音功放
 * 在播放任务中定期调用
 */
static void checkSignalLost() {
    if (currentMode != MODE_RECEIVE || ampCurrentlyMuted) {
        return;
    }
    
    uint32_t elapsed = millis() - lastValidPacketTime;
    if (elapsed > SIGNAL_LOST_TIMEOUT_MS) {
        setAmpMute(true);
        setLed(LED_OFF);  // 信号丢失，LED 熄灭
        updateDisplay(DISPLAY_STANDBY);  // 显示"待机中"
        consecutiveValidPackets = 0;
        lastRssi = 0;  // 重置RSSI，信号格归零
        // Serial.println("[WALKIE] 信号丢失，关断功放");
    }
}

/**
 * 重置信号检测状态（切换模式时调用）
 */
static void resetSignalDetection() {
    lastValidPacketTime = millis();
    consecutiveValidPackets = 0;
    // 切换到接收模式时，先保持静音，等待有效信号
    if (currentMode == MODE_RECEIVE) {
        setAmpMute(true);
    }
}

// =============================================================================
// LED 指示灯控制函数
// =============================================================================
static void setupLed() {
    pinMode(LED_PIN, INPUT);  // 初始高阻态，LED 全灭
    ledState = LED_OFF;
}

static void setLed(LedState state) {
    if (ledState == state) return;
    ledState = state;
    switch (state) {
        case LED_TX:
            pinMode(LED_PIN, OUTPUT);
            digitalWrite(LED_PIN, LOW);   // 发射时低电平，红灯亮
            break;
        case LED_RX:
            pinMode(LED_PIN, OUTPUT);
            digitalWrite(LED_PIN, HIGH);  // 接收有效信号时高电平，绿灯亮
            break;
        case LED_OFF:
        default:
            pinMode(LED_PIN, INPUT);      // 高阻态，LED 全灭
            break;
    }
}

// =============================================================================
// 电池电压检测函数
// =============================================================================
static void updateBatteryStatus() {
    uint32_t now = millis();
    if (now - lastBatteryUpdate < BATTERY_UPDATE_INTERVAL_MS) {
        return;  // 未到更新时间
    }
    lastBatteryUpdate = now;
    
    // 多次采样取平均，提高精度
    uint32_t adcSum = 0;
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        adcSum += analogRead(BATTERY_ADC_PIN);
        delayMicroseconds(100);  // 短暂延时让ADC稳定
    }
    uint16_t adcAverage = adcSum / BATTERY_ADC_SAMPLES;
    
    // 计算电压：ADC值 / 分辨率 * 参考电压 * 分压比
    float voltage = (adcAverage / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;
    
    // 简单的低通滤波，避免电压跳变
    if (batteryVoltage == 0.0f) {
        batteryVoltage = voltage;
    } else {
        batteryVoltage = batteryVoltage * 0.7f + voltage * 0.3f;
    }
    
    // 计算电量百分比（线性近似）
    if (batteryVoltage >= BATTERY_VOLTAGE_MAX) {
        batteryPercentage = 100;
    } else if (batteryVoltage <= BATTERY_VOLTAGE_MIN) {
        batteryPercentage = 0;
    } else {
        batteryPercentage = (uint8_t)((batteryVoltage - BATTERY_VOLTAGE_MIN) / 
                                      (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN) * 100.0f);
    }
}

// =============================================================================
// 图形绘制辅助函数
// =============================================================================

/**
 * 绘制手机信号格图标
 * @param rssi 信号强度(dBm)，0表示无信号
 * @param x 左上角x坐标
 * @param y 左上角y坐标
 */
static void drawSignalBars(int8_t rssi, uint8_t x, uint8_t y) {
    // 信号格参数（缩小尺寸，下边缘齐平）
    const uint8_t barWidth = 3;      // 每格宽度
    const uint8_t barGap = 1;        // 格间距
    const uint8_t baseHeight = 3;    // 最矮格高度
    const uint8_t barStep = 2;       // 每格高度增量
    const uint8_t maxBarHeight = baseHeight + 3 * barStep;  // 最高格 = 9
    
    // 根据RSSI计算信号格数 (4格满格)
    uint8_t bars = 0;
    if (rssi == 0) {
        bars = 0;
    } else if (rssi >= -50) {
        bars = 4;
    } else if (rssi >= -65) {
        bars = 3;
    } else if (rssi >= -80) {
        bars = 2;
    } else if (rssi >= -95) {
        bars = 1;
    } else {
        bars = 0;
    }
    
    // 绘制4格信号条（从左到右，由矮到高，下边缘对齐y+maxBarHeight-1）
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t barHeight = baseHeight + i * barStep;
        uint8_t barX = x + i * (barWidth + barGap);
        uint8_t barY = y + maxBarHeight - barHeight;  // 下边缘齐平
        
        if (i < bars) {
            display.drawBox(barX, barY, barWidth, barHeight);
        } else {
            display.drawFrame(barX, barY, barWidth, barHeight);
        }
    }
}

/**
 * 绘制电池图标（缩小尺寸，下边缘齐平）
 * @param percentage 电量百分比(0-100)
 * @param x 左上角x坐标
 * @param y 左上角y坐标（实际为下边缘参考位置）
 */
static void drawBatteryIcon(uint8_t percentage, uint8_t x, uint8_t y) {
    const uint8_t battWidth = 16;    // 电池主体宽度
    const uint8_t battHeight = 8;    // 电池主体高度
    const uint8_t capWidth = 2;      // 电池正极帽宽度
    const uint8_t capHeight = 3;     // 电池正极帽高度
    
    // 计算实际y坐标（使下边缘对齐到y+8）
    uint8_t topY = y + 1;  // 稍微下移，与信号格下边缘对齐
    
    // 绘制电池外框
    display.drawFrame(x, topY, battWidth, battHeight);
    
    // 绘制电池正极帽（右侧，居中）
    uint8_t capX = x + battWidth;
    uint8_t capY = topY + (battHeight - capHeight) / 2;
    display.drawBox(capX, capY, capWidth, capHeight);
    
    // 计算填充宽度（留1像素边框）
    uint8_t fillMaxWidth = battWidth - 2;
    uint8_t fillWidth = (percentage * fillMaxWidth) / 100;
    if (fillWidth > fillMaxWidth) fillWidth = fillMaxWidth;
    if (fillWidth < 1 && percentage > 0) fillWidth = 1;
    
    // 绘制电量填充
    if (fillWidth > 0) {
        uint8_t fillX = x + 1;
        uint8_t fillY = topY + 1;
        uint8_t fillH = battHeight - 2;
        display.drawBox(fillX, fillY, fillWidth, fillH);
    }
    
    // 低电量时显示警告（电量<20%且不为0）
    if (percentage > 0 && percentage < 20) {
        display.setFont(u8g2_font_5x7_tf);
        display.setCursor(x + battWidth / 2 - 1, topY + battHeight - 1);
        display.print("!");
    }
}

// =============================================================================
// OLED 显示函数
// =============================================================================
static void setupOled() {
    // OLED 初始化重试机制（I2C 总线可能因上电时序不稳定而初始化失败）
    for (int retry = 0; retry < 3; retry++) {
        if (display.begin()) {
            break;
        }
        // Serial.printf("[OLED] 初始化失败，重试 %d/3...\n", retry + 1);
        delay(100);
    }
    display.enableUTF8Print();
    // 使用文泉驿 12px 中文字体，覆盖更多常用汉字
    display.setFont(u8g2_font_wqy12_t_chinese1);
    updateDisplay(DISPLAY_STANDBY);
}

static void updateDisplay(DisplayState state) {
    displayState = state;

    // 获取 I2C 互斥锁，防止多任务并发访问 OLED
    if (i2cMutex != nullptr) {
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
    }

    display.clearBuffer();

    // 使用 6x13 字体，每行高度约 13px
    // 128x64 屏幕可以显示 4-5 行
    display.setFont(u8g2_font_6x13_tf);

    char buf[48];  // 增大缓冲区防止溢出

    // 拷贝 GPS 数据快照，避免读取时被 gpsTask 修改导致脏数据
    gps_data_t localGpsSnapshot;
    gps_data_t* localGps = gpsGetData();
    memcpy(&localGpsSnapshot, localGps, sizeof(gps_data_t));
    bool localGpsValid = gpsIsValid();
    bool localTimeValid = (localGpsSnapshot.flags & GPS_FLAG_TIME_VALID) != 0;
    bool localSivValid = (localGpsSnapshot.flags & GPS_FLAG_SIV_VALID) != 0;
    
    // 拷贝对方 GPS 数据快照
    gps_data_t remoteGpsSnapshot;
    uint8_t remoteCountSnapshot = remoteGpsCount;
    if (remoteCountSnapshot > 0) {
        uint8_t latestIdx = (remoteGpsIndex + remoteCountSnapshot - 1) % GPS_DISPLAY_QUEUE_SIZE;
        memcpy(&remoteGpsSnapshot, &remoteGpsDisplay[latestIdx], sizeof(gps_data_t));
    }

    // 拷贝统计快照
    WalkieStats cur;
    memcpy(&cur, (const void *)&g_stats, sizeof(WalkieStats));
    uint32_t gpsTxSnap = g_gpsTxCount;
    uint32_t gpsRxSnap = g_gpsRxCount;
    int8_t rssiSnap = lastRssi;

    // ========== 第1行: 模式 ==========
    display.setCursor(0, 10);
    switch (state) {
        case DISPLAY_TX:
            display.print("MODE: TX");
            break;
        case DISPLAY_RX:
            display.print("MODE: RX");
            break;
        case DISPLAY_STANDBY:
        default:
            display.print("MODE: STBY");
            break;
    }
    
    // ========== 右上角: 信号格 + 电池图标（贴右上角，下边缘对齐）==========
    // 信号格总宽 = 4*(3+1)-1 = 15像素，紧贴右侧
    // 电池总宽 = 16+2 = 18像素
    // 总宽 = 15 + 4 + 18 = 37像素，从x=128-37=91开始
    drawSignalBars(rssiSnap, 91, 0);
    drawBatteryIcon(batteryPercentage, 91 + 15 + 2, 0);

    // ========== 第3行: 本地GPS状态 ==========
    display.setCursor(0, 24);
    if (localGpsValid) {
        display.print("*LOC:");
        snprintf(buf, sizeof(buf), "%.4f", localGpsSnapshot.latitude);
        display.print(buf);
        display.print(",");
        snprintf(buf, sizeof(buf), "%.4f", localGpsSnapshot.longitude);
        display.print(buf);
    } else {
        display.print("*LOC: NO FIX");
    }

    // ========== 第4行: 北京时间 ==========
    display.setCursor(0, 36);
    if (localTimeValid) {
        uint8_t bjHour = utcToBeijing(localGpsSnapshot.utc_hour);
        snprintf(buf, sizeof(buf), "BJ:%02d:%02d:%02d", 
                 bjHour, localGpsSnapshot.utc_minute, localGpsSnapshot.utc_second);
        display.print(buf);
    } else {
        display.print("BJ:--:--:--");
    }
    
    // 显示卫星数
    display.setCursor(75, 36);
    if (localSivValid) {
        snprintf(buf, sizeof(buf), "SAT:%d", localGpsSnapshot.satellites);
        display.print(buf);
    } else {
        display.print("SAT: --");
    }

    // ========== 第5行: 对方GPS位置 ==========
    display.setCursor(0, 48);
    if (remoteCountSnapshot > 0) {
        display.print("REM:");
        snprintf(buf, sizeof(buf), "%.4f,%.4f", remoteGpsSnapshot.latitude, remoteGpsSnapshot.longitude);
        display.print(buf);
    } else {
        display.print("REM: NO DATA");
    }
    
    // ========== 第6行: 包统计 ==========
    display.setCursor(0, 64);
    
    // 音频包
    display.print("A:T");
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)cur.txSendCbOk);
    display.print(buf);
    display.print(" R");
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)cur.rxPlayed);
    display.print(buf);
    
    // GPS包
    display.print(" G:T");
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)gpsTxSnap);
    display.print(buf);
    display.print(" R");
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)gpsRxSnap);
    display.print(buf);

    display.sendBuffer();

    // 释放 I2C 互斥锁
    if (i2cMutex != nullptr) {
        xSemaphoreGive(i2cMutex);
    }
}

// =============================================================================
// 切换到接收模式（提取为独立函数，避免重复代码）
// =============================================================================
static void switchToRxMode(void) {
    // Serial.println("[WALKIE] 切换到接收模式");
    // 清空发送队列
    audio_packet_t tmp;
    while (xQueueReceive(sendQueue, &tmp, 0) == pdTRUE) {}
    // 重置发射状态
    adpcm_state_reset(&encoderState);
    xSemaphoreGive(txDoneSem);
    // 重置信号检测状态
    resetSignalDetection();
    // LED 熄灭
    setLed(LED_OFF);
    updateDisplay(DISPLAY_STANDBY);
    // 恢复接收任务
    if (playoutTaskHandle != nullptr) vTaskResume(playoutTaskHandle);
    // 发射任务会在循环中检测模式并挂起自己
}

// =============================================================================
// 按键检测任务
// =============================================================================
void buttonTask(void *arg) {
    (void)arg;
    
    // 配置 GPIO5 为输入，启用内部上拉
    pinMode(PTT_BUTTON_PIN, INPUT_PULLUP);
    
    bool lastButtonState = HIGH;  // 上拉默认高电平
    unsigned long lastDebounceTime = 0;
    
    for (;;) {
        bool reading = digitalRead(PTT_BUTTON_PIN);
        
        // 检测状态变化，开始消抖计时
        if (reading != lastButtonState) {
            lastDebounceTime = millis();
        }
        
        // 消抖时间过后确认状态
        if ((millis() - lastDebounceTime) > BUTTON_DEBOUNCE_MS) {
            // 按键按下（低电平）-> 发射模式
            // 按键松开（高电平）-> 接收模式
            WorkMode newMode = (reading == LOW) ? MODE_TRANSMIT : MODE_RECEIVE;
            
            if (newMode != currentMode) {
                currentMode = newMode;
                
                if (currentMode == MODE_TRANSMIT) {
                    // Serial.println("[WALKIE] 切换到发射模式");
                    // LED 指示发射
                    setLed(LED_TX);
                    updateDisplay(DISPLAY_TX);  // 显示"发射中"
                    // 发射时静音功放（避免扬声器啸叫/回声）
                    setAmpMute(true);
                    // 清空接收队列
                    flushAudioQueue();
                    playState = PLAY_STATE_PREFILL;
                    // 恢复发射任务，挂起接收任务
                    if (micTaskHandle != nullptr) vTaskResume(micTaskHandle);
                    if (sendTaskHandle != nullptr) vTaskResume(sendTaskHandle);
                    if (playoutTaskHandle != nullptr) vTaskSuspend(playoutTaskHandle);
                } else {
                    // 松开PTT → 直接切换到接收模式（GPS由gpsTask独立发送，不阻塞音频）
                    switchToRxMode();
                }
            }
        }
        
        lastButtonState = reading;
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms 检测间隔
    }
}

// =============================================================================
// 显示刷新任务 - 实时更新屏幕
// =============================================================================
void displayTask(void *arg) {
    (void)arg;
    
    for (;;) {
        // 每 200ms 刷新一次显示
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // 更新电池电量（内部有5秒间隔控制）
        updateBatteryStatus();
        
        // 根据当前模式刷新显示
        if (currentMode == MODE_TRANSMIT) {
            updateDisplay(DISPLAY_TX);
        } else {
            // 接收模式：根据是否有信号显示不同状态
            if (!ampCurrentlyMuted && consecutiveValidPackets > 0) {
                updateDisplay(DISPLAY_RX);
            } else {
                updateDisplay(DISPLAY_STANDBY);
            }
        }
    }
}

// =============================================================================
// 统计打印
// =============================================================================
static void printStats() {
    // 串口统计输出已注释掉，减少CPU负载和串口干扰
    // 如需调试可取消注释下方代码
    
    /*
    WalkieStats cur;
    WalkieStats prev;
    memcpy(&cur, (const void *)&g_stats, sizeof(WalkieStats));
    memcpy(&prev, (const void *)&g_lastPrint, sizeof(WalkieStats));
    
    gps_data_t* localGps = gpsGetData();
    
    Serial.println();
    Serial.println("========== [对讲机] 统计 ==========");
    Serial.printf("当前模式: %s\n", (currentMode == MODE_TRANSMIT) ? "发射" : "接收");
    
    Serial.println("--- GPS状态 ---");
    if (gpsIsValid()) {
        char latStr[32], lonStr[32];
        formatLatitude(localGps->latitude, 'N', latStr, sizeof(latStr));
        formatLongitude(localGps->longitude, 'E', lonStr, sizeof(lonStr));
        uint8_t bjHour = utcToBeijing(localGps->utc_hour);
        Serial.printf("  本地位置: %s %s\n", latStr, lonStr);
        Serial.printf("  北京时间: %02d:%02d:%02d\n", bjHour, localGps->utc_minute, localGps->utc_second);
        Serial.printf("  高度: %.1fm, 卫星: %d\n", localGps->altitude, localGps->satellites);
    } else {
        Serial.println("  GPS: 无定位");
    }
    
    Serial.println("--- GPS包统计 ---");
    Serial.printf("  发送: %lu, 接收: %lu\n", (unsigned long)g_gpsTxCount, (unsigned long)g_gpsRxCount);
    
    if (remoteGpsCount > 0) {
        uint8_t latestIdx = (remoteGpsIndex + remoteGpsCount - 1) % GPS_DISPLAY_QUEUE_SIZE;
        gps_data_t* remote = &remoteGpsDisplay[latestIdx];
        Serial.printf("  对方位置: %.6f, %.6f\n", remote->latitude, remote->longitude);
    } else {
        Serial.println("  对方位置: 无数据");
    }
    
    Serial.println("--- 发射统计 ---");
    Serial.printf("  编码帧: %lu\n", (unsigned long)cur.txEncoded);
    Serial.printf("  发送调用: %lu\n", (unsigned long)cur.txSendCalls);
    Serial.printf("  发送成功: %lu\n", (unsigned long)cur.txSendCbOk);
    Serial.printf("  发送失败: %lu\n", (unsigned long)cur.txSendCbFail);
    Serial.println("--- 接收统计 ---");
    Serial.printf("  接收成功: %lu\n", (unsigned long)cur.rxRecvOk);
    Serial.printf("  播放帧: %lu\n", (unsigned long)cur.rxPlayed);
    Serial.printf("  静音帧: %lu\n", (unsigned long)cur.rxEmptySlots);
    Serial.printf("  欠载事件: %lu\n", (unsigned long)cur.rxUnderrunEvents);
    Serial.println("====================================");
    
    memcpy((void *)&g_lastPrint, &cur, sizeof(WalkieStats));
    */
}

// =============================================================================
// WiFi 延迟启动任务 - 上电后5秒再启动WiFi/ESP-NOW
// =============================================================================
static TaskHandle_t wifiStartupTaskHandle = nullptr;

void wifiStartupTask(void *arg) {
    (void)arg;
    
    Serial.println("[WALKIE] WiFi启动延迟任务开始，5秒后启动WiFi/ESP-NOW...");
    
    // 延迟5秒，使用vTaskDelay而非delay()，不阻塞其他任务
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // 5秒后启动WiFi和ESP-NOW
    setupWifiRadio();
    setupEspNow();
    
    Serial.println("[WALKIE] WiFi/ESP-NOW延迟启动完成，通信已恢复");
    
    // 任务完成，删除自身
    wifiStartupTaskHandle = nullptr;
    vTaskDelete(nullptr);
}

// =============================================================================
// Arduino setup / loop
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    
    memset((void *)&g_stats, 0, sizeof(WalkieStats));
    memset((void *)&g_lastPrint, 0, sizeof(WalkieStats));
    
    // 初始化 ADPCM 编解码器状态
    adpcm_state_reset(&encoderState);
    adpcm_state_reset(&decoderState);
    
    // 创建队列和信号量
    sendQueue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(audio_packet_t));
    audioQueue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(audio_packet_t));
    txDoneSem = xSemaphoreCreateBinary();
    gpsTxDoneSem = xSemaphoreCreateBinary();
    i2cMutex = xSemaphoreCreateMutex();
    
    if (sendQueue == nullptr || audioQueue == nullptr || txDoneSem == nullptr || 
        gpsTxDoneSem == nullptr || i2cMutex == nullptr) {
        return;
    }
    xSemaphoreGive(txDoneSem);
    xSemaphoreGive(gpsTxDoneSem);
    
    // 初始化 OLED
    setupOled();
    displayEnabled = true;
    
    // 初始化功放静音控制
    setupAmpMute();
    
    // 初始化 LED 指示灯
    setupLed();
    
    // 初始化 GPS 模块
    setupGPS(GPS_RX_PIN, GPS_TX_PIN);
    
    // 初始化电池电压检测ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(BATTERY_ADC_PIN, INPUT);
    
    // 初始化 I2S
    setupI2SMic();
    setupI2SDac();
    
    // 注意：WiFi/ESP-NOW 不在这里初始化，由 wifiStartupTask 延迟5秒后启动
    wifiInitialized = false;
    Serial.println("[WALKIE] 系统初始化完成，WiFi/ESP-NOW将在5秒后启动");
    
    // 创建任务
    xTaskCreatePinnedToCore(micTask, "micTask", 4096, nullptr, 3, &micTaskHandle, 0);
    xTaskCreatePinnedToCore(sendTask, "sendTask", 4096, nullptr, 2, &sendTaskHandle, 1);
    xTaskCreatePinnedToCore(playoutTask, "playoutTask", 8192, nullptr, 3, &playoutTaskHandle, 0);
    xTaskCreatePinnedToCore(gpsTask, "gpsTask", 4096, nullptr, 1, &gpsTaskHandle, 1);
    xTaskCreatePinnedToCore(buttonTask, "buttonTask", 4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(displayTask, "displayTask", 4096, nullptr, 1, nullptr, 1);
    
    // 创建WiFi延迟启动任务（优先级最低，不影响其他任务）
    xTaskCreatePinnedToCore(wifiStartupTask, "wifiStartup", 4096, nullptr, 1, &wifiStartupTaskHandle, 1);
    
    // 初始状态：接收模式，挂起发射任务
    vTaskSuspend(micTaskHandle);
    vTaskSuspend(sendTaskHandle);
}

void loop() {
    printStats();
    delay(STATS_PRINT_INTERVAL_MS > 0 ? STATS_PRINT_INTERVAL_MS : 5000);
}
