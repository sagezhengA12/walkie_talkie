/**
 * @file gps.h
 * @brief GPS NEMA 数据解析和位置信息管理
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

// GPS数据有效性标志
#define GPS_FLAG_VALID      0x01  // 数据有效
#define GPS_FLAG_LAT_VALID   0x02  // 纬度有效
#define GPS_FLAG_LON_VALID   0x04  // 经度有效
#define GPS_FLAG_TIME_VALID  0x08  // 时间有效
#define GPS_FLAG_SIV_VALID   0x10  // 卫星数有效

// GPS位置数据结构
typedef struct {
    float latitude;       // 纬度 (-90 ~ 90)
    float longitude;       // 经度 (-180 ~ 180)
    float altitude;        // 高度(米)
    float speed;           // 速度(km/h)
    float course;          // 航向(度)
    uint8_t flags;         // 有效性标志
    uint8_t satellites;    // 可见卫星数
    uint16_t hdop;         // HDOP * 100 (如 185 表示 1.85)
    
    // UTC时间 (来自GGA/RMC)
    uint16_t utc_year;
    uint8_t utc_month;
    uint8_t utc_day;
    uint8_t utc_hour;
    uint8_t utc_minute;
    uint8_t utc_second;
    
    uint32_t last_update;  // 上次更新时间 (millis)
} gps_data_t;

// GPS信息包 - 通过ESP-NOW发送
#pragma pack(push, 1)
typedef struct {
    uint8_t type;          // 0x01 = GPS信息包
    uint8_t seq;           // 序号
    float latitude;
    float longitude;
    float altitude;
    uint16_t utc_year;
    uint8_t utc_month;
    uint8_t utc_day;
    uint8_t utc_hour;
    uint8_t utc_minute;
    uint8_t utc_second;
    uint8_t satellites;
    uint16_t hdop;
    uint8_t checksum;
} gps_packet_t;
#pragma pack(pop)

#define GPS_PACKET_TYPE 0x01
#define GPS_PACKET_SIZE (sizeof(gps_packet_t))

// GPS NEMA协议最大行长
#define NEMA_MAX_LENGTH 82

/**
 * @brief 初始化GPS模块
 * @param rxPin GPS模块RX连接的ESP32引脚
 * @param txPin GPS模块TX连接的ESP32引脚 (用于调试输出，可选)
 */
void setupGPS(uint8_t rxPin, uint8_t txPin);

/**
 * @brief 处理接收到的GPS字符
 * @param c 接收到的字符
 * @return true 如果收到完整语句
 */
bool gpsProcessChar(uint8_t c);

/**
 * @brief 更新GPS数据
 * @return true 如果有新的有效数据
 */
bool gpsUpdate(void);

/**
 * @brief 获取当前GPS数据
 */
gps_data_t* gpsGetData(void);

/**
 * @brief 检查GPS是否有效定位
 */
bool gpsIsValid(void);

/**
 * @brief UTC时间转北京时间
 * @param utc_hour UTC小时
 * @return 北京时间小时 (0-23)
 */
inline uint8_t utcToBeijing(uint8_t utc_hour) {
    return (utc_hour + 8) % 24;
}

/**
 * @brief 格式化纬度为 度分秒 字符串
 * @param lat 纬度值
 * @param ns 'N' 或 'S'
 * @param buf 输出缓冲区
 * @param len 缓冲区长度
 */
void formatLatitude(float lat, char ns, char* buf, size_t len);

/**
 * @brief 格式化经度为 度分秒 字符串
 * @param lon 经度值
 * @param ew 'E' 或 'W'
 * @param buf 输出缓冲区
 * @param len 缓冲区长度
 */
void formatLongitude(float lon, char ew, char* buf, size_t len);

/**
 * @brief 计算校验和
 * @param nmea NEMA语句 (不含$和*xx)
 * @return 校验和
 */
uint8_t nmeaChecksum(const char* nmea);

/**
 * @brief 从gps_data_t创建GPS数据包
 */
void gpsCreatePacket(const gps_data_t* data, gps_packet_t* packet);

/**
 * @brief 解析GPS数据包
 */
bool gpsParsePacket(const uint8_t* data, size_t len, gps_data_t* out);
