/**
 * @file gps.cpp
 * @brief GPS 数据解析实现 — 同时支持标准NMEA和ATGM336H私有协议
 */
#include "gps.h"
#include <string.h>
#include <stdlib.h>

// 串口接收缓冲区
static char lineBuffer[NEMA_MAX_LENGTH + 1];
static uint8_t lineIndex = 0;

// GPS数据
static gps_data_t gpsData = {};

// GPS串口
static HardwareSerial* gpsSerial = nullptr;

// 调试：上次成功解析的语句类型
static uint32_t lastParseTime = 0;
static uint8_t lastParseCount = 0;

/**
 * @brief 解析逗号分隔的字段
 */
static char* getField(char* s, uint8_t index) {
    static char field[32];
    memset(field, 0, sizeof(field));
    
    uint8_t commaCount = 0;
    uint8_t j = 0;
    
    for (uint8_t i = 0; s[i] != 0 && i < NEMA_MAX_LENGTH; i++) {
        if (s[i] == ',') {
            commaCount++;
            continue;
        }
        if (commaCount == index) {
            if (j < sizeof(field) - 1) {
                field[j++] = s[i];
            }
        }
        if (commaCount > index) {
            break;
        }
    }
    return field;
}

/**
 * @brief 在字符串中查找 "key=value" 模式并提取value
 * @param line 输入行
 * @param key 要查找的key (如 "lat", "lon", "utc")
 * @param value 输出value缓冲区
 * @param maxLen 缓冲区最大长度
 * @return true 如果找到
 */
static bool extractKeyValue(const char* line, const char* key, char* value, size_t maxLen) {
    const char* p = strstr(line, key);
    if (p == nullptr) return false;
    
    // 确保是完整key (前面是空格或行首)
    if (p != line && *(p - 1) != ' ') return false;
    
    p += strlen(key);
    if (*p != '=') return false;
    p++;  // 跳过'='
    
    size_t i = 0;
    while (*p && *p != ' ' && *p != '\r' && *p != '\n' && i < maxLen - 1) {
        value[i++] = *p++;
    }
    value[i] = 0;
    return (i > 0);
}

/**
 * @brief 将度分格式(DDMM.MMMMM)转换为十进制度
 * 如 4000.01359 → 40 + 0.01359/60*100 = 40.000226
 */
static float degMinToDecimal(float degMin) {
    int deg = (int)(degMin / 100.0f);
    float min = degMin - deg * 100.0f;
    return deg + min / 60.0f;
}

/**
 * @brief 从字符串中安全提取2位数字（避免atoi溢出）
 */
static uint8_t parse2Digits(const char* s) {
    return ((s[0] - '0') * 10) + (s[1] - '0');
}

/**
 * @brief 从字符串中安全提取4位数字年份
 */
static uint16_t parse4Digits(const char* s) {
    return ((s[0] - '0') * 1000) + ((s[1] - '0') * 100) + 
           ((s[2] - '0') * 10) + (s[3] - '0');
}

/**
 * @brief 解析GGA语句 (标准NMEA)
 */
static void parseGGA(char* s) {
    char* field;
    
    // UTC时间 HHMMSS.sss
    field = getField(s, 1);
    if (strlen(field) >= 6) {
        gpsData.utc_hour = parse2Digits(field);
        gpsData.utc_minute = parse2Digits(field + 2);
        gpsData.utc_second = parse2Digits(field + 4);
        gpsData.flags |= GPS_FLAG_TIME_VALID;
    }
    
    // 纬度
    field = getField(s, 2);
    if (strlen(field) > 0) {
        gpsData.latitude = degMinToDecimal(atof(field));
        gpsData.flags |= GPS_FLAG_LAT_VALID;
    }
    
    // N/S
    field = getField(s, 3);
    if (strlen(field) > 0 && field[0] == 'S') {
        gpsData.latitude = -gpsData.latitude;
    }
    
    // 经度
    field = getField(s, 4);
    if (strlen(field) > 0) {
        gpsData.longitude = degMinToDecimal(atof(field));
        gpsData.flags |= GPS_FLAG_LON_VALID;
    }
    
    // E/W
    field = getField(s, 5);
    if (strlen(field) > 0 && field[0] == 'W') {
        gpsData.longitude = -gpsData.longitude;
    }
    
    // GPS质量 (0=无效, 1=GPS, 2=DGPS)
    field = getField(s, 6);
    if (strlen(field) > 0 && atoi(field) > 0) {
        gpsData.flags |= GPS_FLAG_VALID;
    }
    
    // 卫星数
    field = getField(s, 7);
    if (strlen(field) > 0) {
        gpsData.satellites = atoi(field);
        gpsData.flags |= GPS_FLAG_SIV_VALID;
    }
    
    // HDOP
    field = getField(s, 8);
    if (strlen(field) > 0) {
        gpsData.hdop = (uint16_t)(atof(field) * 100.0f + 0.5f);
    }
    
    // 高度
    field = getField(s, 9);
    if (strlen(field) > 0) {
        gpsData.altitude = atof(field);
    }
}

/**
 * @brief 解析RMC语句 (标准NMEA)
 */
static void parseRMC(char* s) {
    char* field;
    
    // UTC时间 HHMMSS.sss
    field = getField(s, 1);
    if (strlen(field) >= 6) {
        gpsData.utc_hour = parse2Digits(field);
        gpsData.utc_minute = parse2Digits(field + 2);
        gpsData.utc_second = parse2Digits(field + 4);
        gpsData.flags |= GPS_FLAG_TIME_VALID;
    }
    
    // 状态 A=有效 V=无效
    field = getField(s, 2);
    if (strlen(field) > 0 && field[0] == 'A') {
        gpsData.flags |= GPS_FLAG_VALID;
    }
    
    // 纬度
    field = getField(s, 3);
    if (strlen(field) > 0) {
        gpsData.latitude = degMinToDecimal(atof(field));
        gpsData.flags |= GPS_FLAG_LAT_VALID;
    }
    
    // N/S
    field = getField(s, 4);
    if (strlen(field) > 0 && field[0] == 'S') {
        gpsData.latitude = -gpsData.latitude;
    }
    
    // 经度
    field = getField(s, 5);
    if (strlen(field) > 0) {
        gpsData.longitude = degMinToDecimal(atof(field));
        gpsData.flags |= GPS_FLAG_LON_VALID;
    }
    
    // E/W
    field = getField(s, 6);
    if (strlen(field) > 0 && field[0] == 'W') {
        gpsData.longitude = -gpsData.longitude;
    }
    
    // 速度(节→km/h)
    field = getField(s, 7);
    if (strlen(field) > 0) {
        gpsData.speed = atof(field) * 1.852f;
    }
    
    // 航向
    field = getField(s, 8);
    if (strlen(field) > 0) {
        gpsData.course = atof(field);
    }
    
    // UTC日期 DDMMYY
    field = getField(s, 9);
    if (strlen(field) >= 6) {
        gpsData.utc_day = parse2Digits(field);
        gpsData.utc_month = parse2Digits(field + 2);
        gpsData.utc_year = parse2Digits(field + 4) + 2000;
    }
}

/**
 * @brief 解析GSV语句 (可见卫星数)
 */
static void parseGSV(char* s) {
    char* field = getField(s, 3);
    if (strlen(field) > 0) {
        uint8_t totalSats = atoi(field);
        if (totalSats > 0) {
            gpsData.satellites = totalSats;
            gpsData.flags |= GPS_FLAG_SIV_VALID;
        }
    }
}

/**
 * @brief 解析ZDA语句 (时间日期)
 * $GNZDA,103926.000,01,06,2026,00,00*46
 */
static void parseZDA(char* s) {
    char* field;
    
    // UTC时间 HHMMSS.sss
    field = getField(s, 1);
    if (strlen(field) >= 6) {
        gpsData.utc_hour = parse2Digits(field);
        gpsData.utc_minute = parse2Digits(field + 2);
        gpsData.utc_second = parse2Digits(field + 4);
        gpsData.flags |= GPS_FLAG_TIME_VALID;
    }
    
    // 日
    field = getField(s, 2);
    if (strlen(field) >= 2) {
        gpsData.utc_day = parse2Digits(field);
    }
    // 月
    field = getField(s, 3);
    if (strlen(field) >= 2) {
        gpsData.utc_month = parse2Digits(field);
    }
    // 年
    field = getField(s, 4);
    if (strlen(field) >= 4) {
        gpsData.utc_year = parse4Digits(field);
    }
}

/**
 * @brief 解析ATGM336H私有协议行
 * 格式示例:
 *   fixed=Y(1) utc=103725.000 lat=4000.09706N lon=11648.00100E
 *   utc=103724.000 year=2026 mon=06 day=01
 *   A,V*01
 */
static bool parsePrivateProtocol(char* line) {
    char val[32];
    
    // 解析 fixed=Y(n) — 定位有效
    if (extractKeyValue(line, "fixed", val, sizeof(val))) {
        // fixed=Y(1) 或 fixed=N(0)
        if (val[0] == 'Y' || val[0] == 'y') {
            gpsData.flags |= GPS_FLAG_VALID;
        } else {
            gpsData.flags &= ~GPS_FLAG_VALID;
        }
    }
    
    // 解析 utc=HHMMSS.sss
    if (extractKeyValue(line, "utc", val, sizeof(val))) {
        if (strlen(val) >= 6) {
            gpsData.utc_hour = parse2Digits(val);
            gpsData.utc_minute = parse2Digits(val + 2);
            gpsData.utc_second = parse2Digits(val + 4);
            gpsData.flags |= GPS_FLAG_TIME_VALID;
        }
    }
    
    // 解析 year=YYYY
    if (extractKeyValue(line, "year", val, sizeof(val))) {
        if (strlen(val) >= 4) {
            gpsData.utc_year = parse4Digits(val);
        }
    }
    
    // 解析 mon=MM
    if (extractKeyValue(line, "mon", val, sizeof(val))) {
        if (strlen(val) >= 2) {
            gpsData.utc_month = parse2Digits(val);
        }
    }
    
    // 解析 day=DD
    if (extractKeyValue(line, "day", val, sizeof(val))) {
        if (strlen(val) >= 2) {
            gpsData.utc_day = parse2Digits(val);
        }
    }
    
    // 解析 lat=DDMM.MMMMMN/S
    if (extractKeyValue(line, "lat", val, sizeof(val))) {
        // 去掉末尾的N/S
        size_t len = strlen(val);
        char ns = 'N';
        if (len > 0 && (val[len-1] == 'N' || val[len-1] == 'S' || 
                        val[len-1] == 'n' || val[len-1] == 's')) {
            ns = val[len-1];
            val[len-1] = 0;
        }
        gpsData.latitude = degMinToDecimal(atof(val));
        if (ns == 'S' || ns == 's') {
            gpsData.latitude = -gpsData.latitude;
        }
        gpsData.flags |= GPS_FLAG_LAT_VALID;
    }
    
    // 解析 lon=DDDMM.MMMMME/W
    if (extractKeyValue(line, "lon", val, sizeof(val))) {
        size_t len = strlen(val);
        char ew = 'E';
        if (len > 0 && (val[len-1] == 'E' || val[len-1] == 'W' || 
                        val[len-1] == 'e' || val[len-1] == 'w')) {
            ew = val[len-1];
            val[len-1] = 0;
        }
        gpsData.longitude = degMinToDecimal(atof(val));
        if (ew == 'W' || ew == 'w') {
            gpsData.longitude = -gpsData.longitude;
        }
        gpsData.flags |= GPS_FLAG_LON_VALID;
    }
    
    // 解析 alt=xxx (高度)
    if (extractKeyValue(line, "alt", val, sizeof(val))) {
        gpsData.altitude = atof(val);
    }
    
    // 解析 sat=xx (卫星数)
    if (extractKeyValue(line, "sat", val, sizeof(val))) {
        gpsData.satellites = atoi(val);
        gpsData.flags |= GPS_FLAG_SIV_VALID;
    }
    
    // 如果解析到了经纬度，标记更新时间
    if (gpsData.flags & GPS_FLAG_LAT_VALID) {
        gpsData.last_update = millis();
        return true;
    }
    
    return false;
}

/**
 * @brief 处理一行完整的数据（NMEA或私有协议）
 */
static void processLine(char* line) {
    if (strlen(line) < 3) return;
    
    // 去掉行尾的\r\n
    size_t len = strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n')) {
        line[--len] = 0;
    }
    if (len < 3) return;
    
    if (line[0] == '$') {
        // ===== 标准NMEA语句 =====
        // 去掉校验和部分
        char* star = strchr(line, '*');
        if (star != nullptr) *star = 0;
        
        // 跳过$，从第3个字符开始匹配语句类型
        char* sentence = line + 1;  // 跳过'$'
        
        // 语句类型在第3~5位 (如 $GN -> GNGGA)
        if (strlen(sentence) >= 5) {
            if (strncmp(sentence + 2, "GGA", 3) == 0) {
                parseGGA(sentence);
                lastParseCount++;
            } 
            else if (strncmp(sentence + 2, "RMC", 3) == 0) {
                parseRMC(sentence);
                lastParseCount++;
            }
            else if (strncmp(sentence + 2, "GSV", 3) == 0) {
                parseGSV(sentence);
            }
            else if (strncmp(sentence + 2, "ZDA", 3) == 0) {
                parseZDA(sentence);
            }
        }
    } else {
        // ===== ATGM336H私有协议 =====
        if (strstr(line, "lat=") != nullptr || strstr(line, "lon=") != nullptr ||
            strstr(line, "utc=") != nullptr || strstr(line, "fixed=") != nullptr ||
            strstr(line, "year=") != nullptr) {
            parsePrivateProtocol(line);
            lastParseCount++;
        }
    }
    
    lastParseTime = millis();
}

/**
 * @brief 更新GPS数据 — 从串口读取并解析
 */
bool gpsUpdate(void) {
    if (gpsSerial == nullptr) {
        return false;
    }
    
    bool gotSentence = false;
    
    while (gpsSerial->available()) {
        uint8_t c = gpsSerial->read();
        
        if (c == '\r' || c == '\n') {
            if (lineIndex > 2) {
                lineBuffer[lineIndex] = 0;
                processLine(lineBuffer);
                gotSentence = true;
            }
            lineIndex = 0;
        } else {
            if (lineIndex < NEMA_MAX_LENGTH - 1) {
                lineBuffer[lineIndex++] = c;
            }
        }
    }
    
    return gotSentence;
}

/**
 * @brief 获取当前GPS数据
 */
gps_data_t* gpsGetData(void) {
    return &gpsData;
}

/**
 * @brief 检查GPS是否有效定位
 */
bool gpsIsValid(void) {
    return (gpsData.flags & GPS_FLAG_VALID) && 
           (gpsData.flags & GPS_FLAG_LAT_VALID) && 
           (gpsData.flags & GPS_FLAG_LON_VALID);
}

/**
 * @brief 计算NEMA校验和
 */
uint8_t nmeaChecksum(const char* nmea) {
    uint8_t checksum = 0;
    while (*nmea && *nmea != '*') {
        checksum ^= *nmea++;
    }
    return checksum;
}

/**
 * @brief 初始化GPS模块
 */
void setupGPS(uint8_t rxPin, uint8_t txPin) {
    memset(&gpsData, 0, sizeof(gpsData));
    
    gpsSerial = &Serial1;
    gpsSerial->begin(9600, SERIAL_8N1, rxPin, txPin);
    
    // 增大串口接收缓冲区（默认256字节，GPS每秒约1000字节，100ms读取间隔需要至少200字节）
    gpsSerial->setRxBufferSize(1024);
    
    Serial.printf("[GPS] 初始化完成, RX=GPIO%d, TX=GPIO%d\n", rxPin, txPin);
}

/**
 * @brief 格式化纬度
 */
void formatLatitude(float lat, char ns, char* buf, size_t len) {
    float absLat = abs(lat);
    int deg = (int)absLat;
    float min = (absLat - deg) * 60.0f;
    snprintf(buf, len, "%02d°%05.2f'%c", deg, min, lat >= 0 ? ns : (ns == 'N' ? 'S' : 'N'));
}

/**
 * @brief 格式化经度
 */
void formatLongitude(float lon, char ew, char* buf, size_t len) {
    float absLon = abs(lon);
    int deg = (int)absLon;
    float min = (absLon - deg) * 60.0f;
    snprintf(buf, len, "%03d°%05.2f'%c", deg, min, lon >= 0 ? ew : (ew == 'E' ? 'W' : 'E'));
}

/**
 * @brief 创建GPS数据包
 */
void gpsCreatePacket(const gps_data_t* data, gps_packet_t* packet) {
    memset(packet, 0, sizeof(gps_packet_t));
    packet->type = GPS_PACKET_TYPE;
    packet->seq++;
    packet->latitude = data->latitude;
    packet->longitude = data->longitude;
    packet->altitude = data->altitude;
    packet->utc_year = data->utc_year;
    packet->utc_month = data->utc_month;
    packet->utc_day = data->utc_day;
    packet->utc_hour = data->utc_hour;
    packet->utc_minute = data->utc_minute;
    packet->utc_second = data->utc_second;
    packet->satellites = data->satellites;
    packet->hdop = data->hdop;
    
    // 计算校验和 (简单异或)
    packet->checksum = 0;
    for (size_t i = 0; i < offsetof(gps_packet_t, checksum); i++) {
        packet->checksum ^= ((uint8_t*)packet)[i];
    }
}

/**
 * @brief 解析GPS数据包
 */
bool gpsParsePacket(const uint8_t* data, size_t len, gps_data_t* out) {
    if (len < GPS_PACKET_SIZE) {
        return false;
    }
    
    const gps_packet_t* pkt = (const gps_packet_t*)data;
    
    // 检查类型
    if (pkt->type != GPS_PACKET_TYPE) {
        return false;
    }
    
    // 验证校验和
    uint8_t calcChecksum = 0;
    for (size_t i = 0; i < offsetof(gps_packet_t, checksum); i++) {
        calcChecksum ^= data[i];
    }
    
    if (calcChecksum != pkt->checksum) {
        Serial.printf("[GPS] 包校验和错误: 计算=0x%02X, 收到=0x%02X\n", calcChecksum, pkt->checksum);
        return false;
    }
    
    // 填充输出
    memset(out, 0, sizeof(gps_data_t));
    out->latitude = pkt->latitude;
    out->longitude = pkt->longitude;
    out->altitude = pkt->altitude;
    out->utc_year = pkt->utc_year;
    out->utc_month = pkt->utc_month;
    out->utc_day = pkt->utc_day;
    out->utc_hour = pkt->utc_hour;
    out->utc_minute = pkt->utc_minute;
    out->utc_second = pkt->utc_second;
    out->satellites = pkt->satellites;
    out->hdop = pkt->hdop;
    out->flags = GPS_FLAG_VALID | GPS_FLAG_LAT_VALID | GPS_FLAG_LON_VALID | 
                 GPS_FLAG_TIME_VALID | GPS_FLAG_SIV_VALID;
    out->last_update = millis();
    
    return true;
}
