/**
 * @file adpcm.h
 * @brief IMA ADPCM 编解码器（收发必须使用同一实现）
 *
 * 算法说明：
 * - 标准 IMA ADPCM，4 bit/采样，2 采样/字节
 * - 编解码器各自维护 valprev + index 状态，丢包后需在接收端重置
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** ADPCM 编解码器持久状态 */
typedef struct {
    int16_t valprev; /**< 上一个预测采样值 */
    int8_t index;    /**< 步长表索引 0~88 */
} adpcm_state_t;

/** 将状态清零（丢包、重新同步时调用） */
static inline void adpcm_state_reset(adpcm_state_t *state) {
    state->valprev = 0;
    state->index = 0;
}

/**
 * @brief 编码：PCM -> ADPCM
 * @param indata     输入 PCM，长度 sample_count
 * @param outdata    输出缓冲区，至少 sample_count/2 字节
 * @param sample_count 采样点数（必须为偶数）
 * @param state      跨包保持的状态
 */
void adpcm_encode(const int16_t *indata, uint8_t *outdata, int sample_count, adpcm_state_t *state);

/**
 * @brief 解码：ADPCM -> PCM
 * @param indata       输入 ADPCM 字节流
 * @param outdata      输出 PCM
 * @param sample_count 要解码的采样点数
 * @param state        跨包保持的状态
 */
void adpcm_decode(const uint8_t *indata, int16_t *outdata, int sample_count, adpcm_state_t *state);

#ifdef __cplusplus
}
#endif
