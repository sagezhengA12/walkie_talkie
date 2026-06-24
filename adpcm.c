/**
 * @file adpcm.c
 * @brief IMA ADPCM 编解码实现
 */
#include "adpcm.h"

static const int indexTable[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8,
};

static const int stepsizeTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

void adpcm_encode(const int16_t *indata, uint8_t *outdata, int sample_count, adpcm_state_t *state) {
    int valpred = state->valprev;
    int index = state->index;
    int step = stepsizeTable[index];
    int bufferstep = 0;
    int pending_nibble = 0;
    uint8_t *outp = outdata;

    for (int i = 0; i < sample_count; i++) {
        int diff = (int)indata[i] - valpred;
        int sign = 0;
        if (diff < 0) {
            sign = 8;
            diff = -diff;
        }

        int delta = 0;
        int tempstep = step;
        if (diff >= tempstep) {
            delta = 4;
            diff -= tempstep;
        }
        tempstep >>= 1;
        if (diff >= tempstep) {
            delta |= 2;
            diff -= tempstep;
        }
        tempstep >>= 1;
        if (diff >= tempstep) {
            delta |= 1;
        }
        delta |= sign;

        if (bufferstep) {
            *outp++ = (uint8_t)((pending_nibble << 4) | (delta & 0x0f));
        } else {
            pending_nibble = delta & 0x0f;
        }
        bufferstep = !bufferstep;

        int vpdiff = step >> 3;
        if (delta & 4) {
            vpdiff += step;
        }
        if (delta & 2) {
            vpdiff += step >> 1;
        }
        if (delta & 1) {
            vpdiff += step >> 2;
        }
        if (sign) {
            valpred -= vpdiff;
        } else {
            valpred += vpdiff;
        }
        if (valpred > 32767) {
            valpred = 32767;
        } else if (valpred < -32768) {
            valpred = -32768;
        }

        index += indexTable[delta & 7];
        if (index < 0) {
            index = 0;
        } else if (index > 88) {
            index = 88;
        }
        step = stepsizeTable[index];
    }

    state->valprev = (int16_t)valpred;
    state->index = (int8_t)index;
}

void adpcm_decode(const uint8_t *indata, int16_t *outdata, int sample_count, adpcm_state_t *state) {
    const uint8_t *inp = indata;
    int valpred = state->valprev;
    int index = state->index;
    int step = stepsizeTable[index];
    int bufferstep = 0;
    int inputbuffer = 0;

    for (int i = 0; i < sample_count; i++) {
        int delta;
        if (bufferstep) {
            delta = inputbuffer & 0x0f;
        } else {
            inputbuffer = *inp++;
            delta = (inputbuffer >> 4) & 0x0f;
        }
        bufferstep = !bufferstep;

        index += indexTable[delta];
        if (index < 0) {
            index = 0;
        } else if (index > 88) {
            index = 88;
        }

        int sign = delta & 8;
        delta = delta & 7;

        int vpdiff = step >> 3;
        if (delta & 4) {
            vpdiff += step;
        }
        if (delta & 2) {
            vpdiff += step >> 1;
        }
        if (delta & 1) {
            vpdiff += step >> 2;
        }

        if (sign) {
            valpred -= vpdiff;
        } else {
            valpred += vpdiff;
        }
        if (valpred > 32767) {
            valpred = 32767;
        } else if (valpred < -32768) {
            valpred = -32768;
        }

        step = stepsizeTable[index];
        outdata[i] = (int16_t)valpred;
    }

    state->valprev = (int16_t)valpred;
    state->index = (int8_t)index;
}
