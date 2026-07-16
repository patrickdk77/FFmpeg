/*
 * Sony TRC fixed-point DSP helpers (ETSI-style)
 *
 * Copyright (c) 2026 Patrick Domack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef AVCODEC_TRC_DSP_H
#define AVCODEC_TRC_DSP_H

#include <stdint.h>

static inline int16_t trc_clip_int16(int v)
{
    if (v < -32768)
        return -32768;
    if (v > 32767)
        return 32767;
    return (int16_t)v;
}

static inline int32_t trc_sat32(int64_t acc)
{
    if (acc > 0x7fffffff)
        return 0x7fffffff;
    if (acc < (int64_t)0xffffffff80000000)
        return (int32_t)0x80000000;
    return (int32_t)acc;
}

static inline int32_t trc_l_mult(int32_t acc, int16_t a, int16_t b)
{
    return acc + (int32_t)a * b * 2;
}

static inline int16_t trc_round16(int32_t acc)
{
    return trc_clip_int16((acc + 0x8000) >> 16);
}

static inline uint32_t trc_isqrt64(uint64_t x)
{
    uint64_t r = 0;
    uint64_t bit = (uint64_t)1 << 62;

    if (!x)
        return 0;
    while (bit > x)
        bit >>= 2;
    for (; bit; bit >>= 2) {
        if (x >= r + bit) {
            x -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
    }
    return (uint32_t)r;
}

/* sqrt(x/2) in Q15: isqrt(x) * sqrt(0.5) (23170 = round(sqrt(0.5)*32768)). */
static inline uint16_t trc_sqrt_half_q15(uint32_t param)
{
    uint32_t g = trc_isqrt64((uint64_t)param);

    return (uint16_t)(((g * 23170) + 0x4000) >> 15);
}

#endif /* AVCODEC_TRC_DSP_H */
