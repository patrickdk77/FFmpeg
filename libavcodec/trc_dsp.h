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

/*
 * 15-step binary search for the largest r such that (int64_t)2*r*r <= x.
 * This is floor(sqrt(x/2)), not floor(sqrt(x)); the sqrt(2) factor cancels
 * in the in/out ratio but the quantization grid does not, so the exact
 * algorithm matters for the AGC target and the pitch-filter gain.
 */
static inline int16_t trc_sqrt_q15(int32_t x)
{
    int32_t root = 0;
    int32_t bit  = 0x4000;
    int i;

    for (i = 0; i < 15; i++) {
        int32_t trial = root + bit;
        int64_t sq = ((int64_t)trial * trial) << 1;

        if (sq <= (int64_t)x)
            root = trial;
        bit >>= 1;
    }
    return (int16_t)root;
}

#endif /* AVCODEC_TRC_DSP_H */
