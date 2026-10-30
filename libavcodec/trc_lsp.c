/*
 * Sony TRC LSF/LPC
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

#include "trc_lsp.h"
#include "trc_dsp.h"
#include "trcdata.h"

#include <stdint.h>
#include <string.h>

#define TRC_ORDER 10
/*
 * The reference accumulator constant 0x1800000 is Q16 (short << 16), so
 * the stored short minimum is 0x1800000 >> 16 = 0x180 = 384.
 */
#define TRC_LSF_MIN 0x180
#define TRC_LSF_MAX 0x7a80

/* LSF -> negated LSP by linear interpolation in the cosine table. */
static int16_t trc_lsf_cos_to_lsp(int16_t lsf)
{
    int64_t acc;
    int idx;
    int16_t frac;
    int cos_lo, cos_hi;

    acc = (int64_t)lsf << 16;
    acc >>= 8;
    idx = (int16_t)(trc_sat32(acc) >> 16);
    /* A failed stability pass can leave lsf[0] negative; stay inside the
     * table (the reference decoder reads whatever precedes it). */
    if (idx < 0)
        idx = 0;
    acc -= (int64_t)idx << 16;
    acc <<= 15;
    acc += 0x400000;
    frac = (int16_t)(trc_sat32(acc) >> 16);

    cos_lo = trc_lsp_cos_table[idx * 5];
    cos_hi = trc_lsp_cos_table[idx * 5 + 5];

    acc = (int64_t)(frac * cos_lo) * 2;
    acc -= (int64_t)(frac * cos_hi) * 2;
    acc -= (int64_t)cos_lo * 65536;
    acc *= 2;
    acc += 0x8000;
    acc &= ~0xffffLL;

    return (int16_t)(trc_sat32(acc) >> 16);
}

/* 32x16 Q15 multiply using the reference decoder's high/low split. */
static int64_t trc_mpy_32_16(int32_t x, int16_t l)
{
    return (int64_t)((x >> 16) * l) * 2 + (((x & 0xffff) * l) >> 15);
}

/*
 * Sum polynomials of the even (q) and odd (p) LSPs in Q24, built from the
 * first two roots of each set at once and then one root per step.
 */
static void trc_lsp_poly(int32_t q[6], int32_t p[6], const int16_t *l)
{
    int i, j;

    q[0] = p[0] = 0x1000000;
    q[1] = trc_sat32(((int64_t)l[2] * 65536 + (int64_t)l[0] * 65536) >> 6);
    p[1] = trc_sat32(((int64_t)l[3] * 65536 + (int64_t)l[1] * 65536) >> 6);
    q[2] = trc_sat32(((((int64_t)(l[2] * l[0]) * 2) >> 5) + 0x2000000) & ~1LL);
    p[2] = trc_sat32(((((int64_t)(l[3] * l[1]) * 2) >> 5) + 0x2000000) & ~1LL);

    for (i = 2; i < TRC_ORDER / 2; i++) {
        int16_t le = l[2 * i], lo = l[2 * i + 1];

        q[i + 1] = trc_sat32((trc_mpy_32_16(q[i], le) + q[i - 1]) * 2);
        p[i + 1] = trc_sat32((trc_mpy_32_16(p[i], lo) + p[i - 1]) * 2);
        for (j = i - 1; j >= 1; j--) {
            q[j + 1] = trc_sat32(trc_mpy_32_16(q[j], le) * 2 + q[j - 1] + q[j + 1]);
            p[j + 1] = trc_sat32(trc_mpy_32_16(p[j], lo) * 2 + p[j - 1] + p[j + 1]);
        }
        q[1] = trc_sat32((int64_t)q[1] + (int64_t)le * 1024);
        p[1] = trc_sat32((int64_t)p[1] + (int64_t)lo * 1024);
    }
}

/* Q24 polynomial sum -> LPC Q12 (<<4, round, >>16). */
static int16_t trc_lpc_norm_q24(int64_t v)
{
    v *= 16;
    v += 0x8000;
    v &= ~0xffffLL;
    return (int16_t)(trc_sat32(v) >> 16);
}

/*
 * LSF[] -> LPC Q12 in place.
 * Cosine lookup, then the sum/difference polynomial merge:
 *   a[i]   = norm(p[i] - p[i+1] - q[i] - q[i+1])
 *   a[9-i] = norm(p[i+1] - p[i] - q[i] - q[i+1])
 */
void ff_trc_lsf_to_lpc(int16_t lsf[TRC_ORDER])
{
    int16_t lsp[TRC_ORDER];
    int32_t q[6], p[6];
    int i;

    for (i = 0; i < TRC_ORDER; i++)
        lsp[i] = trc_lsf_cos_to_lsp(lsf[i]);

    trc_lsp_poly(q, p, lsp);

    for (i = 0; i < TRC_ORDER / 2; i++) {
        int64_t fwd = (int64_t)p[i] - p[i + 1] - q[i] - q[i + 1];
        int64_t rev = (int64_t)p[i + 1] - p[i] - q[i] - q[i + 1];

        lsf[i]                 = trc_lpc_norm_q24(fwd);
        lsf[TRC_ORDER - 1 - i] = trc_lpc_norm_q24(rev);
    }
}

/*
 * LSF stability pass.
 * Forward pass pushes pairs closer than 0x140 apart by half the shortfall.
 * Convergence test: all adjacent gaps >= 0x120.
 * On failure after 4 passes, restore the previous frame's LSF.
 */
static void trc_lsf_stabilize(int16_t lsf[TRC_ORDER], const int16_t prev[TRC_ORDER])
{
    int pass, i;

    for (pass = 0; pass < 4; pass++) {
        int converged;

        if (lsf[0] < TRC_LSF_MIN)
            lsf[0] = TRC_LSF_MIN;
        if (lsf[TRC_ORDER - 1] > TRC_LSF_MAX)
            lsf[TRC_ORDER - 1] = TRC_LSF_MAX;

        for (i = 0; i < TRC_ORDER - 1; i++) {
            /* push the pair apart by half the shortfall (rounded down) each */
            int t = (lsf[i] - lsf[i + 1] + 0x140) >> 1;

            if (t > 0) {
                lsf[i]     -= t;
                lsf[i + 1] += t;
            }
        }

        /* convergence check: 0x120 is the settled-enough threshold */
        converged = 1;
        for (i = 0; i < TRC_ORDER - 1; i++) {
            if (lsf[i + 1] - lsf[i] < 0x120) {
                converged = 0;
                break;
            }
        }
        if (converged)
            return;
    }

    /* 4 passes, still unstable: restore previous frame's LSF */
    memcpy(lsf, prev, TRC_ORDER * sizeof(int16_t));
}

void ff_trc_lsf_decode(const int16_t idx[3], const int16_t prev_lsf[TRC_ORDER],
                    int16_t lsf_out[TRC_ORDER])
{
    int i;

    memcpy(lsf_out, trc_lsp_cb0 + idx[0] * 3, 3 * sizeof(int16_t));
    memcpy(lsf_out + 3, trc_lsp_cb1 + idx[1] * 3, 3 * sizeof(int16_t));
    memcpy(lsf_out + 6, trc_lsp_cb2 + idx[2] * 4, 4 * sizeof(int16_t));

    /*
     * MA-predicted dequantization: (ma_coef * (prev - mean)) * 2 added to the
     * accumulator, then the high 16 bits extracted with rounding.  Do NOT add
     * an extra 0x4000 here; trc_round16 supplies the 0x8000 rounding.
     */
    for (i = 0; i < TRC_ORDER; i++) {
        int32_t pred = (int32_t)(prev_lsf[i] - trc_lsf_mean[i]) *
                       trc_lsp_weights[i] * 2;
        int32_t acc  = ((int32_t)lsf_out[i] + trc_lsf_mean[i]) << 16;

        acc += pred;
        lsf_out[i] = trc_round16(acc);
    }

    trc_lsf_stabilize(lsf_out, prev_lsf);
}

void ff_trc_lsf_interpolate(int16_t lsp_out[TRC_ORDER], const int16_t cur[TRC_ORDER],
                         const int16_t prev[TRC_ORDER], int nsub, int sub_idx)
{
    int i;
    int w;

    if (nsub <= 1) {
        memcpy(lsp_out, cur, TRC_ORDER * sizeof(int16_t));
        return;
    }

    /*
     * Subframe interpolation weight w = (sub_idx + 1) * (32768 / nsub) in Q15:
     * out[i] = round16(prev[i]<<16 + (cur[i]-prev[i]) * w * 2)
     */
    w = (sub_idx + 1) * (32768 / nsub);

    for (i = 0; i < TRC_ORDER; i++) {
        int32_t acc = ((int32_t)prev[i] << 16);

        acc += ((int32_t)(cur[i] - prev[i]) * w * 2);
        lsp_out[i] = trc_round16(acc);
    }
}
