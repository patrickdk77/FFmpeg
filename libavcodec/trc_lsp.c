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
#define TRC_LSP_FRAC_BITS 14
/*
 * The reference accumulator constant 0x1800000 is Q16 (short << 16), so
 * the stored short minimum is 0x1800000 >> 16 = 0x180 = 384.
 */
#define TRC_LSF_MIN 0x180
#define TRC_LSF_MAX 0x7a80

static int32_t trc_lsp_mull(int32_t a, int32_t b)
{
    /*
     * The recurrence multiply rounds (not truncates) the Q14 product.
     * Truncating made interpolated-subframe LPC coefs land 1 LSB low at
     * rounding boundaries, which drifted the synthesis state in quiet regions.
     */
    return (int32_t)((((int64_t)a * b) + (1 << (TRC_LSP_FRAC_BITS - 1))) >> TRC_LSP_FRAC_BITS);
}

static int32_t trc_lsf_cos_to_lsp(int16_t lsf)
{
    int64_t acc;
    int idx;
    int16_t frac;
    int cos_lo, cos_hi;

    acc = (int64_t)lsf << 16;
    acc >>= 8;
    idx = (int16_t)(acc >> 16);
    acc -= (int64_t)idx << 16;
    acc <<= 15;
    acc += 0x400000;
    frac = (int16_t)(acc >> 16);

    cos_lo = trc_lsp_cos_table[idx * 5];
    cos_hi = trc_lsp_cos_table[idx * 5 + 5];

    acc = (int64_t)frac * cos_lo * 2;
    acc -= (int64_t)frac * cos_hi * 2;
    acc -= (int64_t)cos_lo << 16;
    acc <<= 1;
    acc += 0x8000;
    acc &= ~0xffff;

    return (int16_t)(acc >> 16);
}

/*
 * G.729-style LSP polynomial (lsp.c lsp2poly), Q22 coefficients.
 * lsp[] is cosine-domain values in Q15 from trc_lsf_cos_to_lsp.
 */
static void trc_lsp2poly(int32_t f[6], const int16_t *lsp, int half)
{
    int i, j;

    f[0] = 0x400000;
    f[1] = -(int32_t)lsp[0] * 256;

    for (i = 2; i <= half; i++) {
        f[i] = f[i - 2];
        for (j = i; j > 1; j--)
            f[j] -= trc_lsp_mull(f[j - 1], lsp[2 * i - 2]) - f[j - 2];
        f[1] -= (int32_t)lsp[2 * i - 2] * 256;
    }
}

/* Final normalization: net >>10 from the Q22 polynomial (<<6 + round >>16). */
static int16_t trc_lpc_norm_q22(int32_t v)
{
    int64_t acc = (int64_t)v << 6;

    acc += 0x8000;
    acc &= ~0xffffLL;
    return trc_clip_int16((int32_t)(acc >> 16));
}

/*
 * LSF[] -> LPC Q12 in place.
 * Cosine lookup table plus a G.729-style polynomial merge recurrence with the
 * reference decoder's final normalization.
 */
void trc_lsf_to_lpc(int16_t lsf[TRC_ORDER])
{
    int16_t lsp[TRC_ORDER];
    int32_t f1[6], f2[6];
    int i;

    for (i = 0; i < TRC_ORDER; i++)
        lsp[i] = (int16_t)-trc_lsf_cos_to_lsp(lsf[i]);

    trc_lsp2poly(f1, lsp,     TRC_ORDER / 2);
    trc_lsp2poly(f2, lsp + 1, TRC_ORDER / 2);

    /*
     * Final merge: for each pair i,
     *   a[i]   = round((f2[i] - f2[i+1] - f1[i] - f1[i+1]) << 6) >> 16
     *   a[9-i] = round((f2[i+1] - f2[i] - f1[i] - f1[i+1]) << 6) >> 16
     */
    for (i = 0; i < TRC_ORDER / 2; i++) {
        int32_t fwd = f2[i] - f2[i + 1] - f1[i] - f1[i + 1];
        int32_t rev = f2[i + 1] - f2[i] - f1[i] - f1[i + 1];

        lsf[i]                 = trc_lpc_norm_q22(fwd);
        lsf[TRC_ORDER - 1 - i] = trc_lpc_norm_q22(rev);
    }
}

/*
 * LSF stability pass.
 * Forward pass enforces min spacing 0x140 via midpoint spread.
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
            if (lsf[i + 1] - lsf[i] < 0x140) {
                int16_t mid = (lsf[i] + lsf[i + 1]) >> 1;

                lsf[i]     = mid - 0xa0;
                lsf[i + 1] = mid + 0xa0;
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

void trc_lsf_decode(const int16_t idx[3], const int16_t prev_lsf[TRC_ORDER],
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

void trc_lsf_interpolate(int16_t lsp_out[TRC_ORDER], const int16_t cur[TRC_ORDER],
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
