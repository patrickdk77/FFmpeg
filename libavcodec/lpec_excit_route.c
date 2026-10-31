/*
 * LPEC excitation route.
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
#include "lpec_excit_route.h"
#include <string.h>

#define LPEC_INV32768 (1.0 / 32768.0)

static void lpec_stabilize_lsf_i16(int16_t *lsf, int order)
{
    int half = order >> 1;
    int i;
    uint16_t hi = 0x7d70;

    if (half < order) {
        for (i = order; i > half; i--) {
            if ((int16_t) lsf[i] < 0)
                lsf[i] = (int16_t) hi;
            hi = (uint16_t) lsf[i] - 0x28f;
        }
    }
    hi = 0x28f;
    for (i = 1; i <= half; i++) {
        if ((int16_t) lsf[i] < 0)
            lsf[i] = (int16_t) hi;
        hi = (uint16_t) lsf[i] + 0x28f;
    }
    lsf[0] = 0;
    for (i = 1; i <= order; i++) {
        if (lsf[i] < lsf[i - 1]) {
            int16_t t = lsf[i - 1];
            lsf[i - 1] = lsf[i];
            lsf[i]     = t;
            if (i >= 2 && lsf[i - 1] < lsf[i - 2]) {
                t = lsf[i - 1];
                lsf[i - 1] = lsf[i - 2];
                lsf[i - 2] = t;
            }
        }
    }
    if (lsf[1] < 0x28f)
        lsf[1] = 0x28f;
    for (i = 2; i <= order; i++) {
        if (lsf[i] - lsf[i - 1] < 0x28f) {
            uint32_t u1 = (uint16_t) lsf[i - 1] + 0x290 + (uint16_t) lsf[i];
            uint32_t u2 = (uint16_t) lsf[i] - 0x28e + (uint16_t) lsf[i - 1];
            lsf[i]     = (int16_t) (u1 >> 1);
            lsf[i - 1] = (int16_t) (u2 >> 1);
        }
    }
    lsf[0] = 0;
}

void ff_lpec_lsf_indices_to_i16(int num_bands, int order, int num_cb,
                             const int *idx, int16_t *lsf_out)
{
    const int16_t *q1, *q2, *q3, *q4;
    int i;

    lsf_out[0] = 0;
    if (num_bands == 10) {
        q1 = lpec_16k_lsf_i16_q1;
        q2 = lpec_16k_lsf_i16_q2;
        q3 = lpec_16k_lsf_i16_q3;
        q4 = lpec_16k_lsf_i16_q4;
    } else {
        q1 = lpec_8k_lsf_i16_q1;
        q2 = lpec_8k_lsf_i16_q2;
        q3 = lpec_8k_lsf_i16_q3;
        q4 = NULL;
    }

    for (i = 0; i < order; i++) {
        int16_t s1 = num_cb > 0 ? q1[idx[0] * order + i] : 0;
        int16_t s2 = num_cb > 1 ? q2[idx[1] * order + i] : 0;
        int16_t s3 = num_cb > 2 ? q3[idx[2] * order + i] : 0;
        int16_t s4 = (num_cb > 3 && q4) ? q4[idx[3] * order + i] : 0;

        /* 4 CB -> q1 + (q2+q3+q4)>>3; else (q3+q2*2+q1*4)>>2. */
        if (num_cb >= 4 && q4)
            lsf_out[i + 1] = (int16_t) (s1 + (int16_t) (((int) s2 + (int) s3 + (int) s4) >> 3));
        else
            lsf_out[i + 1] = (int16_t) (((int) s3 + (int) s2 * 2 + (int) s1 * 4) >> 2);
    }
    lsf_out[0] = 0;
    lpec_stabilize_lsf_i16(lsf_out, order);
}

static int lpec_autocorr_scale(int *out_m, uint32_t *out_l,
                               int16_t *acf_out, const int16_t *lpc,
                               int order)
{
    uint32_t sum_l = 0;
    int sum_h = 0;
    const int16_t *ps;
    uint32_t mag;
    int shift;
    int iexp, i, n;
    uint32_t lo;
    int hi;

    for (ps = lpc; ps <= lpc + order - 1; ps += 2) {
        int32_t a = (int32_t) ps[0] * ps[0];
        int32_t b = (int32_t) ps[1] * ps[1];
        sum_l += (uint32_t) a & 0x7fff;
        sum_h += (a >> 15) + (b >> 15);
        sum_l += (uint32_t) b & 0x7fff;
    }
    for (; ps <= lpc + order; ps++) {
        int32_t a = (int32_t) ps[0] * ps[0];
        sum_l += (uint32_t) a & 0x7fff;
        sum_h += a >> 15;
    }

    mag = (uint32_t) sum_h + (sum_l >> 15);
    if ((int32_t) mag < 0)
        mag = -mag;

    {
        int lead = 0, width = 0;
        uint32_t probe = mag;

        for (;;) {
            if (probe & 0x40000) {
                width = 0x13 - lead;
                break;
            }
            if (lead + 1 > 0x13)
                break;
            if ((probe * 2) & 0x40000) {
                width = 0x13 - (lead + 1);
                break;
            }
            if (lead + 2 > 0x13)
                break;
            if ((probe * 4) & 0x40000) {
                width = 0x13 - (lead + 2);
                break;
            }
            lead += 3;
            probe *= 8;
            if (lead >= 0x14)
                break;
        }
        shift = width;
        iexp = width - 10;
    }
    lo = sum_l & 0x7fff;
    hi = sum_h + (sum_l >> 15);

    if (iexp < 1) {
        if (iexp < 0) {
            uint32_t t = lo >> (shift + 5 & 31);
            lo &= ((1U << ((shift + 5) & 31)) - 1);
            hi = (hi << ((-iexp) & 31)) | t;
        }
    } else {
        lo |= (((1U << (iexp & 31)) - 1) & (uint32_t) hi) << 15;
        if (shift - 0xe > 0)
            lo >>= (shift - 0xe) & 31;
        hi >>= iexp & 31;
    }

    *out_m = hi + 2;
    *out_l = lo;

    if (order <= 0)
        return shift;

    for (n = 1; n <= order; n++) {
        uint32_t sl = 0;
        int sh = 0;
        const int16_t *p = lpc;
        int end = order - n;

        for (i = 0; i <= end; i++) {
            int32_t pprod = (int32_t) p[i] * p[i + n];
            sl += pprod & ((1 << shift) - 1);
            sh += pprod >> shift;
        }
        acf_out[n] = (int16_t) (sh + (int16_t) (sl >> shift));
    }
    return shift;
}

#if defined(__GNUC__)
#pragma GCC push_options
#pragma GCC optimize("O0")
#endif

static int lpec_rt_cos_idx(int ipart)
{
    int32_t v = ipart;
    int32_t t = v + 0x200;

    t >>= 10;
    t = (uint32_t) t >> 21;
    t = t + v + 0x200;
    t &= 0xfffff800;
    t = -t + v + 0x200;
    return (int) t;
}

static int16_t lpec_rt_interp_i16(int16_t in)
{
    int32_t phase = (int32_t) in << 11;
    int ipart = phase >> 16;
    int frac = (phase & 0xffff) >> 1;
    int idx = lpec_rt_cos_idx(ipart);

    return (int16_t) (((0x8000 - frac) * lpec_rtable_a[idx]
                       + lpec_rtable_a[idx + 1] * frac) >> 15);
}

#if defined(__GNUC__)
#pragma GCC pop_options
#endif

static int lpec_fp_mul2(uint32_t a, int16_t s)
{
    int32_t hi = (int32_t) a >> 16;
    int32_t lo = (uint32_t) (a & 0xffff);
    int32_t v = s;

    lo = lo * v;
    hi = hi * v;
    lo >>= 15;
    lo = lo + hi * 2;
    return lo * 2;
}

static int lpec_half_order(int order)
{
    uint32_t v = (uint32_t) order;

    v += 0x80000000u;
    v += 0x80000000u + (v < 0x80000000u);
    return (int32_t) v >> 1;
}

static void lpec_store_lsf_cos(uint8_t *scratch, const int16_t *lsf_in, int order)
{
    int idx, limit;

    if (order < 0)
        return;

    /* Store cos(lsf_in[i]) as int16 at byte 0x90 + 2 * i of the scratch
     * buffer. For an even order the paired loop stops at order - 2, so
     * lsf_in[order] is stored on its own. */
    limit = ((order + 1) & ~1) - 1;
    for (idx = 0; idx <= limit; idx += 2) {
        *(int16_t *) (scratch + idx * 2 + 0x90) = lpec_rt_interp_i16(lsf_in[idx]);
        if (idx + 1 <= order)
            *(int16_t *) (scratch + idx * 2 + 0x92) = lpec_rt_interp_i16(lsf_in[idx + 1]);
    }
    for (idx = limit + 2; idx <= order; idx++)
        *(int16_t *) (scratch + idx * 2 + 0x90) = lpec_rt_interp_i16(lsf_in[idx]);
    /* Even order: paired loop stops at order-2; k=order/2 reads scratch+order*2+0x90. */
    if (!(order & 1))
        *(int16_t *) (scratch + order * 2 + 0x90) = lpec_rt_interp_i16(lsf_in[order]);
}

#if defined(__GNUC__)
#pragma GCC push_options
#pragma GCC optimize("O0")
#endif

/* One 0x1a4-byte buffer; the two polynomials, the cosines and the two
 * product arrays sit at fixed offsets in it. */
int ff_lpec_lsf_to_lpc_i16(const int16_t *lsf_in, int16_t *out, int order)
{
    uint8_t scratch[0x1a4];
    uint32_t *q_poly = (uint32_t *) scratch;
    uint32_t *p_poly = (uint32_t *) (scratch + 0x48);
    int *p_prod = (int *) (scratch + 0xb8);
    int *q_prod = (int *) (scratch + 0x100);
    uint32_t lpc_acc[18];
    uint32_t uacc;
    int half, k, j, i, ret;
    int shift_pos, iexp, round_bias, idx, half_limit;
    uint32_t u;

    if (order < 0)
        return 0;

    memset(scratch, 0, sizeof(scratch));
    lpec_store_lsf_cos(scratch, lsf_in, order);

    p_poly[1] = 0x800000;
    q_poly[1] = 0xff800000;
    p_poly[0] = q_poly[0] = 0xff800000;

    half = lpec_half_order(order);
    if (half > 0) {
        for (k = 1; k <= half; k++) {
            int16_t as_even = *(int16_t *) (scratch + k * 4 + 0x90);
            int16_t as_odd  = *(int16_t *) (scratch + k * 4 + 0x8e);

            p_prod[0] = lpec_fp_mul2(p_poly[0], as_even);
            q_prod[0] = lpec_fp_mul2(q_poly[0], as_odd);

            if (k >= 2) {
                uint8_t *cursor = scratch + 0x08;
                uint8_t *end    = scratch + k * 4;

                while (cursor <= end) {
                    *(int *) (cursor + 0xb4) =
                        lpec_fp_mul2(*(uint32_t *) (cursor + 0x44), as_even)
                        - (int) *(uint32_t *) (cursor + 0x40);
                    *(int *) (cursor + 0xfc) =
                        lpec_fp_mul2(*(uint32_t *) (cursor - 4), as_odd)
                        - (int) *(uint32_t *) (cursor - 8);
                    cursor += 4;
                }
            }

            for (j = 1; j <= k; j++) {
                uint32_t old_b = p_poly[j];
                int mir = (k << 1) + 1 - j;
                int p_sub = p_prod[j - 1];
                int q_sub = q_prod[j - 1];

                p_poly[j] -= p_sub;
                q_poly[j] -= q_sub;
                p_poly[mir] = (uint32_t) (-(int32_t) (old_b - p_sub));
                q_poly[mir] = q_poly[j];
            }
        }
    }

    uacc = 0;
    for (i = 0; i <= order; i++) {
        int32_t neg_sum = -(int32_t) (q_poly[i] + p_poly[i]);
        int32_t mag = neg_sum;

        lpc_acc[i] = (uint32_t) neg_sum;
        if (mag < 0)
            mag = -mag;
        uacc |= (uint32_t) mag;
    }

    /* test uacc, then up to three increments and shifts per outer step. */
    shift_pos = 0;
    while (shift_pos < 0x10) {
        if (uacc & 0x40000000u)
            break;
        shift_pos++;
        uacc <<= 1;
        if (shift_pos >= 0x10)
            break;
        if (uacc & 0x40000000u)
            break;
        shift_pos++;
        uacc <<= 1;
        if (shift_pos >= 0x10)
            break;
        if (uacc & 0x40000000u)
            break;
        shift_pos++;
        uacc <<= 1;
    }

    iexp       = 0x10 - shift_pos;
    ret        = 7 - shift_pos;
    round_bias = 1 << ((0xf - shift_pos) & 31);

    half_limit = ((order + 1) & ~1) - 1;
    idx = 0;
    if (half_limit >= 0) {
        do {
            u = lpc_acc[idx] + round_bias;
            out[idx] = (int16_t) ((int32_t) u >> (iexp & 31));
            lpc_acc[idx] = u;
            u = lpc_acc[idx + 1] + round_bias;
            out[idx + 1] = (int16_t) ((int32_t) u >> (iexp & 31));
            lpc_acc[idx + 1] = u;
            idx += 2;
        } while (idx <= half_limit);
    }
    for (; idx <= order; idx++) {
        u = lpc_acc[idx] + round_bias;
        out[idx] = (int16_t) ((int32_t) u >> (iexp & 31));
        lpc_acc[idx] = u;
    }

    return ret;
}

#if defined(__GNUC__)
#pragma GCC pop_options
#endif

static int lpec_envelope_route_tables(const int16_t *lpc_i16, int pitch_win, double pitch_e,
                           int32_t *route_out, int pitch_len, int order,
                           const int16_t *rt_a, const int16_t *rt_b)
{
    int16_t acf[20];
    int m;
    uint32_t l;
    int16_t pitch_i16;
    int pos, quarter, half, tbl_stride;
    const int16_t *rt;

    if (pitch_len % 3 == 0) {
        tbl_stride = 0x600 / pitch_len;
        rt = rt_b;
    } else {
        tbl_stride = 0x800 / pitch_len;
        rt = rt_a;
    }

    half    = pitch_len >> 1;
    quarter = pitch_len >> 2;

    if (pitch_e >= 0)
        pitch_i16 = (int16_t) (pitch_e * 32768.0 + 0.5);
    else
        pitch_i16 = (int16_t) (pitch_e * 32768.0 - 0.5);

    lpec_autocorr_scale(&m, &l, acf, lpc_i16, order);

    if (!pitch_win) {
        for (pos = 0; pos < half; pos++) {
            int phase = quarter + pos;
            uint32_t sl = l;
            int sh = m;
            int k;

            for (k = 1; k <= order; k++) {
                int idx = tbl_stride * (phase % pitch_len);
                int32_t prod = (int32_t) acf[k] * rt[idx];
                sl += prod & 0x7ffff;
                sh += prod >> 19;
                phase += pos;
            }
            route_out[pos] = sh * 0x2000 + (sl >> 6);
        }
    } else {
        int pitch_q15 = pitch_i16;
        int pitch_pos = quarter;
        int step = 0;

        for (pos = 0; pos < half; pos++) {
            int phase = step + quarter;
            uint32_t sl = l;
            int sh = m;
            int k, weight;

            for (k = 1; k <= order; k++) {
                int idx = tbl_stride * (phase % pitch_len);
                int32_t prod = (int32_t) acf[k] * rt[idx];
                sl += prod & 0x7ffff;
                sh += prod >> 19;
                phase += step;
            }
            weight = (pitch_q15 * (((pitch_q15 - 2 * rt[tbl_stride * (pitch_pos % pitch_len)]) >> 2))
                      + 0x10000000) >> 15;
            weight = (((sl & 0x7ffff) >> 4) * weight >> 14) + (sh + (sl >> 19)) * weight * 2;
            route_out[pos] = weight;
            step++;
            pitch_pos += pitch_win;
        }
    }
    return 0;
}

int ff_lpec_envelope_route(const int16_t *lpc_i16, int pitch_win, double pitch_e,
                    int32_t *route_out, int pitch_len, int order)
{
    return lpec_envelope_route_tables(lpc_i16, pitch_win, pitch_e, route_out,
                                      pitch_len, order,
                                      lpec_rtable_a, lpec_rtable_b);
}
