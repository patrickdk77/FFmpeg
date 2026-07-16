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

void lpec_lsf_indices_to_i16(int num_bands, int order, int num_cb,
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

static int lpec_autocorr_scale_c450(int *out_m, uint32_t *out_l,
                                    int16_t *acf_out, const int16_t *lsf,
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

    for (ps = lsf; ps <= lsf + order - 1; ps += 2) {
        int32_t a = (int32_t) ps[0] * ps[0];
        int32_t b = (int32_t) ps[1] * ps[1];
        sum_l += (uint32_t) a & 0x7fff;
        sum_h += (a >> 15) + (b >> 15);
        sum_l += (uint32_t) b & 0x7fff;
    }
    for (; ps <= lsf + order; ps++) {
        int32_t a = (int32_t) ps[0] * ps[0];
        sum_l += (uint32_t) a & 0x7fff;
        sum_h += a >> 15;
    }

    mag = (uint32_t) sum_h + (sum_l >> 15);
    if ((int32_t) mag < 0)
        mag = -mag;

    {
        int i6 = 0, i11 = 0;
        uint32_t u5 = mag;

        for (;;) {
            if (u5 & 0x40000) {
                i11 = 0x13 - i6;
                break;
            }
            if (i6 + 1 > 0x13)
                break;
            if ((u5 * 2) & 0x40000) {
                i11 = 0x13 - (i6 + 1);
                break;
            }
            if (i6 + 2 > 0x13)
                break;
            if ((u5 * 4) & 0x40000) {
                i11 = 0x13 - (i6 + 2);
                break;
            }
            i6 += 3;
            u5 *= 8;
            if (i6 >= 0x14)
                break;
        }
        shift = i11;
        iexp = i11 - 10;
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
        const int16_t *p = lsf;
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

int lpec_autocorr_scale_c450_test(int *out_m, uint32_t *out_l,
                                  int16_t *acf_out, const int16_t *lsf,
                                  int order)
{
    return lpec_autocorr_scale_c450(out_m, out_l, acf_out, lsf, order);
}

#if defined(__GNUC__)
#pragma GCC push_options
#pragma GCC optimize("O0")
#endif

static int lpec_rt_idx_from_i5(int i5)
{
    int32_t eax = i5;
    int32_t esi = eax + 0x200;

    /* sar 0xa; shr 0x15; and 0xfffff800; neg; lea esi,[esi+eax+0x200] */
    esi >>= 10;
    esi = (uint32_t) esi >> 21;
    esi = esi + eax + 0x200;
    esi &= 0xfffff800;
    esi = -esi + eax + 0x200;
    return (int) esi;
}

static int16_t lpec_rt_interp_i16(int16_t in)
{
    int32_t u4 = (int32_t) in << 11;
    int i5 = u4 >> 16;
    int i8 = (u4 & 0xffff) >> 1;
    int idx = lpec_rt_idx_from_i5(i5);

    return (int16_t) (((0x8000 - i8) * lpec_rtable_a[idx]
                       + lpec_rtable_a[idx + 1] * i8) >> 15);
}

#if defined(__GNUC__)
#pragma GCC pop_options
#endif

static int lpec_fp_mul2(uint32_t a, int16_t s)
{
    int32_t edx = (int32_t) a >> 16;
    int32_t ebx = (uint32_t) (a & 0xffff);
    int32_t eax = s;

    ebx = ebx * eax;
    edx = edx * eax;
    ebx >>= 15;
    ebx = ebx + edx * 2;
    return ebx * 2;
}

static int lpec_a110_dll_half(int order)
{
    uint32_t eax = (uint32_t) order;

    eax += 0x80000000u;
    eax += 0x80000000u + (eax < 0x80000000u);
    return (int32_t) eax >> 1;
}

static void lpec_a110_phase1_stack(uint8_t *stk, const int16_t *lsf_in, int order)
{
    int edx, limit;

    if (order < 0)
        return;

    /* Phase1 writes rt-interp AS words at stk+edx*2+0x90. For even order the
 * paired loop ends at order-2; k=order/2 reads stk+order*2+0x90 (e.g. @b0).*/
    limit = ((order + 1) & ~1) - 1;
    for (edx = 0; edx <= limit; edx += 2) {
        *(int16_t *) (stk + edx * 2 + 0x90) = lpec_rt_interp_i16(lsf_in[edx]);
        if (edx + 1 <= order)
            *(int16_t *) (stk + edx * 2 + 0x92) = lpec_rt_interp_i16(lsf_in[edx + 1]);
    }
    for (edx = limit + 2; edx <= order; edx++)
        *(int16_t *) (stk + edx * 2 + 0x90) = lpec_rt_interp_i16(lsf_in[edx]);
    /* Even order: paired loop stops at order-2; k=order/2 reads stk+order*2+0x90. */
    if (!(order & 1))
        *(int16_t *) (stk + order * 2 + 0x90) = lpec_rt_interp_i16(lsf_in[order]);
}

#if defined(__GNUC__)
#pragma GCC push_options
#pragma GCC optimize("O0")
#endif

/* single 0x1a4-byte stack frame; AS/lattice/path arrays overlap like the reference decoder. */
int lpec_a110_route_shorts(const int16_t *lsf_in, int16_t *out, int order)
{
    uint8_t stk[0x1a4];
    uint32_t *local_1b4 = (uint32_t *) stk;
    uint32_t *local_16c = (uint32_t *) (stk + 0x48);
    int *aiStack_b8  = (int *) (stk + 0xb8);
    int *aiStack_100 = (int *) (stk + 0x100);
    uint32_t local_70[18];
    uint32_t uacc;
    int half, k, j, i, ret;
    int shift_pos, iexp, i8, edx, half_limit;
    uint32_t u;

    if (order < 0)
        return 0;

    memset(stk, 0, sizeof(stk));
    lpec_a110_phase1_stack(stk, lsf_in, order);

    local_16c[1] = 0x800000;
    local_1b4[1] = 0xff800000;
    local_16c[0] = local_1b4[0] = 0xff800000;

    half = lpec_a110_dll_half(order);
    if (half > 0) {
        for (k = 1; k <= half; k++) {
            int16_t as_even = *(int16_t *) (stk + k * 4 + 0x90);
            int16_t as_odd  = *(int16_t *) (stk + k * 4 + 0x8e);

            aiStack_b8[0]  = lpec_fp_mul2(local_16c[0], as_even);
            aiStack_100[0] = lpec_fp_mul2(local_1b4[0], as_odd);

            if (k >= 2) {
                uint8_t *pu  = stk + 0x08;
                uint8_t *end = stk + k * 4;

                while (pu <= end) {
                    *(int *) (pu + 0xb4) =
                        lpec_fp_mul2(*(uint32_t *) (pu + 0x44), as_even)
                        - (int) *(uint32_t *) (pu + 0x40);
                    *(int *) (pu + 0xfc) =
                        lpec_fp_mul2(*(uint32_t *) (pu - 4), as_odd)
                        - (int) *(uint32_t *) (pu - 8);
                    pu += 4;
                }
            }

            for (j = 1; j <= k; j++) {
                uint32_t old_b = local_16c[j];
                int mir = (k << 1) + 1 - j;
                int sb = aiStack_b8[j - 1];
                int s100 = aiStack_100[j - 1];

                local_16c[j] -= sb;
                local_1b4[j] -= s100;
                local_16c[mir] = (uint32_t) (-(int32_t) (old_b - sb));
                local_1b4[mir] = local_1b4[j];
            }
        }
    }

    uacc = 0;
    for (i = 0; i <= order; i++) {
        int32_t neg_sum = -(int32_t) (local_1b4[i] + local_16c[i]);
        int32_t mag = neg_sum;

        local_70[i] = (uint32_t) neg_sum;
        if (mag < 0)
            mag = -mag;
        uacc |= (uint32_t) mag;
    }

    /* shift_pos in edx; test uacc, then up to three +1/shl per outer step. */
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

    iexp = 0x10 - shift_pos;
    ret  = 7 - shift_pos;
    i8   = 1 << ((0xf - shift_pos) & 31);

    half_limit = ((order + 1) & ~1) - 1;
    edx = 0;
    if (half_limit >= 0) {
        do {
            u = local_70[edx] + i8;
            out[edx] = (int16_t) ((int32_t) u >> (iexp & 31));
            local_70[edx] = u;
            u = local_70[edx + 1] + i8;
            out[edx + 1] = (int16_t) ((int32_t) u >> (iexp & 31));
            local_70[edx + 1] = u;
            edx += 2;
        } while (edx <= half_limit);
    }
    for (; edx <= order; edx++) {
        u = local_70[edx] + i8;
        out[edx] = (int16_t) ((int32_t) u >> (iexp & 31));
        local_70[edx] = u;
    }

    return ret;
}

#if defined(__GNUC__)
#pragma GCC pop_options
#endif

/* the reference decoder 0b8d0 (same algorithm, different route tables). */
int lpec_c680_route_tables(const int16_t *a110_out, int pitch_win, double pitch_e,
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

    lpec_autocorr_scale_c450(&m, &l, acf, a110_out, order);

    if (!pitch_win) {
        for (pos = 0; pos < half; pos++) {
            int p0 = quarter + pos;
            uint32_t sl = l;
            int sh = m;
            int k;

            for (k = 1; k <= order; k++) {
                int idx = tbl_stride * (p0 % pitch_len);
                int32_t prod = (int32_t) acf[k] * rt[idx];
                sl += prod & 0x7ffff;
                sh += prod >> 19;
                p0 += pos;
            }
            route_out[pos] = sh * 0x2000 + (sl >> 6);
        }
    } else {
        int local50 = pitch_i16;
        int p28 = quarter;
        int p24 = 0;

        for (pos = 0; pos < half; pos++) {
            int p4 = p24 + quarter;
            uint32_t sl = l;
            int sh = m;
            int k, v6;

            for (k = 1; k <= order; k++) {
                int idx = tbl_stride * (p4 % pitch_len);
                int32_t prod = (int32_t) acf[k] * rt[idx];
                sl += prod & 0x7ffff;
                sh += prod >> 19;
                p4 += p24;
            }
            v6 = (local50 * (((local50 - 2 * rt[tbl_stride * (p28 % pitch_len)]) >> 2))
                  + 0x10000000) >> 15;
            v6 = (((sl & 0x7ffff) >> 4) * v6 >> 14) + (sh + (sl >> 19)) * v6 * 2;
            route_out[pos] = v6;
            p24++;
            p28 += pitch_win;
        }
    }
    return 0;
}

int lpec_c680_route(const int16_t *a110_out, int pitch_win, double pitch_e,
                    int32_t *route_out, int pitch_len, int order)
{
    return lpec_c680_route_tables(a110_out, pitch_win, pitch_e, route_out,
                                  pitch_len, order,
                                  lpec_rtable_a, lpec_rtable_b);
}
