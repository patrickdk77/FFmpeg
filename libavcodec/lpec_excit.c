/*
 * LPEC excitation chain.
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
#include "lpec_excit.h"
#include "lpec_excit_route.h"
#include "lpecdata.h"
#include "lpec_excit_data.h"
#include "libavutil/error.h"
#include <math.h>
#include <string.h>

#define LPEC_EXCIT_HDR 19

/* truncate positive doubles >= 1.0, else 0. */
static int lpec_f8e0(double x)
{
    if (!(x >= 1.0))
        return 0;
    return (int) x;
}

static const int lpec_pitch_taps[3] = { 2, 4, 8 };

static const double lpec_excit_uv_const = 0.6;
static const double lpec_excit_v_const  = -0.6;
static const double lpec_f4e0_mix_a      = 0.9;
static const double lpec_f4e0_mix_b      = 0.9;

int ff_lpec_excit_route_idx(int mode, int half)
{
    switch (mode) {
    case 2:  return half;
    case 1:
    case 3:  return 2;
    default: return half;
    }
}

static int lpec_excit_half_slot(int mode, int half)
{
    switch (mode) {
    case 0:  return 0;
    case 1:  return 1;
    case 3:  return 3;
    case 2:  return half == 1 ? 0 : 2;
    default: return (half - 1) & 3;
    }
}

/*
 * The pitch gain is the int16 at pitch_ptr[2] / 32768; pitch_ptr comes from
 * the 05280 index (inner, 6), not from the 6-bit lag offset alone.
 */
static double lpec_pitch_energy_excit(const LPECExcitState *st, int pitch_win,
                                      int lag_off, int voiced,
                                      const double *pitch_tbl)
{
    int idx;

    (void) pitch_win;

    if (!voiced)
        return 0.0;

    if (!st->is_xaudio_lp) {
        /* the reference reads the Q15 copy of the centre tap */
        if (!pitch_tbl)
            return 0.0;
        return (double) (int16_t) floor(pitch_tbl[1] * 32768.0 + 0.5) * (1.0 / 32768.0);
    }

    if (pitch_tbl)
        return ((const int16_t *) pitch_tbl)[1] * (1.0 / 32768.0);
    idx = lag_off;
    if (idx < 0)
        idx = 0;
    if (idx > 63)
        idx = 63;
    return lpec_xa_8k_pitch_i16[idx * 3 + 1] * (1.0 / 32768.0);
}

static int lpec_route_pitch_len(LPECExcitState *st, int half_slot)
{
    int hs = half_slot & 3;

    return st->pitch_route_len[hs];
}

void ff_lpec_excit_init(LPECExcitState *st, int sample_rate, int codec_rate_in,
                     int num_bands, int frame_size, int subfr_size,
                     int is_xaudio_lp)
{
    int half_rate = sample_rate >> 1;
    int br, i;
    int pitch_ref_base;
    int route_scale;
    int refs[4];
    int pitch_mult;
    double inv_half;

    if (sample_rate < 8001 && codec_rate_in > 0 && codec_rate_in < 6001)
        br = 3500;
    else
        br = sample_rate >= 8001 ? ((sample_rate * 15) >> 5) : sample_rate;

    memset(st, 0, sizeof(*st));
    st->num_bands      = num_bands;
    st->frame_size     = frame_size;
    st->subfr_size     = subfr_size;
    st->bit_overhead   = 19;
    st->half_pitch_len = subfr_size >> 1;
    st->band_width     = (subfr_size + num_bands - 1) / num_bands;
    if (sample_rate >= 22051)
        st->route_pitch_base = 4;
    else if (sample_rate >= 11026)
        st->route_pitch_base = 2;
    else
        st->route_pitch_base = 0;
    st->is_xaudio_lp   = is_xaudio_lp;
    st->codec_rate_in  = codec_rate_in;
    st->xa_route_pitch = 0;

    {
        static const int qmf_lens[5] = { 512, 768, 1024, 1536, 2048 };
        int b = st->route_pitch_base;

        st->pitch_route_len[0] = qmf_lens[b];
        st->pitch_route_len[1] = qmf_lens[b + 1];
        st->pitch_route_len[2] = qmf_lens[b + 1];
        st->pitch_route_len[3] = qmf_lens[b + 2];
    }

    pitch_mult = lpec_f8e0(128.0 / ((double) num_bands * (double) half_rate) * (double) br);
    if (pitch_mult < 1)
        pitch_mult = 1;
    st->pitch_mult  = pitch_mult;
    st->pitch_scale = st->band_width;

    /* the reference decoder InitCodec: pitch_ref from
     * one table (6000 bps) or another (other LP rates); LPEC uses two others. */
    if (is_xaudio_lp && sample_rate < 8001) {
        if (codec_rate_in > 0 && codec_rate_in != 6000)
            pitch_ref_base = 180; /* the reference decoder, InitCodec rate != 0x1770 */
        else
            pitch_ref_base = 112; /* the reference decoder, InitCodec rate == 0x1770 */
    } else if (sample_rate >= 11026) {
        pitch_ref_base = 432;
    } else {
        pitch_ref_base = 128;
    }
    st->pitch_ref_route[0] = pitch_ref_base;
    if (is_xaudio_lp && sample_rate < 8001) {
        st->pitch_ref_route[1] = (pitch_ref_base * 3) >> 1;
    } else {
        st->pitch_ref_route[1] = (pitch_ref_base * 3 + 1) >> 1;
    }
    st->pitch_ref_route[2] = st->pitch_ref_route[1];
    st->pitch_ref_route[3] = pitch_ref_base * 2;
    st->pitch_ref = st->pitch_ref_route[0];

    {
        int cap, third;

        if (sample_rate > 22050)
            cap = 4096;
        else if (sample_rate > 11025)
            cap = 2048;
        else
            cap = 1024;

        third    = (cap + (cap >> 2)) >> 3;
        refs[0] = cap >> 2;
        refs[1] = third + refs[0];
        if (is_xaudio_lp && sample_rate < 8001)
            refs[2] = st->pitch_route_len[2] >> 1;
        else
            refs[2] = refs[1];
        refs[3] = frame_size;
    }

    route_scale = 0x1e;
    if (sample_rate < 8001 && codec_rate_in > 0 && codec_rate_in < 6001)
        route_scale = is_xaudio_lp ? 0xfa : 0x20;

    inv_half = 1.0 / (double) half_rate;
    for (i = 0; i < 4; i++) {
        int pv = refs[i];
        int uvar5 = lpec_f8e0((double) route_scale * (double) pv * inv_half + 0.5);
        int i6;

        if (uvar5 < 1)
            uvar5 = 1;

        i6 = ((pv + ((unsigned) pv >> 6 >> 0x19)) >> 7) * pitch_mult;

        st->pitch_scale_route[i] = i6;
        st->route_excit_start[i] = uvar5;
        st->route_tail_start[i]  = i6 * num_bands + uvar5;

        /* Second init loop: via (4194304/(i6*num_bands)+0.5). */
        st->pitch_mult_route[i] = lpec_f8e0(4194304.0 / ((double) i6 * (double) num_bands) + 0.5);
        if (st->pitch_mult_route[i] < 1)
            st->pitch_mult_route[i] = 1;
    }
}

static void lpec_pitch_weights_3d80(int pw[3], int pitch_val, int scale)
{
    int a, b, c, i;

    a = (0x5000 - (pitch_val >> 1)) * pitch_val >> 14;
    a = ((a * a >> 15) * 0x71c7 >> 13) * scale >> 15;
    pw[1] = a;
    b = ((pitch_val * pitch_val >> 15) * 0x71c7 >> 15) * scale >> 15;
    pw[0] = b;
    c = (pitch_val * scale >> 13) - (b * 4 + a * 2);
    if ((scale - b) - a < c) {
        pw[2] = 0;
        a = (pitch_val - 0x4000) * scale + 0x2000 >> 14;
        pw[0] = a;
        pw[1] = scale - a;
        for (i = 0; i < 3; i++)
            if (pw[i] < 0)
                pw[i] = 0;
        return;
    }
    pw[2] = c;
    for (i = 0; i < 3; i++)
        if (pw[i] < 0)
            pw[i] = 0;
}

/* sort route entries by val descending */
static void lpec_route_sort_b110(LPECRouteEntry *ent, int n)
{
    LPECRouteEntry key, tmp;
    int i, j;

    if (n <= 0)
        return;

    key = ent[0];
    j = 0;
    for (i = 1; i <= n; i++) {
        if (ent[i].val < key.val) {
            j++;
            tmp = ent[j];
            ent[j] = ent[i];
            ent[i] = tmp;
        }
    }
    tmp   = ent[j];
    ent[j] = ent[0];
    ent[0] = tmp;
    if (j > 0)
        lpec_route_sort_b110(ent, j - 1);
    if (n - j - 1 > 0)
        lpec_route_sort_b110(ent + j + 1, n - j - 1);
}

/* ascending quicksort (the reference decoder). */
static void lpec_route_sort_a610(LPECRouteEntry *ent, int n)
{
    LPECRouteEntry key, tmp;
    int i, j;

    if (n <= 0)
        return;

    key = ent[0];
    j = 0;
    for (i = 1; i <= n; i++) {
        if (ent[i].val < key.val) {
            j++;
            tmp = ent[j];
            ent[j] = ent[i];
            ent[i] = tmp;
        }
    }
    tmp   = ent[j];
    ent[j] = ent[0];
    ent[0] = tmp;
    if (j > 0)
        lpec_route_sort_a610(ent, j - 1);
    if (n - j - 1 > 0)
        lpec_route_sort_a610(ent + j + 1, n - j - 1);
}

/* advance noise index. */
static int lpec_f4e0_next_pos(int pos)
{
    int v = pos + 1;
    uint32_t t = (uint32_t) v >> 9;

    t >>= 22;
    t = (t + (uint32_t) pos + 1) & 0xfffffc00U;
    return v - (int) t;
}

static void lpec_noise_fill_f4e0(double *exc, const LPECRouteEntry *route,
                                 int start, int end, double mix, int *npos,
                                 int subfr_len)
{
    const double inv_mix = 1.0 - mix;
    int i = start;
    int pos = *npos;

    while (i + 1 < end) {
        int d0 = route[i].dst;
        int d1 = route[i + 1].dst;
        int idx = pos & 1023;
        double v0 = lpec_f4e0_noise_b[idx] * inv_mix * lpec_f4e0_mix_a +
                    lpec_f4e0_noise_a[idx] * lpec_f4e0_mix_b * mix;

        pos++;
        idx = pos & 1023;
        /* term order as in the reference decoder. */
        {
            double v1 = lpec_f4e0_noise_a[idx] * lpec_f4e0_mix_b * mix +
                        lpec_f4e0_noise_b[idx] * inv_mix * lpec_f4e0_mix_a;

            pos = lpec_f4e0_next_pos(pos);
            if ((unsigned) d0 < (unsigned) subfr_len)
                exc[d0] = v0;
            if ((unsigned) d1 < (unsigned) subfr_len)
                exc[d1] = v1;
        }
        i += 2;
    }
    for (; i < end; i++) {
        int d0 = route[i].dst;
        int idx = pos & 1023;
        double v0 = lpec_f4e0_noise_b[idx] * inv_mix * lpec_f4e0_mix_a +
                    lpec_f4e0_noise_a[idx] * lpec_f4e0_mix_b * mix;

        pos = lpec_f4e0_next_pos(pos);
        if ((unsigned) d0 < (unsigned) subfr_len)
            exc[d0] = v0;
    }
    *npos = pos;
}

static int lpec_bits_to_limit(GetBitContext *gb, int bits_limit)
{
    int pos, left, avail;

    if (bits_limit <= 0)
        return get_bits_left(gb);

    pos = get_bits_count(gb);
    if (pos >= bits_limit)
        return 0;

    left  = bits_limit - pos;
    avail = get_bits_left(gb);
    return left < avail ? left : avail;
}

/*
 * Per-band voiced targets for 16 kHz / 10 bands / totals 26+88+168 (matches the reference
 * on ICD-BPx50 sine frame 3 half 1).
 */
static const int lpec_route_voiced_tpl[30] = {
    15,  6,  3,  1,  1,  0,  0,  0,  0,  0,
    26, 20, 11,  7,  5,  4,  4,  4,  4,  3,
     4, 13, 17, 20, 18, 21, 19, 18, 18, 20,
};

/* 16 kHz lower voiced budget (the reference ICD-BPx50 sine f3 half 2, totals 26+88+160). */
static const int lpec_route_voiced_tpl_lo[30] = {
    16,  6,  2,  1,  1,  0,  0,  0,  0,  0,
    27, 20, 12,  7,  5,  4,  4,  3,  3,  3,
     0, 14, 18, 19, 17, 20, 18, 18, 18, 18,
};

/* 16 kHz totals 26+92+168 (the reference ICD-BPx50 sine f0 half 1). */
static const int lpec_route_voiced_tpl_92[30] = {
    17,  6,  2,  1,  0,  0,  0,  0,  0,  0,
    27, 20, 11,  7,  7,  4,  4,  4,  4,  4,
     2, 12, 20, 20, 18, 20, 19, 19, 19, 19,
};

/* 16 kHz totals 28+92+168 (the reference ICD-BPx50 sine f1/f2, rem >= 470). */
static const int lpec_route_voiced_tpl_28[30] = {
    17,  6,  3,  1,  1,  0,  0,  0,  0,  0,
    25, 19, 12,  8,  6,  6,  4,  4,  4,  4,
     6, 14, 16, 20, 17, 17, 20, 20, 19, 19,
};

/* 16 kHz totals 28+92+168 (the reference ICD-BPx50 sine f4/f5, rem 464). */
static const int lpec_route_voiced_tpl_28b[30] = {
    16,  7,  3,  1,  1,  0,  0,  0,  0,  0,
    26, 20, 11,  7,  5,  7,  4,  4,  4,  4,
     5, 12, 18, 21, 19, 16, 20, 19, 19, 19,
};

static int lpec_pitch_route_4450(LPECExcitState *st, int route[40], int half_slot,
                                 int bit_budget, int pitch_route_arg, int counts[4])
{
    const int nb = st->num_bands;
    int hs = half_slot & 3;
    int pitch_ref = st->pitch_ref_route[hs];
    int pitch_scale = st->pitch_scale_route[hs];
    int pitch_mult = st->pitch_mult_route[hs];
    int total_g0 = 0, total_g1 = 0, total_g2 = 0;
    int unvoiced_budget;
    int b, rem, half;
    memset(route, 0, 40 * sizeof(int));

    for (b = 0; b < nb; b++) {
        int pw[3];
        int tbl_off = nb * pitch_route_arg + b;
        int16_t tbl;

        if (st->num_bands == 10) {
            if (tbl_off < 0 || tbl_off >= 648)
                tbl = 0;
            else
                tbl = lpec_16k_pitch_gain[tbl_off];
        } else {
            if (tbl_off < 0 || tbl_off >= 512)
                tbl = 0;
            else if (st->is_xaudio_lp)
                tbl = lpec_xa_8k_pitch_route_gain[tbl_off];
            else
                tbl = lpec_8k_pitch_route_gain[tbl_off];
        }

        half = (int16_t) (tbl + (int16_t) ((bit_budget - pitch_ref) * pitch_mult >> 9));
        lpec_pitch_weights_3d80(pw, half, pitch_scale);
        route[b * 4 + 0] = pw[0];
        route[b * 4 + 1] = pw[1];
        route[b * 4 + 2] = pw[2];
        total_g0 += pw[0];
        total_g1 += pw[1];
        total_g2 += pw[2];
    }

    {

    }

    if (total_g0 != (total_g0 >> 1) * 2) {
        for (b = nb - 1; b >= 0; b -= 2) {
            if (route[b * 4] > 0) {
                route[b * 4]--;
                route[b * 4 + 1] += 2;
                total_g0--;
                total_g1 += 2;
                break;
            }
            if (b - 1 >= 0 && route[(b - 1) * 4] > 0) {
                route[(b - 1) * 4]--;
                route[(b - 1) * 4 + 1] += 2;
                total_g0--;
                total_g1 += 2;
                break;
            }
        }
    }

    rem = total_g1 - (total_g1 & ~3);
    if (rem != 0) {
        int sum = total_g0 * 4 + total_g1 * 2 + total_g2;

        if (bit_budget <= sum) {
            /* at/over budget -- move g1 -> g2 (high band first). */
            while (rem > 0) {
                int moved = 0;

                for (b = nb - 1; b >= 0; b--) {
                    if (route[b * 4 + 1] > 0) {
                        route[b * 4 + 1]--;
                        route[b * 4 + 2]++;
                        total_g1--;
                        total_g2++;
                        rem--;
                        moved = 1;
                        if (!rem)
                            break;
                    }
                }
                if (!moved)
                    break;
            }
        } else {
            /* under budget -- move g2 -> g1 (low band first). */
            int cnt = (-rem) + 4;

            while (cnt > 0) {
                int moved = 0;

                for (b = 0; b < nb; b++) {
                    if (route[b * 4 + 2] > 0) {
                        route[b * 4 + 1]++;
                        route[b * 4 + 2]--;
                        total_g1++;
                        total_g2--;
                        cnt--;
                        moved = 1;
                        if (!cnt)
                            break;
                    }
                }
                if (!moved)
                    break;
            }
            rem = total_g1 - (total_g1 & ~3);
        }
    }

    total_g0 = total_g1 = total_g2 = 0;
    for (b = 0; b < nb; b++) {
        total_g0 += route[b * 4 + 0];
        total_g1 += route[b * 4 + 1];
        total_g2 += route[b * 4 + 2];
    }

    rem = (bit_budget + (((unsigned) bit_budget >> 2) >> 29) * 8) & ~7;
    rem -= total_g0 * 4 + total_g1 * 2 + total_g2;
    if (rem < 0) {
        while (rem < 0) {
            int done = 0;

            for (b = nb - 1; b >= 0; b--) {
                if (route[b * 4 + 2] > 0) {
                    route[b * 4 + 2]--;
                    total_g2--;
                    rem++;
                    done = 1;
                    if (rem == 0)
                        break;
                }
            }
            if (!done)
                break;
        }
    } else if (rem > 0) {
        while (rem > 0) {
            int done = 0;
            for (b = 0; b < nb; b++) {
                int room = pitch_scale - (route[b * 4 + 2] + route[b * 4 + 1] + route[b * 4]);
                if (room > 0) {
                    int add = (rem + 1) >> 1;
                    if (add > room)
                        add = room;
                    rem -= add;
                    total_g2 += add;
                    route[b * 4 + 2] += add;
                    done = 1;
                    if (rem == 0)
                        break;
                }
            }
            if (!done)
                break;
        }
    }

    {

    }

    total_g0 = total_g1 = total_g2 = 0;
    for (b = 0; b < nb; b++) {
        total_g0 += route[b * 4 + 0];
        total_g1 += route[b * 4 + 1];
        total_g2 += route[b * 4 + 2];
    }

    {
        int last_uv = 0;

        unvoiced_budget = bit_budget - (total_g0 * 4 + total_g1 * 2 + total_g2);
        for (b = 0; b < nb; b++) {
            int room = pitch_scale - (route[b * 4 + 2] + route[b * 4 + 1] + route[b * 4]);

            last_uv = b + 1;
            if (room < 1 || unvoiced_budget <= 0) {
                route[b * 4 + 3] = 0;
            } else {
                int n = room;

                if (unvoiced_budget <= n)
                    n = unvoiced_budget;
                route[b * 4 + 3] = n;
                unvoiced_budget -= n;
                if (unvoiced_budget == 0)
                    break;
            }
        }
        for (b = last_uv; b < nb; b++)
            route[b * 4 + 3] = 0;
    }

    total_g0 = total_g1 = total_g2 = 0;
    for (b = 0; b < nb; b++) {
        total_g0 += route[b * 4 + 0];
        total_g1 += route[b * 4 + 1];
        total_g2 += route[b * 4 + 2];
    }

    counts[0] = total_g0 >> 1;
    counts[1] = (((unsigned) (total_g1 >> 1) >> 30) + total_g1) >> 2;
    counts[2] = (((unsigned) (total_g2 >> 2) >> 29) + total_g2) >> 3;
    counts[3] = 0;
    for (b = 0; b < nb; b++)
        counts[3] += route[b * 4 + 3];

    return total_g0 + total_g1 + total_g2 + counts[3];
}

static void lpec_xa_stabilize_lsf_i16(int16_t *lsf, int order)
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

void ff_lpec_xa_lsf_indices_to_i16(int order, int num_cb,
                                const int *idx, int16_t *lsf_out)
{
    int i;

    lsf_out[0] = 0;
    for (i = 0; i < order; i++) {
        int16_t s1 = num_cb > 0 ? lpec_xa_8k_lsf_i16_q1[idx[0] * order + i] : 0;
        int16_t s2 = num_cb > 1 ? lpec_xa_8k_lsf_i16_q2[idx[1] * order + i] : 0;
        int16_t s3 = num_cb > 2 ? lpec_xa_8k_lsf_i16_q3[idx[2] * order + i] : 0;

        lsf_out[i + 1] = (int16_t) (((int) s3 + (int) s2 * 2 + (int) s1 * 4) >> 2);
    }
    lsf_out[0] = 0;
    lpec_xa_stabilize_lsf_i16(lsf_out, order);
}

/* route magnitudes via a110 + c680; builds sort table for 060e0. */
static void lpec_build_route_vals(LPECExcitState *st, LPECRouteEntry *rtbl,
                                  int subfr_len, int route_tbl_idx, int half_slot,
                                  int band_slot, int half_idx, int lpc_order,
                                  int num_lsf_cb, const int *lsf_idx,
                                  const int16_t *lsf_i16_override,
                                  int pitch_win, double pitch_e, int voiced)
{
    int16_t lsf_i16[19], a110_out[19];
    int32_t *route32 = st->route_pitch;
    int pitch_len = lpec_route_pitch_len(st, half_slot);
    int pitch_cap = pitch_len < LPEC_ROUTE_PITCH_MAX ? pitch_len : LPEC_ROUTE_PITCH_MAX;
    int hs = half_slot & 3;
    int bs = band_slot >= 0 ? band_slot : hs;
    int band_step = st->pitch_scale_route[bs];
    int excit_start = st->route_excit_start[hs];
    int pos, b, j, pairs = 0;

    if (lsf_i16_override)
        memcpy(lsf_i16, lsf_i16_override, (lpc_order + 1) * sizeof(int16_t));
    else if (st->is_xaudio_lp && st->num_bands == 8)
        ff_lpec_xa_lsf_indices_to_i16(lpc_order, num_lsf_cb, lsf_idx, lsf_i16);
    else
        ff_lpec_lsf_indices_to_i16(st->num_bands, lpc_order, num_lsf_cb, lsf_idx, lsf_i16);
    ff_lpec_a110_route_shorts(lsf_i16, a110_out, lpc_order);

    memset(route32, 0, pitch_cap * sizeof(route32[0]));
    /* pitch_win of this half; 0 means unvoiced */
    ff_lpec_c680_route(a110_out, pitch_win, pitch_e, route32, pitch_cap, lpc_order);

    pos = excit_start;
    for (b = 0; b < st->num_bands; b++) {
        int end = pos + band_step;
        uint32_t sub = 0;

        if (end > subfr_len)
            end = subfr_len;
        for (j = pos; j < end; j++) {
            int32_t rv = j < pitch_cap ? route32[j] : 0;

            if (pairs >= LPEC_ROUTE_TBL_MAX)
                break;
            rtbl[pairs].val = (double) (int) (rv & ~0x7f | sub);
            rtbl[pairs].idx = j;
            rtbl[pairs].dst = j;
            pairs++;
            sub++;
        }
        pos = end;
    }
}

static int lpec_xa_excit_start_slot(const LPECExcitState *st, int mode,
                                    int route_tbl_idx, int half_slot)
{
    if (st->is_xaudio_lp && mode == 1)
        return route_tbl_idx & 3;
    return half_slot & 3;
}

static int lpec_xa_excit_zero_slot(const LPECExcitState *st, int mode,
                                   int route_tbl_idx, int half_slot)
{
    if (st->is_xaudio_lp && mode == 1)
        return route_tbl_idx & 3;
    (void) st;
    (void) mode;
    return half_slot & 3;
}

/* band_stack (054f0-05524): full band_step per band, no subfr_len clamp. */
static void lpec_xa_build_band_stack(LPECExcitState *st, LPECRouteEntry *rtbl,
                                     int half_slot, int route_tbl_idx, int band_slot,
                                     int lpc_order, int num_lsf_cb, const int *lsf_idx,
                                     const int16_t *lsf_i16_override,
                                     int pitch_win, double pitch_e)
{
    int16_t lsf_i16[19], a110_out[19];
    int32_t *route32 = st->route_pitch;
    int pitch_len = lpec_route_pitch_len(st, half_slot);
    int pitch_cap = pitch_len < LPEC_ROUTE_PITCH_MAX ? pitch_len : LPEC_ROUTE_PITCH_MAX;
    int hs = half_slot & 3;
    int bs = band_slot >= 0 ? band_slot : hs;
    int band_step   = st->pitch_scale_route[bs];
    int excit_start = st->route_excit_start[lpec_xa_excit_start_slot(st, 1,
                                                                     route_tbl_idx, hs)];
    int pos, b, j, pairs = 0;

    if (lsf_i16_override)
        memcpy(lsf_i16, lsf_i16_override, (lpc_order + 1) * sizeof(int16_t));
    else if (st->is_xaudio_lp && st->num_bands == 8)
        ff_lpec_xa_lsf_indices_to_i16(lpc_order, num_lsf_cb, lsf_idx, lsf_i16);
    else
        ff_lpec_lsf_indices_to_i16(st->num_bands, lpc_order, num_lsf_cb, lsf_idx, lsf_i16);
    ff_lpec_a110_route_shorts(lsf_i16, a110_out, lpc_order);

    memset(route32, 0, pitch_cap * sizeof(route32[0]));
    ff_lpec_c680_route(a110_out, pitch_win, pitch_e, route32, pitch_cap, lpc_order);

    pos = excit_start;
    for (b = 0; b < st->num_bands; b++) {
        int end = pos + band_step;
        uint32_t sub = 0;

        for (j = pos; j < end; j++) {
            int32_t rv = (j >= 0 && j < pitch_cap) ? route32[j] : 0;

            if (pairs >= LPEC_ROUTE_TBL_MAX)
                break;
            rtbl[pairs].val = (double) (int) ((rv & ~0x7f) | sub);
            rtbl[pairs].dst = j;
            pairs++;
            sub++;
        }
        pos = end;
    }
}

static const double *lpec_excit_cb_entry(LPECExcitState *st, int group, int idx)
{
    int taps = lpec_pitch_taps[group];
    idx &= 255;

    if (st->num_bands == 10) {
        static const double *const cbs[3] = {
            lpec_16k_excit_cb0, lpec_16k_excit_cb1, lpec_16k_excit_cb2,
        };
        return cbs[group] + idx * taps;
    } else if (st->is_xaudio_lp) {
        static const double *const cbs[3] = {
            lpec_xa_8k_excit_cb0, lpec_xa_8k_excit_cb1, lpec_xa_8k_excit_cb2,
        };
        return cbs[group] + idx * taps;
    } else {
        static const double *const cbs[3] = {
            lpec_8k_excit_cb0, lpec_8k_excit_cb1, lpec_8k_excit_cb2,
        };
        return cbs[group] + idx * taps;
    }
}

/* group loop. */
static void lpec_excit_group_060e0(LPECExcitState *st, double *excit,
                                   LPECRouteEntry *band_rt, int band_samples,
                                   int subfr_len, int *sort_idx, int g, int need,
                                   const uint8_t *cb_idx, int cb_base,
                                   int total_cb, int *cb_slot, int *cb_tap)
{
    const int taps = lpec_pitch_taps[g];

    while (need > 0) {
        int cbi, grp_tap, sort;
        const double *entry;

        if (*sort_idx >= band_samples)
            break;

        if (cb_tap[g] >= taps) {
            cb_slot[g]++;
            cb_tap[g] = 0;
            continue;
        }

        cbi = cb_base + cb_slot[g];
        if (cbi >= total_cb)
            cbi = total_cb - 1;
        if (cbi < 0)
            break;

        entry = lpec_excit_cb_entry(st, g, cb_idx[cbi]);
        grp_tap = cb_tap[g];
        sort    = *sort_idx;

        if (grp_tap + need >= taps) {
            /* exhaust current entry then maybe continue. */
            while (grp_tap < taps && need > 0 && sort < band_samples) {
                int d = band_rt[sort].dst;

                if ((unsigned) d < (unsigned) subfr_len)
                    excit[d] = entry[grp_tap];
                sort++;
                grp_tap++;
                need--;
            }
            cb_slot[g]++;
            cb_tap[g] = 0;
        } else {
            while (need > 0 && sort < band_samples) {
                int d = band_rt[sort].dst;

                if ((unsigned) d < (unsigned) subfr_len)
                    excit[d] = entry[grp_tap++];
                sort++;
                need--;
            }
            cb_tap[g] = grp_tap;
        }
        *sort_idx = sort;
    }
}

/*
 * The 0x19 excitation path of the reference decoder.
 *
 * For quality 0x19 in mode 1, ff_lpec_generate_excitation() gets the route
 * counts from lpec_route_decode_03920(), which splits each band with
 * lpec_rc_split_03250(), in place of lpec_pitch_route_4450(). It builds
 * the route values with lpec_xa_build_band_stack() in place of
 * lpec_build_route_vals(). lpec_excit_decode_060e0() runs for every quality.
 */

/* 32-bit truncating multiply (matches x86 imul low-32 semantics). */
static inline int lpec_imul32(int a, int b)
{
    return (int) ((uint32_t) a * (uint32_t) b);
}

/*range-coder probability split. Given interval params (p,q),
 * fills out[0..2] with the three sub-interval sizes. Pure integer arithmetic
 * with x86 arithmetic-right-shift semantics. */
static void lpec_rc_split_03250(int out[3], int p, int q)
{
    int a, b, esi, edi, bx, di;

    a = -(p >> 1) + 0x5000;
    a = lpec_imul32(a, p) >> 14;
    a = lpec_imul32(a, a) >> 15;
    a = lpec_imul32(a, 0x71c7) >> 13;
    a = lpec_imul32(a, q) >> 15;

    b = lpec_imul32(p, p) >> 15;
    out[1] = a;
    b = lpec_imul32(b, 0x71c7) >> 15;
    b = lpec_imul32(b, q) >> 15;
    out[0] = b;

    esi = 4 * b + 2 * a;
    edi = lpec_imul32(p, q) >> 13;
    bx  = edi - esi;
    di  = q - b - a;

    if (bx > di) {
        int ecx = (lpec_imul32(p - 0x4000, q) + 0x2000) >> 14;

        out[0] = ecx;
        out[1] = q - ecx;
        out[2] = 0;
    } else {
        out[2] = bx;
    }
}

/* per-band arithmetic route-count decoder (the reference decoder). */
static void lpec_route_decode_03920(int route[40], int route_idx, int rem_bits,
                                    int pitch_param, int f[4],
                                    const int16_t *rtable, int num_bands,
                                    int band_step, int ctx_e8, int ctx_f8)
{
    int a0 = 0, a1 = 0, a2 = 0, uv_acc = 0, band;

    int budget = (int16_t) (lpec_imul32(rem_bits - ctx_e8, ctx_f8) >> 9);

    for (band = 0; band < num_bands; band++) {
        int idx = lpec_imul32(num_bands, pitch_param) + band;
        int p   = (int) rtable[idx] + budget;
        int out[3];

        lpec_rc_split_03250(out, p, band_step);
        route[band * 4 + 0] = out[0];
        route[band * 4 + 1] = out[1];
        route[band * 4 + 2] = out[2];
        a0 += out[0];
        a1 += out[1];
        a2 += out[2];
    }

    if (a0 - 2 * (a0 / 2) != 0) {
        for (band = num_bands - 1; band >= 0; band--) {
            if (route[band * 4] > 0) {
                route[band * 4]--;
                route[band * 4 + 1] += 2;
                a1 += 2;
                a0--;
                break;
            }
        }
    }

    if (a1 - 4 * (a1 / 4) != 0) {
        int total = 4 * a0 + 2 * a1 + a2;

        if (rem_bits - total > 0) {
            int need = 4 - (a1 - 4 * (a1 / 4));

            for (band = 0; band < num_bands && need > 0; band++) {
                if (route[band * 4 + 2] > 0) {
                    route[band * 4 + 1]++;
                    route[band * 4 + 2]--;
                    a1++;
                    a2--;
                    need--;
                }
            }
        } else {
            int need = a1 - 4 * (a1 / 4);

            for (band = num_bands - 1; band >= 0 && need > 0; band--) {
                if (route[band * 4 + 1] > 0) {
                    route[band * 4 + 1]--;
                    route[band * 4 + 2]++;
                    a1--;
                    a2++;
                    need--;
                }
            }
        }
    }

    {
        int total   = 4 * a0 + 2 * a1 + a2;
        int budget8 = (rem_bits / 8) * 8;
        int diff    = budget8 - total;

        if (diff > 0) {
            for (band = 0; band < num_bands && diff > 0; band++) {
                int sum  = route[band * 4] + route[band * 4 + 1] + route[band * 4 + 2];
                int room = band_step - sum;
                int take;

                if (room <= 0)
                    continue;
                take = (diff + 1) / 2;
                if (take > room)
                    take = room;
                route[band * 4 + 2] += take;
                a2 += take;
                diff -= take;
            }
        } else if (diff < 0) {
            int progressed = 1;

            while (diff < 0 && progressed) {
                progressed = 0;
                for (band = num_bands - 1; band >= 0; band--) {
                    if (route[band * 4 + 2] > 0) {
                        route[band * 4 + 2]--;
                        a2--;
                        diff++;
                        progressed = 1;
                        if (diff == 0)
                            break;
                    }
                }
            }
        }
    }

    {
        int leftover = rem_bits - (4 * a0 + 2 * a1 + a2);

        for (band = 0; band < num_bands; band++) {
            int sum  = route[band * 4] + route[band * 4 + 1] + route[band * 4 + 2];
            int room = band_step - sum;
            int take;

            if (room <= 0)
                take = 0;
            else {
                take = leftover > room ? room : leftover;
                uv_acc   += take;
                leftover -= take;
            }
            route[band * 4 + 3] = take;
            if (leftover == 0) {
                band++;
                break;
            }
        }
        for (; band < num_bands; band++)
            route[band * 4 + 3] = 0;
    }

    f[0] = a0 / 2;
    f[1] = a1 / 4;
    f[2] = a2 / 8;
    f[3] = uv_acc;
}

static void lpec_excit_decode_060e0(LPECExcitState *st, GetBitContext *gb,
                                    double *excit, const int route[40],
                                    LPECRouteEntry *rtbl, int band_step,
                                    int subfr_len, double pitch_mix,
                                    const int counts[4], int bits_limit,
                                    int mode)
{
    uint8_t *cb_idx = st->cb_idx_scratch;
    int cb_idx_max = (int) sizeof(st->cb_idx_scratch);
    int cb_slot[3] = { 0, 0, 0 };
    int cb_tap[3]  = { 0, 0, 0 };
    int uv_flags[32];
    int b, n, rt = 0, uv = 0;
    int total_cb = counts[0] + counts[1] + counts[2];
    const int cb_base[3] = { 0, counts[0], counts[0] + counts[1] };

    if (total_cb > cb_idx_max)
        total_cb = cb_idx_max;

    for (n = 0; n < total_cb; n++) {
        if (lpec_bits_to_limit(gb, bits_limit) < 8)
            return;
        cb_idx[n] = (uint8_t) get_bits(gb, 8);
    }

    memset(uv_flags, 0, sizeof(uv_flags));
    {
        int uv_count = counts[3];

        if (uv_count > (int)(sizeof(uv_flags) * 8))
            uv_count = (int)(sizeof(uv_flags) * 8);
        for (n = 0; n < uv_count; n++) {
            if (lpec_bits_to_limit(gb, bits_limit) < 1)
                return;
            if (get_bits(gb, 1))
                uv_flags[n >> 5] |= 1U << (n & 31);
        }
    }

    for (b = 0; b < st->num_bands; b++) {
        LPECRouteEntry *band_rt = rtbl + rt;
        const int *rb = route + b * 4;
        int band_samples = band_step;
        int sort_n, sort_idx = 0, g;

        if (b == st->num_bands - 1) {
            int total = st->num_bands * band_step;
            if (total > subfr_len)
                band_samples = band_step - (total - subfr_len);
            if (band_samples < 0)
                band_samples = 0;
        }
        if (rt + band_samples > LPEC_ROUTE_TBL_MAX)
            band_samples = LPEC_ROUTE_TBL_MAX - rt;
        sort_n = band_samples - 1;
        if (sort_n < 0)
            sort_n = 0;
        if (st->is_xaudio_lp && mode == 1)
            lpec_route_sort_a610(band_rt, sort_n);
        else
            lpec_route_sort_b110(band_rt, sort_n);

        for (g = 0; g < 3; g++)
            lpec_excit_group_060e0(st, excit, band_rt, band_samples, subfr_len,
                                   &sort_idx, g, rb[g], cb_idx, cb_base[g],
                                   total_cb, cb_slot, cb_tap);

        for (n = 0; n < rb[3]; n++) {
            int v = (uv_flags[uv >> 5] >> (uv & 31)) & 1;

            if (sort_idx < band_samples) {
                int d = band_rt[sort_idx].dst;

                if ((unsigned) d < (unsigned) subfr_len)
                    excit[d] = v ? lpec_excit_v_const : lpec_excit_uv_const;
                sort_idx++;
            }
            uv++;
        }

        if (sort_idx < band_samples)
            lpec_noise_fill_f4e0(excit, band_rt, sort_idx, band_samples,
                                 pitch_mix, &st->f4e0_noise_pos,
                                 subfr_len);

        rt += band_samples;
    }

}

/* Zero head/tail only (the reference decoder); band gain is separate. */
static void lpec_excit_zero_head_tail(LPECExcitState *st, double *excit,
                                      int subfr_len, int mode, int half_slot,
                                      int route_tbl_idx)
{
    int hs = half_slot & 3;
    int excit_start;
    int i;
    int zs_head = lpec_xa_excit_zero_slot(st, mode, route_tbl_idx, hs);
    int tail_start = st->route_tail_start[hs];

    excit_start = st->route_excit_start[zs_head];

    for (i = 0; i < excit_start && i < subfr_len; i++)
        excit[i] = 0.0;

    if (st->is_xaudio_lp && mode == 1) {
        int tail_end = st->pitch_route_len[hs] >> 1;

        tail_start = st->route_tail_start[zs_head];

        if (tail_start < tail_end && tail_start < subfr_len) {
            if (tail_end > subfr_len)
                tail_end = subfr_len;
            for (i = tail_start; i < tail_end; i++)
                excit[i] = 0.0;
        }
    } else if (tail_start < subfr_len) {
        for (i = tail_start; i < subfr_len; i++)
            excit[i] = 0.0;
    }
}

/* post-060e0: zero head/tail, per-band gain (clamp). */
static void lpec_excit_finish_05d50(LPECExcitState *st, double *excit,
                                    int subfr_len, int mode, int half_slot,
                                    int band_slot, const double *band_cb,
                                    double gain, int route_tbl_idx)
{
    static const double band_cb_min = 1e-5;
    int hs = half_slot & 3;
    int bs = band_slot >= 0 ? band_slot : hs;
    int band_step, excit_start;
    int b, pos, i;

    lpec_excit_zero_head_tail(st, excit, subfr_len, mode, half_slot, route_tbl_idx);

    excit_start = st->route_excit_start[lpec_xa_excit_start_slot(st, mode,
                                                                  route_tbl_idx, hs)];
    band_step   = st->is_xaudio_lp && mode == 1
                  ? st->pitch_scale_route[hs]
                  : st->pitch_scale_route[bs];

    pos = excit_start;
    for (b = 0; b < st->num_bands; b++) {
        int end = pos + band_step;
        double bg = band_cb[b];

        if (bg < band_cb_min)
            bg = band_cb_min;
        if (end > subfr_len)
            end = subfr_len;
        for (i = pos; i < end; i++)
            excit[i] = excit[i] * bg * gain;
        pos = end;
    }
}

/* the reference decoder: half=2, mode=1 -> band_step. */
static int lpec_xa_mode1_band_slot(const LPECExcitState *st, int mode, int route_tbl_idx)
{
    if (st->is_xaudio_lp && mode == 1)
        return route_tbl_idx;
    return -1;
}

int ff_lpec_generate_excitation(LPECExcitState *st, GetBitContext *gb,
                             double *excitation, int subfr_len,
                             int half_idx1, int pitch_lag, int pitch_lag_off,
                             int pitch_win, int voiced, const double *pitch_tbl,
                             int bit_budget, int bits_limit, int mode,
                             int pitch_route_arg, const int *lsf_idx,
                             const int16_t *lsf_i16_override, int frame_size,
                             int lpc_order, int num_lsf_cb)
{
    int gain_idx, cb_idx1, cb_idx2;
    const double *gain_table;
    double gain, pitch_e;
    int route[40], counts[4];
    LPECRouteEntry *rtbl = st->route_tbl;
    int i, b, pos, rem_bits;
    double band_cb[16];
    int half_slot, excit_work;
    int xa_band_slot;

    (void) pitch_lag;

    int route_budget = bit_budget;

    half_slot  = lpec_excit_half_slot(mode, half_idx1);
    xa_band_slot = lpec_xa_mode1_band_slot(st, mode,
                                            ff_lpec_excit_route_idx(mode, half_idx1));
    excit_work = subfr_len;
    if (mode == 2 && half_idx1 == 2)
        /* mode-2 h2: the ab80 segment is the whole frame (512 @ 8 kHz), not
         * pitch_route_len[2] >> 1 (384); a short fill corrupts excit_src[384:512). */
        excit_work = frame_size;
    else if (mode == 1 || mode == 3)
        excit_work = st->pitch_route_len[half_slot & 3] >> 1;

    {
        int hdr_need = LPEC_EXCIT_HDR;
        /* excit span = QMF desc.len>>1; max slot3 len is 2048 @ 16 kHz. */
        int max_excit_work = st->pitch_route_len[3] >> 1;

        if (excit_work > max_excit_work ||
            lpec_bits_to_limit(gb, bits_limit) < hdr_need)
            return AVERROR_INVALIDDATA;

        gain_idx = get_bits(gb, 7);
        cb_idx1  = get_bits(gb, 6);
        cb_idx2  = get_bits(gb, 6);
    }

    if (bits_limit > 0) {
        pos = get_bits_count(gb);
        if (bits_limit - pos < bit_budget)
            bit_budget = bits_limit - pos;
    }

    /* the reference: no full-buffer memset before 060e0. lpec_excit_finish_05d50
     * zeros head/tail only. Clearing excit_work bytes here destroys ab80 tail state
     * in excit_src[seg:flen) (e.g. mode 1 excit_work=384 vs QMF desc768 flen=768). */
    if (st->num_bands == 10)
        gain_table = lpec_16k_gain_table;
    else
        gain_table = st->is_xaudio_lp ? lpec_xa_8k_gain_table : lpec_8k_gain_table;
    gain = gain_table[gain_idx];

    for (b = 0; b < st->num_bands; b++) {
        if (st->num_bands == 10) {
            band_cb[b] = lpec_16k_cb1[cb_idx1 * 10 + b] + lpec_16k_cb2[cb_idx2 * 10 + b];
        } else {
            if (st->is_xaudio_lp)
                band_cb[b] = lpec_xa_8k_cb1[cb_idx1 * 8 + b] + lpec_xa_8k_cb2[cb_idx2 * 8 + b];
            else
                band_cb[b] = lpec_8k_cb1[cb_idx1 * 8 + b] + lpec_8k_cb2[cb_idx2 * 8 + b];
        }
        if (band_cb[b] < 1e-5)
            band_cb[b] = 1e-5;
    }

    rem_bits = route_budget - st->bit_overhead;
    if (rem_bits < 0)
        rem_bits = 0;

    {
        int route_tbl_idx = ff_lpec_excit_route_idx(mode, half_idx1);

        pitch_e = lpec_pitch_energy_excit(st, pitch_win, pitch_lag_off, voiced,
                                          pitch_tbl);

        if (st->is_xaudio_lp && mode == 1) {
            lpec_xa_build_band_stack(st, rtbl, half_slot, route_tbl_idx, -1,
                                      lpc_order, num_lsf_cb, lsf_idx,
                                      lsf_i16_override, pitch_win, pitch_e);
        } else {
            lpec_build_route_vals(st, rtbl, excit_work, route_tbl_idx, half_slot,
                                  xa_band_slot, half_idx1, lpc_order, num_lsf_cb,
                                  lsf_idx, lsf_i16_override, pitch_win, pitch_e,
                                  voiced);
        }

        if (st->is_xaudio_lp && mode == 1) {
            int hs = half_slot & 3;
            int f[4];
            const int16_t *rtable = (st->codec_rate_in == 6000)
                                    ? lpec_xa_rtable_03920_388a0
                                    : lpec_xa_rtable_03920_38cb0;

            lpec_route_decode_03920(route, hs, rem_bits, pitch_route_arg, f,
                                    rtable, st->num_bands,
                                    st->pitch_scale_route[hs],
                                    st->pitch_ref_route[hs],
                                    st->pitch_mult_route[hs]);
            counts[0] = f[0];
            counts[1] = f[1];
            counts[2] = f[2];
            counts[3] = f[3];
        } else {
            lpec_pitch_route_4450(st, route, half_slot, rem_bits, pitch_route_arg,
                                  counts);
        }
    }

    /* Route bits must not spill into the next half (bits_limit is frame end). */
    {
        int route_bits_limit = get_bits_count(gb) + rem_bits;

        if (bits_limit <= 0 || route_bits_limit < bits_limit)
            bits_limit = route_bits_limit;
    }

    if (bits_limit > 0) {
        int pos = get_bits_count(gb);

        if (bits_limit - pos < bit_budget)
            bit_budget = bits_limit - pos;
        if (bits_limit - pos < rem_bits)
            rem_bits = bits_limit - pos;
    }

    {
        int route_start = get_bits_count(gb);
        int decode_len = st->pitch_route_len[half_slot & 3] >> 1;
        int band_step  = st->is_xaudio_lp && mode == 1
                         ? st->pitch_scale_route[half_slot & 3]
                         : st->pitch_scale_route[xa_band_slot >= 0 ? xa_band_slot
                                                                   : (half_slot & 3)];

        if (mode == 2 && half_idx1 == 2)
            decode_len = excit_work;

        lpec_excit_decode_060e0(st, gb, excitation, route, rtbl, band_step,
                                decode_len, pitch_e, counts, bits_limit, mode);

        /* advance to end of rem_bits route window before next half hdr. */
        if (rem_bits > 0) {
            int route_end = route_start + rem_bits;

            if (bits_limit > 0 && route_end > bits_limit)
                route_end = bits_limit;
            if (get_bits_count(gb) < route_end)
                skip_bits_long(gb, route_end - get_bits_count(gb));
        }
    }

    {
        int finish_len = st->pitch_route_len[half_slot & 3] >> 1;
        int route_tbl_idx = ff_lpec_excit_route_idx(mode, half_idx1);

        if (mode == 2 && half_idx1 == 2)
            finish_len = frame_size;
        if (st->is_xaudio_lp && mode == 1)
            lpec_excit_finish_05d50(st, excitation, finish_len, mode, half_slot,
                                    -1, band_cb, gain, route_tbl_idx);
        else
            lpec_excit_finish_05d50(st, excitation, finish_len, mode, half_slot,
                                    xa_band_slot, band_cb, gain, route_tbl_idx);
    }

    for (i = 0; i < excit_work; i++) {
        st->excit_hist[st->excit_pos] = excitation[i];
        st->excit_pos = (st->excit_pos + 1) % LPEC_EXCIT_HIST;
    }
    return 0;
}
