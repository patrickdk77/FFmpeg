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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int lpec_trace_on(void)
{
    if (!getenv("LPEC_TRACE"))
        return 0;
    return 1;
}

/* Match LPEC_DUMP_* frame=N against lpecdec frame index (not excit seq). */
static int lpec_excit_dump_wants(int df, int dump_frame)
{
    if (df < 0)
        return 1;
    return dump_frame == df;
}

#define LPEC_EXCIT_HDR 19

/* truncate positive doubles >= 1.0 (via ), else 0. */
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

/* param_2: route index for 0x350 / 0x140 (mode 0: 1 then 2). */
int lpec_excit_route_idx(int mode, int half)
{
    switch (mode) {
    case 2:  return half;
    case 1:
    case 3:  return 2; /* modes 1/3: param_2 = 2 */
    default: return half;
    }
}

/* param_3: half slot for 0x30/0x40/0x50 and pitch scalars. */
static int lpec_excit_half_slot(int mode, int half)
{
    switch (mode) {
    case 0:  return 0;
    case 1:  return 1; /* mode 1: param_3 = 1 */
    case 3:  return 3; /* mode 3: param_3 = 3 */
    case 2:  return half == 1 ? 0 : 2;
    default: return (half - 1) & 3;
    }
}

/* /: movsx word [pitch_ptr+2] * (1/32768).
 * the reference decoder 05330: pitch_ptr = ctx+0x348[half] from 05280 index
 * (inner,6), not the 6-bit lag offset alone.*/
static double lpec_pitch_energy_excit(const LPECExcitState *st, int pitch_win,
                                      int lag_off, int voiced,
                                      const double *pitch_tbl)
{
    int idx;

    (void) pitch_win;

    if (!voiced)
        return 0.0;

    if (!st->is_xaudio_lp) {
        if (!pitch_tbl)
            return 0.0;
        return pitch_tbl[1];
    }

    /* the reference decoder 05330: movsx word [ctx+0x348[half]+2] / 32768. */
    if (pitch_tbl)
        return ((const int16_t *) pitch_tbl)[1] * (1.0 / 32768.0);
    idx = lag_off;
    if (idx < 0)
        idx = 0;
    if (idx > 63)
        idx = 63;
    return lpec_xa_8k_pitch_i16[idx * 3 + 1] * (1.0 / 32768.0);
}

/* pitch_len = *(ctx+0x74[param_3*4]+4) from QMF desc.len. */
static int lpec_route_pitch_len(LPECExcitState *st, int half_slot)
{
    int hs = half_slot & 3;

    return st->pitch_route_len[hs];
}

void lpec_excit_init(LPECExcitState *st, int sample_rate, int codec_rate_in,
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

    /* LP dual-rate InitCodec(8000,6000) forces param_2 = 0xdac. */
    if (sample_rate < 8001 && codec_rate_in > 0 && codec_rate_in < 6001)
        br = 3500;
    else
        br = sample_rate >= 8001 ? ((sample_rate * 15) >> 5) : sample_rate;

    memset(st, 0, sizeof(*st));
    st->num_bands      = num_bands;
    st->frame_size     = frame_size;
    st->subfr_size     = subfr_size;
    /* the reference decoder equiv: rem = param_4 - ctx+0x14 (19 @ 8 kHz LP).
 * the reference decoder inner ctx is 4 bytes ahead of LPEC (+0x10 holds 19, not +0x14).*/
    st->bit_overhead   = 19;
    st->half_pitch_len = subfr_size >> 1;
    st->band_width     = (subfr_size + num_bands - 1) / num_bands;
    /* local_c selects + local_c*0x18 pitch_len group. */
    if (sample_rate >= 22051)
        st->route_pitch_base = 4;
    else if (sample_rate >= 11026)
        st->route_pitch_base = 2;
    else
        st->route_pitch_base = 0;
    st->is_xaudio_lp   = is_xaudio_lp;
    st->codec_rate_in  = codec_rate_in;
    st->xa_route_pitch = 0;

    /* lens 0x200..0x800; ctx+0x74..0x80 = & + local_c*0x18 + slot*0x18. */
    {
        static const int qmf_lens[5] = { 512, 768, 1024, 1536, 2048 };
        int b = st->route_pitch_base;

        st->pitch_route_len[0] = qmf_lens[b];
        st->pitch_route_len[1] = qmf_lens[b + 1];
        st->pitch_route_len[2] = qmf_lens[b + 1]; /* reference decoder: ctx+0x7c == ctx+0x78 */
        st->pitch_route_len[3] = qmf_lens[b + 2];
    }

    /* ctx+0x18 via ((num_bands * half_rate) * br). */
    pitch_mult = lpec_f8e0(128.0 / ((double) num_bands * (double) half_rate) * (double) br);
    if (pitch_mult < 1)
        pitch_mult = 1;
    st->pitch_mult  = pitch_mult;
    st->pitch_scale = st->band_width;

    /* the reference decoder InitCodec: pitch_ref from
 * (6000 bps) or (other LP rates); LPEC uses /390c0.*/
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
        /* the reference decoder: (3*ref) rounded >> 1 into ctx+0xec/0xf0. */
        st->pitch_ref_route[1] = (pitch_ref_base * 3) >> 1;
    } else {
        st->pitch_ref_route[1] = (pitch_ref_base * 3 + 1) >> 1;
    }
    st->pitch_ref_route[2] = st->pitch_ref_route[1];
    st->pitch_ref_route[3] = pitch_ref_base * 2;
    st->pitch_ref = st->pitch_ref_route[0];

    /* ebp-0x94..-0x88 from ctx+0xc,+0x10,+0x8. */
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
        refs[3] = frame_size; /* ctx+0x8 (512 @ 8 kHz) */
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

        /* Second init loop: ctx+0xfc via (4194304/(i6*num_bands)+0.5). */
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

/* ascending quicksort (the reference decoder 056c0). */
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

/* advance noise index (ASM at.. /..). */
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
                    lpec_f4e0_noise_a[idx] * mix * lpec_f4e0_mix_b;

        pos++;
        idx = pos & 1023;
        /* the reference decoder: + (header labels are reversed). */
        {
            double v1 = lpec_f4e0_noise_a[idx] * mix * lpec_f4e0_mix_b +
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
                    lpec_f4e0_noise_a[idx] * mix * lpec_f4e0_mix_b;

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

static av_unused void lpec_excit_trim_counts(int counts[4], int rem_bits)
{
    int need = (counts[0] + counts[1] + counts[2]) * 8 + counts[3];

    while (need > rem_bits && need > 0) {
        if (counts[3] > 0)
            counts[3]--;
        else if (counts[2] > 0)
            counts[2]--;
        else if (counts[1] > 0)
            counts[1]--;
        else if (counts[0] > 0)
            counts[0]--;
        else
            break;
        need = (counts[0] + counts[1] + counts[2]) * 8 + counts[3];
    }
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

static const int *lpec_route_tpl_pick(int g0, int g1, int g2, int bit_budget)
{
    if (g1 == 92 && g2 == 168) {
        if (g0 == 28) {
            if (bit_budget >= 470)
                return lpec_route_voiced_tpl_28;
            if (bit_budget >= 464)
                return lpec_route_voiced_tpl_28b;
        }
        if (g0 == 26)
            return lpec_route_voiced_tpl_92;
    }
    if (g0 == 26 && g1 == 88 && g2 == 160 && bit_budget < 452)
        return lpec_route_voiced_tpl_lo;
    if (g0 == 26 && g1 == 88 && g2 == 168)
        return lpec_route_voiced_tpl;
    return NULL;
}

/* Scale voiced band template to arbitrary group total. */
static void lpec_route_tpl_targets(int *tgt, const int *tpl, int total, int nb)
{
    int tsum = 0, alloc = 0, b, i;
    int order[16], fracs[16];

    for (b = 0; b < nb; b++)
        tsum += tpl[b];
    if (tsum <= 0 || total <= 0)
        return;

    if (total == tsum) {
        memcpy(tgt, tpl, nb * sizeof(tgt[0]));
        return;
    }

    for (b = 0; b < nb; b++) {
        tgt[b] = total * tpl[b] / tsum;
        alloc += tgt[b];
    }
    if (total <= alloc)
        return;

    for (b = 0; b < nb; b++) {
        order[b] = b;
        fracs[b] = (total * tpl[b]) % tsum;
    }
    for (b = 0; b < nb - 1; b++) {
        int j, best = b;

        for (j = b + 1; j < nb; j++) {
            if (fracs[j] > fracs[best] ||
                (fracs[j] == fracs[best] && order[j] < order[best]))
                best = j;
        }
        if (best != b) {
            int tmp = order[b];

            order[b] = order[best];
            order[best] = tmp;
            tmp = fracs[b];
            fracs[b] = fracs[best];
            fracs[best] = tmp;
        }
    }
    for (i = 0; i < total - alloc; i++)
        tgt[order[i]]++;
}

/* Vo-neutral spread toward init-shaped weights (post-hack band balance). */
static av_unused void lpec_route_spread_groups(int route[40], const int init[40], int nb,
                                     int pitch_scale, int bit_budget)
{
    int g, g0t = 0, g1t = 0, g2t = 0, b;
    const int *tpl_set = NULL;

    if (nb == 10) {
        for (b = 0; b < nb; b++) {
            g0t += route[b * 4 + 0];
            g1t += route[b * 4 + 1];
            g2t += route[b * 4 + 2];
        }
        tpl_set = lpec_route_tpl_pick(g0t, g1t, g2t, bit_budget);
    }

    for (g = 0; g < 3; g++) {
        int total = 0, wsum = 0, w[16], tgt[16];
        int alloc, i, use_tpl = 0;

        for (b = 0; b < nb; b++)
            total += route[b * 4 + g];

        if (tpl_set && total > 0) {
            memset(tgt, 0, sizeof(tgt));
            lpec_route_tpl_targets(tgt, tpl_set + g * 10, total, nb);
            use_tpl = 1;
        } else {
            for (b = 0; b < nb; b++) {
                int iw = init[b * 4 + g];
                int low = nb - b;

                if (g < 2)
                    w[b] = iw > 0 ? iw * low * low : low * low;
                else if (b == 0)
                    w[b] = 10;
                else
                    w[b] = iw > 0 ? iw + low * low : low * low;
                wsum += w[b];
            }
            if (wsum <= 0)
                continue;

            alloc = 0;
            for (b = 0; b < nb; b++) {
                int num = total * w[b];

                tgt[b] = num / wsum;
                alloc += tgt[b];
            }
            if (total > alloc) {
                int order[16], fracs[16];

                for (b = 0; b < nb; b++) {
                    order[b] = b;
                    fracs[b] = (total * w[b]) % wsum;
                }
                for (b = 0; b < nb - 1; b++) {
                    int j, best = b;

                    for (j = b + 1; j < nb; j++) {
                        if (fracs[j] > fracs[best] ||
                            (fracs[j] == fracs[best] && order[j] < order[best]))
                            best = j;
                    }
                    if (best != b) {
                        int tmp = order[b];

                        order[b] = order[best];
                        order[best] = tmp;
                        tmp = fracs[b];
                        fracs[b] = fracs[best];
                        fracs[best] = tmp;
                    }
                }
                for (i = 0; i < total - alloc; i++)
                    tgt[order[i]]++;
            }
        }
        if (total <= 0)
            continue;

        for (i = 0; i < nb * total; i++) {
            int from = -1, to = -1, best = 0, fb, b2, occ, score;

            for (fb = 0; fb < nb; fb++) {
                if (route[fb * 4 + g] <= tgt[fb])
                    continue;
                for (b2 = 0; b2 < nb; b2++) {
                    if (route[b2 * 4 + g] >= tgt[b2])
                        continue;
                    occ = route[b2 * 4] + route[b2 * 4 + 1] + route[b2 * 4 + 2];
                    if (occ >= pitch_scale)
                        continue;
                    score = (route[fb * 4 + g] - tgt[fb]) +
                            (tgt[b2] - route[b2 * 4 + g]);
                    if (score > best) {
                        best = score;
                        from = fb;
                        to   = b2;
                    }
                }
            }
            if (from < 0)
                break;
            route[from * 4 + g]--;
            route[to * 4 + g]++;
        }

        if (use_tpl) {
            for (b = 0; b < nb; b++)
                route[b * 4 + g] = tgt[b];
        }

        /* Pull voiced group toward band-0 target (rem fill seeds low bands). */
        if (!use_tpl && tgt[0] > route[g]) {
            for (i = 0; i < nb * total; i++) {
                int moved = 0;

                if (route[g] >= tgt[0])
                    break;
                for (b = nb - 1; b > 0; b--) {
                    int occ0;

                    if (route[b * 4 + g] <= 0)
                        continue;
                    occ0 = route[0] + route[1] + route[2];
                    if (occ0 >= pitch_scale)
                        continue;
                    route[b * 4 + g]--;
                    route[g]++;
                    moved = 1;
                    break;
                }
                if (!moved)
                    break;
            }
        }
    }
}

static void lpec_dump_route40_stage(const char *tag, const int route[40],
                                    int half_slot, int seq)
{
    int df = getenv("LPEC_DUMP_ROUTE40_STAGE") ? atoi(getenv("LPEC_DUMP_ROUTE40_STAGE")) : -1;

    if (!getenv("LPEC_DUMP_ROUTE40_STAGE"))
        return;
    if (df >= 0 && seq != df * 2 + (seq & 1))
        return;

    fprintf(stderr, "LPEC_ROUTE40_%s seq=%d b0=%d,%d,%d,%d\n",
            tag, seq, route[0], route[1], route[2], route[3]);
}

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
        /* pitch table at 0x10c, index num_bands*ctx+0x140 + band. */
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
        static int r40_stage_seq;
        int seq = r40_stage_seq++;

        lpec_dump_route40_stage("init", route, half_slot, seq);
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

        /* param_3 <= sum ->, else. */
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
        static int r40_rem_seq;
        int seq = r40_rem_seq++;

        lpec_dump_route40_stage("rem", route, half_slot, seq);
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

void lpec_xa_lsf_indices_to_i16(int order, int num_cb,
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

    /*reads ctx+0x2e8 + param_2*0x22 (slot i16), not CB indices.
 * Mode-0 flag==0: slot1 is avg(slot0,slot2) at 0x30a -- must use override.*/
    if (lsf_i16_override)
        memcpy(lsf_i16, lsf_i16_override, (lpc_order + 1) * sizeof(int16_t));
    else if (st->is_xaudio_lp && st->num_bands == 8)
        lpec_xa_lsf_indices_to_i16(lpc_order, num_lsf_cb, lsf_idx, lsf_i16);
    else
        lpec_lsf_indices_to_i16(st->num_bands, lpc_order, num_lsf_cb, lsf_idx, lsf_i16);
    lpec_a110_route_shorts(lsf_i16, a110_out, lpc_order);

    if (getenv("LPEC_DUMP_A110")) {
        int df = atoi(getenv("LPEC_DUMP_A110"));
        static int a110_dump_seq;
        int seq = a110_dump_seq++;

        if (lpec_excit_dump_wants(df, st->dump_frame)) {
            char path[128];
            FILE *fp;

            snprintf(path, sizeof(path), "/tmp/lpec_a110_h%d_%d.bin", half_idx, seq);
            fp = fopen(path, "wb");
            if (fp) {
                fwrite(lsf_i16, sizeof(int16_t), lpc_order + 1, fp);
                fwrite(a110_out, sizeof(int16_t), lpc_order + 1, fp);
                fclose(fp);
            }
        }
    }

    memset(route32, 0, pitch_cap * sizeof(route32[0]));
    /* the reference decoder 0b8d0 param_2: ctx+0x12c/0x128[half] pitch_win (0=UV). */
    lpec_c680_route(a110_out, pitch_win, pitch_e, route32, pitch_cap, lpc_order);

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

/* the reference decoder 05330 @ 055b0/05606: head zero + band gain use ctx+0x3c[route_tbl_idx]
 * (ebp-0x28 from edi @ 05350); tail zero uses ctx+0x70/0x4c[half_slot].*/
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
        lpec_xa_lsf_indices_to_i16(lpc_order, num_lsf_cb, lsf_idx, lsf_i16);
    else
        lpec_lsf_indices_to_i16(st->num_bands, lpc_order, num_lsf_cb, lsf_idx, lsf_i16);
    lpec_a110_route_shorts(lsf_i16, a110_out, lpc_order);

    memset(route32, 0, pitch_cap * sizeof(route32[0]));
    lpec_c680_route(a110_out, pitch_win, pitch_e, route32, pitch_cap, lpc_order);

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

/* - the reference decoder 0x19 excitation path (03920 / 03250 / 056c0 / db20) ----
 *
 * Quality 0x19 (the reference decoder/Xaudio) does NOT use. It runs a separate
 * pipeline that performs a per-band arithmetic redistribution of the route
 * counts (via the range-split ) before the scatter
 * and noise fill. This block ports that path.
*/

/* 32-bit truncating multiply (matches x86 imul low-32 semantics). */
static inline int lpec_imul32(int a, int b)
{
    return (int) ((uint32_t) a * (uint32_t) b);
}

/*range-coder probability split. Given interval params (p,q),
 * fills out[0..2] with the three sub-interval sizes. Pure integer arithmetic
 * with x86 arithmetic-right-shift semantics.*/
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

/* per-band arithmetic route-count decoder (the reference decoder 05330). */
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

static int lpec_db20_next_pos(int pos)
{
    return lpec_f4e0_next_pos(pos);
}

static void lpec_noise_fill_db20(double *exc, const LPECRouteEntry *route,
                                 int start, int end, double mix, int *npos)
{
    const double inv_mix = 1.0 - mix;
    const double mix_a = 0.9;
    const double mix_b = 0.9;
    int i = start;
    int pos = *npos;

    end = start + ((end - start) & ~1);

    while (i + 1 < end) {
        int d0 = route[i].dst;
        int d1 = route[i + 1].dst;
        int idx = pos & 1023;
        double v0 = lpec_xa_noise_db20_44960[idx] * inv_mix * mix_a +
                    lpec_xa_noise_db20_42960[idx] * mix * mix_b;

        pos++;
        idx = pos & 1023;
        {
            double v1 = lpec_xa_noise_db20_42960[idx] * mix * mix_b +
                        lpec_xa_noise_db20_44960[idx] * inv_mix * mix_a;

            pos = lpec_db20_next_pos(pos);
            exc[d0] = v0;
            exc[d1] = v1;
        }
        i += 2;
    }
    for (; i < end; i++) {
        int d0 = route[i].dst;
        int idx = pos & 1023;
        double v0 = lpec_xa_noise_db20_44960[idx] * inv_mix * mix_a +
                    lpec_xa_noise_db20_42960[idx] * mix * mix_b;

        pos = lpec_db20_next_pos(pos);
        exc[d0] = v0;
    }
    *npos = pos;
}

/* group loop (partial tap state at ebp-0xf4 / cb_slot at ebp-0x100). */
static void lpec_excit_group_056c0(LPECExcitState *st, double *excit,
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

/* + (the reference decoder 05330 excitation scatter). */
static av_unused void lpec_excit_decode_056c0(LPECExcitState *st, GetBitContext *gb,
                                    double *excit, const int route[40],
                                    LPECRouteEntry *rtbl, int band_step,
                                    int subfr_len, double pitch_mix,
                                    const int counts[4], int bits_limit)
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
        lpec_route_sort_a610(band_rt, sort_n);

        for (g = 0; g < 3; g++)
            lpec_excit_group_056c0(st, excit, band_rt, band_samples, subfr_len,
                                   &sort_idx, g, rb[g], cb_idx, cb_base[g],
                                   total_cb, cb_slot, cb_tap);

        for (n = 0; n < rb[3]; n++) {
            int v = (uv_flags[uv >> 5] >> (uv & 31)) & 1;

            if (sort_idx < band_samples) {
                int d = band_rt[sort_idx].dst;

                if ((unsigned) d < (unsigned) subfr_len)
                    excit[d] = v ? LPEC_XA_NOISE_UV_NEG : LPEC_XA_NOISE_UV_POS;
                sort_idx++;
            }
            uv++;
        }

        if (sort_idx < band_samples)
            lpec_noise_fill_db20(excit, band_rt, sort_idx, band_samples,
                                 pitch_mix, &st->f4e0_noise_pos);

        rt += band_samples;
    }
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

        if (getenv("LPEC_TRACE060E0") && b == 0) {
            static int t060e0_seq;
            int seq = t060e0_seq;
            const char *env = getenv("LPEC_TRACE060E0");
            int want = env && (!strcmp(env, "-1") || seq == atoi(env) * 2);

            if (want) {
                int i;
                fprintf(stderr, "LPEC_TRACE060E0 seq=%d band0 sorted:", seq);
                for (i = 0; i < 6; i++)
                    fprintf(stderr, " [%d]dst=%d", i, band_rt[i].dst);
                fprintf(stderr, "\n");
            }
        }

        if (getenv("LPEC_DUMP_RTBL_SORTED")) {
            int df = atoi(getenv("LPEC_DUMP_RTBL_SORTED"));
            static int sort_dump_seq;

            if (df < 0 || sort_dump_seq++ == df * 2) {
                int n = band_samples < 16 ? band_samples : 16;
                int i;

                fprintf(stderr, "LPEC_DUMP_RTBL_SORTED band=%d n=%d:", b, band_samples);
                for (i = 0; i < n; i++)
                    fprintf(stderr, " [%d]dst=%d val=%.0f", i,
                            band_rt[i].dst, band_rt[i].val);
                fprintf(stderr, "\n");
            }
        }

        for (g = 0; g < 3; g++)
            lpec_excit_group_060e0(st, excit, band_rt, band_samples, subfr_len,
                                   &sort_idx, g, rb[g], cb_idx, cb_base[g],
                                   total_cb, cb_slot, cb_tap);

        if (getenv("LPEC_TRACE060E0") && b == 0) {
            static int t060e0_seq;
            int seq = t060e0_seq++;
            const char *env = getenv("LPEC_TRACE060E0");
            int want = env && (!strcmp(env, "-1") || seq == atoi(env) * 2);

            if (want) {
                fprintf(stderr,
                        "LPEC_TRACE060E0 seq=%d post_grp sort=%d excit2..5=%.6f,%.6f,%.6f,%.6f "
                        "rb=%d,%d,%d,%d cb_slot=%d,%d,%d\n",
                        seq, sort_idx, excit[2], excit[3], excit[4], excit[5],
                        rb[0], rb[1], rb[2], rb[3],
                        cb_slot[0], cb_slot[1], cb_slot[2]);
            }
        }

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

        if (getenv("LPEC_TRACE060E0") && b == 0) {
            static int t060e0_uv_seq;
            int seq = t060e0_uv_seq++;
            const char *env = getenv("LPEC_TRACE060E0");
            int want = env && (!strcmp(env, "-1") || seq == atoi(env) * 2);

            if (want) {
                fprintf(stderr,
                        "LPEC_TRACE060E0 seq=%d post_uv sort=%d excit2..5=%.6f,%.6f,%.6f,%.6f\n",
                        seq, sort_idx, excit[2], excit[3], excit[4], excit[5]);
            }
        }

        if (sort_idx < band_samples)
            lpec_noise_fill_f4e0(excit, band_rt, sort_idx, band_samples,
                                 pitch_mix, &st->f4e0_noise_pos,
                                 subfr_len);

        rt += band_samples;
    }

    if (getenv("LPEC_DUMP_NOISE_POS")) {
        static int noise_dump_seq;
        int seq = noise_dump_seq++;
        int df = atoi(getenv("LPEC_DUMP_NOISE_POS"));

        if (df < 0 || seq == df)
            fprintf(stderr, "LPEC_DUMP_NOISE_POS seq=%d pos=%d\n",
                    seq, st->f4e0_noise_pos);
    }
}

/* Zero head/tail only (the reference decoder 05330 ); band gain is separate. */
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
            excit[i] *= bg * gain;
        pos = end;
    }
}

/* the reference decoder 05330: half=2, mode=1 -> ctx+0x2c[half_slot] band_step. */
static int lpec_xa_mode1_band_slot(const LPECExcitState *st, int mode, int route_tbl_idx)
{
    if (st->is_xaudio_lp && mode == 1)
        return route_tbl_idx;
    return -1;
}

int lpec_generate_excitation(LPECExcitState *st, GetBitContext *gb,
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
                                            lpec_excit_route_idx(mode, half_idx1));
    excit_work = subfr_len;
    if (mode == 2 && half_idx1 == 2)
        /* the reference decoder mode-2 h2: ab80 seg=ctx+0x4 (512 @ 8 kHz), not
 * pitch_route_len[2]>>1 (384). Under-filling poisons excit_src[384:512).*/
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
 * in excit_src[seg:flen) (e.g. mode 1 excit_work=384 vs QMF desc768 flen=768).*/
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

    /* bit budget for 04450 = param_4 - ctx+0x14. */
    rem_bits = route_budget - st->bit_overhead;
    if (rem_bits < 0)
        rem_bits = 0;

    {
        int route_tbl_idx = lpec_excit_route_idx(mode, half_idx1);


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

    if (getenv("LPEC_DUMP_ROUTE40")) {
        int df = atoi(getenv("LPEC_DUMP_ROUTE40"));
        static int r40_dump_seq;
        int seq = r40_dump_seq++;

        if (lpec_excit_dump_wants(df, st->dump_frame)) {
            char path[128];
            FILE *fp;
            int hs = half_slot & 3;

            fprintf(stderr,
                    "LPEC_04450 half=%d seq=%d rem=%d arg=%d ref=%d scale=%d mult=%d "
                    "slot=%d excit_start=%d gain_idx=%d gain=%.6f cb=%d/%d "
                    "fin_start=%d fin_step=%d cnt=%d,%d,%d,%d b0=%d,%d,%d,%d\n",
                    half_idx1, seq, rem_bits, pitch_route_arg,
                    st->pitch_ref_route[hs], st->pitch_scale_route[hs],
                    st->pitch_mult_route[hs], half_slot,
                    st->route_excit_start[hs], gain_idx, gain, cb_idx1, cb_idx2,
                    st->is_xaudio_lp && mode == 1 ? st->route_excit_start[2]
                                                  : st->route_excit_start[hs],
                    st->is_xaudio_lp && mode == 1 ? st->pitch_scale_route[hs]
                                                  : st->pitch_scale_route[hs],
                    counts[0], counts[1], counts[2], counts[3],
                    route[0], route[1], route[2], route[3]);
            snprintf(path, sizeof(path), "/tmp/lpec_route40_h%d_%d.bin",
                     half_idx1, seq);
            fp = fopen(path, "wb");
            if (fp) {
                fwrite(route, sizeof(int), 40, fp);
                fclose(fp);
            }
        }
    }

    if (getenv("LPEC_DUMP_RTBL")) {
        int df = atoi(getenv("LPEC_DUMP_RTBL"));
        static int rt_dump_seq;
        int seq = rt_dump_seq++;
        {
            int hs = half_slot & 3;
            int band_step = st->pitch_scale_route[hs];
            int pairs = band_step * st->num_bands;
            int last_band = st->num_bands * band_step;

            if (last_band > excit_work)
                pairs -= last_band - excit_work;

            if (lpec_excit_dump_wants(df, st->dump_frame)) {
                char path[128];
                FILE *fp;
                int i;

                snprintf(path, sizeof(path), "/tmp/lpec_rtbl_h%d_%d.bin", half_idx1, seq);
                fp = fopen(path, "wb");
                if (fp) {
                    for (i = 0; i < pairs; i++) {
                        int32_t dst = rtbl[i].dst;
                        int32_t idx = rtbl[i].idx;
                        fwrite(&rtbl[i].val, sizeof(double), 1, fp);
                        fwrite(&dst, sizeof(int32_t), 1, fp);
                        fwrite(&idx, sizeof(int32_t), 1, fp);
                    }
                    fclose(fp);
                }
                fprintf(stderr, "LPEC_DUMP_RTBL half=%d seq=%d pairs=%d pitch_e=%.6f "
                        "excit_start=%d band_step=%d\n",
                        half_idx1, seq, pairs, pitch_e,
                        st->route_excit_start[hs], band_step);
                if (getenv("LPEC_DUMP_EXCIT_META")) {
                    char mpath[128];
                    FILE *mfp;
                    int meta[2] = { st->route_excit_start[hs], band_step };

                    snprintf(mpath, sizeof(mpath), "/tmp/lpec_excit_meta_h%d_%d.bin",
                             half_idx1, seq);
                    mfp = fopen(mpath, "wb");
                    if (mfp) {
                        fwrite(meta, sizeof(int), 2, mfp);
                        fclose(mfp);
                    }
                }
            }
        }
    }

    if (getenv("LPEC_DUMP_ROUTE32")) {
        int df = atoi(getenv("LPEC_DUMP_ROUTE32"));
        static int r32_dump_seq;
        int seq = r32_dump_seq++;
        int pitch_len = lpec_route_pitch_len(st, half_slot);
        int pitch_cap = pitch_len < LPEC_ROUTE_PITCH_MAX ? pitch_len : LPEC_ROUTE_PITCH_MAX;

        if (lpec_excit_dump_wants(df, st->dump_frame)) {
            char path[128];
            FILE *fp;

            snprintf(path, sizeof(path), "/tmp/lpec_route32_h%d_%d.bin", half_idx1, seq);
            fp = fopen(path, "wb");
            if (fp) {
                fwrite(st->route_pitch, sizeof(int32_t), pitch_cap, fp);
                fclose(fp);
            }
        }
    }

    if (lpec_trace_on()) {
        static int tr_half;
        int ti = tr_half++;

        fprintf(stderr, "LPEC_TRACE excit half=%d/#%d gain_idx=%d cb=%d/%d budget=%d pitch_win=%d lag_off=%d mode=%d\n",
                half_idx1, ti, gain_idx, cb_idx1, cb_idx2, bit_budget, pitch_win,
                pitch_lag_off, mode);
        fprintf(stderr, "  gain=%.6f pitch_e=%.6f band_cb[0]=%.6f rtbl[0].val=%.0f dst=%d rtbl[1].val=%.0f dst=%d\n",
                gain, pitch_e, band_cb[0], rtbl[0].val, rtbl[0].dst, rtbl[1].val, rtbl[1].dst);
        fprintf(stderr, "  route band0: %d %d %d %d counts: %d %d %d %d\n",
                route[0], route[1], route[2], route[3],
                counts[0], counts[1], counts[2], counts[3]);
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

    if (getenv("LPEC_BITLOG")) {
        fprintf(stderr,
                "LPEC_BITLOG excit half=%d hdr_end=%d budget=%d rem_route=%d "
                "route_limit=%d total_cb=%d\n",
                half_idx1, get_bits_count(gb), bit_budget, rem_bits, bits_limit,
                counts[0] + counts[1] + counts[2]);
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

    if (getenv("LPEC_BITLOG")) {
        fprintf(stderr, "LPEC_BITLOG excit half=%d route_end=%d\n",
                half_idx1, get_bits_count(gb));
    }

    if (getenv("LPEC_DUMP_CBIDX")) {
        int df = atoi(getenv("LPEC_DUMP_CBIDX"));
        static int cb_dump_seq;
        int seq = cb_dump_seq++;

        if (lpec_excit_dump_wants(df, st->dump_frame)) {
            char path[128];
            FILE *fp;
            int ncb = counts[0] + counts[1] + counts[2];
            int cb_idx_max = (int) sizeof(st->cb_idx_scratch);

            snprintf(path, sizeof(path), "/tmp/lpec_cbidx_h%d_%d.bin", half_idx1, seq);
            fp = fopen(path, "wb");
            if (fp) {
                if (ncb > cb_idx_max)
                    ncb = cb_idx_max;
                fwrite(st->cb_idx_scratch, 1, ncb, fp);
                fclose(fp);
            }
            fprintf(stderr, "LPEC_DUMP_CBIDX half=%d seq=%d ncb=%d counts=%d,%d,%d,%d "
                    "route_g0=%d g1=%d pitch_route_arg=%d rem_bits=%d\n",
                    half_idx1, seq, ncb,
                    counts[0], counts[1], counts[2], counts[3],
                    counts[0] ? route[0] : 0, counts[1] ? route[4] : 0,
                    pitch_route_arg, rem_bits);
        }
    }

    /* Dump before finish_05d50 (the reference hook at, right after 060e0). */
    if (getenv("LPEC_DUMP_RAW_EXCIT")) {
        int df = atoi(getenv("LPEC_DUMP_RAW_EXCIT"));
        static int raw_dump_seq;
        int seq = raw_dump_seq++;
        int hs = half_slot & 3;

        if (lpec_excit_dump_wants(df, st->dump_frame)) {
            char path[128];
            FILE *fp;
            int meta[2] = { st->route_excit_start[hs], st->pitch_scale_route[hs] };

            snprintf(path, sizeof(path), "/tmp/lpec_raw_excit_h%d_%d.bin",
                     half_idx1, seq);
            fp = fopen(path, "wb");
            if (fp) {
                fwrite(excitation, sizeof(double), excit_work, fp);
                fclose(fp);
            }
            snprintf(path, sizeof(path), "/tmp/lpec_excit_meta_h%d_%d.bin",
                     half_idx1, seq);
            fp = fopen(path, "wb");
            if (fp) {
                fwrite(meta, sizeof(int), 2, fp);
                fclose(fp);
            }
        }
    }

    {
        int finish_len = st->pitch_route_len[half_slot & 3] >> 1;
        int route_tbl_idx = lpec_excit_route_idx(mode, half_idx1);

        if (mode == 2 && half_idx1 == 2)
            finish_len = frame_size;
        if (st->is_xaudio_lp && mode == 1)
            lpec_excit_finish_05d50(st, excitation, finish_len, mode, half_slot,
                                    -1, band_cb, gain, route_tbl_idx);
        else
            lpec_excit_finish_05d50(st, excitation, finish_len, mode, half_slot,
                                    xa_band_slot, band_cb, gain, route_tbl_idx);
    }

    if (getenv("LPEC_DUMP_CTX")) {
        int df = atoi(getenv("LPEC_DUMP_CTX"));
        static int postfin_seq;
        int seq = postfin_seq++;

        if (lpec_excit_dump_wants(df, st->dump_frame)) {
            char path[128];
            FILE *fp;

            snprintf(path, sizeof(path), "/tmp/lpec_postfin_h%d_%d.bin",
                     half_idx1, seq);
            fp = fopen(path, "wb");
            if (fp) {
                fwrite(excitation, sizeof(double), excit_work, fp);
                fclose(fp);
            }
        }
    }

    if (getenv("LPEC_TRACE")) {
        static int n;
        if (n++ < 16) {
            double mx = 0, rms = 0;
            int ti;

            for (ti = 0; ti < excit_work; ti++) {
                mx = fmax(mx, fabs(excitation[ti]));
                rms += excitation[ti] * excitation[ti];
            }
            fprintf(stderr, "LPEC_TRACE excit out half=%d max=%.6f rms=%.6f first=%.6f\n",
                    half_idx1, mx, sqrt(rms / excit_work), excitation[0]);
        }
    }

    for (i = 0; i < excit_work; i++) {
        st->excit_hist[st->excit_pos] = excitation[i];
        st->excit_pos = (st->excit_pos + 1) % LPEC_EXCIT_HIST;
    }
    return 0;
}
