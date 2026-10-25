/*
 * Sony AT-X high frequency compensation (HFC) for ATRAC3+
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/*
 * This code reproduces the reference decoder bit for bit. Where the
 * reference keeps a value in an x87 register at 53-bit precision, this code
 * uses a double. Where it stores a value to memory, this code uses a float.
 * The order of operations is the same. Fused multiply-add would change the
 * results. GCC does not contract in ISO C mode, and clang needs the pragma.
 */
#ifdef __clang__
#pragma STDC FP_CONTRACT OFF
#endif

#include <math.h>
#include <string.h>

#include "libavutil/attributes.h"
#include "get_bits.h"
#include "atrac3plus_hfc.h"
#include "atrac3plus_hfc_data.h"

#define HFC_FRAME_LEN 2048
#define HFC_VLC_BITS  9

static VLCElem hfc_vlc_buf[8056];
static const VLCElem *hfc_vlc[8];

av_cold void ff_atrac3p_hfc_init_static(void)
{
    VLCInitState state = VLC_INIT_STATE(hfc_vlc_buf);

    for (int i = 0; i < 8; i++)
        hfc_vlc[i] = ff_vlc_init_tables_from_lengths(&state, HFC_VLC_BITS, 64,
                                                     hfc_huff_lens[i], 1,
                                                     hfc_huff_syms[i], 1, 1,
                                                     -32, 0);
}

static void build_patches(Atrac3pHfcContext *s, int start, int end, int base)
{
    int w = start - base, span = end - start + 1, n, k, b;

    s->base_band = base;
    n = span / w;
    if (span % w)
        n++;
    s->num_patches = n;
    s->patch_start[0] = start;
    s->patch_end[0]   = base - 1 + w * 2;
    for (k = 1; k < n; k++) {
        s->patch_start[k] = s->patch_end[k - 1] + 1;
        s->patch_end[k]   = s->base_band - 1 + w * (k + 2);
    }
    if (end < s->patch_end[n - 1])
        s->patch_end[n - 1] = end;

    for (k = 0; k < n; k++)
        for (b = s->patch_start[k]; b <= s->patch_end[k]; b++) {
            if (!s->mirror) {
                s->patch_map[b] = b - (s->patch_start[0] - s->base_band) * (k + 1);
            } else {
                int src = s->patch_start[k] * 2 - b - 1;

                s->patch_map[b] = src;
                if (s->patch_start[0] <= src)
                    s->patch_map[b] = s->patch_map[src];
            }
        }
}

static void group_bands(int start, int end, int *list, int *count)
{
    int p;

    list[0] = start;
    *count  = 1;
    for (p = start + 1; p <= end; p++) {
        switch (p) {
        case 0: case 1: case 2: case 3: case 4: case 5: case 7: case 9: case 12:
            list[(*count)++] = p;
        }
    }
}

static void group_bands13(int start, int end, int *list, int *count)
{
    int p;

    list[0] = start;
    *count  = 1;
    for (p = start + 1; p <= end; p++)
        if (p < 13)
            list[(*count)++] = p;
}

static void fill_groups(int *a, int start, int end, int count, const int *list)
{
    int i, j;

    for (j = 0; j < start; j++)
        a[j] = 0;
    for (i = 0; i < count; i++) {
        int last = i == count - 1 ? end : list[i + 1] - 1;

        for (j = list[i] + 1; j <= last; j++)
            a[j] = a[j - 1];
    }
    for (j = end + 1; j < HFC_BANDS; j++)
        a[j] = 0;
}

static int tone_decode(int start, int end, int code, int *count, int *idx)
{
    if (code >= 0xd3) {
        *count = 0;
        return AVERROR_INVALIDDATA;
    }
    if (!code) {
        *count = 0;
        return 0;
    }
    if (code - 1 < 20) {
        *count = 1;
        idx[0]  = code - 1;
        return hfc_tone_band[idx[0]] + start <= end ? 0 : AVERROR_INVALIDDATA;
    }
    *count = 2;
    {
        int q = (code - 21) / 19, r = (code - 21) % 19;

        if (r < q) {
            idx[0] = 19 - q;
            idx[1] = 20 - q + r;
        } else {
            idx[0] = q;
            idx[1] = r + 1;
        }
    }
    if (hfc_tone_band[idx[0]] + start <= end && hfc_tone_band[idx[1]] + start <= end)
        return 0;
    return AVERROR_INVALIDDATA;
}

static uint16_t payload_hash(const uint8_t *buf, int bits)
{
    uint32_t h = buf[0];

    for (int i = 1; i < bits >> 3; i++)
        h = buf[i] + h * 0x7d33;
    return h;
}

int ff_atrac3p_hfc_parse(Atrac3pHfcContext *s, const uint8_t *buf, int len)
{
    GetBitContext gb;
    int start, end, end0, ch, nst, b, t, k, v, ret, nch = s->nch;
    int dt[2] = { 0 };

    if ((ret = init_get_bits8(&gb, buf, len)) < 0)
        return ret;

#define CHECK_POS() do { if (get_bits_left(&gb) < 0) return AVERROR_INVALIDDATA; } while (0)

    v = get_bits(&gb, 3);
    CHECK_POS();
    start = s->patch_start[0];
    if (start < 6) {
        end = v - 2 + start * 2;
    } else {
        int e = 8 - start;

        if (e < 0)
            e = 0;
        end = start + e + v;
    }
    if (end > 15)
        end = 15;
    if (end < start)
        return AVERROR_INVALIDDATA;
    end0 = end;
    build_patches(s, start, end, s->base_band);
    group_bands(start, end, s->groups, &s->num_groups);
    s->noise_sel = get_bits(&gb, 2);
    v = get_bits(&gb, 1);
    CHECK_POS();
    s->num_slots = v ? 8 : 16;
    s->slot_len  = HFC_FRAME_LEN / s->num_slots;
    s->env_mode[0] = get_bits(&gb, 2);
    CHECK_POS();
    if (nch == 2)
        s->env_mode[1] = s->env_mode[0];
    s->smooth_mode = get_bits(&gb, 2);
    CHECK_POS();
    s->limit_level = get_bits(&gb, 3);
    CHECK_POS();
    if (nch < 2) {
        s->coupled = 0;
    } else {
        s->coupled = get_bits(&gb, 1);
        CHECK_POS();
    }
    nst = s->coupled ? 1 : nch;
    end = s->patch_end[s->num_patches - 1];
    for (ch = 0; ch < nst; ch++) {
        int *sel = s->band_sel[ch];

        s->sel_grouped[ch] = get_bits(&gb, 1);
        CHECK_POS();
        if (!s->sel_grouped[ch]) {
            sel[start] = get_bits(&gb, 2);
            CHECK_POS();
            for (b = start + 1; b <= end; b++)
                sel[b] = sel[start];
        } else {
            for (k = 0; k < s->num_groups; k++) {
                sel[s->groups[k]] = get_bits(&gb, 2);
                CHECK_POS();
            }
            fill_groups(sel, start, end, s->num_groups, s->groups);
        }
    }
    if (s->coupled) {
        s->sel_grouped[1] = s->sel_grouped[0];
        memcpy(s->band_sel[1], s->band_sel[0], sizeof(s->band_sel[0]));
    }
    s->tones_present = get_bits(&gb, 1);
    CHECK_POS();
    if (!s->tones_present) {
        for (ch = 0; ch < nch; ch++)
            s->tone_count[ch] = 0;
    } else {
        if (nch < 2) {
            s->tones_coupled = 0;
        } else {
            s->tones_coupled = get_bits(&gb, 1);
            CHECK_POS();
        }
        for (ch = 0; ch < nch; ch++) {
            s->tone_code[ch] = get_bits(&gb, 8);
            CHECK_POS();
            if (tone_decode(start, end, s->tone_code[ch], &s->tone_count[ch], s->tone_index[ch]) < 0)
                return AVERROR_INVALIDDATA;
            s->tone_flag[ch] = get_bits(&gb, 1);
            CHECK_POS();
            if (s->tones_coupled) {
                s->tone_count[1] = s->tone_count[0];
                for (k = 0; k < s->tone_count[0]; k++)
                    s->tone_index[1][k] = s->tone_index[0][k];
                s->tone_flag[1] = s->tone_flag[0];
                break;
            }
        }
    }
    for (ch = 0; ch < nch; ch++) {
        int *env = s->env_idx[ch], nslots = s->num_slots, cnt;
        const int *list;

        v = get_bits(&gb, nslots - 1);
        CHECK_POS();
        for (t = 0; t < nslots - 1; t++)
            s->time_flags[ch][t] = (v >> t) & 1;
        s->time_flags[ch][nslots - 1] = 0;
        if (s->env_mode[ch] == 0) {
            group_bands13(start, end0, s->groups13, &s->num_groups13);
            for (k = 0; k < s->num_groups13; k++) {
                env[s->groups13[k]] = get_bits(&gb, 3);
                CHECK_POS();
            }
            cnt  = s->num_groups13;
            list = s->groups13;
        } else if (s->env_mode[ch] == 1) {
            env[s->groups[0]] = get_bits(&gb, 4);
            CHECK_POS();
            for (k = 1; k < s->num_groups; k++)
                env[s->groups[k]] = env[s->groups[0]];
            cnt  = s->num_groups;
            list = s->groups;
        } else {
            for (k = 0; k < s->num_groups; k++) {
                env[s->groups[k]] = get_bits(&gb, 4);
                CHECK_POS();
            }
            cnt  = s->num_groups;
            list = s->groups;
        }
        fill_groups(env, start, end, cnt, list);
        s->time_group[ch] = get_bits(&gb, 1);
        CHECK_POS();
        s->env_offset[ch] = get_bits(&gb, 4);
        CHECK_POS();
        dt[ch] = get_bits(&gb, 1);
        CHECK_POS();
        for (b = start; b <= end; b++) {
            int sel = b == start ? 0 : s->band_sel[ch][b];
            int q   = get_vlc2(&gb, hfc_vlc[sel], HFC_VLC_BITS, 3);

            CHECK_POS();
            s->env_q[ch][0][b] = q;
            for (t = 1; t < s->num_slots; t++) {
                if (s->time_flags[ch][t - 1]) {
                    s->env_q[ch][t][b] = s->env_q[ch][t - 1][b];
                    continue;
                }
                q = get_vlc2(&gb, hfc_vlc[sel + dt[ch] * 4], HFC_VLC_BITS, 3);
                CHECK_POS();
                s->env_q[ch][t][b] = q;
                if (dt[ch]) {
                    s->env_q[ch][t][b] += s->env_q[ch][0][b];
                    if (s->env_q[ch][t][b] < -32 || s->env_q[ch][t][b] > 31)
                        return AVERROR_INVALIDDATA;
                }
            }
        }
    }
#undef CHECK_POS
    s->seed     = payload_hash(buf, get_bits_count(&gb));
    s->no_parse = 0;
    return 0;
}

static void noise_seed(Atrac3pHfcContext *s, int start, int end)
{
    static const int off[4] = { 199, 0x1c9, 0x2ef, 0x38f };
    int b;

    s->noise_idx[start] = (off[s->noise_sel] + (int16_t)s->seed) & 0x3ff;
    for (b = start + 1; b <= end; b++)
        s->noise_idx[b] = (s->noise_idx[b - 1] + 0x80) & 0x3ff;
}

static void env_resample(float e[][HFC_BANDS], int hi, int nsl)
{
    int b;

    if (nsl == 8) {
        for (b = 0; b <= hi; b++) {
            double p0 = e[0][b], p1 = e[1][b], p2 = e[2][b], p3 = e[3][b];

            e[0][b] = (float)((p0 * p0 + p1 * p1) * 0.5);
            e[1][b] = (float)((p2 * p2 + p3 * p3) * 0.5);
            e[0][b] = (float)sqrt(e[0][b]);
            e[1][b] = (float)sqrt(e[1][b]);
        }
    } else {
        for (b = 0; b <= hi; b++) {
            e[3][b] = e[1][b];
            e[2][b] = e[1][b];
            e[1][b] = e[0][b];
        }
    }
}

#define TONE_REC(s, j) ((s)->tone_rec[(s)->tone_sel[j]])

static void tone_setup(Atrac3pHfcContext *s)
{
    int ch, k, i, start = s->patch_start[0];

    for (ch = 0; ch < s->nch; ch++) {
        const int *cur = s->env_idx[ch], *prev = s->prev_env_idx[ch];
        int cnt[3] = { 0 }, t;
        Atrac3pHfcWord *P, *C;

        t = s->tone_sel[2 * ch + 1];
        s->tone_sel[2 * ch + 1] = s->tone_sel[2 * ch];
        s->tone_sel[2 * ch]     = t;
        P = TONE_REC(s, 2 * ch);
        for (i = 0; i < P[0].i; i++) {
            P[12 + 2 * i]   = P[13 + 2 * i];
            P[13 + 2 * i].f = hfc_gain_src[s->env_mode[ch] * 16 + cur[P[2 + i].i + start]];
        }
        C = TONE_REC(s, 2 * ch + 1);
        C[0].i = s->tone_count[ch];
        C[1].i = s->tone_flag[ch];
        for (k = 0; k < C[0].i; k++) {
            int idx = s->tone_index[ch][k], off, ph, b;

            off       = hfc_tone_band[idx];
            C[2 + k].i  = off;
            ph        = hfc_tone_step[idx + (start & 1) * 20];
            C[4 + k].i  = ph;
            C[10 + k].f = hfc_tone_amp[ph];
            C[16 + k].f = hfc_tone_leak[ph];
            b = off + start;
            if (ph < 4) {
                C[6 + k].i = 12;
                C[8 + k].i = hfc_tone_sign[(b & 1) * 2];
                if (b == 0 || b == 15) {
                    C[10 + k].f = 1.0f;
                    C[16 + k].i = 0;
                    C[6 + k].i  = 0;
                    C[8 + k].i  = 0;
                }
            } else {
                C[8 + k].i = hfc_tone_sign[(b & 1) * 2 + 1];
                C[6 + k].i = 4;
            }
            cnt[k] = 0;
            C[12 + 2 * k].f = hfc_gain_src[s->prev_env_mode[ch] * 16 + prev[b]];
            C[13 + 2 * k].f = hfc_gain_src[s->env_mode[ch] * 16 + cur[b]];
            if (k > 0 && C[2 + k - 1].i == off) {
                int j = k - 1;

                cnt[k] = cnt[k - 1] + 1;
                do {
                    cnt[j] = cnt[k];
                    j--;
                } while (j >= 0 && C[2 + j].i == off);
            }
        }
        for (k = 0; k < C[0].i; k++)
            C[10 + k].f = (float)((double)hfc_tone_share[cnt[k]] * C[10 + k].f);
    }
}

static void coded_energy(float **sb, float **ev, const int *flags, int group,
                         float *gain, int hi, int nsl, int n)
{
    int t, b, i;

    for (t = 0; t < nsl; t++) {
        gain[t] = 0;
        for (b = 0; b <= hi; b++) {
            const float *x = sb[b] + t * n;
            double acc = 0.0;

            for (i = 0; i < n; i++)
                acc = acc + (double)x[i] * x[i];
            acc     = acc * hfc_recip[n];
            gain[t] = (float)(acc + gain[t]);
            ev[t][b] = (float)sqrt(acc);
        }
        gain[t] = (float)((double)gain[t] * hfc_recip[hi + 1]);
    }
    if (!group) {
        double acc = 0.0;

        for (t = 0; t < nsl; t++)
            acc = acc + gain[t];
        gain[0] = (float)sqrt(acc * hfc_recip[nsl]);
        for (t = 1; t < nsl; t++)
            gain[t] = gain[0];
    } else {
        t = 0;
        while (t < nsl) {
            double acc = gain[t];
            int len = 1, e, j;

            if (flags[t]) {
                j = t;
                do {
                    acc = acc + gain[j + 1];
                    j++;
                    len++;
                } while (flags[j]);
            }
            e = len + t;
            gain[t] = (float)sqrt(acc * hfc_recip[len]);
            for (i = t + 1; i < e; i++)
                gain[i] = gain[t];
            t = e;
        }
    }
}

static void env_dequant(float dst[][HFC_BANDS], const int src[][HFC_BANDS], int off,
                        const int *sel, int start, int end, int nsl)
{
    int t, b;

    for (t = 0; t < nsl; t++)
        dst[t][start] = (float)((double)src[t][start] * hfc_env_step[sel[start]] + hfc_env_offset[off]);
    for (b = start + 1; b <= end; b++)
        for (t = 0; t < nsl; t++)
            dst[t][b] = (float)((double)src[t][b] * hfc_env_step[sel[b]]);
}

static void env_to_linear(float **ev, const float db[][HFC_BANDS], const float *gain,
                          int start, int end, int nsl)
{
    const double scale = 0.05f;
    int t, b;

    for (t = 0; t < nsl; t++)
        ev[t][start] = (float)(pow(10.0, db[t][start] * scale) * gain[t]);
    for (b = start + 1; b <= end; b++)
        for (t = 0; t < nsl; t++)
            ev[t][b] = (float)(pow(10.0, db[t][b] * scale) * ev[t][b - 1]);
    for (b = start; b <= end; b++)
        for (t = 0; t < nsl; t++) {
            if (ev[t][b] > 32768.0)
                ev[t][b] = 32768.0f;
            if (ev[t][b] < 0.0005)
                ev[t][b] = 0.0005f;
        }
    for (b = end + 1; b < HFC_BANDS; b++)
        for (t = 0; t < nsl; t++)
            ev[t][b] = 0;
}

static void tone_adjust(int start, int end, int nsl, const Atrac3pHfcWord *rec,
                        float **ev)
{
    int mark[18] = { 0 }, cnt = rec[0].i, j, t, k;

    mark[start] = 1;
    for (j = 0; j < cnt; j++)
        mark[rec[2 + j].i + start + 1] = 1;
    for (j = cnt - 1; j >= 0; j--) {
        int b = rec[2 + j].i + start;

        if (mark[b]) {
            if (!mark[b + 2]) {
                for (t = 0; t < nsl; t++) {
                    double f = 1.0;

                    if (ev[t][b] > 0)
                        f = rec[1].i ? hfc_tone_scale[2] : hfc_tone_scale[3];
                    for (k = b + 1; k <= end; k++)
                        ev[t][k] = (float)(f * ev[t][k]);
                }
            }
        } else if (mark[b + 2]) {
            for (t = 0; t < nsl; t++) {
                double f = 1.0;

                if (ev[t][b] > 0)
                    f = rec[1].i ? hfc_tone_scale[0] : hfc_tone_scale[1];
                for (k = b; k <= end; k++)
                    ev[t][k] = (float)(f * ev[t][k]);
            }
        } else {
            for (t = 0; t < nsl; t++) {
                float *p = &ev[t][b];
                double r = 1.0;

                if (p[0] > 0)
                    r = (double)p[-1] / p[0];
                for (k = b + 1; k <= end; k++)
                    ev[t][k] = (float)(r * ev[t][k]);
                if (rec[1].i == 1) {
                    double r2 = 1.0;

                    if (p[-1] > 0)
                        r2 = (double)p[0] / p[-1];
                    p[0] = (float)(r2 * r2 * p[-1]);
                }
                p[0] = (float)((double)p[0] * hfc_tone_scale[1]);
            }
        }
        while (j > 0 && b == rec[2 + j - 1].i + start)
            j--;
    }
}

static void tone_at(int start, int b, const Atrac3pHfcWord *rec,
                    int *cursor, int *flag)
{
    if (*cursor < rec[0].i && b == rec[2 + *cursor].i + start) {
        *flag = 1;
        do {
            (*cursor)++;
        } while (*cursor < rec[0].i && b == rec[2 + *cursor].i + start);
        return;
    }
    *flag = 0;
}

static const float hfc_limits[8] = {
    1.0f, 1.999396f, 4.0f, 7.997584f, 16.0f, 64.0f, 256.0f, 32768.0f,
};

static void limit_band(float **E, float R[][19], const int *flags, int b, int src,
                       int k0, int kend, int nsl, int level)
{
    double acc = 0.0;
    int k;

    for (k = k0; k < kend; k++)
        if (flags[src + 16 * (k - k0)])
            E[k][b] = 0;
    for (k = k0; k < kend; k++) {
        double r = R[src][k];

        acc = acc + r * r * E[k][b] * E[k][b];
    }
    acc = acc * hfc_recip[nsl];
    if (acc > hfc_limits[level]) {
        float g = (float)sqrt(hfc_limits[level] / acc);

        for (k = k0; k < kend; k++)
            E[k][b] = (float)((double)g * E[k][b]);
    }
}

static void source_recip(float **E, float R[][19], int *flags, int base, int start, int nsl, int q)
{
    int b, k;

    for (b = base; b < start; b++)
        for (k = 0; k < nsl + q + 2; k++) {
            if (E[k][b] != 0.0) {
                R[b][k] = (float)(1.0 / E[k][b]);
            } else {
                R[b][k] = 0;
                if (k >= q + 2)
                    flags[b + 16 * (k - (q + 2))] = 1;
            }
        }
}

/* patch with a window crossfade between slots */
static void gen_hf(Atrac3pHfcContext *s, int ch, float **E, float **sb, int nsl, int n, int lim)
{
    const int *cur = s->env_idx[ch], *prev = s->prev_env_idx[ch], *map = s->patch_map;
    const float *win = nsl == 8 ? hfc_win8 : hfc_win16;
    int cm = s->env_mode[ch], pm = s->prev_env_mode[ch], start = s->patch_start[0];
    int q = nsl / 8 - 1, half = n / 2, b, k, i, cur0 = 0, cur1 = 0;
    int flags[256] = { 0 };

    source_recip(E, s->recip, flags, s->base_band, start, nsl, q);
    for (b = start; b <= lim; b++) {
        int fA, fB, src = map[b];
        float (*R)[19] = s->recip;
        float T0, U0, Tp, Up, Dn, Sn, Bf;
        double A, T1, U1, P, Q;

        tone_at(start, b, TONE_REC(s, 2 * ch), &cur0, &fA);
        tone_at(start, b, TONE_REC(s, 2 * ch + 1), &cur1, &fB);
        if (!fB && s->limit_level < 7)
            limit_band(E, R, flags, b, src, q + 2, q + nsl + 2, nsl, s->limit_level);
        A  = hfc_gain_src[prev[b] + pm * 16];
        Bf = hfc_gain_noise[prev[b] + pm * 16];
        T0 = (float)(A * R[src][0] * E[0][b]);
        U0 = (float)((double)Bf * E[0][b]);
        T1 = A * R[src][1] * E[1][b];
        U1 = (double)Bf * E[1][b];
        if (fA) {
            P = Q = 0.0;
        } else {
            Q = ((double)T0 - T1) * 0.5;
            P = (T1 + T0) * 0.5;
        }
        Dn = (float)(((double)U0 - U1) * 0.5);
        Sn = (float)((U1 + U0) * 0.5);
        for (i = 0; i < half; i++) {
            sb[b][i] = (float)((Q * win[half + i] + P) * sb[src][i] +
                               ((double)Dn * win[half + i] + Sn) * hfc_noise[s->noise_idx[b]]);
            s->noise_idx[b] = (s->noise_idx[b] + 1) & 0x3ff;
        }
        Tp = (float)T1;
        Up = (float)U1;
        for (k = 2; k < nsl + 1; k++) {
            int idx = nsl == 16 && k == 2 ? prev[b] + pm * 16 : cur[b] + cm * 16;
            int o = (k - 2) * n + half;
            double T, U;

            A  = hfc_gain_src[idx];
            Bf = hfc_gain_noise[idx];
            T  = (double)R[src][k] * E[k][b] * A;
            U  = (double)Bf * E[k][b];
            if (nsl == 16 && k == 2 ? fA : fB) {
                P = Q = 0.0;
            } else {
                Q = ((double)Tp - T) * 0.5;
                P = (T + Tp) * 0.5;
            }
            Dn = (float)(((double)Up - U) * 0.5);
            Sn = (float)((U + Up) * 0.5);
            for (i = 0; i < n; i++) {
                sb[b][o + i] = (float)((Q * win[i] + P) * sb[src][o + i] +
                                       ((double)Dn * win[i] + Sn) * hfc_noise[s->noise_idx[b]]);
                s->noise_idx[b] = (s->noise_idx[b] + 1) & 0x3ff;
            }
            Tp = (float)T;
            Up = (float)U;
        }
        {
            int o = (nsl - 1) * n + half;
            float Tl = (float)((double)R[src][nsl + 1] * E[nsl + 1][b] * A);
            float Ul = (float)((double)Bf * E[nsl + 1][b]);

            if (fB) {
                P = Q = 0.0;
            } else {
                Q = ((double)Tp - Tl) * 0.5;
                P = ((double)Tl + Tp) * 0.5;
            }
            Dn = (float)(((double)Up - Ul) * 0.5);
            Sn = (float)(((double)Ul + Up) * 0.5);
            for (i = 0; i < half; i++) {
                sb[b][o + i] = (float)((Q * win[i] + P) * sb[src][o + i] +
                                       ((double)Dn * win[i] + Sn) * hfc_noise[s->noise_idx[b]]);
                s->noise_idx[b] = (s->noise_idx[b] + 1) & 0x3ff;
            }
        }
    }
}

/* patch with a 3-tap temporal smoothing of the envelope */
static void gen_hf_smooth(Atrac3pHfcContext *s, int ch, float **E, float **sb, int nsl, int n, int lim)
{
    static const float wts[3][3] = { { 0, 1, 0 }, { 1, 2, 1 }, { 1, 1, 1 } };
    static const float norm_tab[4] = { 0, 1.0f, 0.25f, 0.33333334f };
    const int *cur = s->env_idx[ch], *prev = s->prev_env_idx[ch], *map = s->patch_map;
    const float *w = wts[s->smooth_mode - 1];
    const float norm = norm_tab[s->smooth_mode];
    int cm = s->env_mode[ch], pm = s->prev_env_mode[ch], start = s->patch_start[0];
    int q = nsl / 8 - 1, b, k, i, cur0 = 0, cur1 = 0;
    int flags[256] = { 0 };

    source_recip(E, s->recip, flags, s->base_band, start, nsl, q);
    for (b = start; b <= lim; b++) {
        float (*R)[19] = s->recip;
        int fA, fB, src = map[b];
        float T0, U0, T1, U1, U2, Tp, Up, Tn, Un, NA, Bc;
        double A, Bv, SA;

        tone_at(start, b, TONE_REC(s, 2 * ch), &cur0, &fA);
        tone_at(start, b, TONE_REC(s, 2 * ch + 1), &cur1, &fB);
        if (!fB && s->limit_level < 7)
            limit_band(E, R, flags, b, src, q + 2, q + nsl + 2, nsl, s->limit_level);
        A  = hfc_gain_src[prev[b] + pm * 16];
        Bv = hfc_gain_noise[prev[b] + pm * 16];
        T0 = (float)(A * R[src][0] * E[0][b]);
        U0 = (float)(Bv * E[0][b]);
        T1 = (float)(A * R[src][1] * E[1][b]);
        U1 = (float)(Bv * E[1][b]);
        if (nsl == 8) {
            A  = hfc_gain_src[cur[b] + cm * 16];
            Bv = hfc_gain_noise[cur[b] + cm * 16];
        }
        U2 = (float)(Bv * E[2][b]);
        if (fA) {
            SA = 0.0;
        } else {
            double T2 = A * ((double)R[src][2] * E[2][b]);

            SA = ((T2 * w[2] + (double)T1 * w[1]) + (double)T0 * w[0]) * norm;
        }
        NA = (float)((((double)U2 * w[2] + (double)U1 * w[1]) + (double)U0 * w[0]) * norm);
        for (i = 0; i < n; i++) {
            sb[b][i] = (float)(SA * sb[src][i] + (double)hfc_noise[s->noise_idx[b]] * NA);
            s->noise_idx[b] = (s->noise_idx[b] + 1) & 0x3ff;
        }
        Tp = T1;
        Up = U1;
        for (k = 2; k < nsl + 1; k++) {
            double Tk, Uk, Ac;

            if (nsl == 16 && k == 2) {
                double Ap = hfc_gain_src[prev[b] + pm * 16];

                Tk = Ap * R[src][2] * E[2][b];
                Uk = (double)hfc_gain_noise[prev[b] + pm * 16] * E[2][b];
                Ac = hfc_gain_src[cur[b] + cm * 16];
                Bc = hfc_gain_noise[cur[b] + cm * 16];
            } else {
                float Acf = hfc_gain_src[cur[b] + cm * 16];

                Bc = hfc_gain_noise[cur[b] + cm * 16];
                Tk = (double)R[src][k] * E[k][b] * Acf;
                Uk = (double)Bc * E[k][b];
                Ac = Acf;
            }
            Tn = (float)((double)R[src][k + 1] * E[k + 1][b] * Ac);
            Un = (float)((double)Bc * E[k + 1][b]);
            if (nsl == 16 && k == 2 ? fA : fB)
                SA = 0.0;
            else
                SA = (((double)Tn * w[2] + Tk * w[1]) + (double)Tp * w[0]) * norm;
            NA = (float)((((double)Un * w[2] + Uk * w[1]) + (double)Up * w[0]) * norm);
            for (i = 0; i < n; i++) {
                int o = (k - 1) * n + i;

                sb[b][o] = (float)(SA * sb[src][o] + (double)hfc_noise[s->noise_idx[b]] * NA);
                s->noise_idx[b] = (s->noise_idx[b] + 1) & 0x3ff;
            }
            Tp = (float)Tk;
            Up = (float)Uk;
        }
    }
}

/* noise only, used when a frame had no HFC payload */
static void gen_hf_noise(Atrac3pHfcContext *s, int ch, float **E, float **sb, int nsl, int n, int end)
{
    const int *cur = s->env_idx[ch], *prev = s->prev_env_idx[ch];
    const float *tB = &hfc_gain_noise[16 * s->env_mode[ch]];
    const float *win = nsl == 8 ? hfc_win8 : hfc_win16;
    int half = n / 2, b, k, i;

    for (b = s->patch_start[0]; b <= end; b++) {
        double B = tB[prev[b]], Ud, Uprev;
        float U0 = (float)(B * E[0][b]), D, S, Bc, Ukf;

        Ud = B * E[1][b];
        D  = (float)(((double)U0 - Ud) * 0.5);
        S  = (float)(((double)U0 + Ud) * 0.5);
        for (i = 0; i < half; i++) {
            sb[b][i] = (float)(((double)D * win[half + i] + S) * hfc_noise[s->noise_idx[b]] + sb[b][i]);
            s->noise_idx[b] = (s->noise_idx[b] + 1) & 0x3ff;
        }
        Bc    = tB[cur[b]];
        Uprev = Ud;
        for (k = 2; k < nsl + 1; k++) {
            int o = half + (k - 2) * n;
            double Uk = (double)Bc * E[k][b];

            Ukf = (float)Uk;
            D   = (float)((Uprev - Uk) * 0.5);
            S   = (float)(((double)Ukf + Uprev) * 0.5);
            for (i = 0; i < n; i++) {
                sb[b][o + i] = (float)(((double)D * win[i] + S) * hfc_noise[s->noise_idx[b]] + sb[b][o + i]);
                s->noise_idx[b] = (s->noise_idx[b] + 1) & 0x3ff;
            }
            Uprev = Ukf;
        }
        {
            int o = (nsl - 1) * n + half;
            double Ul = (double)Bc * E[nsl + 1][b];

            D = (float)((Uprev - Ul) * 0.5);
            S = (float)((Ul + Uprev) * 0.5);
            for (i = 0; i < half; i++) {
                sb[b][o + i] = (float)(((double)D * win[i] + S) * hfc_noise[s->noise_idx[b]] + sb[b][o + i]);
                s->noise_idx[b] = (s->noise_idx[b] + 1) & 0x3ff;
            }
        }
    }
}

static void tone_synth(int start, int n, const float *win, const int *range, int q,
                       float E[][HFC_BANDS], const Atrac3pHfcWord *rec,
                       float **sb)
{
    int k, t, i;

    for (k = 0; k < rec[0].i; k++) {
        int b = rec[2 + k].i + start, b2 = b + rec[8 + k].i, ph = rec[4 + k].i, ty = rec[6 + k].i;
        float amp = (float)((double)rec[10 + k].f * 0.5), gp = rec[12 + 2 * k].f, gc = rec[13 + 2 * k].f;
        float leak = rec[16 + k].f, aprev;
        int half = n / 2, phase;

        t = range[0];
        aprev = (float)((double)E[t + q - 1][b] * gp);
        if (t >= range[1])
            continue;
        phase = t * ph * n;
        for (; t < range[1]; t++) {
            int o = t * n, p2;
            double acur = (double)E[t + q][b] * (t < q ? gp : gc), anext, v;
            float S, D, X;

            S  = (float)(((double)aprev + acur) * amp);
            D  = (float)(((double)aprev - acur) * amp);
            p2 = phase + ty;
            for (i = 0; i < half; i++) {
                v = (double)D * win[half + i] + S;
                sb[b][o + i]  = (float)(v * hfc_tone_mod[phase & 0xf] + sb[b][o + i]);
                phase        += ph;
                sb[b2][o + i] = (float)(v * leak * hfc_tone_mod[p2 & 0xf] + sb[b2][o + i]);
                p2           += ph;
            }
            X     = (float)acur;
            anext = (double)E[t + q + 1][b] * (t + 1 < q ? gp : gc);
            S     = (float)(((double)X + anext) * amp);
            D     = (float)(((double)X - anext) * amp);
            p2    = phase + ty;
            for (i = 0; i < n - half; i++) {
                v = (double)D * win[i] + S;
                sb[b][o + half + i]  = (float)(v * hfc_tone_mod[phase & 0xf] + sb[b][o + half + i]);
                phase               += ph;
                sb[b2][o + half + i] = (float)(v * leak * hfc_tone_mod[p2 & 0xf] + sb[b2][o + half + i]);
                p2                  += ph;
            }
            phase = (t + 1) * ph * n;
            aprev = (float)anext;
        }
    }
}

void ff_atrac3p_hfc_synth(Atrac3pHfcContext *s, float *sb[])
{
    float *ev[HFC_MAX_SLOTS], *ec[20];
    int ch, b, t, k, nsl, n, end, lim, start = s->patch_start[0];

    for (ch = 0; ch < s->nch; ch++)
        for (b = 0; b < start; b++)
            memcpy(&s->hist[ch][b][16], sb[16 * ch + b] + HFC_BAND_LEN - 16, 16 * sizeof(float));
    end = s->patch_end[s->num_patches - 1];
    noise_seed(s, start, end);
    if (s->num_slots != s->prev_num_slots)
        for (ch = 0; ch < s->nch; ch++)
            env_resample(s->env_gain[ch], s->prev_end, s->num_slots);
    lim = s->prev_end > end ? s->prev_end : end;
    tone_setup(s);
    nsl = s->num_slots;
    n   = s->slot_len / 16;
    for (ch = 0; ch < s->nch; ch++) {
        float **x = &sb[16 * ch];

        for (t = 0; t < nsl; t++)
            ev[t] = s->env_gain[ch][nsl / 4 + t];
        for (k = 0; k < nsl / 8 + 1 + nsl; k++)
            ec[k] = s->env_gain[ch][nsl / 8 - 1 + k];
        coded_energy(x, ev, s->time_flags[ch], s->time_group[ch], s->slot_gain, start - 1, nsl, n);
        env_dequant(s->env_db[ch], s->env_q[ch], s->env_offset[ch], s->band_sel[ch], start, end, nsl);
        env_to_linear(ev, s->env_db[ch], s->slot_gain, start, end, nsl);
        tone_adjust(start, end, nsl, TONE_REC(s, 2 * ch + 1), ev);
        for (b = 0; b < start; b++) {
            memmove(x[b] + 16, x[b], (HFC_BAND_LEN - 16) * sizeof(float));
            memcpy(x[b], s->hist[ch][b], 16 * sizeof(float));
            memmove(s->hist[ch][b], s->hist[ch][b] + 16, 16 * sizeof(float));
        }
        if (s->no_parse) {
            memset(s->prev_env_idx[ch], 0, sizeof(s->prev_env_idx[ch]));
            memset(s->env_idx[ch], 0, sizeof(s->env_idx[ch]));
            for (b = start; b <= end; b++)
                memset(x[b], 0, HFC_BAND_LEN * sizeof(float));
            gen_hf_noise(s, ch, ec, x, nsl, n, end);
        } else if (!s->smooth_mode) {
            gen_hf(s, ch, ec, x, nsl, n, lim);
        } else {
            gen_hf_smooth(s, ch, ec, x, nsl, n, lim);
        }
        for (b = lim + 1; b < HFC_BANDS; b++)
            memset(x[b], 0, HFC_BAND_LEN * sizeof(float));
    }
    for (ch = 0; ch < s->nch; ch++) {
        const int range[3] = { 0, nsl / 8, nsl };
        const float *win = nsl == 8 ? hfc_win8 : hfc_win16;

        tone_synth(start, 0x80 / nsl, win, &range[0], nsl / 8, s->env_gain[ch], TONE_REC(s, 2 * ch), &sb[16 * ch]);
        tone_synth(start, 0x80 / nsl, win, &range[1], nsl / 8, s->env_gain[ch], TONE_REC(s, 2 * ch + 1), &sb[16 * ch]);
    }
    s->prev_end       = end;
    s->prev_num_slots = s->num_slots;
    s->no_parse       = 1;
    for (ch = 0; ch < s->nch; ch++) {
        memcpy(s->prev_env_idx[ch], s->env_idx[ch], sizeof(s->env_idx[ch]));
        s->prev_env_mode[ch] = s->env_mode[ch];
    }
    for (ch = 0; ch < s->nch; ch++)
        for (b = 0; b < HFC_BANDS; b++)
            for (t = 0; t < nsl / 4; t++)
                s->env_gain[ch][t][b] = s->env_gain[ch][nsl + t][b];
}

void ff_atrac3p_hfc_reset(Atrac3pHfcContext *s)
{
    int ch, b;

    for (ch = 0; ch < s->nch; ch++) {
        memset(s->prev_env_idx[ch], 0, sizeof(s->prev_env_idx[ch]));
        memset(s->env_gain[ch], 0, sizeof(s->env_gain[ch]));
        memset(s->env_q[ch], 0, sizeof(s->env_q[ch]));
        for (b = 0; b < s->patch_start[0]; b++)
            s->hist[ch][b][0] = s->hist[ch][b][1] = 0;
    }
    for (ch = 0; ch < 2 * s->nch; ch++)
        memset(TONE_REC(s, ch), 0, sizeof(s->tone_rec[0]));
}

int ff_atrac3p_hfc_init(Atrac3pHfcContext *s, int nch, int base, int start, int end, int mirror)
{
    int ch, b, k;

    if (nch < 1 || nch > 2 || base < 0 || base > 3 || start < 1 || start > 15 ||
        start <= base || end < start || end > 15 || (!mirror && (start & 1) != (base & 1)))
        return AVERROR(EINVAL);
    memset(s, 0, sizeof(*s));
    s->nch            = nch;
    s->mirror         = mirror;
    s->patch_start[0] = start;
    build_patches(s, start, end, base);
    s->no_parse       = 1;
    s->slot_len       = HFC_FRAME_LEN / 8;
    s->num_slots      = 8;
    s->prev_num_slots = 8;
    s->prev_end       = start;
    for (ch = 0; ch < nch; ch++)
        s->env_offset[ch] = 15;
    for (b = 0; b < HFC_BANDS; b++)
        for (k = 0; k < 18; k++)
            s->recip[b][k] = 1.0f;
    for (k = 0; k < 4; k++)
        s->tone_sel[k] = k;
    return 0;
}
