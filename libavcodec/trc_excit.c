/*
 * Sony TRC excitation (CELP pitch + algebraic codebook)
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

/*
 * Adaptive (pitch) codebook, fixed algebraic codebook and the long-term
 * pitch postfilter that runs on the assembled excitation.
 *
 * Subframe parameter layout (TRCSubParams):
 *   track 1 position code <- sf->signs2
 *   track 1 sign bits     <- sf->signs1 (left-aligned, bit 15 = first sign)
 *   track 2 position code <- sf->pos2
 *   track 2 sign bits     <- sf->pos1   (left-aligned, bit 15 = first sign)
 */

#include "trc_excit.h"
#include "trc_dsp.h"
#include "trcdata.h"
#include "trc_unpack.h"

#include <string.h>

#define TRC_LT_HIST   144
#define TRC_LT_TOTAL  304
#define TRC_LT_WIN    40

/*
 * Contiguous combinadic position-threshold table.  Row r holds C(19-col, 3-r):
 * row0=C(.,3), row1=C(.,2), row2=C(.,1), row3=C(.,0) (all ones).  Pulse
 * decoding walks one column per candidate position and jumps +20 (next row)
 * on each placed pulse, so the rows must be contiguous.
 */
static const int16_t trc_pulse_thr[80] = {
    969, 816, 680, 560, 455, 364, 286, 220, 165, 120,
     84,  56,  35,  20,  10,   4,   1,   0,   0,   0,
    171, 153, 136, 120, 105,  91,  78,  66,  55,  45,
     36,  28,  21,  15,  10,   6,   3,   1,   0,   0,
     19,  18,  17,  16,  15,  14,  13,  12,  11,  10,
      9,   8,   7,   6,   5,   4,   3,   2,   1,   0,
      1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
      1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
};

/*
 * Fixed-codebook gain: first-order MA prediction on the gain index pulling
 * toward 7 (0.75 in Q15), then the transmitted 4-bit delta, clamped [0, 25].
 */
static int16_t trc_gain_decode(int16_t *state, int16_t delta)
{
    int32_t g = (((int32_t)(*state - 7) * 0x6000) + 0x4000 >> 15) + 7;

    if (g < 7)
        g = 7;
    if (g > 17)
        g = 17;
    g = (g - 7) + delta;
    if (g < 0)
        g = 0;
    if (g > 25)
        g = 25;
    *state = (int16_t)g;
    return trc_gain_quant[g];
}

/*
 * Pitch synthesis feedback applied additively to the scratch buffer,
 * starting at position lag-1.
 *
 * For i = lag-1: scratch[i] += round(scratch[0]*coef[2])
 * For i = lag:   scratch[i] += round(scratch[0]*coef[1] + scratch[1]*coef[2])
 * For i > lag:   scratch[i] += round(scratch[i-lag-1]*coef[0]
 *                                   + scratch[i-lag]*coef[1]
 *                                   + scratch[i-lag+1]*coef[2])
 *
 * Reads from positions < lag-1 (unmodified pulse positions), writes to lag-1
 * and beyond.  In-place safe for lag >= 2 (always true: minimum lag is 18).
 */
static void trc_pitch_on_scratch(int16_t *scratch, int lag, int pitch_idx,
                                 int sublen)
{
    const int16_t *coef = trc_coef_table + pitch_idx * 9;
    int32_t acc;
    int j;

    if (sublen <= lag)
        return;

    /* i = lag-1 */
    acc = trc_l_mult(0, scratch[0], coef[2]);
    scratch[lag - 1] = trc_clip_int16(scratch[lag - 1] + trc_round16(acc));

    /* i = lag */
    acc = trc_l_mult(0, scratch[0], coef[1]);
    acc = trc_l_mult(acc, scratch[1], coef[2]);
    scratch[lag] = trc_clip_int16(scratch[lag] + trc_round16(acc));

    /* i = lag+1 .. sublen-1 */
    for (j = 0; j < sublen - lag - 1; j++) {
        acc = trc_l_mult(0,   scratch[j],     coef[0]);
        acc = trc_l_mult(acc, scratch[j + 1], coef[1]);
        acc = trc_l_mult(acc, scratch[j + 2], coef[2]);
        scratch[j + lag + 1] = trc_clip_int16(scratch[j + lag + 1] +
                                               trc_round16(acc));
    }
}

/*
 * Pulse-placement engine.
 *
 * Iterates over subframe positions in steps, comparing pos_code (Q16) against
 * the cumulative distribution in `table`.  Positions where code < threshold
 * get a pulse; code is not reduced on a hit so subsequent pulses use the same
 * code.  On each hit the table pointer jumps +20 (next pulse section), plus +1
 * at the loop bottom for non-last hits.
 */
static void trc_place_track(int16_t *exc, int len, int16_t pos_code,
                            int16_t *signs, int16_t gain, int start, int step,
                            int npulses, const int16_t *table)
{
    const int16_t *tab = table;
    int32_t code = (int32_t)pos_code << 16;
    int left = npulses;
    int pos;

    for (pos = start; pos < len && left > 0; pos += step) {
        int32_t thr = (int32_t)*tab << 16;

        if (code < thr) {
            int16_t amp = (*signs & 0x8000) ? (int16_t)-gain : gain;

            exc[pos] = trc_clip_int16(exc[pos] + amp);
            *signs = (int16_t)((unsigned)(*signs) << 1);
            left--;
            tab += 20;
            if (!left)
                break;
        } else {
            code -= thr;
        }
        tab++;
    }
}

void trc_adaptive_excit(int16_t *exc, const TRCSubParams *sf,
                        int16_t *exc_hist, int len)
{
    int lag = sf->lag;
    int16_t tmp[256];
    int n, t;
    const int16_t *coef = trc_coef_table + sf->pitch_idx * 9;

    if (lag < 1)
        lag = 1;
    if (len > 256)
        return;

    if (lag < len + 1) {
        for (n = 0; n <= len - lag; n++)
            exc[n] = exc[n - lag];
    }

    for (n = 0; n < len; n++) {
        int32_t acc = 0;

        for (t = 0; t < 3; t++) {
            int p = n - lag - 1 + t;

            if (exc + p >= exc_hist)
                acc = trc_l_mult(acc, coef[t], exc[p]);
        }
        tmp[n] = trc_round16(acc << 1);
    }
    memcpy(exc, tmp, len * sizeof(int16_t));
}

/* Fixed algebraic codebook excitation, added onto the adaptive contribution. */
void trc_fixed_excit(int16_t *exc, const TRCSubParams *sf, int rate, int len,
                     int16_t *gain_state)
{
    int16_t gain;
    int16_t scratch[80];
    int16_t signs1 = sf->signs1;       /* track 1 signs, left-aligned */
    int16_t signs2 = sf->pos1;         /* track 2 signs, left-aligned */
    int i;

    memset(scratch, 0, len * sizeof(int16_t));
    gain = trc_gain_decode(gain_state, sf->gain_idx);

    /* Track 1: 4 pulses; combinadic table starts at row 0 (C(.,3)). */
    trc_place_track(scratch, len, sf->signs2, &signs1, gain,
                    sf->field2, len / 20, 4, trc_pulse_thr);

    /* Track 2 (rate 2 only): 3 pulses at odd positions; table starts at
     * row 1 (C(.,2)), i.e. trc_pulse_thr + 20. */
    if (rate == 2)
        trc_place_track(scratch, len, sf->pos2, &signs2, gain,
                        1, 2, 3, trc_pulse_thr + 20);

    trc_pitch_on_scratch(scratch, sf->lag, sf->pitch_idx, len);

    for (i = 0; i < len; i++)
        exc[i] = trc_clip_int16(exc[i] + scratch[i]);
}

static int16_t trc_peak_abs16(const int16_t *x, int n)
{
    int32_t peak = 0;
    int i;

    for (i = 0; i < n; i++) {
        int32_t v = x[i];
        v = v < 0 ? -v : v;
        if (v > peak)
            peak = v;
    }
    return peak > 0x7fff ? 0x7fff : (int16_t)peak;
}

/* Normalization shift for the peak (power-of-two scale table lookup). */
static int16_t trc_norm_peak_shift(int16_t peak)
{
    int64_t v = (int32_t)peak << 16;
    int s = 0;

    if (!peak)
        return 0;
    while (v > 0x7fffffffLL || v < -0x80000000LL) {
        v >>= 1;
        s--;
    }
    while (v >= -0x40000000LL && v <= 0x3fffffffLL) {
        v <<= 1;
        s++;
    }
    return (int16_t)s;
}

static int16_t trc_pow2_scale(int16_t idx)
{
    static const int16_t ext[28] = {
        24576, 13107, 4096, 0, 0, 0,            /* overlap window */
        11, 18, 24, 4, 2, 0,                     /* packet sizes */
        1, 2, 4, 8, 16, 32, 64, 128, 256, 512,  /* powers of two */
        1024, 2048, 4096, 8192, 16384, 0,
    };

    if (idx < -12 || idx > 15)
        return 0;
    return ext[idx + 12];
}

/* Normalize the buffer by its peak: dst = clip((scale * src * 2) >> 4). */
static void trc_norm_buf(int16_t *dst, const int16_t *src, int16_t peak, int n)
{
    int16_t idx = trc_norm_peak_shift(peak);
    int32_t scale;
    int i;

    if (peak < 1) {
        memset(dst, 0, n * sizeof(*dst));
        return;
    }
    scale = trc_pow2_scale(idx);
    if (!scale) {
        memset(dst, 0, n * sizeof(*dst));
        return;
    }
    for (i = 0; i < n; i++) {
        int64_t acc = (int64_t)scale * src[i] << 1;

        dst[i] = trc_clip_int16((int32_t)(acc >> 4));
    }
}

static int64_t trc_corr_win(const int16_t *a, const int16_t *b, int n)
{
    int64_t s = 0;
    int i;

    for (i = 0; i < n; i++)
        s += (int32_t)a[i] * b[i];
    return s;
}

/*
 * Forward lag search (backward-in-time direction): best SIGNED
 * corr(x[n-lag], x[n]) over lag in [center-2, center+2].  The search starts
 * from "no lag" (0) and only picks a lag whose correlation is strictly
 * positive; the candidate range is clamped to [0x8b, 0x8f] when center+2
 * exceeds 0x8f.
 */
static int trc_refine_lag_fwd(const int16_t *work, int pos, int center)
{
    int lo = center - 2, hi = center + 2;
    int best = 0;
    int64_t best_e = 0;
    int lag;

    if (hi > 0x8f) {
        hi = 0x8f;
        lo = 0x8b;
    }

    for (lag = lo; lag <= hi; lag++) {
        int64_t e;

        if (lag < 1 || pos - lag < 0)
            continue;
        e = trc_corr_win(work + pos - lag, work + pos, TRC_LT_WIN) << 1;
        if (e > best_e) {
            best_e = e;
            best = lag;
        }
    }
    return best;
}

/*
 * Backward lag search (forward-in-time direction): best SIGNED
 * corr(x[n], x[n+lag]).  The whole search is skipped when the highest
 * candidate window would read past the 304-sample buffer (all-or-nothing
 * bound on hi, not per candidate).
 */
static int trc_refine_lag_bwd(const int16_t *work, int pos, int center)
{
    int lo = center - 2, hi = center + 2;
    int best = 0;
    int64_t best_e = 0;
    int lag;

    if (hi > 0x8f) {
        hi = 0x8f;
        lo = 0x8b;
    }

    if (pos + hi + TRC_LT_WIN > TRC_LT_TOTAL)
        return 0;

    for (lag = lo; lag <= hi; lag++) {
        int64_t e;

        if (lag < 1)
            continue;
        e = trc_corr_win(work + pos, work + pos + lag, TRC_LT_WIN) << 1;
        if (e > best_e) {
            best_e = e;
            best = lag;
        }
    }
    return best;
}

static int16_t trc_lt_lag(const TRCFrameParams *fp, int k)
{
    if (fp->nsub == 4)
        return fp->sf[k].lag;
    return fp->sf[k >> 1].lag;
}

static int16_t trc_lt_overlap_w(int rate)
{
    if (rate < 0 || rate >= 3)
        return 0;
    return trc_overlap_win[rate];
}

static int64_t trc_lt_energy(const int16_t *x, int n)
{
    int64_t e = 0;
    int i;

    for (i = 0; i < n; i++)
        e += (int32_t)x[i] * x[i];
    return e;
}

/* Pack a window energy: sat32((sum_sq << 1) >> 4). */
static int32_t trc_lt_pack_energy(int64_t sum_sq)
{
    return trc_sat32(sum_sq << 1 >> 4);
}

/* Normalization exponent from the larger of two packed energies. */
static int trc_lt_norm_exp(int32_t e0, int32_t e1)
{
    int32_t maxv, maxh;
    int64_t u;
    int exp = 0;
    int32_t h0 = e0 >> 31, h1 = e1 >> 31;

    maxv = e1;
    maxh = h1;
    if (h1 < h0 || (h1 == h0 && (uint32_t)e1 <= (uint32_t)e0)) {
        maxv = e0;
        maxh = h0;
    }
    u = ((int64_t)maxh << 32) | (uint32_t)maxv;
    if (!maxv && !maxh)
        return 0;

    for (;;) {
        int32_t uh = (int32_t)(u >> 32);
        uint32_t ul = (uint32_t)u;
        int adjust = 0;

        if (uh > 0 || (uh == 0 && ul > 0x7fffffffU)) {
            exp++;
            u >>= 1;
            continue;
        }
        if (u < 0 || ul < 0x40000000U) {
            if (u > (int64_t)-0x100000001LL) {
                exp--;
                u <<= 1;
                adjust = 1;
            }
        }
        if (!adjust)
            break;
    }
    return -exp;
}

static int16_t trc_lt_norm_scalar(int32_t e, int exp)
{
    int64_t v = e;

    if (exp < 0)
        v >>= -exp;
    else if (exp > 0)
        v <<= exp;
    return (int16_t)(trc_sat32((v + 0x8000) >> 16));
}

/* Enhance only when (norm_in * norm_enh >> 1) < norm_cross^2. */
static int trc_lt_gate_pass(int16_t norm_in, int16_t norm_enh, int16_t norm_cross)
{
    int64_t prod = (int64_t)norm_in * norm_enh;
    int64_t cross_sq = (int64_t)norm_cross * norm_cross;

    return (prod >> 1) < cross_sq;
}

/*
 * Taper a scalar by the overlap window weight:
 * v = 2*(x*ow + 0x4000) = 2*x*ow + 0x8000 (rounds once), then take the high
 * word.  Do NOT round again; the +0x4000 << 1 IS the rounding.
 */
static int16_t trc_lt_ov_scale(int16_t x, int16_t ow)
{
    int64_t acc = ((int64_t)x * ow + 0x4000) << 1;

    return (int16_t)(trc_sat32(acc) >> 16);
}

static void trc_lt_overlap_taper(int16_t *harm, int16_t ow)
{
    int n;

    for (n = 0; n < TRC_LT_WIN; n++)
        harm[n] = trc_lt_ov_scale(harm[n], ow);
}

/* Overlap-taper the norms: cross once, enh twice, in unchanged. */
static void trc_lt_overlap_norms(int16_t *norm_enh, int16_t *norm_cross,
                                 int16_t ow)
{
    *norm_cross = trc_lt_ov_scale(*norm_cross, ow);
    *norm_enh   = trc_lt_ov_scale(*norm_enh, ow);
    *norm_enh   = trc_lt_ov_scale(*norm_enh, ow);
}

static int64_t trc_lt_cross_energy(const int16_t *in, const int16_t *harm, int n)
{
    int64_t e = 0;
    int i;

    for (i = 0; i < n; i++)
        e += (int32_t)in[i] * harm[i];
    return e;
}

/* Keep the low 32 bits when the high dword is zero, else clip. */
static int32_t trc_lt_sat64_lo(int32_t lo, int32_t hi)
{
    if (hi > 0)
        return 0x7fffffff;
    if (hi < 0)
        return (int32_t)0x80000000;
    return lo;
}

static int16_t trc_lt_den(int16_t norm_in, int16_t norm_enh,
                          int16_t norm_cross)
{
    int64_t acc;
    int32_t lo, hi, v;

    /* Linear 2*(n<<13) sum, low-dword mask, >>16. */
    acc  = (int64_t)2 * ((int32_t)norm_in << 13);
    acc += (int64_t)2 * ((int32_t)norm_enh << 13);
    acc += (int64_t)2 * (((int32_t)norm_cross << 14) + 0x4000);
    lo = (int32_t)acc;
    hi = (int32_t)(acc >> 32);
    lo &= 0xffff0000;
    v  = trc_lt_sat64_lo(lo, hi);
    v  = trc_lt_sat64_lo(v, 0);
    return (int16_t)(v >> 16);
}

static int16_t trc_lt_gain(int16_t norm_in, int16_t norm_enh,
                           int16_t norm_cross)
{
    int32_t num, den;
    uint32_t g_raw, g;

    den = trc_lt_den(norm_in, norm_enh, norm_cross);
    if (den <= 0)
        return 0x4000;

    num = (int32_t)norm_in << 14;
    if (num >= (int32_t)den << 16)
        g_raw = 0x7fff;
    else
        g_raw = (uint32_t)(((int64_t)num >> 1) / den);

    if (g_raw > 0x7fff)
        g_raw = 0x7fff;

    g = trc_sqrt_half_q15(g_raw << 16);
    if (g < 0x2000)
        g = 0x2000;
    if (g > 0x7fff)
        g = 0x7fff;
    return (int16_t)g;
}

/* Mix output: round((harm*g + in*g) << 1 >> 16). */
static int16_t trc_lt_mix_out(int16_t harm, int16_t in, int16_t g)
{
    int64_t acc = (int64_t)harm * g;

    acc += (int64_t)in * g;
    return trc_round16(trc_sat32(acc << 1));
}

static int16_t trc_norm_sample(const int16_t *norm, int idx)
{
    if (idx < 0 || idx >= TRC_LT_TOTAL)
        return 0;
    return norm[idx];
}

/* Harmonic-synthesis rounding: acc<<4, +0x8000, mask, >>16. */
static int16_t trc_lt_harm_round(int32_t acc)
{
    int64_t v = (int64_t)acc << 4;

    v += 0x8000;
    v &= ~0xffffLL;
    return trc_clip_int16((int16_t)(v >> 16));
}

/* Harmonic synthesis: 3 taps at center = fpos + 1 + lag + n. */
static void trc_lt_synth_harm(int16_t *harm, const int16_t *raw, int fpos,
                             int lag, const int16_t taps[3])
{
    int n;

    for (n = 0; n < TRC_LT_WIN; n++) {
        int center = fpos + 1 + lag + n;
        int32_t acc = 0;

        acc = trc_l_mult(acc, taps[0], trc_norm_sample(raw, center - 2));
        acc = trc_l_mult(acc, taps[1], trc_norm_sample(raw, center - 1));
        acc = trc_l_mult(acc, taps[2], trc_norm_sample(raw, center));
        harm[n] = trc_lt_harm_round(acc);
    }
}

/* Fractional multiply: sat32(2*a*b) >> 16. */
static int16_t trc_lt_fmul(int16_t a, int16_t b)
{
    return (int16_t)(trc_sat32(((int64_t)a * b) << 1) >> 16);
}

/* Cramer determinant term: ((int64)(fmul(p,q) * r)) << 1. */
static int64_t trc_lt_det_term(int16_t p, int16_t q, int16_t r)
{
    return ((int64_t)trc_lt_fmul(p, q) * r) << 1;
}

/* 40-sample lag correlation, packed as sat32(sum << 1). */
static int32_t trc_lt_corr40(const int16_t *norm, int ia, int ib)
{
    int64_t s = 0;
    int i;

    for (i = 0; i < TRC_LT_WIN; i++)
        s += (int32_t)trc_norm_sample(norm, ia + i) *
             trc_norm_sample(norm, ib + i);
    return trc_sat32(s << 1);
}

/* Normalization exponent from |max| of the 9 packed lags. */
static int trc_lt_norm9_exp(const int32_t r[9])
{
    int64_t maxabs = 0;
    int i;

    for (i = 0; i < 9; i++) {
        int64_t v = r[i];

        if (v < 0)
            v = -v;
        if (v > maxabs)
            maxabs = v;
    }
    if (maxabs > 0x7fffffff)
        maxabs = 0x7fffffff;
    return trc_lt_norm_exp((int32_t)maxabs, 0);
}

/* Normalize a packed lag: sat32(v shifted by exp) >> 16 (no rounding). */
static int16_t trc_lt_norm9_scalar(int32_t v, int exp)
{
    int64_t x = v;

    if (exp < 0)
        x >>= -exp;
    else if (exp > 0)
        x <<= exp;
    return (int16_t)(trc_sat32(x) >> 16);
}

/* Final divide: tap = sign(num) * ((|num|>>5) / det_h), clamped. */
static int16_t trc_lt_solve_tap(int32_t num, int16_t det_h)
{
    int64_t n = num < 0 ? -(int64_t)num : (int64_t)num;

    n >>= 4;
    if (det_h > 0 && n < ((int64_t)det_h << 16)) {
        int64_t q = (n >> 1) / det_h;

        if (q > 0x7fff)
            q = 0x7fff;
        return (int16_t)(num < 0 ? -q : q);
    }
    return 0x7fff;
}

/*
 * 3-tap pitch predictor over a 40-sample block.
 *
 * Builds 9 autocorrelation lags for the three lag-aligned windows A/B/C at
 * offsets lag-1/lag/lag+1 from the subframe base `pos` (signed lag, negative
 * = backward/history), normalizes them to int16 q[9], then solves the
 * symmetric 3x3 normal equations by Cramer's rule.  Taps h[0..2] map to
 * windows A/B/C; cross[0..2] are the normalized R(in,A/B/C) and *exp_out the
 * normalization exponent used by the direction dot metric.
 */
static void trc_lt_coefs(int16_t h[3], int16_t cross[3], int *exp_out,
                         const int16_t *norm, int pos, int lag)
{
    int a = pos + lag - 1, b = pos + lag, c = pos + lag + 1;
    int32_t r[9];
    int16_t q[9];
    int exp, i;
    int64_t det_sum, n0, n1, n2;
    int32_t det;
    int16_t det_h;

    r[0] = trc_lt_corr40(norm, pos, a);   /* R(in, A) */
    r[1] = trc_lt_corr40(norm, pos, b);   /* R(in, B) */
    r[2] = trc_lt_corr40(norm, pos, c);   /* R(in, C) */
    r[3] = trc_lt_corr40(norm, a, a);     /* R(A, A) */
    r[4] = trc_lt_corr40(norm, b, b);     /* R(B, B) */
    r[5] = trc_lt_corr40(norm, c, c);     /* R(C, C) */
    r[6] = trc_lt_corr40(norm, a, b);     /* R(A, B) */
    r[7] = trc_lt_corr40(norm, b, c);     /* R(B, C) */
    r[8] = trc_lt_corr40(norm, a, c);     /* R(A, C) */

    exp = trc_lt_norm9_exp(r);
    for (i = 0; i < 9; i++)
        q[i] = trc_lt_norm9_scalar(r[i], exp);
    if (exp_out)
        *exp_out = exp;
    cross[0] = q[0];
    cross[1] = q[1];
    cross[2] = q[2];

    /*
     * Matrix M = [[d0,e0,e2],[e0,d1,e1],[e2,e1,d2]], rhs = [c0,c1,c2] with
     *   c0=q[0] c1=q[1] c2=q[2] d0=q[3] d1=q[4] d2=q[5] e0=q[6] e1=q[7] e2=q[8]
     */
    det_sum = trc_lt_det_term(q[4], q[3], q[5])
            + trc_lt_det_term(q[7], q[6], q[8])
            + trc_lt_det_term(q[7], q[6], q[8])
            - trc_lt_det_term(q[8], q[4], q[8])
            - trc_lt_det_term(q[7], q[3], q[7])
            - trc_lt_det_term(q[5], q[6], q[6]);
    det   = trc_sat32((det_sum + 0x8000) & ~(int64_t)0xffff);
    det_h = (int16_t)(det >> 16);

    n0 = trc_lt_det_term(q[0], q[4], q[5])
       + trc_lt_det_term(q[7], q[6], q[2])
       + trc_lt_det_term(q[8], q[1], q[7])
       - trc_lt_det_term(q[8], q[4], q[2])
       - trc_lt_det_term(q[0], q[7], q[7])
       - trc_lt_det_term(q[1], q[6], q[5]);
    n1 = trc_lt_det_term(q[1], q[3], q[5])
       + trc_lt_det_term(q[0], q[7], q[8])
       + trc_lt_det_term(q[8], q[6], q[2])
       - trc_lt_det_term(q[8], q[1], q[8])
       - trc_lt_det_term(q[7], q[3], q[2])
       - trc_lt_det_term(q[0], q[6], q[5]);
    n2 = trc_lt_det_term(q[4], q[3], q[2])
       + trc_lt_det_term(q[1], q[6], q[8])
       + trc_lt_det_term(q[0], q[6], q[7])
       - trc_lt_det_term(q[0], q[4], q[8])
       - trc_lt_det_term(q[3], q[1], q[7])
       - trc_lt_det_term(q[6], q[2], q[6]);

    h[0] = trc_lt_solve_tap(trc_sat32(n0), det_h);
    h[1] = trc_lt_solve_tap(trc_sat32(n1), det_h);
    h[2] = trc_lt_solve_tap(trc_sat32(n2), det_h);
}

/*
 * Direction-selection dot metric: predicted energy = (sum(cross[i]*h[i]) << 1),
 * exponent-adjusted so both direction candidates compare at the same scale
 * (exp < 0 means shift back up).
 */
static int64_t trc_lt_dir_metric(const int16_t cross[3], const int16_t h[3],
                                 int exp)
{
    int64_t dot = 0;
    int i;

    for (i = 0; i < 3; i++)
        dot += (int64_t)cross[i] * h[i];
    dot <<= 1;
    if (exp < 0)
        return dot << -exp;
    return dot >> exp;
}

/*
 * Long-term pitch postfilter on the excitation (144 history + 160 frame).
 *
 * Pass 1: four 40-sample windows; forward/backward lag refine (+-2) and
 * 3-tap coefficients on the peak-normalized buffer; the direction with the
 * stronger predicted energy wins.
 * Pass 2: harmonic synthesis, 3-energy gain, and a harm/input mix per window.
 */
void trc_lt_pitch_filter(int16_t *buf, const TRCFrameParams *fp)
{
    int16_t norm[TRC_LT_TOTAL];
    int16_t work[TRC_LT_TOTAL];
    int16_t harm[TRC_LT_WIN];
    int16_t lag[4], h[4][3];
    int16_t peak, ow;
    int w, k;

    if (!fp || fp->type != 1)
        return;

    peak = trc_peak_abs16(buf, TRC_LT_TOTAL);
    trc_norm_buf(norm, buf, peak, TRC_LT_TOTAL);
    ow = trc_lt_overlap_w(fp->rate);

    for (k = 0; k < 4; k++) {
        int pos = TRC_LT_HIST + k * TRC_LT_WIN;
        int center = trc_lt_lag(fp, k);
        int fwd = trc_refine_lag_fwd(norm, pos, center);
        int bwd = trc_refine_lag_bwd(norm, pos, center);
        int16_t hf[3], hb[3], cf[3], cb[3];
        int expf, expb;
        int64_t mf, mb;

        /*
         * Forward = negative lag (history), backward = positive lag.
         * A zero lag means the search found no positive correlation in that
         * direction; with both zero the window stays unfiltered (coefs 0).
         */
        mf = mb = 0;
        lag[k] = 0;
        h[k][0] = h[k][1] = h[k][2] = 0;
        if (fwd)
            trc_lt_coefs(hf, cf, &expf, norm, pos, -fwd);
        if (bwd)
            trc_lt_coefs(hb, cb, &expb, norm, pos, bwd);
        if (fwd && bwd) {
            mf = trc_lt_dir_metric(cf, hf, expf);
            mb = trc_lt_dir_metric(cb, hb, expb);
        }
        if (fwd && (!bwd || (bwd && mb < mf))) {
            lag[k] = (int16_t)-fwd;
            h[k][0] = hf[0];
            h[k][1] = hf[1];
            h[k][2] = hf[2];
        } else if (bwd) {
            lag[k] = (int16_t)bwd;
            h[k][0] = hb[0];
            h[k][1] = hb[1];
            h[k][2] = hb[2];
        }
    }

    /* Pass 2 works on the raw (unnormalized) excitation. */
    memcpy(work, buf, TRC_LT_TOTAL * sizeof(int16_t));

    for (w = 0; w < 4; w++) {
        int fpos = TRC_LT_HIST + w * TRC_LT_WIN;
        const int16_t *sf_in = work + fpos;
        int64_t e_in, e_enh;
        int32_t e_raw[3];
        int exp, n;
        int16_t norm_in, norm_enh, norm_cross, g;

        trc_lt_synth_harm(harm, work, fpos, lag[w], h[w]);

        e_in  = trc_lt_energy(sf_in, TRC_LT_WIN);
        e_enh = trc_lt_energy(harm, TRC_LT_WIN);

        e_raw[0] = trc_lt_pack_energy(e_in);
        e_raw[1] = trc_lt_pack_energy(e_enh);
        e_raw[2] = trc_lt_pack_energy(trc_lt_cross_energy(sf_in, harm, TRC_LT_WIN));
        exp = trc_lt_norm_exp(e_raw[0], e_raw[1]);
        norm_in    = trc_lt_norm_scalar(e_raw[0], exp);
        norm_enh   = trc_lt_norm_scalar(e_raw[1], exp);
        norm_cross = trc_lt_norm_scalar(e_raw[2], exp);

        if (!trc_lt_gate_pass(norm_in, norm_enh, norm_cross))
            continue;

        if (ow > 0) {
            trc_lt_overlap_taper(harm, ow);
            trc_lt_overlap_norms(&norm_enh, &norm_cross, ow);
        }

        g = trc_lt_gain(norm_in, norm_enh, norm_cross);

        for (n = 0; n < TRC_LT_WIN; n++)
            buf[fpos + n] = trc_lt_mix_out(harm[n], sf_in[n], g);
    }
}
