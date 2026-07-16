/*
 * Sony TRC synthesis / postfilter / comfort noise
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

#include "trc_synth.h"
#include "trc_dsp.h"
#include "trcdata.h"

#include <string.h>

#define TRC_ORDER 10

/*
 * The postfilter works in one scratch ring holding three staggered bands of
 * the subframe, mirroring the reference decoder's in-place buffer reuse:
 * the formant-filtered signal, the tilt-compensated signal written ten
 * samples behind it, and the final (AGC scaled) output taken from the tilt
 * band.
 */
#define TRC_PF_PCM_LEAD     20
#define TRC_PF_FORMANT_LEAD (TRC_PF_PCM_LEAD - 4)
#define TRC_PF_TILT_LEAD    (TRC_PF_FORMANT_LEAD - TRC_ORDER)
#define TRC_PF_RING_SAMPLES (TRC_PF_PCM_LEAD + TRC_FRAME_SAMPLES + TRC_ORDER)

static const int16_t *trc_weight_for_rate(int rate)
{
    static const int16_t *const wins[] = {
        trc_weight_win0, trc_weight_win1, trc_weight_win2,
        trc_weight_win2, trc_weight_win2,
    };

    if (rate < 0)
        rate = 0;
    if (rate > 4)
        rate = 4;
    return wins[rate];
}

/* 1/A(z) synthesis on the excitation (in place). */
void trc_lpc_synth(int16_t *exc, const int16_t lpc[TRC_ORDER], int len)
{
    int n, k;

    for (n = 0; n < len; n++) {
        int64_t acc = (int64_t)exc[n] << 14;

        for (k = 0; k < TRC_ORDER; k++)
            acc += (int64_t)lpc[k] * exc[n - 1 - k] * 2;
        /* 64-bit through the <<2 and rounding, then a single saturation. */
        acc = trc_sat32((acc << 2) + 0x8000);
        exc[n] = (int16_t)(acc >> 16);
    }
}

static inline int16_t trc_agc_smooth(int16_t agc, int16_t g_tgt)
{
    int64_t acc;

    /*
     * The smoothed term rounds half-up before the target is added
     * (verified against the reference decoder's settle-from-below fixed
     * point: 4089 for g=256; truncation would give 4081).
     */
    acc = (int64_t)agc * 0x7800 * 2;
    acc += 0x8000;
    acc &= ~0xffffLL;
    acc += (int64_t)g_tgt << 16;
    if (acc > 0x7fff0000LL)
        acc = 0x7fff0000LL;
    else if (acc < 0)
        acc = 0;
    return (int16_t)(acc >> 16);
}

/*
 * 15-step binary search for the largest r such that (int64_t)2*r*r <= x.
 * This is floor(sqrt(x/2)), not floor(sqrt(x)); the sqrt(2) factor cancels
 * in the in/out ratio but the quantization grid does not, so the exact
 * algorithm matters for the AGC target.
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

static inline int16_t trc_agc_target(uint64_t in_e, uint64_t out_e)
{
    uint64_t emax;
    int16_t exp;
    uint32_t sin, sout, si, so;
    uint64_t g;

    if (!in_e)
        in_e = 1;
    if (!out_e)
        out_e = 1;

    /*
     * The input energy is saturated to 32 bits before the max/normalize/
     * ratio; the output energy stays 64-bit.
     */
    if (in_e > 0x7fffffffULL)
        in_e = 0x7fffffffULL;

    emax = in_e > out_e ? in_e : out_e;
    exp  = 0;
    while (emax > 0x7fffffffULL) {
        emax >>= 1;
        exp++;
    }
    while (emax && emax < 0x40000000ULL) {
        emax <<= 1;
        exp--;
    }

    if (exp >= 0) {
        sin  = (uint32_t)(in_e  >> exp);
        sout = (uint32_t)(out_e >> exp);
    } else {
        int up = -exp;

        sin  = (uint32_t)(in_e  << up);
        sout = (uint32_t)(out_e << up);
    }
    if (!sin)
        sin = 1;
    if (!sout)
        sout = 1;

    si = trc_sqrt_q15((int32_t)sin);
    so = trc_sqrt_q15((int32_t)sout);
    if (!so)
        so = 1;

    /*
     * g_tgt = (sqrt(in) << 8) / sqrt(out); the normalization exponent comes
     * from max(in_e, out_e) and is applied to both energies.
     */
    g = ((uint64_t)si << 8) / so;
    if (g < 1)
        g = 1;
    if (g > 0x7fff)
        g = 0x7fff;
    return (int16_t)g;
}

static void trc_expand_lpc(int16_t aw[TRC_ORDER], const int16_t lpc[TRC_ORDER],
                           const int16_t *win)
{
    int i;

    for (i = 0; i < TRC_ORDER; i++)
        aw[i] = trc_round16(trc_l_mult(0, lpc[i], win[i]));
}

/*
 * Tilt-compensation coefficient: run a unit impulse through the weighted
 * pole/zero filter to get 30 samples of its impulse response, then take the
 * scaled lag-1/lag-0 autocorrelation ratio.
 */
static int16_t trc_tilt_coef_ir(const int16_t lpc[TRC_ORDER], const int16_t *win,
                                const int16_t *den)
{
    static const int rels[TRC_ORDER] = { 2, 0, -2, -4, -6, -8, -10, -12, -14, -16 };
    int16_t aw_num[TRC_ORDER], aw_den[TRC_ORDER];
    int16_t buf[40];
    int64_t r0, r1, a, c, d, div, num;
    int ebx, step, rel, ki;
    int widx, hidx;

    for (ki = 0; ki < TRC_ORDER; ki++) {
        aw_num[ki] = trc_round16(trc_l_mult(0, lpc[ki], win[ki]));
        aw_den[ki] = trc_round16(trc_l_mult(0, lpc[ki], den[ki]));
    }

    memset(buf, 0, sizeof(buf));
    buf[10] = 0x2000;
    memcpy(buf + 11, aw_num, TRC_ORDER * sizeof(int16_t));

    ebx = 16;
    for (step = 0; step < 30; step++) {
        int64_t acc;
        int32_t v;

        widx = (ebx + 4) / 2;
        v    = buf[widx];
        acc  = (int64_t)v << 14;
        for (ki = 0; ki < TRC_ORDER; ki++) {
            rel  = rels[ki];
            hidx = (ebx + rel) / 2;
            if (hidx >= 0 && hidx < 40)
                acc += (int64_t)aw_den[ki] * buf[hidx] * 2;
        }
        acc = trc_sat32(acc);
        acc = trc_sat32(acc << 2);
        buf[widx] = trc_round16((int32_t)acc);
        ebx += 2;
    }

    r0 = 0;
    r1 = 0;
    for (ki = 0; ki < 30; ki++) {
        r0 += (int64_t)buf[10 + ki] * buf[10 + ki];
        if (ki < 29)
            r1 += (int64_t)buf[10 + ki] * buf[11 + ki];
    }

    div = trc_sat32(r0 << 1) >> 16;
    a   = r1 << 1;
    c   = a + (a >> 4);
    d   = c >> 1;
    num = d;
    if (div < 1 || r1 < 0)
        return r1 < 0 ? 0 : 0x7fff;
    num /= div;
    if (num > 0x7fff)
        num = 0x7fff;

    return (int16_t)num;
}

/*
 * Formant postfilter B(z/g1)/A(z/g2) plus tilt compensation, both written
 * into the staggered scratch ring: the formant output at ring[pcm_off + n]
 * feeds the IIR recursion; the tilt output lands ten samples behind it.
 * formant_hist (the previous subframe's formant tail) primes the recursion.
 */
static void trc_postfilter_formant_ring(int16_t *ring, int ring_off,
                                        const int16_t *syn,
                                        const int16_t lpc[TRC_ORDER],
                                        int rate, int len)
{
    const int16_t *win = trc_weight_for_rate(rate);
    int16_t aw_num[TRC_ORDER], aw_den[TRC_ORDER];
    int16_t tilt_coef;
    int pcm_off = ring_off + TRC_PF_PCM_LEAD - TRC_ORDER;
    int16_t *ps8;
    const int16_t *ps7;
    int n, k;

    if (len > 256)
        len = 256;

    trc_expand_lpc(aw_num, lpc, win);
    trc_expand_lpc(aw_den, lpc, trc_lag_window);
    tilt_coef = trc_tilt_coef_ir(lpc, win, trc_lag_window);

    ps8 = ring + pcm_off - 8;
    ps7 = syn - 2;

    for (n = 0; n < len; n++) {
        int64_t acc = 0, r;
        int16_t formant, tilted;

        for (k = 0; k < TRC_ORDER; k++)
            acc += (int64_t)aw_num[k] * ps7[1 - k] * 2;
        acc += (int64_t)aw_den[0] * ps8[3] * 2;
        acc += (int64_t)aw_den[1] * ps8[2] * 2;
        acc += (int64_t)aw_den[2] * ps8[1] * 2;
        acc += (int64_t)aw_den[3] * ps8[0] * 2;
        acc += (int64_t)aw_den[4] * ps8[-1] * 2;
        acc += (int64_t)aw_den[5] * ps8[-2] * 2;
        acc += (int64_t)aw_den[6] * ps8[-3] * 2;
        acc += (int64_t)aw_den[7] * ps8[-4] * 2;
        acc += (int64_t)aw_den[8] * ps8[-5] * 2;
        acc += (int64_t)aw_den[9] * ps8[-6] * 2;
        /*
         * The accumulator stays 64-bit through the <<2 and the direct-term
         * add; a single saturation happens at extraction.
         */
        acc = (acc << 2) + ((int64_t)syn[n] << 16);
        r = trc_sat32(acc + 0x8000);
        formant = (int16_t)(r >> 16);

        if (tilt_coef) {
            r = trc_sat32(acc - (int64_t)trc_l_mult(0, ps8[3], tilt_coef)
                              + 0x8000);
            tilted = (int16_t)(r >> 16);
        } else
            tilted = formant;

        ps8[4]  = formant;
        ps8[-6] = tilted;
        ps7++;
        ps8++;
    }
}

/* AGC: scale the tilt band ring[agc_off + TRC_PF_TILT_LEAD ..] in place. */
static void trc_postfilter_agc_ring(int16_t *ring, int agc_off,
                                    const int16_t *syn, int16_t *agc_mem,
                                    int len)
{
    uint64_t in_e = 1, out_e = 1;
    int form_off = agc_off + TRC_PF_TILT_LEAD;
    int16_t g_tgt;
    int n;

    if (len > 256)
        len = 256;

    /* Energy loops accumulate ((x*x) << 1) >> 5; the <<1 keeps an LSB. */
    for (n = 0; n < len; n++)
        in_e += (uint64_t)(((int64_t)((int32_t)syn[n] * syn[n]) << 1) >> 5);
    for (n = 0; n < len; n++)
        out_e += (uint64_t)(((int64_t)((int32_t)ring[form_off + n] *
                                       ring[form_off + n]) << 1) >> 5);

    g_tgt = trc_agc_target(in_e, out_e);

    for (n = 0; n < len; n++) {
        int64_t acc;
        int16_t agc;

        agc = trc_agc_smooth(*agc_mem, g_tgt);
        *agc_mem = agc;

        /* Apply: round(sample * agc * 2 << 4) >> 16. */
        acc = trc_l_mult(0, ring[form_off + n], agc);
        acc = trc_sat32(acc << 4);
        ring[form_off + n] = trc_round16((int32_t)acc);
    }
}

void trc_postfilter(int16_t *pcm, int16_t *exc, const int16_t lpc[TRC_ORDER],
                    int16_t *formant_hist, int16_t *agc_mem, int rate, int len)
{
    int16_t ring[TRC_PF_RING_SAMPLES];

    if (len > 256)
        len = 256;

    memset(ring, 0, sizeof(ring));
    /* Prime the formant IIR history with the previous subframe's tail. */
    memcpy(ring + TRC_PF_TILT_LEAD, formant_hist, TRC_ORDER * sizeof(int16_t));
    trc_postfilter_formant_ring(ring, TRC_ORDER, exc, lpc, rate, len);
    trc_postfilter_agc_ring(ring, 0, exc, agc_mem, len);
    memcpy(pcm, ring + TRC_PF_TILT_LEAD, len * sizeof(int16_t));

    /* formant_hist seeds the next call's feedback: save the formant tail. */
    if (len >= TRC_ORDER)
        memcpy(formant_hist,
               ring + TRC_PF_FORMANT_LEAD + len - TRC_ORDER,
               TRC_ORDER * sizeof(int16_t));
}

static uint32_t trc_noise_step(uint32_t *state)
{
    *state = *state * 1664525u + 1013904223u;
    return *state;
}

void trc_noise_fill(int16_t *exc_block, int block_off, int16_t *gain_idx,
                    int16_t *gain_val, uint32_t *noise_state)
{
    int i;
    int16_t gain;

    if (*gain_idx < 0)
        *gain_idx = 0;
    if (*gain_idx > 25)
        *gain_idx = 25;

    gain = trc_gain_quant[*gain_idx];
    *gain_val = gain;

    for (i = 0; i < 40; i++) {
        int32_t n = (int32_t)(int16_t)(trc_noise_step(noise_state) >> 16);
        int32_t acc = n * gain;
        exc_block[block_off + i] = trc_clip_int16(acc >> 14);
    }
}
