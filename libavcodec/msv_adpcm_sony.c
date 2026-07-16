/*
 * Sony MSV ADPCM decoder core (the reference decoder, bit-exact)
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
/* preamble + helpers for msv_adpcm_sony.c */
#include "msv_adpcm_sony.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifndef AV_RL32
# define AV_RL32(x) ( ((uint32_t)((const uint8_t*)(x))[0]) | \
                      ((uint32_t)((const uint8_t*)(x))[1] << 8) | \
                      ((uint32_t)((const uint8_t*)(x))[2] << 16) | \
                      ((uint32_t)((const uint8_t*)(x))[3] << 24) )
#endif

static MSVADPCMState g;

static int16_t msv_scale_pcm(int16_t si);
static void msv_bs_reset(MSVADPCMBitstream *bs, const uint8_t *base, int limit);
static void msv_bs_set_mode(MSVADPCMBitstream *bs, int mode);
static uint32_t msv_bs_read_symbol(MSVADPCMBitstream *bs, uint32_t *sym_out);
static uint64_t msv_decode_symbol(uint32_t sym, uint32_t init_flag, int mode);

/* Per-symbol entry for msv/tools/msv_adpcm_trace.c only (not in msv_adpcm_sony.h). */
uint64_t msv_adpcm_decode_one(uint32_t sym, uint32_t init_flag, int mode);

static void msv_bs_reset(MSVADPCMBitstream *bs, const uint8_t *base, int limit)
{
    bs->base     = base;
    bs->limit    = limit;
    bs->byte_off = 0;
    bs->sym_idx  = 0;
    bs->mode     = 1;
    bs->adv      = 3;
    bs->sym_max  = 8;
}

static void msv_bs_set_mode(MSVADPCMBitstream *bs, int mode)
{
    bs->mode = mode;
    if (mode == 2) {
        bs->sym_max = 4;
        bs->adv     = 1;
    } else if (mode == 3) {
        bs->adv     = 3;
        bs->sym_max = 8;
    } else if (mode == 4) {
        bs->sym_max = 2;
        bs->adv     = 1;
    }
}

static uint32_t msv_bs_read_symbol(MSVADPCMBitstream *bs, uint32_t *sym_out)
{
    const uint8_t *p = bs->base + bs->byte_off;
    int bits, mask;

    switch (bs->mode) {
    case 2: bits = 2; mask = 0x3; break;   /* 4 syms / byte (LP) */
    case 3: bits = 3; mask = 0x7; break;   /* 8 syms / 3 bytes (SP) */
    case 4: bits = 4; mask = 0xf; break;   /* 2 syms / byte (HQ) */
    default: return 0;
    }

    /* symbols are packed LSB-first within a little-endian word */
    *sym_out = (AV_RL32(p) >> (bits * bs->sym_idx)) & mask;
    bs->sym_idx++;
    if (bs->sym_idx >= bs->sym_max) {
        bs->byte_off += bs->adv;
        bs->sym_idx = 0;
        if (bs->limit < bs->byte_off + bs->adv)
            return 0;
    }
    return 1;
}

static void msv_postfilter(uint32_t sym, uint32_t *gain, uint32_t *exp_class, int mode)
{
    uint32_t idx = sym & 0xf;

    if (mode == 4) {
        static const uint16_t lo[8] = {
            0x800, 0x004, 0x087, 0x0d5, 0x111, 0x143, 0x175, 0x1a9
        };
        static const uint16_t hi[8] = {
            0x1a9, 0x175, 0x143, 0x111, 0x0d5, 0x087, 0x004, 0x800
        };
        *exp_class = idx >> 3;
        *gain = (idx < 8) ? lo[idx & 7] : hi[idx & 7];
        return;
    }
    if (mode == 3) {
        static const uint16_t lo[4] = { 0x800, 0x087, 0x111, 0x175 };
        static const uint16_t hi[4] = { 0x175, 0x111, 0x087, 0x800 };
        *exp_class = idx >> 2;
        *gain = (idx < 4) ? lo[idx & 3] : hi[idx & 3];
        return;
    }
    if (mode == 2) {
        *exp_class = idx >> 1;
        if (idx >> 1 == 0)
            *gain = (idx == 0) ? 0x74 : 0x16d;
        else {
            if (idx == 2)
                *gain = 0x16d;
            else if (idx == 3)
                *gain = 0x74;
        }
    }
}

static uint32_t msv_state_word(int sel, uint32_t cur, uint32_t val)
{
    return sel ? val : cur;
}

static uint32_t msv_sym_index(uint32_t sym, int mode)
{
    sym &= 0xf;
    if (mode == 4) {
        if (sym & 8)
            return (uint32_t)(-(int32_t)sym - 1);
        return sym;
    }
    if (mode == 3) {
        if (sym & 0xc)
            return (uint32_t)(-(int32_t)sym - 1);
        return sym;
    }
    if (mode == 2) {
        if (sym & 2)
            return (uint32_t)(-(int32_t)sym - 1);
        return sym;
    }
    return sym;
}

static const uint16_t msv_quant_tab[3][8] = {
    { 0xfea, 0x1b7 },                         /* mode 2: only [0],[1] used */
    { 0xffc, 0x01e, 0x089, 0x246 },           /* mode 3 */
    { 0xff4, 0x012, 0x029, 0x040, 0x070, 0x0c6, 0x163, 0x462 }, /* mode 4 */
};

static const uint8_t msv_bw_tab[3][8] = {
    { 0, 7 },                                 /* mode 2 */
    { 0, 1, 2, 7 },                           /* mode 3 */
    { 0, 0, 0, 1, 1, 1, 3, 7 },               /* mode 4 */
};

static uint32_t msv_quant_step(uint32_t sym, int mode)
{
    uint32_t idx = msv_sym_index(sym, mode);

    if (mode == 4)
        return msv_quant_tab[2][idx & 7];
    if (mode == 3)
        return msv_quant_tab[1][idx & 3];
    if (mode == 2)
        return msv_quant_tab[0][idx & 1];
    return sym;
}

static uint32_t msv_sym_bw_scale(uint32_t sym, int mode)
{
    uint32_t idx = msv_sym_index(sym, mode);

    if (mode == 4)
        return msv_bw_tab[2][idx & 7];
    if (mode == 3)
        return msv_bw_tab[1][idx & 3];
    if (mode == 2)
        return msv_bw_tab[0][idx & 1];
    return sym;
}

static uint32_t msv_del_e4_step(uint32_t sign_e4, uint32_t del_e4)
{
    if (sign_e4 == 0) {
        if (del_e4 < 0x2000)
            return del_e4 * 4;
        return 0x7ffc;
    }
    if (del_e4 < 0xe001)
        return 0x18004;
    return (del_e4 & 0x7fff) * 4;
}

static uint32_t msv_del_e0_inc(uint32_t tmp_b, int adapt_cmp)
{
    if (tmp_b >> 16 == 0)
        return adapt_cmp ? 0 : tmp_b >> 7;
    if ((tmp_b >> 16) != 1 || adapt_cmp)
        return 0;
    return (tmp_b >> 7) + 0xfc00;
}

static uint32_t msv_del_e0_saturate(uint32_t sum)
{
    if (sum >= 0x8000 && sum <= 0xd000)
        return 0xd000;
    if (sum > 0x2fff && sum < 0x8000)
        return 0x3000;
    return sum;
}

static uint32_t msv_del_e4_predict(uint32_t sign_e4, uint32_t del_e4, uint32_t tap_seed,
                                    uint32_t del_e0_sum)
{
    uint32_t acc, lo, hi;

    if (sign_e4)
        acc = 0x100 - (del_e4 >> 8);
    else
        acc = -(del_e4 >> 8);
    acc = ((acc & 0xffff) + tap_seed + del_e4) & 0xffff;
    lo = (0x3c00 - del_e0_sum) & 0xffff;
    hi = (del_e0_sum - 0x3c00) & 0xffff;
    if ((((acc < 0x8000) || (hi < acc)) && (hi = acc, lo <= acc)) && (acc < 0x8000))
        return lo;
    return hi;
}

static uint32_t msv_tap0_seed(uint32_t acc_xor, int adapt_cmp)
{
    if (acc_xor == 0)
        return adapt_cmp ? 0 : 0xc0;
    if (acc_xor == 1 && !adapt_cmp)
        return 0xff40;
    return adapt_cmp ? 0 : 0;
}

static uint32_t msv_adapt_ac(int ovfl, uint32_t pred_cmp)
{
    return ~-(uint32_t)(ovfl != 0) & pred_cmp;
}

static int msv_adapt_pred_cmp(int sum)
{
    return (int)('\x01' - (0x51ff < sum - 0x8000U));
}

/* Delay-line magnitude + tap scale; must match asm bit-exact. */
static uint32_t msv_norm_mag_13bit(uint32_t word, int *exp_shift)
{
    uint32_t mag;
    int exp;

    if ((word >> 15) == 0)
        mag = word >> 2;
    else
        mag = -(word >> 2) & 0x1fff;

    if (mag >= 0x1000) {
        *exp_shift = 0xd;
        if (mag == 0)
            return 0x20;
        return (mag << 6) >> (int8_t)*exp_shift;
    }
    if (mag == 0) {
        *exp_shift = 0;
        return 0x20;
    }
    if (mag == 1) {
        *exp_shift = 1;
        return (mag << 6) >> 1;
    }
    if (mag - 0x800 < 0x800)
        exp = 0xc;
    else if (mag - 0x400 < 0x400)
        exp = 0xb;
    else if (mag - 0x200 < 0x200)
        exp = 10;
    else if (mag - 0x100 < 0x100)
        exp = 9;
    else if (mag - 0x80 < 0x80)
        exp = 8;
    else if (mag - 0x40 < 0x40)
        exp = 7;
    else if (mag - 0x20 < 0x20)
        exp = 6;
    else if (mag - 0x10 < 0x10)
        exp = 5;
    else if (mag - 8 < 8)
        exp = 4;
    else if (mag - 4 < 4)
        exp = 3;
    else if (mag - 2 < 2)
        exp = 2;
    else
        exp = 0;
    *exp_shift = exp;
    return (mag << 6) >> (int8_t)exp;
}

static uint32_t msv_scale_tap(uint32_t coef_word, uint32_t mag_norm, int exp_shift,
                              uint32_t sign_ref)
{
    uint32_t coeff = (coef_word >> 6 & 0xf) + exp_shift;
    uint32_t out = (((coef_word & 0x3f) * mag_norm + 0x30) >> 4) << 7;
    char sb = (char)coeff;

    if (coeff < 0x1b)
        out = out >> ((0x1aU - sb) & 0x1f);
    else
        out = out << ((sb - 0x1aU) & 0x1f) & 0x7fff;
    if ((coef_word >> 10) != sign_ref)
        out = -out & 0xffff;
    return out;
}

/* 15-bit PCM magnitude exponent (0x4000 ladder); packs mantissa before *0x40 term. */
static uint32_t msv_exp_mant_15bit(uint32_t mag, int *exp_idx)
{
    int exp;

    if (mag < 0x4000) {
        if (mag - 0x2000 < 0x2000)
            exp = 0xe;
        else if (mag - 0x1000 < 0x1000)
            exp = 0xd;
        else if (mag - 0x800 < 0x800)
            exp = 0xc;
        else if (mag - 0x400 < 0x400)
            exp = 0xb;
        else if (mag - 0x200 < 0x200)
            exp = 10;
        else if (mag - 0x100 < 0x100)
            exp = 9;
        else if (mag - 0x80 < 0x80)
            exp = 8;
        else if (mag - 0x40 < 0x40)
            exp = 7;
        else if (mag - 0x20 < 0x20)
            exp = 6;
        else if (mag - 0x10 < 0x10)
            exp = 5;
        else if (mag - 8 < 8)
            exp = 4;
        else if (mag - 4 < 4)
            exp = 3;
        else if (mag - 2 < 2)
            exp = 2;
        else if (mag == 1) {
            *exp_idx = 1;
            return (mag << 6) >> 1;
        }
        if (mag == 0) {
            *exp_idx = 0;
            return 0x20;
        }
        *exp_idx = exp;
        return (mag << 6) >> ((uint8_t)exp & 0x1f);
    }
    *exp_idx = 0xf;
    if (mag == 0)
        return 0x20;
    return (mag << 6) >> 0xf;
}

typedef struct {
    uint32_t delay;
    uint32_t delta;
} MSVDelayStep;

static MSVDelayStep msv_delay_line_step(uint32_t tap_sign, uint32_t pcm_sign, uint32_t pcm_mag,
                                        uint32_t *saved_delta, uint32_t delta, int first_tap,
                                        uint32_t delay_sign, uint32_t delay_word, uint32_t pf_ovfl)
{
    MSVDelayStep r;
    uint32_t acc;

    if ((tap_sign ^ pcm_sign) == 0)
        delta = pcm_mag ? 0x80 : 0;
    else if ((tap_sign ^ pcm_sign) == 1)
        delta = pcm_mag ? 0xff80 : 0;
    else if (first_tap)
        delta = pcm_mag ? *saved_delta : 0;
    else if (!pcm_mag)
        delta = 0;

    if (delay_sign == 0)
        acc = -(delay_word >> 8);
    else
        acc = 0x100 - (delay_word >> 8);
    if (!pf_ovfl)
        r.delay = ((acc & 0xffff) + delta + delay_word) & 0xffff;
    else
        r.delay = 0;
    r.delta = delta;
    return r;
}

static uint32_t msv_sub_shift(uint32_t val, uint32_t mul, uint32_t sub,
                              uint32_t mask, uint32_t sign_msk,
                              unsigned shr, uint32_t bias)
{
    uint32_t d = val * mul - sub;
    uint32_t t = d & mask;

    if ((d & sign_msk) == 0)
        return t >> shr;
    return (t >> shr) + bias;
}

static void msv_adapt_filter(uint32_t bw, uint32_t bc_pred, int adapt_cmp,
                             uint32_t *filt_lo, uint32_t *filt_hi,
                             uint32_t *pred_abs, int *adapt_sel)
{
    uint32_t tmp_a, tmp_c, tmp_d, coeff;

    tmp_d = msv_sub_shift(bw, 0x200, *filt_hi, 0x1fff, 0x1000, 5, 0xf00);
    tmp_c = msv_sub_shift(bw, 0x800, *filt_lo, 0x7fff, 0x4000, 7, 0x3f00);
    tmp_d = (tmp_d + *filt_hi) & 0xfff;
    tmp_a = (tmp_c + *filt_lo) & 0x3fff;
    coeff = tmp_d * 4 - tmp_a;
    tmp_c = coeff & 0x7fff;
    if (coeff & 0x4000)
        tmp_c = -tmp_c & 0x3fff;
    *pred_abs = tmp_c;
    *adapt_sel = (bc_pred < 0x600 || (tmp_a >> 3 <= tmp_c) || adapt_cmp) ? 1 : 0;
    *filt_lo = tmp_a;
    *filt_hi = tmp_d;
}

static uint32_t msv_bw_scale_update(int adapt_sel, uint32_t pf_ovfl, uint32_t bw_scale)
{
    uint32_t d = adapt_sel * 0x200 - bw_scale;
    uint32_t t = d & 0x7ff;

    if ((d & 0x400) == 0)
        t = t >> 4;
    else
        t = (t >> 4) + 0x380;
    if (!pf_ovfl)
        return (t + bw_scale) & 0x3ff;
    return 0x100;
}

static uint32_t msv_quant_base_update(uint32_t sym, int mode, uint32_t bc_pred)
{
    uint32_t q = msv_quant_step(sym, mode) * 0x20 - bc_pred;
    uint32_t t = q & 0x1ffff;

    if ((q & 0x10000) == 0)
        t = t >> 5;
    else
        t = (t >> 5) + 0x1000;
    t = (t + bc_pred) & 0x1fff;
    if ((t - 0x220) & 0x2000)
        return 0x220;
    if (((t - 0x1400) & 0x2000) == 0)
        return 0x1400;
    return t;
}

static uint32_t msv_sym_acc_mag(uint32_t quant_base, uint32_t sym_acc)
{
    uint32_t next = (-sym_acc >> 6) + quant_base;
    uint32_t m = next & 0x3fff;

    if (next & 0x2000)
        m += 0x7c000;
    return m;
}

typedef struct {
    uint32_t tap0, del_e8_mag, e4, e0, dc, d8, d4, d0, cc, c8;
    uint32_t adapt_ctrl, bw_scale, filt_lo, filt_hi;
} MSVSymCommit;

typedef struct MSVSymbolCtx {
    int mode;
    int exp_shift;
    int adapt_cmp;
    int exp_idx;
    uint32_t sym_lo;
    uint32_t pf_ovfl;
    uint32_t synth;
    uint32_t pcm_mag;
    uint32_t bc_pred;
    uint32_t adapt_sum;
    uint32_t scl_c4_scaled;
    uint32_t tap_c0_scaled;
    uint32_t f0_tap_scaled;
    uint32_t tap1_scaled;
    uint32_t tap2_scaled, tap3_scaled, tap4_scaled, tap5_scaled;
    uint32_t sign_e4, sign_e0;
    uint32_t sign_t0, sign_t1, sign_t2, sign_t3, sign_t4, sign_t5;
    uint32_t sign_dc, sign_d8, sign_d4, sign_d0, sign_cc, sign_c8;
    uint32_t del_e0_new, del_e0_sum;
    uint32_t new_tap0;
    uint32_t saved_delta;
    uint32_t acc, mag, coeff;
    uint32_t tmp_a, tmp_b, tmp_c, tmp_d;
    uint32_t sym_acc_out;
} MSVSymbolCtx;

static void msv_state_commit(const MSVSymCommit *c, uint32_t scl_c4_scaled)
{
    g.tap5 = g.tap4;
    g.tap4 = g.tap3;
    g.tap3 = g.tap2;
    g.tap2 = g.tap1;
    g.tap1 = g.tap0;
    g.tap0 = c->tap0;
    g.del_ec = g.del_e8;
    g.del_e8 = c->del_e8_mag;
    g.del_e4 = c->e4;
    g.del_e0 = c->e0;
    g.del_dc = c->dc;
    g.del_d8 = c->d8;
    g.del_d4 = c->d4;
    g.del_d0 = c->d0;
    g.scl_c4 = g.scl_c0;
    g.del_cc = c->cc;
    g.scl_c0 = scl_c4_scaled;
    g.del_c8 = c->c8;
    g.adapt_ctrl = c->adapt_ctrl;
    g.bw_scale = c->bw_scale;
    g.filt_lo = c->filt_lo;
    g.filt_hi = c->filt_hi;
}

static uint16_t msv_decode_symbol_init(void)
{
    g.tap0 = msv_state_word(1, 0, 0x20);
    g.tap1 = g.tap0;
    g.tap2 = g.tap0;
    g.tap3 = g.tap0;
    g.tap4 = g.tap0;
    g.tap5 = g.tap0;
    g.del_c8 = msv_state_word(1, 0, 0);
    g.scl_c0 = g.tap0;
    g.scl_c4 = g.tap0;
    g.del_cc = g.del_c8;
    g.del_d0 = g.del_c8;
    g.del_d4 = g.del_c8;
    g.del_d8 = g.del_c8;
    g.del_dc = g.del_c8;
    g.del_e0 = g.del_c8;
    g.del_e4 = g.del_c8;
    g.del_e8 = g.del_c8;
    g.del_ec = g.del_c8;
    g.quant_base = msv_state_word(1, 0, 0x220);
    g.sym_acc = msv_state_word(1, 0, 0x8800);
    g.adapt_ctrl = 0;
    g.bw_scale = 0;
    g.filt_lo = 0;
    g.filt_hi = 0;
    return (uint16_t)msv_scale_pcm(0);
}

static void msv_decode_tap_sum(MSVSymbolCtx *c)
{
    c->sign_e4 = g.del_e4 >> 0xf;
    c->mag = msv_norm_mag_13bit(g.del_e4, &c->exp_shift);
    c->tap_c0_scaled = msv_scale_tap(g.scl_c0, c->mag, c->exp_shift, c->sign_e4);
    c->sign_e0 = g.del_e0 >> 0xf;
    c->mag = msv_norm_mag_13bit(g.del_e0, &c->exp_shift);
    c->scl_c4_scaled = msv_scale_tap(g.scl_c4, c->mag, c->exp_shift, c->sign_e0);
    c->sign_dc = g.del_dc >> 0xf;
    c->sign_t0 = g.tap0 >> 10;
    c->mag = msv_norm_mag_13bit(g.del_dc, &c->exp_shift);
    c->f0_tap_scaled = msv_scale_tap(g.tap0, c->mag, c->exp_shift, c->sign_dc);
    c->sign_d8 = g.del_d8 >> 0xf;
    c->sign_t1 = g.tap1 >> 10;
    c->mag = msv_norm_mag_13bit(g.del_d8, &c->exp_shift);
    c->tap1_scaled = msv_scale_tap(g.tap1, c->mag, c->exp_shift, c->sign_d8);
    c->sign_d4 = g.del_d4 >> 0xf;
    c->sign_t2 = g.tap2 >> 10;
    c->mag = msv_norm_mag_13bit(g.del_d4, &c->exp_shift);
    c->tap2_scaled = msv_scale_tap(g.tap2, c->mag, c->exp_shift, c->sign_d4);
    c->sign_d0 = g.del_d0 >> 0xf;
    c->sign_t3 = g.tap3 >> 10;
    c->mag = msv_norm_mag_13bit(g.del_d0, &c->exp_shift);
    c->tap3_scaled = msv_scale_tap(g.tap3, c->mag, c->exp_shift, c->sign_d0);
    c->sign_cc = g.del_cc >> 0xf;
    c->sign_t4 = g.tap4 >> 10;
    c->mag = msv_norm_mag_13bit(g.del_cc, &c->exp_shift);
    c->tap4_scaled = msv_scale_tap(g.tap4, c->mag, c->exp_shift, c->sign_cc);
    c->sign_c8 = g.del_c8 >> 0xf;
    c->sign_t5 = g.tap5 >> 10;
    c->mag = msv_norm_mag_13bit(g.del_c8, &c->exp_shift);
    c->tap5_scaled = msv_scale_tap(g.tap5, c->mag, c->exp_shift, c->sign_c8);
    c->tmp_b = (c->tap5_scaled + c->tap4_scaled + c->tap3_scaled + c->tap2_scaled +
                c->tap1_scaled + c->f0_tap_scaled) & 0xffff;
    c->mag = (c->tmp_b + c->scl_c4_scaled + c->tap_c0_scaled) >> 1;
    c->tmp_b = c->tmp_b >> 1;
    c->coeff = c->mag & 0x7fff;
}

static void msv_decode_synth(MSVSymbolCtx *c)
{
    if (g.bw_scale < 0x100)
        c->tmp_c = g.bw_scale >> 2;
    else
        c->tmp_c = 0x40;
    c->tmp_d = (g.quant_base - (g.sym_acc >> 6)) & 0x3fff;
    c->tmp_a = c->tmp_d >> 0xd;
    if (c->tmp_a != 0)
        c->tmp_d = -c->tmp_d & 0x1fff;
    c->tmp_c = c->tmp_d * c->tmp_c >> 6;
    if (c->tmp_a != 0)
        c->tmp_c = -c->tmp_c & 0x3fff;
    c->tmp_c = ((g.sym_acc >> 6) + c->tmp_c) & 0x1fff;
    c->bc_pred = c->tmp_c;
    msv_postfilter(c->sym_lo, &c->pf_ovfl, &c->synth, c->mode);
    c->tmp_c = (c->tmp_c >> 2) + c->pf_ovfl;
    if ((c->tmp_c & 0x800) == 0) {
        c->tmp_c = ((c->tmp_c & 0x7f) + 0x80) * 0x80 >>
                   (((0xe - (uint8_t)((c->tmp_c & 0xfff) >> 7)) & 0xf) & 0x1f);
    } else {
        c->tmp_c = 0;
    }
    c->tmp_c = c->synth * 0x8000 + c->tmp_c;
    c->pcm_mag = c->tmp_c & 0x7fff;
    c->tmp_d = c->acc >> 0xf;
    if (c->tmp_d < 9)
        c->acc = (((c->acc >> 10) & 0x1f) + 0x20) << ((uint8_t)c->tmp_d & 0x1f);
    else
        c->acc = 0x3e00;
    if ((c->pcm_mag <= ((c->acc >> 1) + c->acc) >> 1) ||
        (c->pf_ovfl = 1, g.adapt_ctrl != 1))
        c->pf_ovfl = 0;
    c->tap1_scaled = c->tmp_c >> 0xf;
    c->acc = c->tmp_c;
    if (c->tap1_scaled != 0)
        c->acc = -c->pcm_mag & 0xffff;
    if ((c->mag & 0x4000) != 0)
        c->coeff = c->coeff + 0x8000;
    c->synth = (c->acc + c->coeff) & 0xffff;
    c->acc = c->synth;
    if (c->synth >> 0xf != 0)
        c->acc = -c->synth & 0x7fff;
    c->scl_c4_scaled = msv_exp_mant_15bit(c->acc, &c->exp_idx);
    c->scl_c4_scaled = ((c->synth >> 0xf) * 0x10 + c->exp_idx) * 0x40 + c->scl_c4_scaled;
}

static void msv_decode_adapt(MSVSymbolCtx *c)
{
    if (c->tap1_scaled != 0)
        c->tmp_c = -c->pcm_mag & 0xffff;
    if ((c->tmp_b & 0xffffc000) != 0)
        c->tmp_b = c->tmp_b + 0x8000;
    c->acc = (c->tmp_c + c->tmp_b) & 0xffff;
    c->mag = c->acc >> 0xf;
    c->adapt_cmp = c->acc == 0;
    c->acc = c->mag ^ g.del_e8;
    c->tmp_b = msv_del_e4_step(c->sign_e4, g.del_e4);
    if (c->acc != 1)
        c->tmp_b = -c->tmp_b & 0x1ffff;
    c->tmp_b = ((-(uint32_t)(c->mag != g.del_ec) & 0x18000) + 0x4000 + c->tmp_b) & 0x1ffff;
    c->del_e0_new = msv_del_e0_inc(c->tmp_b, c->adapt_cmp);
    if (c->sign_e0 == 0)
        c->tmp_b = -(g.del_e0 >> 7);
    else
        c->tmp_b = 0x200 - (g.del_e0 >> 7);
    c->del_e0_sum = msv_del_e0_saturate(((c->tmp_b & 0xffff) + c->del_e0_new + g.del_e0) & 0xffff);
    c->adapt_sum = c->del_e0_sum;
    c->del_e0_new = ~-(uint32_t)(c->pf_ovfl != 0) & c->del_e0_sum;
    c->new_tap0 = msv_tap0_seed(c->acc, c->adapt_cmp);
    c->sign_e0 = msv_del_e4_predict(c->sign_e4, g.del_e4, c->new_tap0, c->del_e0_sum);
    c->sign_e0 = ~-(uint32_t)(c->pf_ovfl != 0) & c->sign_e0;
    c->new_tap0 = msv_exp_mant_15bit(c->pcm_mag, &c->exp_idx);
    c->new_tap0 = (c->tap1_scaled * 0x10 + c->exp_idx) * 0x40 + c->new_tap0;
    {
        MSVDelayStep ds = { 0, 0 };

        ds = msv_delay_line_step(c->sign_t0, c->tap1_scaled, c->pcm_mag, &c->saved_delta,
                                 ds.delta, 1, c->sign_dc, g.del_dc, c->pf_ovfl);
        c->sign_dc = ds.delay;
        ds = msv_delay_line_step(c->sign_t1, c->tap1_scaled, c->pcm_mag, &c->saved_delta,
                                 ds.delta, 0, c->sign_d8, g.del_d8, c->pf_ovfl);
        c->sign_d8 = ds.delay;
        ds = msv_delay_line_step(c->sign_t2, c->tap1_scaled, c->pcm_mag, &c->saved_delta,
                                 ds.delta, 0, c->sign_d4, g.del_d4, c->pf_ovfl);
        c->sign_d4 = ds.delay;
        ds = msv_delay_line_step(c->sign_t3, c->tap1_scaled, c->pcm_mag, &c->saved_delta,
                                 ds.delta, 0, c->sign_d0, g.del_d0, c->pf_ovfl);
        c->sign_d0 = ds.delay;
        ds = msv_delay_line_step(c->sign_t4, c->tap1_scaled, c->pcm_mag, &c->saved_delta,
                                 ds.delta, 0, c->sign_cc, g.del_cc, c->pf_ovfl);
        c->sign_cc = ds.delay;
        ds = msv_delay_line_step(c->sign_t5, c->tap1_scaled, c->pcm_mag, &c->saved_delta,
                                 ds.delta, 0, c->sign_c8, g.del_c8, c->pf_ovfl);
        c->tmp_b = ds.delay;
    }
}

static void msv_decode_filter_commit(MSVSymbolCtx *c)
{
    MSVSymCommit commit;

    c->exp_idx = c->mag;
    c->adapt_cmp = msv_adapt_pred_cmp((int)c->adapt_sum);
    c->acc = msv_adapt_ac(c->pf_ovfl, c->adapt_cmp);
    c->coeff = msv_sym_bw_scale(c->sym_lo, c->mode);
    c->mag = c->bc_pred;
    msv_adapt_filter(c->coeff, c->mag, c->adapt_cmp, &g.filt_lo, &g.filt_hi,
                     &c->tmp_c, &c->exp_shift);
    c->coeff = msv_bw_scale_update(c->exp_shift, c->pf_ovfl, g.bw_scale);
    g.quant_base = msv_quant_base_update(c->sym_lo, c->mode, c->mag);
    c->mag = msv_sym_acc_mag(g.quant_base, g.sym_acc);
    c->sym_acc_out = (c->mag + g.sym_acc) & 0x7ffff;
    commit.tap0       = c->new_tap0;
    commit.del_e8_mag = c->exp_idx;
    commit.e4         = c->sign_e0;
    commit.e0         = c->del_e0_new;
    commit.dc         = c->sign_dc;
    commit.d8         = c->sign_d8;
    commit.d4         = c->sign_d4;
    commit.d0         = c->sign_d0;
    commit.cc         = c->sign_cc;
    commit.c8         = c->tmp_b;
    commit.adapt_ctrl = c->acc;
    commit.bw_scale   = c->coeff;
    commit.filt_lo    = g.filt_lo;
    commit.filt_hi    = g.filt_hi;
    msv_state_commit(&commit, c->scl_c4_scaled);
    g.sym_acc = c->sym_acc_out;
}

void msv_adpcm_bs_init(MSVADPCMBitstream *bs, const uint8_t *buf, int limit, int mode)
{
    msv_bs_reset(bs, buf, limit);
    msv_bs_set_mode(bs, mode);
}

int msv_adpcm_bs_next(MSVADPCMBitstream *bs, uint32_t *sym)
{
    return msv_bs_read_symbol(bs, sym);
}

void msv_adpcm_get_state(MSVADPCMState *st)
{
    if (st)
        *st = g;
}

void msv_adpcm_set_state(const MSVADPCMState *st)
{
    if (st)
        g = *st;
}

uint64_t msv_adpcm_decode_one(uint32_t sym, uint32_t init_flag, int mode)
{
    return msv_decode_symbol(sym, init_flag, mode);
}

static uint64_t msv_decode_symbol(uint32_t sym, uint32_t init_flag, int mode)
{
    MSVSymbolCtx c;

    memset(&c, 0, sizeof(c));
    c.sym_lo = sym & 0xf;
    c.mode   = mode;
    c.acc    = g.sym_acc;

    if (init_flag == 1)
        return msv_decode_symbol_init();

    msv_decode_tap_sum(&c);
    msv_decode_synth(&c);
    msv_decode_adapt(&c);
    msv_decode_filter_commit(&c);
    return (uint16_t)msv_scale_pcm((int16_t)(uint16_t)c.synth);
}
static int16_t msv_scale_pcm(int16_t si)
{
    double v = (double)si * 3.5;
    return (int16_t)(int32_t)v;
}

int msv_adpcm_samples_per_frame(int mode)
{
    switch (mode) {
    case 2:  return 192;   /* 48 bytes * 4 syms/byte */
    case 4:  return 96;    /* 48 bytes * 2 syms/byte */
    case 3:
    default: return 128;   /* 48 bytes * 8 syms/3 bytes */
    }
}

int msv_adpcm_decode_block(const uint8_t *buf, int16_t *out, int mode,
                          int *need_init, MSVADPCMBitstream *bs)
{
    static MSVADPCMBitstream bs_fallback;
    MSVADPCMBitstream *work = bs ? bs : &bs_fallback;
    int nsamples = msv_adpcm_samples_per_frame(mode);
    uint32_t sym;
    int i;

    if (!bs)
        memset(&bs_fallback, 0, sizeof(bs_fallback));
    msv_bs_reset(work, buf, MSV_ADPCM_FRAME_SIZE);
    msv_bs_set_mode(work, mode);
    for (i = 0; i < nsamples; i++) {
        int ok = msv_bs_read_symbol(work, &sym);
        out[i] = (int16_t)msv_decode_symbol(sym, *need_init ? 1u : 0u, mode);
        *need_init = 0;
        if (!ok)
            break;
    }
    return i + 1;
}
