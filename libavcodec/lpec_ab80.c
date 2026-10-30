/*
 * LPEC QMF synthesis filterbank (cosine-modulated), reverse-engineered from
 * the Sony LPEC decoder.
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
#include "libavutil/mathematics.h"
#include "lpec_ab80.h"

#include <math.h>
#include <string.h>

#define LPEC_QMF_SCALE 2.0
#define LPEC_QMF_FFT_NORM 1.0
#define LPEC_COS1536_WI_OFF 384 /* wi table: same index as wr, phase +768 sin indices */

/*
 * The reference tables use truncated pi constants and an extended-precision
 * sine rounded to double.
 */

/* fills (2048 entries). */
static void lpec_qmf_fill_cos2048(double *dst, int n)
{
    int i;
    double step = 6.283185307 / n;

    for (i = 0; i < n; i++)
        dst[i] = (double) sinl((double) i * step);
}

/* fills (1536 entries, even indices). */
static void lpec_qmf_fill_cos1536(double *dst, int n)
{
    int i;
    double step = 3.1415926535 / n;

    for (i = 0; i < n; i++)
        dst[i] = (double) sinl((double) (2 * i) * step);
}

static void lpec_qmf_fill_sin(double *dst, int len)
{
    int i;
    double step = 3.14159265359 / (2.0 * len);

    /* len/2 entries at sin((2k+1)*pi/(2*len)), k=0..len/2-1. */
    for (i = 1; i < len; i += 2)
        dst[i >> 1] = (double) sinl((double) i * step);
}

static void lpec_qmf_build_desc(LPECQmfDesc *desc, double *sin_buf, int len,
                                const double *cos)
{
    desc->scale = len % 3 ? (2048 / len) : (6144 / (len * 4));
    desc->len   = len;
    desc->sin   = sin_buf;
    desc->cos   = cos;
    lpec_qmf_fill_sin(sin_buf, len);
}

/* bit-reversal permutation. */
static void lpec_qmf_permute012c0(double *a, double *b, int n)
{
    int i, m = 0;
    int half = n >> 1;

    for (i = 0; i < n - 1; i++) {
        if (i < m) {
            double ta = a[i], tb = b[i];
            a[i] = a[m]; a[m] = ta;
            b[i] = b[m]; b[m] = tb;
        }
        {
            int j = half;
            if (j <= m) {
                do {
                    m -= j;
                    j >>= 1;
                } while (j > 0 && j <= m);
            }
            m += j;
        }
    }
}

/*
 * Pass1 modulation bin count (pairs fill lo/hi[0..mod_n)).
 * Empirically flen/4 matches the reference decoder; the decompiler labels as (flen*3/4)/4.
 */
static int lpec_ab80_mod_n(int flen)
{
    return flen >> 2;
}

static int lpec_cos_period(int flen)
{
    return flen % 3 ? 2048 : 1536;
}

/* Pass1/2 walk cos pointers backward; reference decoder table is periodic sin steps. */
static double lpec_cos_at(const double *cos, int idx, int period)
{
    idx %= period;
    if (idx < 0)
        idx += period;
    return cos[idx];
}

/* (cos2048) / twiddles (cos1536: wr @ i, wi @ i+384). */
static void lpec_qmf_fft01430(double *a, double *b, int n, const double *cos,
                              int tw_base)
{
    int tw_step;
    int size;
    int wi_off = (tw_base == 1536) ? LPEC_COS1536_WI_OFF : 512;

    if (n <= 4 || !cos)
        return;

    tw_step = tw_base / n;
    size = n;

    if (tw_step <= 0)
        return;

    while (size > 4) {
        int half = size >> 1;
        int j = 0;
        int tw_idx = 0;
        int step = tw_step;

        while (j < half) {
            double wr = cos[tw_idx];
            double wi = cos[tw_idx + wi_off];
            int k;

            for (k = j; k < n; k += size) {
                int p = k + half;
                double dr = a[k] - a[p];
                double di = b[k] - b[p];

                a[k] += a[p];
                b[k] += b[p];
                a[p] = dr * wi - di * wr;
                b[p] = dr * wr + di * wi;
            }
            j++;
            tw_idx += step;
        }
        tw_step *= 2;
        size = half;
    }
}

static void lpec_qmf_stage01340(double *a, double *b, int n, double invn)
{
    int q = n >> 2;
    int i;
    double sc = LPEC_QMF_FFT_NORM * invn;

    for (i = 0; i < q; i++) {
        double a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3];
        double b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3];

        a[1] = ((a2 + a0) - (a3 + a1)) * sc;
        a[0] = ((a2 + a0) + (a3 + a1)) * sc;
        b[1] = ((b0 + b2) - (b1 + b3)) * sc;
        b[0] = ((b0 + b2) + (b1 + b3)) * sc;
        a[3] = ((b1 - b3) + (a0 - a2)) * sc;
        a[2] = ((a0 - a2) - (b1 - b3)) * sc;
        b[2] = ((b0 - b2) + (a1 - a3)) * sc;
        b[3] = ((b0 - b2) - (a1 - a3)) * sc;
        a += 4;
        b += 4;
    }
}

/* Radix-3 output shuffle after three sub-FFTs.
 * Interleave block0[k], block1[k], block2[k] -> buf[3*k..3*k+2]. */
static void lpec_qmf_shuffle01590(double *buf, int n)
{
    int u = n / 3;
    int k;
    double tmp[LPEC_QMF_FFT_MAX];

    if (u <= 0 || n > LPEC_QMF_FFT_MAX)
        return;

    memcpy(tmp, buf, n * sizeof(double));
    for (k = 0; k < u; k++) {
        buf[3 * k]     = tmp[k];
        buf[3 * k + 1] = tmp[u + k];
        buf[3 * k + 2] = tmp[2 * u + k];
    }
}

/*
 * Radix-3 first pass of the reference transform for n = 3 * 2^k. The
 * reference runs the x87 at 53-bit precision, so plain double arithmetic in
 * the same order reproduces it exactly.
 */
static void lpec_qmf_pre01590_ref(double *re, double *im, int n, const double *cos)
{
    const double *wi = cos + LPEC_COS1536_WI_OFF;
    const double h = sqrt(3.0) * 0.5;
    int u = n / 3, stride = 0x1800 / (n * 4), t = 0, k;

    for (k = 0; k < u; k++, t += stride) {
        int b = u + k, c = 2 * u + k;
        double ra = re[k], ia = im[k];
        double s1 = re[c] + re[b];
        double d1 = (re[b] - re[c]) * h;
        double s2 = im[c] + im[b];
        double d2 = (im[b] - im[c]) * h;
        double t1 = ra - 0.5 * s1;
        double t2 = s2 * 0.5 + -ia;
        double e  = t2 - d1;
        double cc = d1 + t2;
        double aa = t1 - d2;
        double bb = d2 + t1;
        double c1 = cos[t], w1 = wi[t], c2 = cos[2 * t], w2 = wi[2 * t];

        re[k] = ra + s1;
        im[k] = ia + s2;
        re[b] = c1 * e + w1 * aa;
        im[b] = aa * c1 - e * w1;
        re[c] = c2 * cc + w2 * bb;
        im[c] = bb * c2 - cc * w2;
    }
}

/*
 * desc768: radix-3 preprocess, then three n/3 blocks of 01430 + 01340 +
 * 012c0 on cos2048.
 */
static void lpec_qmf_fft01590(double *lo, double *hi, int n, const double *cos1536,
                              const double *cos2048)
{
    int sub = n / 3;
    int i;
    double sc = 1.0 / (double) n;

    if (sub <= 4)
        return;

    /* radix-3 preprocess */
    lpec_qmf_pre01590_ref(lo, hi, n, cos1536);

    for (i = 0; i < n; i += sub) {
        lpec_qmf_fft01430(lo + i, hi + i, sub, cos2048, 2048);
        lpec_qmf_stage01340(lo + i, hi + i, sub, sc);
        lpec_qmf_permute012c0(lo + i, hi + i, sub);
    }

    /* radix-3 reorder (opt-out LPEC_NO_SHUFFLE01590=1). */
    lpec_qmf_shuffle01590(lo, n);
    lpec_qmf_shuffle01590(hi, n);

}

/* /: FFT + radix-4 + bit-reverse. */
static void lpec_qmf_stage1530(double *lo, double *hi, int n, const double *cos,
                               int tw_base, const double *cos2048)
{
    if (tw_base == 1536) {
        lpec_qmf_fft01590(lo, hi, n, cos, cos2048);
    } else {
        lpec_qmf_fft01430(lo, hi, n, cos, tw_base);
        lpec_qmf_stage01340(lo, hi, n, 1.0 / (double) n);
        lpec_qmf_permute012c0(lo, hi, n);
    }
}

static void lpec_ab80_core(double *inout, int seg_len, const LPECQmfDesc *desc,
                           const double *cos2048)
{
    const int scale  = desc->scale;
    const int flen   = desc->len;
    const double *cos = desc->cos;
    const double *sin = desc->sin;
    double lo[LPEC_QMF_FFT_MAX + 16];
    double hi[LPEC_QMF_FFT_MAX + 16];
    int nband = flen >> 2;
    int half  = flen >> 1;
    int mod_n = lpec_ab80_mod_n(flen);
    int fft_n = flen >> 1;             /* FFT + synthesis length */
    /* cos1536 table is 1536 entries; reference decoder uses 6144/n but different layout. */
    int tw_base = flen % 3 ? 2048 : 1536;
    int i9 = 0;
    const double *pd_in = inout;
    int cos_n = lpec_cos_period(flen);
    int hi_idx = scale * nband;
    int lo_idx = 0;
    int pass1_end = mod_n & ~1;
    int i15 = half;

    memset(lo, 0, fft_n * sizeof(double));
    memset(hi, 0, fft_n * sizeof(double));

    for (; i9 < pass1_end; i9 += 2) {
        lo[i9]     = pd_in[0] * lpec_cos_at(cos, hi_idx, cos_n);
        hi[i9]     = pd_in[0] * lpec_cos_at(cos, lo_idx, cos_n);
        lo[i9 + 1] = pd_in[2] * lpec_cos_at(cos, hi_idx - scale, cos_n);
        hi[i9 + 1] = pd_in[2] * lpec_cos_at(cos, lo_idx + scale, cos_n);
        pd_in  += 4;
        hi_idx -= scale + scale;
        lo_idx += scale + scale;
    }
    for (; i9 < mod_n; i9++) {
        lo[i9] = pd_in[0] * lpec_cos_at(cos, hi_idx, cos_n);
        hi[i9] = pd_in[0] * lpec_cos_at(cos, lo_idx, cos_n);
        pd_in  += 2;
        hi_idx -= scale;
        lo_idx += scale;
    }
    /* Second pass: cos base for lo, cos @ +scale*3*nband for hi. */
    {
        int pass2_end = flen >> 1;

        hi_idx = scale * (3 * nband);
        lo_idx = 0;
        /* pass2: end index flen/2, input at buf + flen/2 - 1. */
        pd_in = inout + (flen >> 1) - 1;

        for (; i9 < pass2_end && pd_in >= inout; i9 += 2) {
            lo[i9]     = pd_in[0]  * lpec_cos_at(cos, lo_idx, cos_n);
            hi[i9]     = pd_in[0]  * lpec_cos_at(cos, hi_idx, cos_n);
            lo[i9 + 1] = pd_in[-2] * lpec_cos_at(cos, lo_idx + scale, cos_n);
            hi[i9 + 1] = pd_in[-2] * lpec_cos_at(cos, hi_idx - scale, cos_n);
            pd_in  -= 4;
            lo_idx += scale + scale;
            hi_idx -= scale + scale;
        }
        for (; i9 < pass2_end && pd_in >= inout; i9++) {
            lo[i9] = pd_in[0] * lpec_cos_at(cos, lo_idx, cos_n);
            hi[i9] = pd_in[0] * lpec_cos_at(cos, hi_idx, cos_n);
            pd_in  -= 2;
            lo_idx += scale;
            hi_idx -= scale;
        }
    }

    lpec_qmf_stage1530(lo, hi, fft_n, cos, tw_base, cos2048);
    if (fft_n > LPEC_QMF_FFT_MAX)
        return;

    {
        /* synthesize fft_n samples backward into inout. */
        int out = flen - (seg_len >> 1) - 1;
        int k;

        for (k = 0; k < fft_n; ) {
            double wf = sin[k];
            double wr = sin[fft_n - 1 - k];

            if (k + 1 < fft_n) {
                inout[out]     = (hi[k]     * wf - lo[k]     * wr) * LPEC_QMF_SCALE;
                inout[out - 1] = (hi[k + 1] * sin[k + 1] - lo[k + 1] * sin[fft_n - 2 - k]) *
                                 LPEC_QMF_SCALE;
                out -= 2;
                k   += 2;
            } else {
                inout[out] = (hi[k] * wf - lo[k] * wr) * LPEC_QMF_SCALE;
                k++;
            }
        }
    }
    {
        /* mirror seg_len/2 samples at buf+flen-seg/2. */
        int n_mirror = seg_len >> 1;
        int n_tail   = flen - seg_len;
        double *dst, *src;
        int i, j;

        if (n_mirror > 0) {
            dst = inout + flen - n_mirror;
            src = dst - 1;
            for (i = 0; i + 1 < (n_mirror & ~1); i += 2) {
                dst[0] = src[0];
                dst[1] = src[-1];
                src -= 2;
                dst += 2;
            }
            for (; i < n_mirror; i++)
                *dst++ = *src--;
        }

        /* negated copy into buf[0:(flen-seg)/2) from buf+flen-seg-1. */
        if (n_tail > 0) {
            int tail_n = (flen - seg_len + (unsigned) (flen - seg_len >> 31)) >> 1;

            dst = inout;
            src = inout + flen - seg_len - 1;
            for (j = 0; j + 1 < (tail_n & ~1); j += 2) {
                dst[0] = -src[0];
                dst[1] = -src[-1];
                src -= 2;
                dst += 2;
            }
            for (; j < tail_n; j++)
                *dst++ = -(*src--);
        }
    }

    (void) i15;
}

void ff_lpec_qmf_init(LPECQmfState *st)
{
    /* Require zero-init; stale inited with len==0 must rebuild tables. */
    if (st->inited && st->desc512.len)
        return;
    st->inited = 0;

    lpec_qmf_fill_cos2048(st->cos2048, 2048);
    lpec_qmf_fill_cos1536(st->cos1536, 1536);

    lpec_qmf_build_desc(&st->desc512, st->sin512, 512, st->cos2048);
    lpec_qmf_build_desc(&st->desc768, st->sin768, 768, st->cos1536);
    lpec_qmf_build_desc(&st->desc1024, st->sin1024, 1024, st->cos2048);
    lpec_qmf_build_desc(&st->desc1536, st->sin1536, 1536, st->cos1536);
    lpec_qmf_build_desc(&st->desc2048, st->sin2048, 2048, st->cos2048);

    st->lp_dual_rate = 0;
    st->active      = NULL;
    st->inited      = 1;
}

void ff_lpec_qmf_configure(LPECQmfState *st, int out_rate, int in_rate)
{
    ff_lpec_qmf_init(st);
    st->lp_dual_rate = out_rate == 8000 && in_rate > 0 && in_rate < 6001;

    if (out_rate >= 11026)
        st->active = &st->desc1024;
    else
        st->active = &st->desc512;
}

void ff_lpec_qmf_process_slot(LPECQmfState *st, double *buf, int len, int desc_idx)
{
    const LPECQmfDesc *desc;

    if (!st->inited)
        ff_lpec_qmf_init(st);

    if (desc_idx == 1)
        desc = &st->desc768;
    else if (desc_idx == 2)
        desc = &st->desc1024;
    else
        desc = &st->desc512;

    lpec_ab80_core(buf, len, desc, st->cos2048);
}

void ff_lpec_qmf_process_seg(LPECQmfState *st, double *buf, int seg_len,
                          const LPECQmfDesc *desc)
{
    if (!st->inited)
        ff_lpec_qmf_init(st);
    if (!desc || seg_len <= 0)
        return;
    lpec_ab80_core(buf, seg_len, desc, st->cos2048);
}

void ff_lpec_qmf_process_scratch(LPECQmfState *st, double *buf, int len,
                              const LPECQmfDesc *desc)
{
    double tmp[LPEC_QMF_MAX];
    int tail, half;

    if (!st->inited)
        ff_lpec_qmf_init(st);
    if (!desc || len <= 0 || desc->len <= len)
        return;

    tail = desc->len - len;
    half = len >> 1;

    memcpy(tmp, buf, len * sizeof(double));
    if (half > 0 && half < len)
        memcpy(tmp + half, buf + half, half * sizeof(double));
    if (tail > 0)
        memcpy(tmp + len, buf + len, tail * sizeof(double));
    lpec_ab80_core(tmp, len, desc, st->cos2048);
    memcpy(buf, tmp, len * sizeof(double));
    if (tail > 0)
        memcpy(buf + len, tmp + len, tail * sizeof(double));
}
