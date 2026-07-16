/*
 * Sony TRC bitstream unpacker
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

#include <string.h>

#include "trc_unpack.h"
#include "trcdata.h"
#include "libavutil/attributes.h"
#include "libavutil/error.h"
#include "get_bits.h"

int trc_decode_rate(uint8_t byte0)
{
    int top2 = byte0 >> 6;

    if (top2 != 3)
        return top2;
    return (byte0 & 0x20) ? 4 : 3;
}

int trc_packet_bytes(int rate)
{
    if (rate < 0 || rate > 5)
        return 0;
    return trc_small_lut[rate];
}

static av_always_inline int trc_rate_prefix_bits(int rate)
{
    return rate > 2 ? 3 : 2;
}

static void trc_decode_lags(GetBitContext *gb, TRCFrameParams *fp, int rate)
{
    int raw = get_bits(gb, 7);

    fp->base_lag  = raw + 0x12;
    fp->sf[0].lag = fp->base_lag;

    if (rate == 0) {
        int v = get_bits(gb, 12);
        int rem = v % 209;

        /* Track-1 pos_code lives at sf->signs2: the coarse index goes in
         * the upper byte; fine bits are added below. */
        fp->sf[0].signs2 = (v / 209) << 8;
        fp->sf[1].signs2 = (rem / 11) << 8;
        /* The reference decoder applies no lag clamp here. */
        fp->sf[1].lag  = fp->base_lag - rem % 11 + 5;
        return;
    }

    {
        int s8, temp;

        s8 = raw + 0x11;
        if (s8 < 0x12)
            s8 = 0x12;
        if (raw + 0x14 > 0x8e)
            s8 = 0x8c;
        fp->sf[1].lag = get_bits(gb, 2) + s8;

        temp = fp->base_lag - get_bits(gb, 4);
        fp->sf[2].lag = temp + 7;

        s8 = temp + 6;
        if (s8 < 0x12)
            s8 = 0x12;
        if (temp + 9 > 0x8e)
            s8 = 0x8c;
        fp->sf[3].lag = get_bits(gb, 2) + s8;
    }
}

int trc_unpack_frame(const uint8_t *buf, int buf_size, TRCFrameParams *fp)
{
    GetBitContext gb;
    int rate, pbytes, ret;
    int i;

    if (!buf || !fp || buf_size < 1)
        return AVERROR_INVALIDDATA;

    memset(fp, 0, sizeof(*fp));

    rate = trc_decode_rate(buf[0]);
    pbytes = trc_packet_bytes(rate);
    if (pbytes <= 0 || buf_size < pbytes)
        return AVERROR_INVALIDDATA;

    fp->rate         = rate;
    fp->packet_bytes = pbytes;
    fp->nsub         = rate == 0 ? 2 : 4;
    fp->sublen       = rate == 0 ? 80 : 40;
    fp->type         = rate == 4 ? 0 : rate == 3 ? 2 : 1;

    ret = init_get_bits8(&gb, buf, pbytes);
    if (ret < 0)
        return ret;

    skip_bits(&gb, trc_rate_prefix_bits(rate));

    if (rate == 4)
        return pbytes;

    fp->lsf_idx[0] = get_bits(&gb, 7);
    fp->lsf_idx[1] = get_bits(&gb, 7);
    fp->lsf_idx[2] = get_bits(&gb, 7);

    if (rate == 3) {
        fp->aux_gain = get_bits(&gb, 5);
        return pbytes;
    }

    trc_decode_lags(&gb, fp, rate);

    for (i = 0; i < fp->nsub; i++) {
        fp->sf[i].pitch_idx = get_bits(&gb, 5);
        fp->sf[i].gain_idx  = get_bits(&gb, 4);
    }

    if (rate == 0) {
        /* Grouped reads (rate-0 layout): all field2, then all
         * signs1, then all fine pos_code bits (added into the coarse value). */
        for (i = 0; i < fp->nsub; i++)
            fp->sf[i].field2 = get_bits(&gb, 2);
        for (i = 0; i < fp->nsub; i++)
            /* signs1: 4 bits, left-aligned so bit 15 = first sign */
            fp->sf[i].signs1 = get_bits(&gb, 4) << 12;
        for (i = 0; i < fp->nsub; i++)
            /* track-1 pos_code: fine 8 bits added into sf->signs2 */
            fp->sf[i].signs2 += get_bits(&gb, 8);
    } else if (rate == 1) {
        /* Grouped reads (rate-1 layout): all field2 (1 bit each,
         * read as raw bit-shorts), then all signs1, then all pos_code. */
        for (i = 0; i < fp->nsub; i++)
            fp->sf[i].field2 = get_bits(&gb, 1);
        for (i = 0; i < fp->nsub; i++)
            /* signs1: 4 bits, left-aligned */
            fp->sf[i].signs1 = get_bits(&gb, 4) << 12;
        for (i = 0; i < fp->nsub; i++)
            /* track-1 pos_code at sf->signs2 */
            fp->sf[i].signs2 = get_bits(&gb, 10);
        {
            int v = get_bits(&gb, 5);

            fp->sf[0].signs2 += (v / 5) * 0x400;
            fp->sf[1].signs2 += (v % 5) * 0x400;

            v = get_bits(&gb, 5);
            fp->sf[2].signs2 += (v / 5) * 0x400;
            fp->sf[3].signs2 += (v % 5) * 0x400;
        }
    } else if (rate == 2) {
        int v, b, d;

        /* Grouped reads (rate-2 layout): all (signs1,pos1) pairs
         * first, then all (signs2,pos2) pairs, then two 11-bit composites. */
        for (i = 0; i < fp->nsub; i++) {
            fp->sf[i].field2 = 0;
            /* signs1: 4 bits, left-aligned (codebook reads bit 15 then <<1) */
            fp->sf[i].signs1 = get_bits(&gb, 4) << 12;
            /* track2 sign bits: 3 bits, left-aligned at sf->pos1 */
            fp->sf[i].pos1   = get_bits(&gb, 3) << 13;
        }
        for (i = 0; i < fp->nsub; i++) {
            /* track-1 pos_code: 10 bits at sf->signs2 */
            fp->sf[i].signs2 = get_bits(&gb, 10);
            /* track-2 pos_code: 7 bits at sf->pos2 */
            fp->sf[i].pos2   = get_bits(&gb, 7);
        }

        /* Composite 1: distributes to sf0/sf1 pos2 (track2) and signs2 (track1). */
        v = get_bits(&gb, 11);
        b = v % 0xe1;          /* 225 */
        d = b % 0x2d;          /* 45 */
        fp->sf[0].pos2   += (v / 0xe1) * 0x80;
        fp->sf[0].signs2 += (b / 0x2d) * 0x400;
        fp->sf[1].pos2   += (d / 5)    * 0x80;
        fp->sf[1].signs2 += (d % 5)    * 0x400;

        /* Composite 2: distributes to sf2/sf3 pos2 (track2) and signs2 (track1). */
        v = get_bits(&gb, 11);
        b = v % 0xe1;
        d = b % 0x2d;
        fp->sf[2].pos2   += (v / 0xe1) * 0x80;
        fp->sf[2].signs2 += (b / 0x2d) * 0x400;
        fp->sf[3].pos2   += (d / 5)    * 0x80;
        fp->sf[3].signs2 += (d % 5)    * 0x400;
    }

    /* Allow unused trailing bits; fail only on overread. */
    if (get_bits_left(&gb) < 0)
        return AVERROR_INVALIDDATA;

    return pbytes;
}
