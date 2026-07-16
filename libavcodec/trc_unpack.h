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

#ifndef AVCODEC_TRC_UNPACK_H
#define AVCODEC_TRC_UNPACK_H

#include <stdint.h>

#define TRC_MAX_SUBFRAMES 4
#define TRC_FRAME_SAMPLES 160

/* Per-subframe parameter block (8 shorts, reference decoder layout). */
typedef struct TRCSubParams {
    int16_t lag;          /* [0] pitch lag (only sf0 uses base_lag at frame level) */
    int16_t pitch_idx;    /* [1] 5-bit pitch-filter VQ index */
    int16_t field2;       /* [2] RATE0: 2-bit; RATE1: literal; RATE2: 0 */
    int16_t gain_idx;     /* [3] 4-bit fixed-codebook gain delta */
    int16_t signs1;       /* [4] track-1 sign bits (left-aligned) */
    int16_t pos1;         /* [5] track-2 signs (RATE2); NOT track-1 pos, see [6] */
    int16_t signs2;       /* [6] track-1 position code */
    int16_t pos2;         /* [7] track-2 position code (RATE2) */
} TRCSubParams;

typedef struct TRCFrameParams {
    int      rate;           /* 0..4, from the packet's rate prefix */
    int      type;           /* 0=silence 1=CELP 2=low-rate noise */
    int      packet_bytes;   /* trc_small_lut[rate] */
    int      nsub;           /* 2 (RATE0) or 4 */
    int      sublen;         /* 80 (RATE0) or 40 */

    int16_t  aux_gain;       /* RATE3 only */
    int16_t  lsf_idx[3];     /* 7-bit split-VQ indices */
    int16_t  base_lag;       /* integer pitch lag base (18..145) */

    TRCSubParams sf[TRC_MAX_SUBFRAMES];
} TRCFrameParams;

/* Decode RATE from the first packet byte. */
int trc_decode_rate(uint8_t byte0);

/* Packet byte count for a RATE code. */
int trc_packet_bytes(int rate);

/* Parse one TRC frame packet. Returns consumed bytes or AVERROR. */
int trc_unpack_frame(const uint8_t *buf, int buf_size, TRCFrameParams *fp);

#endif /* AVCODEC_TRC_UNPACK_H */
