/*
 * LPEC excitation (chain).
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
#ifndef AVCODEC_LPEC_EXCIT_H
#define AVCODEC_LPEC_EXCIT_H

#include "get_bits.h"

#define LPEC_EXCIT_HIST      2048
#define LPEC_ROUTE_TBL_MAX   1024 /* num_bands * max pitch_scale_route @ 16 kHz */
#define LPEC_ROUTE_PITCH_MAX 2048 /* max lpec_route_pitch_len (route idx 4) */

typedef struct LPECRouteEntry {
    double val;
    int    dst; /* int at local_512c[i*2]+8 */
    int    idx;
} LPECRouteEntry;

typedef struct LPECExcitState {
    int    num_bands;
    int    frame_size;
    int    subfr_size;
    int    excit_pos;
    int    f4e0_noise_pos;
    int    cb_idx_pos;              /* ctx+0x70 scratch for 060e0 */
    int    bit_overhead;            /* ctx+0x14 = 19 */
    int    pitch_ref;               /* legacy; use pitch_ref_route */
    int    pitch_scale;             /* legacy; use pitch_scale_route */
    int    pitch_mult;              /* legacy; use pitch_mult_route */
    int    pitch_ref_route[4];      /* ctx+0xec + route*4 */
    int    pitch_scale_route[4];    /* ctx+0x40 + route*4 (05d50 band step / c680) */
    int    pitch_mult_route[4];     /* ctx+0xfc + route*4 */
    int    route_excit_start[4];    /* ctx+0x40 + route*4 (05d50 head zero / gain start) */
    int    route_tail_start[4];     /* ctx+0x50 + route*4 (05d50 tail zero start) */
    int    band_width;              /* ceil(subfr / num_bands); legacy alias */
    int    half_pitch_len;          /* subfr_size >> 1 */
    int    route_pitch_base;        /* local_c -> pitch_len */
    int    pitch_route_len[4];      /* *(ctx+0x74[slot]+4) QMF desc.len */
    int    is_xaudio_lp;            /* quality 0x19 selects the reference decoder tables */
    int    codec_rate_in;           /* InitCodec rate (6000 -> 03920 rtable_a) */
    int    xa_route_pitch;          /* ctx[0x13c + half_idx1*4] for 03920 rtable row */
    int    mode0_half2_gain;        /* mode 0: 2nd header read after half 1 */
    int    mode0_half2_cb1;
    int    mode0_half2_cb2;
    int    mode0_half2_valid;
    int    dump_frame;              /* lpecdec frame idx for LPEC_DUMP_* gating */
    double excit_hist[LPEC_EXCIT_HIST];
    uint32_t cb_idx_buf[256];
    LPECRouteEntry route_tbl[LPEC_ROUTE_TBL_MAX];
    int32_t  route_pitch[LPEC_ROUTE_PITCH_MAX];
    uint8_t  cb_idx_scratch[768];
} LPECExcitState;

int lpec_excit_route_idx(int mode, int half);

void lpec_excit_init(LPECExcitState *st, int sample_rate, int codec_rate_in,
                     int num_bands, int frame_size, int subfr_size,
                     int is_xaudio_lp);

void lpec_xa_lsf_indices_to_i16(int order, int num_cb,
                                const int *idx, int16_t *lsf_out);

int lpec_generate_excitation(LPECExcitState *st, GetBitContext *gb,
                             double *excitation, int subfr_len,
                             int half_idx1, int pitch_lag, int pitch_lag_off,
                             int pitch_win, int voiced, const double *pitch_tbl,
                             int bit_budget, int bits_limit, int mode,
                             int pitch_route_arg, const int *lsf_idx,
                             const int16_t *lsf_i16_override, int frame_size,
                             int lpc_order, int num_lsf_cb);

#endif /* AVCODEC_LPEC_EXCIT_H */
