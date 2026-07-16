/*
 * Sony MSV ADPCM decoder (the reference decoder)
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
#ifndef AVCODEC_MSV_ADPCM_SONY_H
#define AVCODEC_MSV_ADPCM_SONY_H

#include <stdint.h>

#define MSV_ADPCM_FRAME_SIZE   48
#define MSV_ADPCM_SAMPLES_MAX  128
#define MSV_ADPCM_MODE_3BIT    3   /* ICD-MS1 / msvapcmw.spi */

typedef struct MSVADPCMBitstream {
    const uint8_t *base;
    int limit;      /* exclusive byte offset */
    int byte_off;
    int sym_idx;
    int mode;
    int adv;
    int sym_max;
} MSVADPCMBitstream;

/* Persistent decoder state (the reference decoder xx). */
typedef struct MSVADPCMState {
    uint32_t sym_acc;     /* symbol / synthesis accumulator */
    uint32_t adapt_ctrl;  /* adaptation control */
    uint32_t bw_scale;    /* bandwidth scale */
    uint32_t filt_lo;     /* filter state (low) */
    uint32_t filt_hi;     /* filter state (high) */
    uint32_t quant_base;  /* quantizer predictor base */
    uint32_t scl_c0;      /* scale / delay pair 0 */
    uint32_t scl_c4;
    uint32_t del_c8;
    uint32_t del_cc;
    uint32_t del_d0;
    uint32_t del_d4;
    uint32_t del_d8;
    uint32_t del_dc;
    uint32_t del_e0;
    uint32_t del_e4;
    uint32_t del_e8;
    uint32_t del_ec;
    uint32_t tap0;
    uint32_t tap1;
    uint32_t tap2;
    uint32_t tap3;
    uint32_t tap4;
    uint32_t tap5;
} MSVADPCMState;

void msv_adpcm_bs_init(MSVADPCMBitstream *bs, const uint8_t *buf, int limit, int mode);
int  msv_adpcm_bs_next(MSVADPCMBitstream *bs, uint32_t *sym);

void msv_adpcm_get_state(MSVADPCMState *st);
void msv_adpcm_set_state(const MSVADPCMState *st);

int msv_adpcm_samples_per_frame(int mode);

int msv_adpcm_decode_block(const uint8_t *buf, int16_t *out, int mode,
                           int *need_init, MSVADPCMBitstream *bs);

#endif /* AVCODEC_MSV_ADPCM_SONY_H */
