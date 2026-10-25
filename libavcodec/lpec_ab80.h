/*
 * LPEC QMF excitation post-process.
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
#ifndef AVCODEC_LPEC_AB80_H
#define AVCODEC_LPEC_AB80_H

#define LPEC_QMF_MAX 2048
#define LPEC_QMF_FFT_MAX ((LPEC_QMF_MAX) >> 1)

typedef struct LPECQmfDesc {
    int           scale;
    int           len;
    const double *sin;
    const double *cos;
} LPECQmfDesc;

typedef struct LPECQmfState {
    LPECQmfDesc desc512;
    LPECQmfDesc desc768;
    LPECQmfDesc desc1024;
    LPECQmfDesc desc1536;
    LPECQmfDesc desc2048;
    double sin512[512 / 2 + 2];
    double sin768[768 / 2 + 2];
    double sin1024[1024 / 2 + 2];
    double sin1536[1536 / 2 + 2];
    double sin2048[2048 / 2 + 2];
    double cos2048[2048];
    double cos1536[1536];
    const LPECQmfDesc *active;
    int    inited;
    int    lp_dual_rate;
} LPECQmfState;

void ff_lpec_qmf_init(LPECQmfState *st);
void ff_lpec_qmf_configure(LPECQmfState *st, int out_rate, int in_rate);
void ff_lpec_qmf_process_slot(LPECQmfState *st, double *buf, int len, int desc_idx);
/* desc512 with seg < flen (mode-2 h1): scratch path, same as process_slot tail logic. */
void ff_lpec_qmf_process_scratch(LPECQmfState *st, double *buf, int len,
                              const LPECQmfDesc *desc);
/* In-place (mode-2 h2: buf must span desc->len). */
void ff_lpec_qmf_process_seg(LPECQmfState *st, double *buf, int seg_len,
                          const LPECQmfDesc *desc);


#endif /* AVCODEC_LPEC_AB80_H */
