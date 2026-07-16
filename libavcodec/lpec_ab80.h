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
    int           scale; /* param_1[0] */
    int           len;   /* param_1[1] */
    const double *sin;   /* param_1[2] */
    const double *cos;   /* param_1[3] */
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
    const LPECQmfDesc *active; /* ctx+0x74 = & + local_c*0x18 */
    int    inited;
    int    lp_dual_rate; /* @ 8000 Hz, param_2 == 6000 */
} LPECQmfState;

void lpec_qmf_init(LPECQmfState *st);
void lpec_qmf_configure(LPECQmfState *st, int out_rate, int in_rate);
void lpec_qmf_process(LPECQmfState *st, double *buf, int len);
/* desc_idx: 0=ctx+0x74 (512), 1=ctx+0x7c (768), 2=ctx+0x80 (1024) per */
void lpec_qmf_process_slot(LPECQmfState *st, double *buf, int len, int desc_idx);
/* Exact desc768 QMF (lpec_qmf_a080.c): correct inverse DFT instead of the buggy
 * radix-3 x87 emulation. Used for the 8 kHz the reference decoder mode-1 slot only.*/
void lpec_qmf_a080_process(LPECQmfState *st, double *buf, int seg_len);
void lpec_qmf_a080_process_desc(LPECQmfState *st, double *buf, int seg_len,
                                const LPECQmfDesc *desc);
/* desc512 with seg < flen (mode-2 h1): scratch path, same as process_slot tail logic. */
void lpec_qmf_process_scratch(LPECQmfState *st, double *buf, int len,
                              const LPECQmfDesc *desc);
/* In-place (mode-2 h2: buf must span desc->len). */
void lpec_qmf_process_seg(LPECQmfState *st, double *buf, int seg_len,
                          const LPECQmfDesc *desc);

/* Set by lpecdec when LPEC_DUMP_AB80_FRAME selects a single frame. */
extern int lpec_ab80_dump_wanted;
extern int lpec_ab80_dump_frame; /* 1 if unset; tags /tmp/lpec_f{N}_h{H}_*.bin */
extern int lpec_ab80_dump_half;
extern int lpec_qmf_frame_idx; /* lpecdec frame during QMF; -1 if unset */
extern int lpec_qmf_chain_late; /* 0 off; 1 mode-1 after m2; 2 mode-2 h2 before m1 */
int lpec_pre01590_env_frame_listed(const char *env_name, int frame);
int lpec_pre01590_chain_skip_frame(int frame);

/* Isolated (for tools/fft01590_test.c). */
void lpec_qmf_fft01590_pub(double *lo, double *hi, int n, LPECQmfState *st);
void lpec_qmf_pre01590_pub(double *lo, double *hi, int n, LPECQmfState *st);
void lpec_qmf_fft01430_pub(double *a, double *b, int n, LPECQmfState *st);
void lpec_qmf_stage01340_pub(double *a, double *b, int n, double invn);
void lpec_qmf_permute012c0_pub(double *a, double *b, int n);

#if defined(__x86_64__)
void lpec_pre01590_x87(double *lo, double *hi, int n,
                       const double *cos_wr, const double *cos_wi);
void lpec_pre01590_x87_iter(int k, int u, int ecx_off, int edi_byte,
                            double *lo, double *hi,
                            const double *cos_wr, const double *cos_wi,
                            double *defer);
void lpec_pre01590_x87_dump_postbfly(int k, const unsigned char *fnsave_img);
void lpec_pre01590_x87_dump_enable(int on);
void lpec_pre01590_x87_dump_flush(void);
#endif

#endif /* AVCODEC_LPEC_AB80_H */
