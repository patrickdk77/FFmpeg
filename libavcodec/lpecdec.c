/*
 * LPEC audio decoder
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

/**
 * @file
 * Sony LPEC audio decoder
 *
 * Reverse-engineered from the Sony LPEC decoder.
 *
 * Bitstream is MSB-first (big-endian bit order).
 *
 * Frame structure (all modes output LPEC_FRAME_OUT = 512 samples):
 * 2 bits mode
 * Mode 0: two half-frames, each with own LSF + pitch + excitation
 * Mode 1: one frame, single LSF + pitch + excitation (only half decoded)
 * Mode 2: one shared LSF + two halves of pitch + excitation
 * Mode 3: one frame, all silent / comfort noise
 *
 * LSF: 3 VQ codebooks, 6 bits each (with esi+0xac=6, esi+0xb0=3)
 *
 * Mode 0 pitch: (1) and (2) first,
 * then per half-frame: 1 bit + optional 7-bit pitch CB,
 * then excitation (7+6+6).
 * Pitch (mode 1/2/3 via ):
 * 1 bit voiced flag; if voiced: 6-bit lag offset from base lag
 * Excitation: 7-bit gain idx + 6-bit cb1 idx + 6-bit cb2 idx
 *
 * LPC order: 10 for 8 kHz, 16 for >=16 kHz (from esi+0xa4).
 * Synthesis: interpolated LPC: param_2+1 passes (8->9 @ 16 kHz),
 * first/last pass length frame/(2*param_2), middle passes frame/param_2.
*/

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
/* NOTE: NO BITSTREAM_READER_LE -- reference decoder uses MSB-first (standard get_bits) */
#include "get_bits.h"
#include "libavutil/channel_layout.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "libavutil/macros.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lpec_tables.h"
#include "lpec_excit.h"
#include "lpec_excit_route.h"
#include "lpec_ab80.h"
/* Mode-1 half2 LPC helpers (defined at end of file). */
void lpec_mode1_half2_synthesis(const double *prev_lsf, const double *curr_lsf,
                                const double *excit, double *synth_buf,
                                double *output, const double *frame_out,
                                int order, int frame_size, int subfr_size,
                                int num_seg);
void lpec_interpolated_lpc_inplace(const double *prev_lsf, const double *curr_lsf,
                                   const double *excit, double *synth_buf,
                                   double *output, int order, int frame_size,
                                   int num_seg);
void lpec_mode1_half2_isolated(const double *prev_lsf, const double *curr_lsf,
                               const double *excit, double *synth_buf,
                               double *output, int order, int frame_size,
                               int subfr_size, int num_seg);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* - Constants ---- */
#define LPEC_LPC_ORDER_MAX  16      /* 10 @ 8kHz, 16 @ 16kHz (the reference decoder +0xa4) */
#define LPEC_LSF_MAX        (LPEC_LPC_ORDER_MAX + 1) /* lsf[0]=0, lsf[1..order] */
#define LPEC_NUM_LSF_CB_MAX 4       /* 3 @ 8kHz, 4 @ 16kHz (the reference decoder +0xb0) */
#define LPEC_LSF_BITS       6       /* esi+0xac = 6 */
/*
 * Frame size from the reference decoder (ctx+8) / InitDecoder:
 * 8 kHz: 512 samples per; 16 kHz: 1024 (two 512-sample halves).
 * Bit budgets at ctx+0x1c..0x28 match ctx+8, not fixed 128/432.
*/
#define LPEC_FRAME_8K     512
#define LPEC_FRAME_16K    1024
#define LPEC_SUBFR_8K     256
#define LPEC_SUBFR_16K    512
#define LPEC_NUM_BANDS_MAX  10      /* 8 @ 8kHz, 10 @ 16kHz (the reference decoder +0x2c) */
#define LPEC_NUM_SEGMENTS 4         /* LPC interpolation segments per frame */
#define LPEC_MAX_FRAME    1024
#define LPEC_EXCIT_SRC_MAX (LPEC_MAX_FRAME * 2) /* ctx+0x37c: ctx+4 @ 16 kHz */
#define LPEC_MAX_FRAMES_PKT 32
#define LPEC_SCRATCH_SAMPLES (LPEC_MAX_FRAME * LPEC_MAX_FRAMES_PKT)
#define LPEC_WORK_SLACK   256         /* ctx+0x388 + 0x800 bytes */
#define LPEC_F640_SHAPE_M3 (2 * LPEC_MAX_FRAME) /* 2*frame_size */
#define LPEC_QUIET_EXCIT_RMS 10.0     /* mode-3 half2 below this uses tail hist */

/*
 * Pitch lag (mode 1/2/3: ): lag = base + 6-bit-offset.
 * The 6-bit offset indexes into a table via lag = base_lag + offset * step.
 * base_lag is encoded in the context at offset 0xd0.
 * Mode 0: 7-bit index into lpec_8k_pitch_cb (128 entries).
*/
#define LPEC_PITCH_LAG_BASE  20     /* smallest allowed pitch lag (mode 1/2/3) */
#define LPEC_PITCH_LAG_MAX   83     /* 20 + 63 */

/* Excitation history in LPECExcitState (lpec_excit.h). */

/* Carry buffer: holds unconsumed bits from previous packet.
 * In the worst case the carry is almost a full 80-byte packet (640 bits).
 * We need: max_carry (80 bytes) + new packet (80 bytes) + alignment slack (8).*/
#define LPEC_CARRY_BUF_BYTES  4096 /* must hold partial frame + full MSV payload */

typedef struct LPECContext {
    AVCodecContext *avctx;
    int            codec_rate_in;     /* the reference decoder d430; 6000 for LP 8 kHz */
    int            frame_size;        /* 104 or 208 depending on sample_rate */
    int            subfr_size;        /* frame_size / 2 */
    int            lpc_order;         /* 10 or 16 */
    int            num_lsf_cb;        /* 3 or 4 */
    int            num_bands;         /* 8 or 10 */
    double prev_lsf[LPEC_LSF_MAX];
    double lsf1[LPEC_LSF_MAX];  /* LSF-A from last decoded frame */
    double lsf2[LPEC_LSF_MAX];  /* LSF-B from last decoded frame */
    double synth_buf[LPEC_LPC_ORDER_MAX + LPEC_MAX_FRAME]; /* ctx+900/380 */
    LPECExcitState excit;
    int    half_lsf_cb0[2];             /* [0]=ctx+0x144, [1]=ctx+0x148 */
    int    half_lsf_idx[3][LPEC_NUM_LSF_CB_MAX];
    int16_t half_lsf_i16[3][LPEC_LSF_MAX]; /* @ ctx+0x2e8+param*0x22 */
    int    noise_pos;                   /* legacy noise walk (unused by excit chain) */
    int    prev_pitch_lag;
    int    pitch_voiced_bits;         /* ctx+0xa0 (7 @ 8k, 8 @ 16k) */
    int    frame_out_size;            /* ctx+0x10: frame_size / 4 */
    int    pitch_win_prev;            /* ctx+300: rolled from prior half-2 */
    int    pitch_win_half1;           /* ctx+0x130: (1) */
    int    pitch_win_half2;           /* ctx+0x134: (2) */
    int    pitch_lag_off_half1;       /* 6-bit lag table index when voiced */
    int    pitch_lag_off_half2;
    int    pitch_lag_off_prev;        /* rolled lag index (debug) */
    const double *pitch_tbl_prev;     /* ctx+0x368: f430 seg0, rolled from half2 */
    const double *pitch_tbl_half1;    /* ctx+0x36c: f430 seg1 */
    const double *pitch_tbl_half2;    /* ctx+0x370: f430 seg2 */
    int    frame_count;
    int    lpec_frame_idx;              /* decoded LPEC frames (not AVPackets) */
    int    pending_frame_start;       /* bit pos at start of current frame decode */
    int    pending_frame_mode;        /* mode byte for padding on resync */
    int    pad_pending_start;         /* decoded frame awaiting mode padding */
    int    pad_pending_mode;
    /* MSV packet tail + chunk stream (the reference: decode then drop chunk). */
    uint8_t carry_buf[LPEC_CARRY_BUF_BYTES + 80 + 4];
    int    chunk_carry_len;
    uint8_t stream_buf[LPEC_CARRY_BUF_BYTES + 80 + 4];
    int    stream_len;
    int    stream_bit_off;            /* 0..7 bits already consumed in stream_buf[0] */
    uint16_t chunk_sz[4];             /* lead 0x00/0x40/0x80/0xC0 */
    int    frame_bit_target[4];         /* padding (ctx+0x1c..0x28) */
    int    synth_half_flag;             /* ctx+0x128: mode-0 LSF flag, persists */
    double excit_src[LPEC_EXCIT_SRC_MAX]; /* ctx+0x37c: excitation work (2048 @ 16 kHz) */
    double frame_excit[LPEC_MAX_FRAME]; /* ctx+0x38c: f640 output / saved excitation */
    double excit_work[LPEC_WORK_SLACK + LPEC_MAX_FRAME]; /* ctx+0x388 + slack */
    double excit_ola_hist[LPEC_MAX_FRAME]; /* ctx+0x390 */
    double frame_out[LPEC_MAX_FRAME];   /* retain untouched half for modes 1-3 */
    double scratch[LPEC_SCRATCH_SAMPLES];
    double pitch_shape[LPEC_MAX_FRAME];
    double pitch_weight[LPEC_MAX_FRAME];
    double f640_shape_m3[LPEC_F640_SHAPE_M3]; /* ctx+0x8c: */
    double pitch_cb_aux[LPEC_SUBFR_16K]; /* the reference decoder ctx+0x38c: dc80/03e30 OLA hist */
    double mode0_shaped_tail[LPEC_SUBFR_16K]; /* cross-frame carry */
    LPECQmfState qmf;
    int    prev_half2_pitch_cb;         /* 0x138: prior frame half-2 CB flag */
    int    prev_half2_pitch_cb_idx;     /* prior frame half-2 CB index */
    int    pitch_cb_flag_h1;            /* ctx+0x13c: mode-0 half-1 pitch CB bit */
    int    pitch_cb_flag_h2;            /* ctx+0x140: mode-0 half-2 pitch CB bit */
    int    pitch_cb_half2_idx;          /* ctx+0x364: current frame half-2 CB idx */
    int    pitch_cb_tbl_prev_idx;       /* ctx+0x35c: rolled from prior half-2 tbl */
    int    prev_lsf_cb_half2;           /* rolled LSF CB for half 1 */
    int    mode0_excit_rem;             /* rem after pitch, before pitch CB */
    int    prev_decode_mode;            /* prior frame mode for cross-mode hist */
    int    next_frame_mode_peek;        /* show_bits(gb,2) at next frame; -1 unknown */
    int    eof_flush_done;              /* one-shot trailing frame at EOF */
    int    eof_draining;                /* avpkt flush: relax padding lookahead */
    int    is_xaudio_lp;                /* codec_tag == 0x19 (the reference decoder) */
    int    qmf_seg_m2_h1;               /* ctx+0xc; mode-2 h1 ab80 len */
} LPECContext;

static const double lpec_pitch_cb_stub[8] = {
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
};

static const double *lpec_pitch_cb_ptr(LPECContext *s, int idx)
{
    if (idx < 0)
        return lpec_pitch_cb_stub;
    if (s->num_bands == 10)
        return lpec_16k_pitch_cb + (idx & 127) * 8;
    if (s->is_xaudio_lp)
        return lpec_xa_8k_pitch_cb + (idx & 127) * 8;
    return lpec_8k_pitch_cb + (idx & 127) * 8;
}

/* coef table @ ctx+0xc4: 16 kHz ->, 8 kHz ->. */
static const double lpec_16k_f430_coef[192] = {
    0.028729972606089303, 0.15328675419214269, 0.017482854866846635,
    0.10878427834873362, 0.36091455724888222, 0.15450039144327535,
    -0.037912732848629414, 0.13402055378102512, -0.05640739089096812,
    -0.095810860415539115, 0.22953521704456253, 0.13770390032460919,
    0.038338820504945768, 0.18852875052358917, 0.124109632322392,
    0.059198818145002982, 0.071288007191259481, -0.00071052372631450898,
    0.014247479769313636, 0.22323341873378946, -0.061118843132450473,
    -0.067736931284241367, 0.18913880237276362, 0.033022559269365197,
    -0.054642149210748572, 0.51341028423105839, -0.013791261255198233,
    0.014869778311855293, 0.08598482484787677, -0.16670320324117285,
    -0.042988132588338203, 0.102197686075399, 0.074773063059378725,
    0.1535806418890629, 0.27675837172135109, -0.019378533816776764,
    -0.1007054843263069, 0.21256647310614615, -0.071223448850374016,
    -0.018281319888986593, 0.322252719547516, -0.11854152510991044,
    -0.046622655289896731, 0.55533480407402713, 0.4124866162781517,
    0.48178200976672214, 0.61082553988031429, -0.13163494158785058,
    -0.047011526423114909, 0.47550923109643273, 0.17137478892132851,
    0.30232387024689783, 0.33785094587443498, 0.098251600547813442,
    0.26278961520453414, 0.45710782638910086, 0.18174499802345231,
    -0.042785671156202548, 0.20075747124376522, -0.19479352892426127,
    0.025969530357584508, 0.69070595977297244, 0.19598077718542548,
    0.16697891872993831, 0.80054487743018299, -0.027305370338162598,
    0.24709452307320348, 0.32658981078404603, 0.26873899927946371,
    0.12179172832423819, 0.18904225086113655, 0.029406148628194271,
    -0.17374437920223315, 0.11932080351460912, -0.0046716640142863792,
    0.01286185295854061, 0.27337602977216652, 0.050337397732274705,
    -0.17836897659608555, 0.73176141896340341, 0.37865688295464384,
    -0.049140363587001565, 0.85755475030562178, 0.092475304647522377,
    0.064958629094484827, 0.40050681560041562, 0.34528623944011499,
    -0.2505798469279083, 0.46026748625580427, -0.018418527913073433,
    0.16207169440656125, 0.24503840426089069, 0.134713769107867,
    0.079742102388540265, 0.40324900074357511, 0.015229826756279746,
    0.13850503966597724, 0.53227365551965244, -0.028696278418361358,
    0.34652297262377929, 0.73003080083388239, -0.16361624953102108,
    0.041279939331865877, 0.90000000000000013, 0.054603239132815291,
    -0.10681390109714731, 0.71649860701116741, -0.038564291184854695,
    -0.14238914166629621, 0.1049638204385686, -0.14371521041545468,
    0.29047782581071435, 0.50983121918110286, -0.2596310054441972,
    -0.085350422331284231, 0.34283987326740489, 0.22472607988499074,
    -0.27290364685621649, 0.38907757160737122, 0.17075550488153196,
    0.14085730914489536, 0.5391504982168448, 0.27105842841904326,
    -0.043791212853548513, 0.041548334287569079, -0.02246947520320576,
    -0.25336591989358093, 0.62453262562775991, -0.11569370186635035,
    0.24433637646807269, 0.3664768687878916, -0.10897982108340647,
    -0.26435132625820451, 0.50425838607926199, 0.31373139195434746,
    0.12138539226541471, 0.25494780934978611, -0.22763471440883193,
    -0.21547921347979329, 0.24595287195186277, -0.13802897939207842,
    -0.11489354867351798, 0.34500176546041339, 0.012920473416825888,
    -0.17034430958995331, 0.58916035660230803, -0.17557225493785608,
    0.60145395181647698, 0.18943870261314219, -0.25625488260597284,
    0.092843210480641952, 0.16527343379158657, -0.097102957226228581,
    0.1489142660219453, 0.90000000000000013, -0.048993871998479585,
    -0.17088157098464316, 0.39332482695695792, -0.23449683024567222,
    -0.058058487910880398, 0.82742687449053443, -0.10787855639491233,
    0.07152800348241789, 0.67975115156543664, -0.19253574472201593,
    0.24195135279312804, 0.62973287316169335, 0.079760298410232494,
    0.059532812377510264, 0.44995896608413433, -0.23066816022552866,
    0.36710182996727153, 0.47732365844035324, 0.0080308570084168564,
    -0.20241249488224278, 0.6359140538467325, 0.12379316474146698,
    0.29383330446101663, 0.19251414845253564, 0.11889079589527415,
    0.21478583820351094, 0.18044218654621838, 0.25819246983736727,
    -0.21941493355392386, 0.23159496269108953, 0.03725220835278905,
    0.056998823685529158, 0.26997752802043051, 0.224761970892077,
    0.11156547038317294, 0.11686665850784031, 0.10520788955080092,
};

/*raised-cosine windows for /.
 * reference decoder uses float10 fcos; param_1=shape (sqrt w), param_2=weight @ ctx+0x84/0x88.*/
static void lpec_init_pitch_windows(LPECContext *s)
{
    int n = s->frame_size;
    int i;
    double scale = 2.0 * M_PI / (double) n;

    for (i = 0; i < n; i++) {
        float phase = (float)(((double) i + 0.5) * scale);
        float w     = (1.0f - cosf(phase)) * 0.5f;

        s->pitch_weight[i] = w;
        s->pitch_shape[i]  = sqrtf(w);
    }
}

/* sqrt raised-cosine for modes 1/3 (ctx+0x8c). */
static void lpec_init_f640_shape_m3(LPECContext *s)
{
    /* the reference decoder b380: sqrt raised-cosine over overlap=frame_size needs phase 0..pi,
 * so table period n = 2*frame_size (2048 @ 16 kHz, 1024 @ 8 kHz).*/
    int n = 2 * s->frame_size;
    double scale = 2.0 * M_PI / (double) n;

    /* InitCodec: b380 count=0x800, step=(i+0.5)*2*pi/n. */
    for (int i = 0; i < n; i++) {
        float phase = (float)(((double) i + 0.5) * scale);
        float w     = (1.0f - cosf(phase)) * 0.5f;

        s->f640_shape_m3[i] = sqrtf(w);
    }
}

/* mode 2 uses ctx+0x84 (b400 shape); modes 1/3 use ctx+0x8c (b380). */
static const double *lpec_f640_window(const LPECContext *s, int mode)
{
    if (mode == 1 || mode == 3)
        return s->f640_shape_m3;
    return s->pitch_shape;
}

/* local_1c -> ctx+0x1c/0x28 (mode 0 excit budget base). */
static int lpec_frame_bit_base(const LPECContext *s)
{
    int l1c = s->frame_size;
    int rate_out = s->avctx->sample_rate;
    int rate_in  = s->codec_rate_in;

    /* Dual-rate LP (InitCodec 8000/6000): local_1c scales with rate_in/rate_out. */
    if (rate_in > 0 && rate_in < 0x1771 && rate_out > 0 && rate_in != rate_out)
        l1c = (l1c * rate_in + rate_out / 2) / rate_out;
    return l1c;
}

/* Per-mode bit budgets (skips to ctx+0x1c+mode*4). */
static void lpec_set_frame_bit_targets(LPECContext *s)
{
    int l1c = lpec_frame_bit_base(s);
    int l30 = l1c * 3;

    /* ctx+0x1c/0x20/0x24/0x28 from local_1c and local_30. */
    s->frame_bit_target[0] = l1c;
    s->frame_bit_target[3] = l1c;
    s->frame_bit_target[2] = (l1c * 5 + (unsigned) (l1c * 5 >> 1 >> 30)) >> 2;
    s->frame_bit_target[1] = (l30 + (unsigned) (l30 >> 1 >> 30)) >> 2;
}

static void lpec_skip_to_frame_boundary(GetBitContext *gb, int mode,
                                        const int *frame_bit_target,
                                        int frame_start)
{
    int limit = frame_start + frame_bit_target[mode & 3];
    int pos   = get_bits_count(gb);

    if (limit > pos) {
        int skip = limit - pos;

        if (skip > get_bits_left(gb))
            skip = get_bits_left(gb);
        if (skip > 0)
            skip_bits_long(gb, skip);
    }

    if (getenv("LPEC_BITPOS"))
        fprintf(stderr, "LPEC_BITPOS mode=%d start=%d end=%d target=%d\n",
                mode, frame_start, get_bits_count(gb), limit);
}

/* True when the full mode bit budget is present (padding). */
static int lpec_frame_bits_complete(GetBitContext *gb, int mode,
                                    const int *frame_bit_target,
                                    int frame_start)
{
    int limit = frame_start + frame_bit_target[mode & 3];

    return get_bits_count(gb) >= limit ||
           get_bits_left(gb) >= limit - get_bits_count(gb);
}

static int lpec_complete_frame_bits(LPECContext *s, GetBitContext *gb, int mode,
                                    int frame_start, int frame_size)
{
    if (lpec_frame_bits_complete(gb, mode, s->frame_bit_target, frame_start)) {
        lpec_skip_to_frame_boundary(gb, mode, s->frame_bit_target, frame_start);
        s->pad_pending_start = -1;
    } else {
        s->pad_pending_start = frame_start;
        s->pad_pending_mode  = mode;
    }
    s->pending_frame_start = -1;
    return frame_size;
}

static int lpec_finish_frame(LPECContext *s, GetBitContext *gb, int mode,
                             int frame_start, int frame_size)
{
    s->prev_decode_mode = mode;
    return lpec_complete_frame_bits(s, gb, mode, frame_start, frame_size);
}

/* --------------------------------------------------------------- */
/* LSF decoding */
/* --------------------------------------------------------------- */

/* the reference decoder: sort + min spacing only (no endpoint clamp loops). */
static void lpec_xa_stabilize_lsf_once(double *lsf, int order)
{
    lsf[0] = 0.0;
    for (int i = 1; i <= order; i++) {
        if (lsf[i] < lsf[i - 1]) {
            double t = lsf[i];
            lsf[i]     = lsf[i - 1];
            lsf[i - 1] = t;
            if (i >= 2 && lsf[i - 1] < lsf[i - 2]) {
                t = lsf[i - 1];
                lsf[i - 1] = lsf[i - 2];
                lsf[i - 2] = t;
            }
        }
    }
    lsf[0] = 0.0;
    for (int i = 1; i <= order; i++) {
        if (lsf[i] - lsf[i - 1] < 0.01) {
            if (i < 2) {
                lsf[i] = 0.01;
            } else {
                const double curr = lsf[i];
                const double prev = lsf[i - 1];

                lsf[i]     = (0.01 + prev + curr) * 0.5;
                lsf[i - 1] = ((prev + curr) - 0.01) * 0.5;
            }
        }
    }
    lsf[0] = 0.0;
}

/* the reference decoder: single sort + min-spacing pass (no multi-pass loop). */
static void lpec_xa_stabilize_lsf(double *lsf, int order)
{
    lpec_xa_stabilize_lsf_once(lsf, order);
}

/* lsf[0] sentinel (0), active lines at lsf[1..order]. */
static void lpec_stabilize_lsf(double *lsf, int order)
{
    int half = order / 2;
    double d2;

    if (half < order) {
        d2 = 0.49;
        for (int i = order; i > half; i--) {
            if (lsf[i] >= 0.5)
                lsf[i] = d2;
            d2 = lsf[i] - 0.01;
        }
    }
    if (half > 0) {
        d2 = 0.01;
        for (int i = 1; i <= half; i++) {
            if (lsf[i] < 0.0)
                lsf[i] = d2;
            d2 = lsf[i] + 0.01;
        }
    }
    for (int i = 1; i <= order; i++) {
        if (lsf[i] < lsf[i - 1]) {
            double t = lsf[i];
            lsf[i]     = lsf[i - 1];
            lsf[i - 1] = t;
            if (i >= 2 && lsf[i - 1] < lsf[i - 2]) {
                t = lsf[i - 1];
                lsf[i - 1] = lsf[i - 2];
                lsf[i - 2] = t;
            }
        }
    }
    for (int i = 1; i <= order; i++) {
        if (lsf[i] - lsf[i - 1] < 0.01) {
            if (i < 2) {
                lsf[i] = 0.01;
            } else {
                /* uses pre-update lsf[i] for both lines. */
                const double curr = lsf[i];
                const double prev = lsf[i - 1];

                lsf[i]     = (0.01 + prev + curr) * 0.5;
                lsf[i - 1] = ((prev + curr) - 0.01) * 0.5;
            }
        }
    }
    lsf[0] = 0.0;
}

/* refresh ctx+0x2e8 + param*0x22 (Sony param 1 -> slot1, 2 -> slot2). */
static void lpec_refresh_half_lsf_i16(LPECContext *s, int sony_half)
{
    if ((unsigned) sony_half < 1 || sony_half > 2)
        return;
    if (s->is_xaudio_lp)
        lpec_xa_lsf_indices_to_i16(s->lpc_order, s->num_lsf_cb,
                                   s->half_lsf_idx[sony_half], s->half_lsf_i16[sony_half]);
    else
        lpec_lsf_indices_to_i16(s->num_bands, s->lpc_order, s->num_lsf_cb,
                                s->half_lsf_idx[sony_half], s->half_lsf_i16[sony_half]);
}

/* flag==0: average slot0 (0x2e8) and slot2 (0x32c) into slot1 (0x30a). */
static void lpec_avg_half_lsf_i16(LPECContext *s)
{
    int i, order = s->lpc_order;

    for (i = 0; i <= order; i++) {
        uint32_t u = (uint32_t) (int) s->half_lsf_i16[0][i] +
                     (uint32_t) (int) s->half_lsf_i16[2][i];

        s->half_lsf_i16[1][i] = (int16_t) ((int) (u + (u > 0x7fffffffU)) >> 1);
    }
}

/* --------------------------------------------------------------- */
/* LSF decode */
/* --------------------------------------------------------------- */

static int lpec_decode_lsf_dump_seq;
static int lpec_dump_wants_frame(const LPECContext *s);

static void decode_lsf(LPECContext *s, const int *idx, double *lsf)
{
    const int order = s->lpc_order;
    const int cb    = s->num_lsf_cb;

    lsf[0] = 0.0; /* /: line 0 unused */
    for (int i = 0; i < order; i++) {
        lsf[i + 1] = 0;
        if (order == 16) {
            if (cb > 0)
                lsf[i + 1] += lpec_16k_lsf_quant_1[idx[0] * 16 + i];
            if (cb > 1)
                lsf[i + 1] += lpec_16k_lsf_quant_2[idx[1] * 16 + i];
            if (cb > 2)
                lsf[i + 1] += lpec_16k_lsf_quant_3[idx[2] * 16 + i];
            if (cb > 3)
                lsf[i + 1] += lpec_16k_lsf_quant_4[idx[3] * 16 + i];
        } else if (order == 10) {
            if (s->is_xaudio_lp) {
                if (cb > 0)
                    lsf[i + 1] += lpec_xa_8k_lsf_quant_1[idx[0] * 10 + i];
                if (cb > 1)
                    lsf[i + 1] += lpec_xa_8k_lsf_quant_2[idx[1] * 10 + i];
                if (cb > 2)
                    lsf[i + 1] += lpec_xa_8k_lsf_quant_3[idx[2] * 10 + i];
            } else {
                if (cb > 0)
                    lsf[i + 1] += lpec_8k_lsf_quant_1[idx[0] * 10 + i];
                if (cb > 1)
                    lsf[i + 1] += lpec_8k_lsf_quant_2[idx[1] * 10 + i];
                if (cb > 2)
                    lsf[i + 1] += lpec_8k_lsf_quant_3[idx[2] * 10 + i];
            }
        }
    }

    if (lpec_dump_wants_frame(s)) {
        char path[256];
        FILE *df;

        snprintf(path, sizeof(path), "/tmp/lpec_f%d_lsf_idx_%d.bin",
                 s->lpec_frame_idx, lpec_decode_lsf_dump_seq);
        df = fopen(path, "wb");
        if (df) {
            fwrite(idx, sizeof(int), cb, df);
            fwrite(lsf, sizeof(double), order + 1, df);
            fclose(df);
        }
        lpec_decode_lsf_dump_seq++;
    }

    if (s->is_xaudio_lp)
        lpec_xa_stabilize_lsf(lsf, order);
    else
        lpec_stabilize_lsf(lsf, order);
}

/* --------------------------------------------------------------- */
/* LSF -> LPC conversion */
/* --------------------------------------------------------------- */

static void lsf_to_lpc(const double *lsf, double *lpc, int order)
{
    double P[LPEC_LPC_ORDER_MAX + 2];
    double Q[LPEC_LPC_ORDER_MAX + 2];
    double cosv[LPEC_LPC_ORDER_MAX + 2];
    double A144[LPEC_LPC_ORDER_MAX + 2];
    double Ab4[LPEC_LPC_ORDER_MAX + 2];
    int edi, ecx, k, i;

    for (k = 0; k <= order; k++) {
        P[k] = 0.0;
        Q[k] = 0.0;
    }
    /* cos on lsf[1..order] (param_1 skips sentinel at lsf[0]). */
    for (k = 1; k <= order; k++)
        cosv[k + 1] = cos(lsf[k] * 2.0 * M_PI);

    Q[0] = -1.0;
    P[0] = -1.0;
    P[1] =  1.0;
    Q[1] = -1.0;

    if (order <= 1)
        goto finish;

    ecx = 3;
    for (edi = 2; edi <= order; edi += 2, ecx += 2) {
        A144[1] = cosv[edi]     * 2.0 * Q[0];
        Ab4[1]  = cosv[edi + 1] * 2.0 * P[0];

        for (k = 2; k <= order; k++) {
            Ab4[k] = P[k - 1] * cosv[edi + 1] * 2.0 - P[k - 2];
            A144[k] = Q[k - 1] * cosv[edi]     * 2.0 - Q[k - 2];
        }

        Ab4[order + 1] = cosv[edi + 1] * P[order] * 2.0;
        A144[order + 1] = cosv[edi]     * Q[order] * 2.0;

        for (i = 1; i <= edi / 2; i++) {
            double old_p = P[i];
            double old_q = Q[i];
            double new_p = old_p - Ab4[i];
            double new_q = old_q - A144[i];

            P[i] = new_p;
            Q[i] = new_q;
            P[ecx - i] = -new_p;
            Q[ecx - i] =  new_q;
        }
    }

finish:
    for (k = 0; k < order; k += 2) {
        lpc[k]     = (P[k]     + Q[k])     * -0.5;
        lpc[k + 1] = (P[k + 1] + Q[k + 1]) * -0.5;
    }
    if (k < order)
        lpc[k] = (P[k] + Q[k]) * -0.5;
    if (!(order & 1))
        lpc[order] = (P[order] + Q[order]) * -0.5;
}

/* --------------------------------------------------------------- */
/* Interpolated LPC synthesis */
/* --------------------------------------------------------------- */

static void lpec_dump_lpc_coef(int pass, int seg,
                               const double *lsf, const double *lpc, int order);

static int lpec_dump_lpc_frame = -1;

/**
 * Interpolated LPC synthesis.
 *
 * Sony layout: out_base = synth_buf+order (ctx+0x380); hist in out_base[-k].
 * Tail roll via lpec_roll_lpc_hist after each frame.
*/
static void interpolated_lpc_synthesis(const double *prev_lsf,
                                       const double *curr_lsf,
                                       const double *excitation,
                                       double *out_base,
                                       int out_off,
                                       int order,
                                       int total_len,
                                       int num_seg)
{
    double interp_lsf[LPEC_LSF_MAX];
    double lpc[LPEC_LPC_ORDER_MAX + 2];
    const double step = num_seg > 0 ? 1.0 / (double)num_seg : 1.0;
    int out_pos = 0;

    for (int seg = 0; seg <= num_seg; seg++) {
        double w_curr = (double)seg * step;
        double w_prev = 1.0 - w_curr;
        int div = (seg == 0 || seg == num_seg) ? num_seg * 2 : num_seg;
        int seg_len = total_len / div;

        for (int i = 0; i <= order; i++)
            interp_lsf[i] = prev_lsf[i] * w_prev + curr_lsf[i] * w_curr;

        lsf_to_lpc(interp_lsf, lpc, order);

        if (getenv("LPEC_LPC_SKIP"))
            memset(lpc, 0, sizeof(lpc));

        if (lpec_dump_lpc_frame >= 0 && seg == 0 && out_off == 0 && out_pos == 0) {
            lpec_dump_lpc_coef(0, 0, interp_lsf, lpc, order);
            lpec_dump_lpc_frame = -1;
        }

        for (int n = 0; n < seg_len; n++) {
            int pos = out_off + out_pos + n;
            double s = excitation[out_pos + n];

            /* LPC taps via out[pos-k]; hist in out_base[-k] (ctx+0x380). */
            for (int k = 1; k <= order; k++)
                s -= lpc[k] * out_base[pos - k];

            if (!isfinite(s))
                s = 0.0;

            out_base[pos] = s;
        }
        out_pos += seg_len;
    }
}

/* tail: last order PCM samples -> ctx hist before next frame. */
static void lpec_roll_lpc_hist(LPECContext *s)
{
    int order = s->lpc_order;
    int fs    = s->frame_size;
    double *pcm = s->synth_buf + order;

    if (order > 0 && fs > 0)
        memcpy(s->synth_buf, pcm + fs - order, order * sizeof(double));
}

/* --------------------------------------------------------------- */
/* Pitch gain (+ ) */
/* --------------------------------------------------------------- */

static void lpec_get_pitch_weights(const LPECContext *s, int pitch_idx,
                                   double w[3])
{
    if (pitch_idx < 0)
        pitch_idx = 0;
    if (pitch_idx > 63)
        pitch_idx = 63;

    if (s->num_bands == 10) {
        int off = pitch_idx * 9;
        w[0] = lpec_16k_pitch_gain[off + 0] * (1.0 / 32768.0);
        w[1] = lpec_16k_pitch_gain[off + 1] * (1.0 / 32768.0);
        w[2] = lpec_16k_pitch_gain[off + 2] * (1.0 / 32768.0);
    } else if (s->is_xaudio_lp) {
        w[0] = lpec_xa_8k_pitch_gain[pitch_idx * 3 + 0] * (1.0 / 32768.0);
        w[1] = lpec_xa_8k_pitch_gain[pitch_idx * 3 + 1] * (1.0 / 32768.0);
        w[2] = lpec_xa_8k_pitch_gain[pitch_idx * 3 + 2] * (1.0 / 32768.0);
    } else {
        w[0] = lpec_8k_pitch_gain[pitch_idx * 3 + 0] * (1.0 / 32768.0);
        w[1] = lpec_8k_pitch_gain[pitch_idx * 3 + 1] * (1.0 / 32768.0);
        w[2] = lpec_8k_pitch_gain[pitch_idx * 3 + 2] * (1.0 / 32768.0);
    }
}

static double lpec_pitch_energy_sum(const double w[3])
{
    double s = w[0] + w[1] + w[2];

    return s > 1.0 ? 1.0 : s;
}

static double lpec_buf_rms(const double *buf, int len);

static av_unused double lpec_excit_pitch_energy(LPECContext *s, int pitch_idx, int use_pitch)
{
    double pw[3];

    if (!use_pitch)
        return 0.0;
    lpec_get_pitch_weights(s, pitch_idx, pw);
    return lpec_pitch_energy_sum(pw);
}

/* param_2: always 8 per half, 16 full frame. */
static int lpec_lpc_segments_half(const LPECContext *s)
{
    (void) s;
    return 8;
}

static int lpec_lpc_segments_full(const LPECContext *s)
{
    (void) s;
    return 16;
}

static void lpec_roll_excit_work(LPECContext *s);

/* the reference decoder ctx+0x38c (not LPEC ctx+0x390) holds excit OLA / 03e30 aux. */
static double *lpec_ola_hist(LPECContext *s)
{
    return s->is_xaudio_lp ? s->pitch_cb_aux : s->excit_ola_hist;
}

static void lpec_roll_pitch_win(LPECContext *s)
{
    /* ctx+300 <- 0x134, ctx+0x368 <- 0x370 */
    s->pitch_win_prev     = s->pitch_win_half2;
    s->pitch_lag_off_prev = s->pitch_lag_off_half2;
    s->pitch_tbl_prev     = s->pitch_tbl_half2;
}

/* roll half-LSF CB indices + pitch (LSF doubles handled per mode). */
static void lpec_roll_half_lsf(LPECContext *s)
{
    int ncb = s->num_lsf_cb;

    memcpy(s->half_lsf_idx[0], s->half_lsf_idx[2],
           ncb * sizeof(s->half_lsf_idx[0][0]));
    memcpy(s->half_lsf_i16[0], s->half_lsf_i16[2],
           sizeof(s->half_lsf_i16[0]));
    lpec_roll_pitch_win(s);
    /* ctx+0x35c <- 0x364; ctx+0x138 <- 0x140 (flag, not idx). */
    s->pitch_cb_tbl_prev_idx   = s->pitch_cb_half2_idx;
    s->prev_half2_pitch_cb     = s->pitch_cb_flag_h2;
    s->prev_half2_pitch_cb_idx = s->pitch_cb_half2_idx;
}

/* roll LSF / pitch state at end of each decoded frame. */
static void lpec_end_frame(LPECContext *s)
{
    int order = s->lpc_order;

    /* then */
    lpec_roll_excit_work(s);
    lpec_roll_lpc_hist(s);
    memcpy(s->lsf1, s->lsf2, (order + 1) * sizeof(double));
    lpec_roll_half_lsf(s);
}

static void lpec_dump_half_buf(const LPECContext *s, const char *tag, int half_idx,
                               const double *buf);
static void lpec_dump_f430_seg(const LPECContext *s, int seg, const double *buf, int len);

/* Mode-2 ab80 @ 16 kHz: h1 ctx+0x74 desc1024 seg ctx+0xc; h2 ctx+0x7c desc1536 seg ctx+0x8. */
static const LPECQmfDesc *lpec_qmf_mode2_desc(const LPECContext *s, int half_idx)
{
    if (s->frame_size >= LPEC_FRAME_16K) {
        if (half_idx == 2)
            return &s->qmf.desc1536;
        return &s->qmf.desc1024;
    }
    if (half_idx == 2)
        return &s->qmf.desc768;
    return &s->qmf.desc512;
}

static void lpec_qmf_mode2_process(LPECContext *s, double *buf, int off,
                                   int seg_len, int half_idx)
{
    const LPECQmfDesc *desc = lpec_qmf_mode2_desc(s, half_idx);

    /* the reference zeros excit_src[seg:flen) before mode-2 h2 ab80 only. */
    if (half_idx == 2 && seg_len < desc->len &&
        off + desc->len <= LPEC_EXCIT_SRC_MAX)
        memset(buf + off + seg_len, 0, (desc->len - seg_len) * sizeof(double));

    /* In-place when buf spans desc->len (the reference path). Scratch tmp is only
 * LPEC_QMF_MAX; desc1536+seg1024 needs 1536 samples -- must not scratch.*/
    if (off + desc->len <= LPEC_EXCIT_SRC_MAX)
        lpec_qmf_process_seg(&s->qmf, buf + off, seg_len, desc);
    else if (desc->len > seg_len)
        lpec_qmf_process_scratch(&s->qmf, buf + off, seg_len, desc);
    else
        lpec_qmf_process_seg(&s->qmf, buf + off, seg_len, desc);
}

static void lpec_qmf_mode2_h2_a080(LPECContext *s, double *buf, int off,
                                    int seg_len)
{
    const LPECQmfDesc *desc = lpec_qmf_mode2_desc(s, 2);

    if (seg_len < desc->len && off + desc->len <= LPEC_EXCIT_SRC_MAX)
        memset(buf + off + seg_len, 0, (desc->len - seg_len) * sizeof(double));
    if (off + desc->len <= LPEC_EXCIT_SRC_MAX)
        lpec_qmf_a080_process_desc(&s->qmf, buf + off, seg_len, desc);
    else
        lpec_qmf_mode2_process(s, buf, off, seg_len, 2);
}

/* Mode-1/3 ab80 @ 16 kHz: mode 1 ctx+0x78 desc1536; mode 3 ctx+0x80 desc2048. */
static const LPECQmfDesc *lpec_qmf_mode13_desc(const LPECContext *s, int mode)
{
    if (s->frame_size >= LPEC_FRAME_16K)
        return mode == 3 ? &s->qmf.desc2048 : &s->qmf.desc1536;
    return mode == 1 ? &s->qmf.desc768 : &s->qmf.desc1024;
}

static void lpec_qmf_mode13_process(LPECContext *s, int mode, double *buf, int off,
                                    int seg_len)
{
    const LPECQmfDesc *desc = lpec_qmf_mode13_desc(s, mode);

    /* the reference zeros excit_src[seg:flen) before ab80 when desc spans more than seg. */
    if (desc->len > seg_len && off + desc->len <= LPEC_EXCIT_SRC_MAX)
        memset(buf + off + seg_len, 0, (desc->len - seg_len) * sizeof(double));

    /* In-place when excit_src spans desc->len (the reference path); scratch tmp is only
 * LPEC_QMF_MAX wide, so prefer in-place for the large 2048-tap descriptor.*/
    if (off + desc->len <= LPEC_EXCIT_SRC_MAX)
        lpec_qmf_process_seg(&s->qmf, buf + off, seg_len, desc);
    else if (desc->len > seg_len)
        lpec_qmf_process_scratch(&s->qmf, buf + off, seg_len, desc);
    else
        lpec_qmf_process_seg(&s->qmf, buf + off, seg_len, desc);
}

/* overlap-add excitation segment into frame buffer. */
static void lpec_excit_window_f640(LPECContext *s, int half_idx, double *dst,
                                   const double *src, double *hist,
                                   const double *window, int overlap, int copy_len)
{
    if (s && getenv("LPEC_DUMP_F640")) {
        int df = atoi(getenv("LPEC_DUMP_F640"));

        if (df >= 0 && s->lpec_frame_idx != df)
            goto f640_done;
    } else if (!getenv("LPEC_DUMP_CTX") || !lpec_dump_wants_frame(s)) {
        goto f640_done;
    }
    {
        char path[256];
        FILE *df;

        snprintf(path, sizeof(path), "/tmp/lpec_f%d_f640_h%d_hist.bin",
                 s->lpec_frame_idx, half_idx);
        df = fopen(path, "wb");
        if (df) {
            fwrite(hist, sizeof(double), overlap, df);
            fclose(df);
        }
        snprintf(path, sizeof(path), "/tmp/lpec_f%d_f640_h%d_src.bin",
                 s->lpec_frame_idx, half_idx);
        df = fopen(path, "wb");
        if (df) {
            int n = overlap + copy_len;

            if (n > LPEC_EXCIT_SRC_MAX)
                n = LPEC_EXCIT_SRC_MAX;
            fwrite(src, sizeof(double), n, df);
            fclose(df);
        }
        fprintf(stderr, "LPEC_F640 f%d h%d overlap=%d copy_len=%d dst_off=%td src0=%.3f hist0=%.3f\n",
                s->lpec_frame_idx, half_idx, overlap, copy_len,
                dst - s->frame_excit, src[0], hist[0]);
    }

f640_done:
    for (int i = 0; i < overlap; i++)
        dst[i] = src[i] * window[i] + hist[i] * window[overlap - 1 - i];
    memcpy(hist, src + overlap, copy_len * sizeof(double));
    if (s && getenv("LPEC_DUMP_F640")) {
        int df = atoi(getenv("LPEC_DUMP_F640"));

        if (df < 0 || s->lpec_frame_idx == df) {
            char path[256];
            FILE *dfp;

            snprintf(path, sizeof(path), "/tmp/lpec_f%d_f640_h%d_src_tail.bin",
                     s->lpec_frame_idx, half_idx);
            dfp = fopen(path, "wb");
            if (dfp) {
                fwrite(src + overlap, sizeof(double), overlap + copy_len, dfp);
                fclose(dfp);
            }
        }
    }
    if (s)
        lpec_dump_half_buf(s, "postf640", half_idx, dst);
}

/* ctx+0xc4 pitch triple table: 16 kHz ->, 8 kHz ->. */
static const double *lpec_f430_pitch_table(const LPECContext *s)
{
    if (s->num_bands == 10)
        return lpec_16k_f430_coef;
    /* the reference decoder 8 kHz: ctx+0xc4 = PTR -> pitch_cb,
 * not the LPEC 8 kHz pitch_gain.*/
    if (s->is_xaudio_lp)
        return lpec_xa_8k_pitch_cb;
    return lpec_8k_pitch_gain;
}

/* unvoiced: *(ctx+half*4+0x368) = & (identity triple). */
static const double lpec_f430_unvoiced_tbl[3] = { 1.0, 1.0, 1.0 };

static const double *lpec_pitch_f430_tbl(LPECContext *s, int lag_off, int voiced)
{
    if (!voiced)
        return lpec_f430_unvoiced_tbl;
    if (lag_off < 0)
        lag_off = 0;
    if (lag_off > 63)
        lag_off = 63;
    if (s->is_xaudio_lp)
        return (const double *)(lpec_xa_8k_pitch_i16 + lag_off * 3);
    return lpec_f430_pitch_table(s) + lag_off * 3;
}

/* round to nearest integer (saturating for huge values). */
static double lpec_f8e0_round(double x)
{
    if (!isfinite(x))
        return x;
    return llrint(x);
}

static double lpec_e330_quant(double x)
{
    return lpec_f8e0_round(x * 4096.0) / 32768.0;
}

/* Levinson-style coefficient refinement. */
static int lpec_e330(int n, const double *diag, double *mat, double *tmp, double *upd)
{
    int m, k;

    if (n < 0)
        return 0;

    for (m = 0; m <= n; m++)
        tmp[m] = lpec_e330_quant(diag[m]);

    while (n > 0) {
        double mn;

        if (fabs(tmp[n]) > 0.1225)
            return -1;

        mn = tmp[n] * 8.0;
        mat[n] = mn;

        for (k = 0; k < n; k++)
            upd[k] = lpec_e330_quant(((tmp[k] - tmp[n] * tmp[n - k] * 8.0) /
                                      (1.0 - mn * mn)) * 32768.0) / 32768.0;

        for (k = 0; k < n; k++)
            tmp[k] = upd[k];

        n--;
    }
    return 0;
}

/* pitch triple for voiced OLA. */
static void lpec_e4a0_coefs(const double *tbl, int pitch_win, double out[3])
{
    int n, pw = pitch_win;
    double *diag, *mat, *tmp, *upd;
    int sz;

    if (pw <= 0) {
        out[0] = tbl[0];
        out[1] = tbl[1];
        out[2] = tbl[2];
        return;
    }

    if (pw > 1020)
        pw = 1020;

    n  = pw + 1;
    sz = n + 3;
    diag = av_malloc_array(sz, 4 * sizeof(double));
    if (!diag) {
        out[0] = tbl[0];
        out[1] = tbl[1];
        out[2] = tbl[2];
        return;
    }
    mat = diag + sz;
    tmp = mat + sz;
    upd = tmp + sz;

    memset(diag, 0, sz * sizeof(double));
    if (pw + 2 < sz) {
        diag[pw]     = -tbl[0];
        diag[pw + 1] = -tbl[1];
        diag[pw + 2] = -tbl[2];
    }

    if (lpec_e330(n, diag, mat, tmp, upd) < 0) {
        out[0] = 0.0;
        out[1] = tbl[1];
        out[2] = 0.0;
    } else {
        out[0] = tbl[0];
        out[1] = tbl[1];
        out[2] = tbl[2];
    }
    av_free(diag);
}

/*copy or voiced OLA-filter into synthesis work buffer.
 * param_4 (pitch_win) is lookback distance into work (field).
 * When lookback exceeds len, only tap from work_base (slack) for early samples.*/
static void lpec_excit_ola_f430(LPECContext *s, int seg, double *work, const double *src,
                                int len, int pitch_win, const double *lag_tbl)
{
    const double *work_base = s->excit_work;
    double c[3];
    double xa_tbl[3];
    int w;

    if (getenv("LPEC_F430_OFF"))
        pitch_win = 0;
    if (getenv("LPEC_F430_SEG0_OFF") && seg == 0)
        pitch_win = 0;
    if (getenv("LPEC_F430_H2ZERO") && work >= s->excit_work + LPEC_WORK_SLACK + s->subfr_size)
        pitch_win = 0;

    /* the reference decoder voiced 3-tap OLA: identical recurrence to the LPEC path
 * below; the lag coefs come from xa_pitch_f430 (i16/32768). Off-switch for A/B.*/
    if (s->is_xaudio_lp && getenv("LPEC_F430_XA_OFF")) {
        memcpy(work, src, len * sizeof(double));
        lpec_dump_f430_seg(s, seg, work, len);
        return;
    }

    if (getenv("LPEC_TRACE_F430") &&
        (!getenv("LPEC_TRACE_F430_FRAME") ||
         s->lpec_frame_idx == atoi(getenv("LPEC_TRACE_F430_FRAME")))) {
        if (seg == 0)
            fprintf(stderr, "LPEC_TRACE_F430 f%d seg%d len=%d win=%d tbl=%p slack_rms=%.1f src0=%.3f\n",
                    s->lpec_frame_idx, seg, len, pitch_win, (void *)lag_tbl,
                    lpec_buf_rms(s->excit_work, LPEC_WORK_SLACK), src[0]);
        if (pitch_win > 0 && lag_tbl)
            fprintf(stderr, "LPEC_TRACE_F430 f%d seg%d win=%d tbl=%.6f,%.6f,%.6f\n",
                    s->lpec_frame_idx, seg, pitch_win,
                    lag_tbl[0], lag_tbl[1], lag_tbl[2]);
    }

    if (seg == 0 && lpec_dump_wants_frame(s)) {
        char path[256];
        FILE *df;

        snprintf(path, sizeof(path), "/tmp/lpec_f%d_f430_slack.bin",
                 s->lpec_frame_idx);
        df = fopen(path, "wb");
        if (df) {
            fwrite(s->excit_work, sizeof(double), LPEC_WORK_SLACK, df);
            fclose(df);
        }
    }

    if (pitch_win == 0) {
        memcpy(work, src, len * sizeof(double));
        lpec_dump_f430_seg(s, seg, work, len);
        return;
    }

    if (!lag_tbl)
        lag_tbl = lpec_f430_unvoiced_tbl;

    /* the reference decoder keeps the f430 lag table as int16 triples (ctx+0x360); the excit
 * generator (lpec_pitch_energy_excit) re-casts the same pointer to int16,
 * so convert here for the OLA coefs instead of changing the stored type.*/
    if (s->is_xaudio_lp && lag_tbl != lpec_f430_unvoiced_tbl) {
        const int16_t *i16 = (const int16_t *)lag_tbl;
        xa_tbl[0] = i16[0] * (1.0 / 32768.0);
        xa_tbl[1] = i16[1] * (1.0 / 32768.0);
        xa_tbl[2] = i16[2] * (1.0 / 32768.0);
        lag_tbl = xa_tbl;
    }

    lpec_e4a0_coefs(lag_tbl, pitch_win, c);

    w = pitch_win;
    if (w < 1)
        w = 1;

    for (int i = 0; i < len; i++) {
        double v = src[i];

        if (work + i - (w - 1) >= work_base)
            v += work[i - (w - 1)] * c[0];
        if (work + i - w >= work_base)
            v += work[i - w] * c[1];
        if (work + i - (w + 1) >= work_base)
            v += work[i - (w + 1)] * c[2];

        work[i] = v;
    }

    lpec_dump_f430_seg(s, seg, work, len);
}

/**
 *: f640 (modes 1-3) + f430, then synthesis from excit_work+slack.
 * Mode 0 has no f640 in the reference decoder. src (ctx+0x37c) and dst (ctx+0x38c) are separate.
*/
static av_unused void lpec_refresh_mode1_upper_hist(LPECContext *s, const double *src, int fs, int sf)
{
    double blend = 0.0;
    int i;

    if (getenv("LPEC_MODE1_UPPER"))
        blend = atof(getenv("LPEC_MODE1_UPPER"));
    if (blend <= 0.0) {
        if (fs + sf + sf <= LPEC_EXCIT_SRC_MAX)
            memcpy(lpec_ola_hist(s) + sf, src + fs + sf, sf * sizeof(double));
        return;
    }
    if (blend > 1.0)
        blend = 1.0;
    for (i = 0; i < sf; i++) {
        double tail = (fs + sf + sf <= LPEC_EXCIT_SRC_MAX) ? src[fs + sf + i] : 0.0;
        lpec_ola_hist(s)[sf + i] = tail * (1.0 - blend) + src[sf + i] * blend;
    }
}

static av_unused void lpec_stage_quiet_hist(LPECContext *s, const double *src, int fs, int sf,
                                            double scale_lo, double scale_hi)
{
    int i;

    if (fs + sf > LPEC_EXCIT_SRC_MAX)
        return;
    for (i = 0; i < sf; i++)
        lpec_ola_hist(s)[i] = src[fs + i] * scale_lo;
    if (scale_hi > 0.0) {
        /* Rolled tail at src+fs holds prior voiced energy for both OLA halves. */
        for (i = 0; i < sf; i++)
            lpec_ola_hist(s)[sf + i] = src[fs + i] * scale_hi;
    }
}

/* zero synthesis work areas before f430 (not excit_ola_hist). */
static double lpec_buf_rms(const double *buf, int len)
{
    double e = 0;
    int i;

    if (len <= 0)
        return 0.0;
    for (i = 0; i < len; i++)
        e += buf[i] * buf[i];
    return sqrt(e / len);
}

static int lpec_load_doubles(const char *path, double *buf, int n)
{
    FILE *f = fopen(path, "rb");
    size_t got;

    if (!f)
        return AVERROR(EIO);
    got = fread(buf, sizeof(double), n, f);
    fclose(f);
    return got == (size_t) n ? 0 : AVERROR(EINVAL);
}

static void lpec_maybe_inject_frame_excit(LPECContext *s)
{
    const char *path = getenv("LPEC_INJECT_FRAME_EXCIT");
    const char *fj   = getenv("LPEC_INJECT_FRAME");

    if (path && fj && s->lpec_frame_idx == atoi(fj))
        lpec_load_doubles(path, s->frame_excit, s->frame_size);
}

static void lpec_dump_lpc_coef(int pass, int seg,
                               const double *lsf, const double *lpc, int order)
{
    char path[256];
    FILE *df;

    if (lpec_dump_lpc_frame < 0 || pass != 0 || seg != 0)
        return;

    snprintf(path, sizeof(path), "/tmp/lpec_f%d_lpc_lsf_p%d_s%d.bin",
             lpec_dump_lpc_frame, pass, seg);
    df = fopen(path, "wb");
    if (df) {
        fwrite(lsf, sizeof(double), order + 1, df);
        fclose(df);
    }
    snprintf(path, sizeof(path), "/tmp/lpec_f%d_lpc_coef_p%d_s%d.bin",
             lpec_dump_lpc_frame, pass, seg);
    df = fopen(path, "wb");
    if (df) {
        fwrite(lpc, sizeof(double), order + 1, df);
        fclose(df);
    }
}

static int lpec_dump_wants_frame(const LPECContext *s)
{
    const char *env = getenv("LPEC_DUMP_CTX");

    if (!env)
        return 0;
    if (!*env)
        return 1;
    return atoi(env) < 0 || s->lpec_frame_idx == atoi(env);
}

/* the reference decoder mode-1: a080 seg = ctx+0x8 (subfr_size=256 @ 8 kHz). */
static int lpec_mode1_ab80_len(const LPECContext *s)
{
    if (s->is_xaudio_lp && s->frame_size < LPEC_FRAME_16K)
        return s->subfr_size;
    return s->qmf_seg_m2_h1;
}

static void lpec_dump_half_buf(const LPECContext *s, const char *tag, int half_idx,
                               const double *buf)
{
    char path[256];
    FILE *f;
    int n = s->subfr_size;

    if (getenv("LPEC_DUMP_F640")) {
        int df = atoi(getenv("LPEC_DUMP_F640"));

        if (df >= 0 && s->lpec_frame_idx != df)
            return;
    } else if (!lpec_dump_wants_frame(s)) {
        return;
    }

    if (!strcmp(tag, "postf640") && s->avctx->sample_rate < 8001)
        n = s->qmf_seg_m2_h1; /* mode-2: OLA writes overlap samples at dst */

    snprintf(path, sizeof(path), "/tmp/lpec_f%d_%s_h%d.bin",
             s->lpec_frame_idx, tag, half_idx);
    f = fopen(path, "wb");
    if (f) {
        fwrite(buf, sizeof(double), n, f);
        fclose(f);
    }
}

static void lpec_dump_f430_seg(const LPECContext *s, int seg, const double *buf, int len)
{
    char path[256];
    FILE *f;

    if (!lpec_dump_wants_frame(s) || len <= 0)
        return;

    snprintf(path, sizeof(path), "/tmp/lpec_f%d_postf430_s%d.bin",
             s->lpec_frame_idx, seg);
    f = fopen(path, "wb");
    if (f) {
        fwrite(buf, sizeof(double), len, f);
        fclose(f);
    }
}

static void lpec_dump_f1c0_pass(const LPECContext *s, int pass,
                               const double *out, int out_off, int len)
{
    char path[256];
    FILE *f;

    if (!lpec_dump_wants_frame(s) || len <= 0)
        return;

    snprintf(path, sizeof(path), "/tmp/lpec_f%d_postf1c0_p%d.bin",
             s->lpec_frame_idx, pass);
    f = fopen(path, "wb");
    if (f) {
        fwrite(out + out_off, sizeof(double), len, f);
        fclose(f);
    }
}

static void lpec_dump_ctx_bufs(const LPECContext *s, const char *tag)
{
    char path[256];
    FILE *f;

    if (!lpec_dump_wants_frame(s))
        return;

    snprintf(path, sizeof(path), "/tmp/lpec_f%d_%s.bin", s->lpec_frame_idx, tag);
    f = fopen(path, "wb");
    if (!f)
        return;
    if (!strcmp(tag, "excit_src"))
        fwrite(s->excit_src, sizeof(double), s->frame_size * 2, f);
    else if (!strcmp(tag, "frame_excit"))
        fwrite(s->frame_excit, sizeof(double), s->frame_size, f);
    else if (!strcmp(tag, "synth_work"))
        fwrite(s->excit_work + LPEC_WORK_SLACK, sizeof(double), s->frame_size, f);
    else if (!strcmp(tag, "excit_ola"))
        fwrite(lpec_ola_hist((LPECContext *)s), sizeof(double), s->frame_size, f);
    fclose(f);
}

static void lpec_trace_stage(const LPECContext *s, int mode, const char *tag,
                             const double *buf, int off, int len)
{
    const char *env = getenv("LPEC_STAGE");
    int want_frame = 7;
    int want_mode  = 1;
    const char *mf, *mm;

    if (!env)
        return;
    mf = getenv("LPEC_STAGE_FRAME");
    mm = getenv("LPEC_STAGE_MODE");
    if (mf)
        want_frame = atoi(mf);
    if (mm)
        want_mode = atoi(mm);
    if (s->lpec_frame_idx != want_frame || mode != want_mode)
        return;
    fprintf(stderr, "LPEC_STAGE f%d mode%d %s rms=%.1f off=%d len=%d",
            want_frame, want_mode, tag, lpec_buf_rms(buf + off, len), off, len);
    if (len >= 4) {
        fprintf(stderr, " first4=%.1f,%.1f,%.1f,%.1f",
                buf[off], buf[off + 1], buf[off + 2], buf[off + 3]);
    }
    fputc('\n', stderr);
}
static void lpec_begin_synth_frame(LPECContext *s)
{
    const char *inj = getenv("LPEC_INJECT_LPC_HIST");
    const char *ijf = getenv("LPEC_INJECT_FRAME");
    int order = s->lpc_order;
    int inj_frame = ijf ? atoi(ijf) : -1;

    /* clears synth_buf only at init/flush, not each frame.
 * Cross-frame LPC history lives in synth_buf[0..order-1] (tail copy).*/
    if (inj && order > 0 &&
        (inj_frame < 0 || s->lpec_frame_idx == inj_frame))
        lpec_load_doubles(inj, s->synth_buf, order);
}

/* the reference decoder /: dc80 dst and f430 src = ctx+0x388
 * (= excit_work + 0x800 slack). the reference decoder uses ctx+0x38c (frame_excit).
 * pitch_cb_aux is dc80 OLA hist only (lpec_ola_hist for xaudio).*/
static double *lpec_shaped_excit_dst(LPECContext *s, int mode, double *dst)
{
    if (s->is_xaudio_lp && s->frame_size < LPEC_FRAME_16K && (mode == 1 || mode == 3))
        return s->excit_work + LPEC_WORK_SLACK;
    return dst;
}

static const double *lpec_prepare_synth_excit(LPECContext *s, int mode,
                                              const double *src, double *dst)
{
    const int fo = s->frame_out_size;
    const int sf = s->subfr_size;
    const int fs = s->frame_size;
    double *work = s->excit_work + LPEC_WORK_SLACK;
    double *shaped = lpec_shaped_excit_dst(s, mode, dst);

    lpec_begin_synth_frame(s);

    if (mode == 1) {
        /* the reference mode 1: f640 uses ctx+0x390 unchanged (no excit_src->ola copy). */
        if (getenv("LPEC_MODE1_M0HIST") &&
            s->prev_decode_mode == 0 && fs + sf + sf <= LPEC_EXCIT_SRC_MAX) {
            memcpy(lpec_ola_hist(s), src + fs, sf * sizeof(double));
            memcpy(lpec_ola_hist(s) + sf, src + fs + sf,
                   sf * sizeof(double));
        }
        /* mode 1: f640 overlap=ctx+0x8, copy=ctx+0xc. */
        lpec_excit_window_f640(s, 1, shaped, src, lpec_ola_hist(s),
                               lpec_f640_window(s, mode), fs,
                               lpec_mode1_ab80_len(s));
        lpec_trace_stage(s, mode, "src_h2", src, sf, sf);
        lpec_trace_stage(s, mode, "src_h1", src, 0, sf);
        lpec_trace_stage(s, mode, "dst_h2_post_f640", shaped, sf, sf);
        lpec_trace_stage(s, mode, "dst_h1_post_f640", shaped, 0, sf);
        /* the reference mode 1: single f640 then f430 -- no excit_src->ola refresh.
 * lpec_refresh_mode1_upper_hist copied src[fs+sf:fs+2*sf] but 8 kHz desc768
 * only spans 768 samples, so hist[sf:fs] was zeroed and broke mode-3 f640
 * after long mode-0 runs (e.g. f185->f201 on 0x2c).*/
    } else if (mode == 3) {
        lpec_excit_window_f640(s, 2, shaped, src, lpec_ola_hist(s),
                               lpec_f640_window(s, mode), fs, fs);
    }
    if (mode == 3 || mode == 1) {
        /* the reference decoder: f430 src = ctx+0x388; the reference decoder uses frame_excit. */
        int seg = s->qmf_seg_m2_h1;

        lpec_excit_ola_f430(s, 0, work, shaped, seg, s->pitch_win_prev, s->pitch_tbl_prev);
        lpec_excit_ola_f430(s, 1, work + seg, shaped + seg, seg,
                            s->pitch_win_half2, s->pitch_tbl_half2);
        return work;
    }
    if (mode == 2) {
        /* the reference decoder: modes 0 and 2 share da70 lens ctx+0xc, ctx+0x8,
 * ctx+0xc (fo/sf/fo = 128/256/128 @ 8 kHz). Mode 1/3 use
 * ctx+0x8 twice. the reference decoder mode 2 uses ctx+0x10/0xc/0x10.*/
        if (s->is_xaudio_lp) {
            lpec_excit_ola_f430(s, 0, work, src, fo, s->pitch_win_prev,
                                s->pitch_tbl_prev);
            lpec_excit_ola_f430(s, 1, work + fo, src + fo, sf,
                                s->pitch_win_half1, s->pitch_tbl_half1);
            lpec_excit_ola_f430(s, 2, work + fo + sf, src + fo + sf, fo,
                                s->pitch_win_half2, s->pitch_tbl_half2);
        } else {
            /* mode 2: f430 lens ctx+0x10, ctx+0xc, ctx+0x10. */
            const int qmf_seg = s->qmf_seg_m2_h1;

            lpec_excit_ola_f430(s, 0, work, src, fo, s->pitch_win_prev,
                                s->pitch_tbl_prev);
            lpec_excit_ola_f430(s, 1, work + fo, src + fo, qmf_seg,
                                s->pitch_win_half1, s->pitch_tbl_half1);
            lpec_excit_ola_f430(s, 2, work + fo + qmf_seg, src + fo + qmf_seg, fo,
                                s->pitch_win_half2, s->pitch_tbl_half2);
        }
        return work;
    }

    /* Mode 0: pitch-CB output in src; three f430 segments (no f640 in reference decoder).
 * fo/sf/fo layout (128/256/128 @ 8 kHz) for the reference decoder and the reference decoder.*/
    lpec_excit_ola_f430(s, 0, work, src, fo, s->pitch_win_prev, s->pitch_tbl_prev);
    lpec_trace_stage(s, mode, "work_h1_post_f430", work, 0, sf);
    lpec_excit_ola_f430(s, 1, work + fo, src + fo, sf, s->pitch_win_half1,
                        s->pitch_tbl_half1);
    lpec_trace_stage(s, mode, "work_h2_post_f430", work, sf, sf);
    lpec_excit_ola_f430(s, 2, work + fo + sf, src + fo + sf, fo,
                        s->pitch_win_half2, s->pitch_tbl_half2);
    lpec_trace_stage(s, mode, "work_tail_post_f430", work, fo + sf, sf);
    return work;
}

static void lpec_roll_excit_work(LPECContext *s)
{
    /*memmove(ctx+0x388, ctx+0x388 + ctx+8*8, 0x800).
 * Roll is always 0x800 bytes (= 2 * frame_out_size doubles) for 8 and 16 kHz.*/
    const int roll_bytes = 0x800;
    double *base = s->excit_work;
    int fs = s->frame_size;

    if (fs > 0 && roll_bytes > 0 &&
        fs + roll_bytes / (int)sizeof(double) <= LPEC_WORK_SLACK + LPEC_MAX_FRAME)
        memmove(base, base + fs, roll_bytes);
}

static void lpec_dump_excit_carry(const LPECContext *s, const char *tag)
{
    char path[256];
    FILE *f;
    int sf = s->subfr_size;

    if (!getenv("LPEC_DUMP_CARRY"))
        return;
    if (atoi(getenv("LPEC_DUMP_CARRY")) >= 0 &&
        s->lpec_frame_idx != atoi(getenv("LPEC_DUMP_CARRY")))
        return;

    snprintf(path, sizeof(path), "/tmp/lpec_f%d_%s_excit_src_lo.bin",
             s->lpec_frame_idx, tag);
    f = fopen(path, "wb");
    if (f) {
        fwrite(s->excit_src, sizeof(double), sf * 2, f);
        fclose(f);
    }
    snprintf(path, sizeof(path), "/tmp/lpec_f%d_%s_excit_src_hi.bin",
             s->lpec_frame_idx, tag);
    f = fopen(path, "wb");
    if (f) {
        fwrite(s->excit_src + sf, sizeof(double), sf * 2, f);
        fclose(f);
    }
    snprintf(path, sizeof(path), "/tmp/lpec_f%d_%s_frame_excit_hi.bin",
             s->lpec_frame_idx, tag);
    f = fopen(path, "wb");
    if (f) {
        fwrite(s->frame_excit + sf, sizeof(double), sf * 2, f);
        fclose(f);
    }
}

static void lpec_save_frame(LPECContext *s, const double *out,
                            const double *excit, int frame_size, int mode)
{
    int sf = s->subfr_size;

    lpec_dump_excit_carry(s, "pre_save");

    memcpy(s->mode0_shaped_tail, s->frame_excit + sf, sf * sizeof(double));
    memcpy(s->frame_out,   out,   frame_size * sizeof(double));
    memcpy(s->frame_excit, excit, frame_size * sizeof(double));
    /* reference decoder keeps ctx+0x37c and ctx+0x38c separate; ab80 (desc768/1024) updates
 * excit_src[seg:flen) in place -- no memmove at end of frame.*/

    lpec_dump_excit_carry(s, "post_save");
}

static av_unused void lpec_tame_pitch_aux_buf(double *aux, int len, double target)
{
    double e = 0;
    int i;

    for (i = 0; i < len; i++)
        e += aux[i] * aux[i];
    if (e / len < 150.0 * 150.0)
        return;

    if (getenv("LPEC_HIST_SCALE_MO")) {
        double sc = atof(getenv("LPEC_HIST_SCALE_MO"));

        for (i = 0; i < len; i++)
            aux[i] *= sc;
        return;
    }

    {
        double rms = sqrt(e / len);
        double sc  = target / rms;

        if (sc > 1.0)
            sc = 1.0;
        for (i = 0; i < len; i++)
            aux[i] *= sc;
    }
}

static av_unused int lpec_tail_loud_enough(const double *tail, const double *hist, int len)
{
    double te = 0, he = 0;
    int i;

    for (i = 0; i < len; i++) {
        te += tail[i] * tail[i];
        he += hist[i] * hist[i];
    }
    te /= len;
    he /= len;
    if (he <= 150.0 * 150.0)
        return 1;
    return te >= he * 0.01;
}

/**
 * Mode-0 pitch codebook shaping.
 * param_4 aux copy is always ctx+0x37c+half (excit_src[512:1024] for 16 kHz).
 * Half2 05d50/04960: reference decoder uses ctx+0x37c only -- half2 overwrites lower slot,
 * QMF filters [0:subfr), 04960 src is always lower post-QMF.
 * param_5 aux is ctx+0x390 (LPEC) or ctx+0x38c (the reference decoder pitch_cb_aux).
*/
static void lpec_apply_pitch_cb_mode0(LPECContext *s, double *dst_half,
                                      const double *src_lo,
                                      const double *aux_copy_src,
                                      int subfr_len,
                                      const double *aux_in,
                                      int cb_a, int cb_b, int idx_a, int idx_b,
                                      int update_aux, int half_idx)
{
    const double *shape  = s->pitch_shape;
    const double *weight = s->pitch_weight;
    double *aux_store = lpec_ola_hist(s);
    const double *aux = aux_in ? aux_in : aux_store;
    int half = subfr_len;
    int frame_len = s->frame_size;

    if (!cb_a && !cb_b) {
        for (int i = 0; i < half; i++)
            dst_half[i] = shape[i] * src_lo[i] + aux[i] * shape[i + half];
    } else {
        const double *tbl0 = lpec_pitch_cb_ptr(s, cb_a ? idx_a : -1);
        const double *tbl1 = lpec_pitch_cb_ptr(s, cb_b ? idx_b : -1);

        if (getenv("LPEC_DUMP_04960") &&
            (!getenv("LPEC_DUMP_04960_FRAME") ||
             s->lpec_frame_idx == atoi(getenv("LPEC_DUMP_04960_FRAME")))) {
            fprintf(stderr, "LPEC_DUMP_04960 f%d h%d cb=%d/%d idx=%d/%d tbl1[0..3]=%.6f,%.6f,%.6f,%.6f\n",
                    s->lpec_frame_idx, half_idx, cb_a, cb_b, idx_a, idx_b,
                    tbl1[0], tbl1[1], tbl1[2], tbl1[3]);
        }

        for (int seg = 0; seg < 4; seg++) {
            int start = (seg * frame_len) >> 3;
            int end   = ((seg + 1) * frame_len) >> 3;
            double d6 = tbl0[4 + seg];
            double d7 = tbl1[seg];
            double d8 = tbl0[7 - seg];
            double d9 = tbl1[3 - seg];

            if (end > half)
                end = half;
            for (int i = start; i < end; i++) {
                int j = half + i;
                double den = weight[i] * (d9 / d7) + weight[j] * (d8 / d6);

                if (fabs(den) < 1e-30)
                    den = 1e-30;
                dst_half[i] = (d9 * shape[i] * src_lo[i]
                               + d8 * shape[j] * aux[i]) / den;
            }
        }
    }

    if (getenv("LPEC_STAGE") && getenv("LPEC_STAGE_FRAME")) {
        int wf = atoi(getenv("LPEC_STAGE_FRAME"));
        if (s->lpec_frame_idx == wf) {
            fprintf(stderr, "LPEC_STAGE f%d 04960 h%d dst_rms=%.1f aux_rms=%.1f src_rms=%.1f cb=%d/%d idx=%d/%d\n",
                    wf, half_idx, lpec_buf_rms(dst_half, half), lpec_buf_rms(aux, half),
                    lpec_buf_rms(src_lo, half), cb_a, cb_b, idx_a, idx_b);
        }
    }

    if (update_aux)
        memcpy(aux_store, aux_copy_src + half, half * sizeof(double));

    lpec_dump_half_buf(s, "post04960", half_idx, dst_half);
}

/* --------------------------------------------------------------- */
/* Excitation generation */
/* --------------------------------------------------------------- */

/**
 * Per-half pitch codebook select (loop at 0x13c):
 * 1 bit; if set, 7-bit index scaled to lpec_*_pitch_cb (idx * 0x40 + base).
 * Returns CB index 0..127, or -1 if unvoiced.
*/
/* pitch table idx = *(ctx + half*4 + 0x140); if <0 use +0x144. */
static int lpec_pitch_route_arg(LPECContext *s, int mode, int half)
{
    int route = lpec_excit_route_idx(mode, half);
    int idx = route - 1;
    int v;

    if (idx < 0)
        return 0;
    if (idx >= FF_ARRAY_ELEMS(s->half_lsf_cb0))
        idx = FF_ARRAY_ELEMS(s->half_lsf_cb0) - 1;

    v = s->half_lsf_cb0[idx];
    if (v < 0 && idx + 1 < FF_ARRAY_ELEMS(s->half_lsf_cb0))
        v = s->half_lsf_cb0[idx + 1];
    return v >= 0 ? v : 0;
}

static int read_pitch_cb_half(GetBitContext *gb)
{
    if (get_bits_left(gb) < 1)
        return AVERROR(EAGAIN);
    if (!get_bits(gb, 1))
        return -1;
    if (get_bits_left(gb) < 7)
        return AVERROR(EAGAIN);
    return get_bits(gb, 7);
}

/* store lag table pointer at ctx+0x368/0x36c/0x370 per half. */
static void lpec_bind_pitch_tbl(LPECContext *s, int *lag_off_out, int voiced, int lag_off)
{
    const double *tbl = lpec_pitch_f430_tbl(s, lag_off, voiced);

    if (lag_off_out == &s->pitch_lag_off_half1)
        s->pitch_tbl_half1 = tbl;
    else if (lag_off_out == &s->pitch_lag_off_half2)
        s->pitch_tbl_half2 = tbl;
}

/**
 * Read pitch parameters:
 * pitch_voiced_bits-bit field; 0 = unvoiced, else 6-bit lag offset.
 * Returns lag, or 0 if unvoiced.
*/
static int read_pitch(LPECContext *s, GetBitContext *gb, int *voiced_out,
                      int *win_idx_out, int *lag_off_out)
{
    int n = s->pitch_voiced_bits;

    if (get_bits_left(gb) < n)
        return AVERROR(EAGAIN);
    int v = get_bits(gb, n);
    if (win_idx_out)
        *win_idx_out = v;
    *voiced_out = v != 0;
    if (!v) {
        if (lag_off_out)
            *lag_off_out = 0;
        lpec_bind_pitch_tbl(s, lag_off_out, 0, 0);
        return 0;
    }
    if (get_bits_left(gb) < 6)
        return AVERROR(EAGAIN);
    int off = get_bits(gb, 6);
    if (lag_off_out)
        *lag_off_out = off;
    lpec_bind_pitch_tbl(s, lag_off_out, 1, off);
    return LPEC_PITCH_LAG_BASE + off;
}

/* mode 2 half1 must not read past ctx+0x1c (only half2 uses 0x24). */
static int lpec_excit_bits_limit(const LPECContext *s, int mode, int half,
                                 int frame_start)
{
    if (mode == 2 && half == 1)
        return frame_start + s->frame_bit_target[0];
    return frame_start + s->frame_bit_target[mode & 3];
}

#define LPEC_GEN_EXCIT_HALF(s, gb, dst, half, lag, lag_off, win, voiced,      \
                            pitch_tbl, frame_start, mode, frame_size, order,  \
                            pitch_cb, lsf_i16_override)                       \
    ((s)->excit.dump_frame = (s)->lpec_frame_idx,                             \
     lpec_generate_excitation(&(s)->excit, gb, dst, (s)->subfr_size, half, lag,  \
                             lag_off, win, voiced, pitch_tbl,                   \
                             lpec_excit_bit_budget(s, gb, frame_start, mode,    \
                                                  pitch_cb,                     \
                                                  mode == 0 || mode == 2, half),\
                             lpec_excit_bits_limit(s, mode, half, frame_start), \
                             mode, lpec_pitch_route_arg(s, mode, half),         \
                             (s)->half_lsf_idx[half],                            \
                             lsf_i16_override, frame_size, order,               \
                             (s)->num_lsf_cb))
static int lpec_excit_bit_budget(LPECContext *s, GetBitContext *gb,
                                 int frame_start, int mode, int cb_overhead,
                                 int per_half, int half)
{
    int used = get_bits_count(gb) - frame_start;
    int rem, budget;
    int avail = get_bits_left(gb);

    if (mode == 2 && half == 1) {
        /* mode 2 half1: (ctx+0x1c - bitpos) / 2, no pitch CB. */
        rem = s->frame_bit_target[0] - used;
        budget = (rem >> 1) - cb_overhead;
    } else if (mode == 2 && half == 2) {
        rem = s->frame_bit_target[2] - used;
        budget = rem - cb_overhead;
    } else if (per_half && mode == 0) {
        /* mode 0: uVar4 after pitch, before pitch CB; same for both halves. */
        budget = (s->mode0_excit_rem >> 1) - cb_overhead;
    } else if (per_half) {
        rem = s->frame_bit_target[mode & 3] - used;
        budget = (rem >> 1) - cb_overhead;
    } else {
        rem = s->frame_bit_target[mode & 3] - used;
        budget = rem - cb_overhead;
    }

    if (budget < 0)
        budget = 0;
    if (budget > avail)
        budget = avail;
    return budget;
}

/* QMF post-process on excitation before f640 / pitch CB. */
static void lpec_postprocess_excit(LPECContext *s, int mode, double *buf, int off,
                                   int len, int half_idx)
{
    static int qmf_mode = -1;

    if (qmf_mode < 0) {
        const char *env = getenv("LPEC_QMF");

        if (env) {
            if (!strcmp(env, "0"))
                qmf_mode = 0;
            else if (!strcmp(env, "m1"))
                qmf_mode = 1; /* mode 1 only */
            else if (!strcmp(env, "13"))
                qmf_mode = 2; /* modes 1 and 3 */
            else if (!strcmp(env, "1"))
                qmf_mode = 3; /* all modes */
            else if (!strcmp(env, "m0"))
                qmf_mode = 4; /* mode 0 only */
            else
                qmf_mode = 2;
        } else {
            qmf_mode = 3; /* calls for every half incl. mode 0/2 */
        }
    }

    if (!qmf_mode)
        return;
    if (qmf_mode == 1 && mode != 1)
        return;
    if (qmf_mode == 2 && mode != 1 && mode != 3)
        return;
    if (qmf_mode == 4 && mode != 0)
        return;

    if (s->is_xaudio_lp && mode == 2 && half_idx == 2 &&
        getenv("LPEC_XA_M2H2_NO_QMF"))
        return;

    {
        const char *from = getenv("LPEC_QMF_FROM");

        if (from && s->lpec_frame_idx < atoi(from))
            return;
    }

    if (getenv("LPEC_DUMP_QMF")) {
        int want = atoi(getenv("LPEC_DUMP_QMF"));
        if (want < 0 || s->lpec_frame_idx == want) {
            char path[128];
            FILE *df;

            snprintf(path, sizeof(path), "/tmp/qmf_in_f%d_h%d.bin",
                     s->lpec_frame_idx, half_idx);
            df = fopen(path, "wb");
            if (df) {
                fwrite(buf + off, sizeof(double), len, df);
                fclose(df);
            }
        }
    }

    /* QMF on excit_src; seg len/desc vary by mode (the reference decoder). */
    lpec_qmf_frame_idx = s->lpec_frame_idx;
    lpec_qmf_chain_late = 0;
    if (!getenv("LPEC_PRE01590_NO_CHAIN_LATE")) {
        const char *force = getenv("LPEC_PRE01590_CHAIN_LATE");
        int chain_m1 = getenv("LPEC_PRE01590_CHAIN_M1") != NULL;
        int chain_m2 = !getenv("LPEC_PRE01590_NO_CHAIN_M2H2");

        if (force && force[0] && strcmp(force, "0"))
            lpec_qmf_chain_late = 1;
        else if (chain_m1 && mode == 1 && s->prev_decode_mode == 2)
            lpec_qmf_chain_late = 1;
        else if (chain_m2 && mode == 2 && half_idx == 2 &&
                 s->next_frame_mode_peek == 1 &&
                 !lpec_pre01590_chain_skip_frame(s->lpec_frame_idx))
            lpec_qmf_chain_late = 2;
        else if (chain_m2 && mode == 2 && half_idx == 2 &&
                 s->next_frame_mode_peek == 3 &&
                 !lpec_pre01590_chain_skip_frame(s->lpec_frame_idx))
            lpec_qmf_chain_late = 3;
    }
    {
        const char *ab80_frame = getenv("LPEC_DUMP_AB80_FRAME");
        int saved_wanted = lpec_ab80_dump_wanted;
        int saved_frame  = lpec_ab80_dump_frame;
        int saved_half   = lpec_ab80_dump_half;

        lpec_ab80_dump_wanted = !ab80_frame || s->lpec_frame_idx == atoi(ab80_frame);
        if (lpec_ab80_dump_wanted) {
            lpec_ab80_dump_frame = s->lpec_frame_idx;
            lpec_ab80_dump_half  = half_idx;
        } else {
            lpec_ab80_dump_frame = -1;
        }

    if (mode == 0 || mode == 2) {
        int qmf_len;
        int desc_idx = 0;

        if (mode == 2 && half_idx == 2) {
            qmf_len  = s->frame_size;
            desc_idx = 1;
        } else if (mode == 2 && half_idx == 1) {
            qmf_len  = s->qmf_seg_m2_h1;
            desc_idx = 0;
        } else if (mode == 0) {
            qmf_len = s->subfr_size;
        } else {
            qmf_len = s->qmf_seg_m2_h1;
        }
        {
            /* Mode 2: ab80 in-place on excit_src (same ptr, seg=len). */
            if (mode == 2 && half_idx == 2) {
                const LPECQmfDesc *desc = lpec_qmf_mode2_desc(s, 2);
                int ext = desc->len - qmf_len;

                /* the reference leaves excit_src[len:flen) zero before h2 ab80. */
                if (ext > 0 && off + qmf_len + ext <= LPEC_EXCIT_SRC_MAX)
                    memset(buf + off + qmf_len, 0, ext * sizeof(double));
                if (getenv("LPEC_DUMP_CTX") && lpec_dump_wants_frame(s) &&
                    desc->len <= LPEC_EXCIT_SRC_MAX) {
                    char path[256];
                    FILE *df;

                    snprintf(path, sizeof(path), "/tmp/lpec_f%d_preqmf_h2_768.bin",
                             s->lpec_frame_idx);
                    df = fopen(path, "wb");
                    if (df) {
                        fwrite(buf + off, sizeof(double), desc->len, df);
                        fclose(df);
                    }
                }
            }
            if (mode == 2 && (half_idx == 1 || half_idx == 2)) {
                /* Mode-2 h2: exact-iDFT a080 (8 kHz desc768, 16 kHz desc1536).
 * ab80 radix-3 caps h2 corr ~0.984 and poisons persistent tail /
 * f430 s2 on the same frame. LPEC_M2H2_A080_OFF restores ab80.*/
                if (half_idx == 2 && !getenv("LPEC_M2H2_A080_OFF"))
                    lpec_qmf_mode2_h2_a080(s, buf, off, qmf_len);
                else
                    lpec_qmf_mode2_process(s, buf, off, qmf_len, half_idx);
            } else if (mode == 0 && s->frame_size >= LPEC_FRAME_16K) {
                const LPECQmfDesc *desc = s->qmf.active ? s->qmf.active : &s->qmf.desc1024;

                /* 16 kHz mode-0 follows local_c=2/4 descriptor selection (1024 taps). */
                if (desc->len > qmf_len)
                    lpec_qmf_process_scratch(&s->qmf, buf + off, qmf_len, desc);
                else
                    lpec_qmf_process_seg(&s->qmf, buf + off, qmf_len, desc);
            } else
                lpec_qmf_process_slot(&s->qmf, buf + off, qmf_len, desc_idx);
            if (getenv("LPEC_DUMP_QMF")) {
                int want = atoi(getenv("LPEC_DUMP_QMF"));

                if (want < 0 || s->lpec_frame_idx == want) {
                    char path[128];
                    FILE *df;
                    int dump_len = qmf_len;
                    const LPECQmfDesc *qdesc = NULL;

                    if (mode == 2 && (half_idx == 1 || half_idx == 2))
                        qdesc = lpec_qmf_mode2_desc(s, half_idx);

                    snprintf(path, sizeof(path), "/tmp/qmf_out_f%d_h%d.bin",
                             s->lpec_frame_idx, half_idx);
                    df = fopen(path, "wb");
                    if (df) {
                        fwrite(buf + off, sizeof(double), dump_len, df);
                        fclose(df);
                    }
                    if (qdesc && qdesc->len > dump_len) {
                        snprintf(path, sizeof(path), "/tmp/qmf_post_f%d_h%d.bin",
                                 s->lpec_frame_idx, half_idx);
                        df = fopen(path, "wb");
                        if (df) {
                            fwrite(buf + off, sizeof(double), qdesc->len, df);
                            fclose(df);
                        }
                    }
                }
            }
        }
    } else if (mode == 3) {
        if (s->frame_size >= LPEC_FRAME_16K) {
            /* 16 kHz: local_c remaps mode-3 to 1024 family descriptor. */
            lpec_qmf_mode13_process(s, mode, buf, off, len);
        } else {
            /* 8 kHz keeps slot path (ctx+0x80) with persistent tail state. */
            lpec_qmf_process_slot(&s->qmf, buf + off, len, 2);
        }
    } else if (mode == 1) {
        int qmf_len = lpec_mode1_ab80_len(s);
        const LPECQmfDesc *desc = s->frame_size >= LPEC_FRAME_16K
            ? lpec_qmf_mode13_desc(s, mode)
            : &s->qmf.desc768;

        /*
 * Mode-1 ab80/a080 in-place on excit_src; do not zero excit_src[seg:flen)
 * (mode13_process memset breaks pass-2: fills buf[seg:flen/2)
 * before ab80 @ 16 kHz desc1536). lpec_qmf_a080_process_desc uses an exact
 * inverse DFT instead of lpec_ab80.c's radix-3 FFT emulation.
*/
        if (!getenv("LPEC_QMF_A080_8K_OFF")) {
            lpec_qmf_a080_process_desc(&s->qmf, buf + off, qmf_len, desc);
        } else {
            lpec_qmf_process_seg(&s->qmf, buf + off, qmf_len, desc);
        }
    }

        lpec_ab80_dump_wanted = saved_wanted;
        lpec_ab80_dump_frame  = saved_frame;
        lpec_ab80_dump_half   = saved_half;
    }
}

/* --------------------------------------------------------------- */
/* Initialiser / close */
/* --------------------------------------------------------------- */

/* the reference decoder profile rates (d430 in, d598 out). */
static int lpec_rates_from_quality(int quality, int *rate_in, int *rate_out)
{
    switch (quality) {
    case 0x15:
    case 0x2a:
    case 0x4a:
        *rate_in = *rate_out = 16000;
        return 0;
    case 0x19:
    case 0x2c:
    case 0x4c:
        *rate_in  = 6000;
        *rate_out = 8000;
        return 0;
    default:
        return AVERROR(EINVAL);
    }
}

static int lpec_chunk_bytes(const LPECContext *s, uint8_t lead)
{
    switch (lead & 0xC0) {
    case 0x00: return s->chunk_sz[0];
    case 0x40: return s->chunk_sz[1];
    case 0x80: return s->chunk_sz[2];
    default:   return s->chunk_sz[3];
    }
}

static void lpec_consume_chunk(LPECContext *s, int nbytes)
{
    if (nbytes <= 0 || nbytes > s->chunk_carry_len)
        return;
    memmove(s->carry_buf, s->carry_buf + nbytes, s->chunk_carry_len - nbytes);
    s->chunk_carry_len -= nbytes;
}

static int decode_one_lpec_frame(LPECContext *s, GetBitContext *gb,
                                 double *output, int frame_size, int subfr_size);

/* Drop consumed frame bits from the front of stream_buf (bit-accurate). */
static void lpec_stream_consume_bits(LPECContext *s, int bits)
{
    int total = s->stream_bit_off + bits;
    int nbytes = total >> 3;
    int new_off = total & 7;

    if (nbytes > s->stream_len)
        nbytes = s->stream_len;
    if (nbytes > 0) {
        memmove(s->stream_buf, s->stream_buf + nbytes, s->stream_len - nbytes);
        s->stream_len -= nbytes;
    }
    s->stream_bit_off = s->stream_len ? new_off : 0;
}

/* Feed 48-byte LPEC chunks from carry_buf (the reference Decode chunk size). */
static void lpec_feed_chunks(LPECContext *s)
{
    while (s->chunk_carry_len > 0) {
        int need = lpec_chunk_bytes(s, s->carry_buf[0]);

        if (need <= 0 || s->chunk_carry_len < need)
            break;
        if (s->stream_len + need > (int)sizeof(s->stream_buf))
            break;
        memcpy(s->stream_buf + s->stream_len, s->carry_buf, need);
        s->stream_len += need;
        lpec_consume_chunk(s, need);
    }
}

/* Decode as many frames as stream_buf allows (called before new packet bytes). */
static int lpec_decode_from_stream(LPECContext *s, double *out, int out_cap,
                                   int frame_size, int subfr_size,
                                   int max_frames, int *frames_out)
{
    GetBitContext gb;
    int total_samples = 0;
    int num_frames = 0;
    int ret;

    while (s->stream_len > 0 &&
           num_frames < max_frames &&
           total_samples + frame_size <= out_cap) {
        int n, bits_pos;

        if ((ret = init_get_bits(&gb, s->stream_buf, s->stream_len * 8)) < 0)
            return ret;

        if (s->stream_bit_off > 0)
            skip_bits(&gb, s->stream_bit_off);

        if (get_bits_left(&gb) < 2)
            break;

        bits_pos = get_bits_count(&gb);
        n = decode_one_lpec_frame(s, &gb, out + total_samples,
                                  frame_size, subfr_size);

        if (n == AVERROR(EAGAIN))
            break;

        if (n < 0) {
            if (s->pending_frame_start >= 0) {
                lpec_skip_to_frame_boundary(&gb, s->pending_frame_mode,
                                            s->frame_bit_target,
                                            s->pending_frame_start);
                s->pending_frame_start = -1;
                lpec_stream_consume_bits(s, get_bits_count(&gb));
            } else {
                lpec_stream_consume_bits(s, 8);
            }
            continue;
        }

        lpec_stream_consume_bits(s, get_bits_count(&gb) - bits_pos);
        total_samples += n;
        num_frames++;
        s->lpec_frame_idx++;
    }

    *frames_out = num_frames;
    return total_samples;
}

static av_cold int lpec_decode_init(AVCodecContext *avctx)
{
    LPECContext *s = avctx->priv_data;

    if (avctx->ch_layout.nb_channels != 1) {
        av_log(avctx, AV_LOG_ERROR, "LPEC: only mono supported\n");
        return AVERROR_PATCHWELCOME;
    }
    s->avctx = avctx;
    avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    s->codec_rate_in = 0;
    s->is_xaudio_lp = avctx->codec_tag == 0x19;

    {
        int rate_in = 0, rate_out = 0;

        if (lpec_rates_from_quality(avctx->codec_tag, &rate_in, &rate_out) >= 0) {
            avctx->sample_rate = rate_out;
            s->codec_rate_in   = rate_in;
        } else if (avctx->sample_rate == 6000) {
            /* LP LPEC header rate without a known quality tag. */
            avctx->sample_rate = 8000;
            s->codec_rate_in   = 6000;
        }
    }

    /* Parameters from the reference decoder init (the reference decoder ) */
    if (avctx->sample_rate >= 8001) {
        int cap = avctx->sample_rate >= 22051 ? 4096 : 2048;

        s->frame_size = LPEC_FRAME_16K;
        s->subfr_size = LPEC_SUBFR_16K;
        s->frame_out_size = LPEC_FRAME_16K / 4;
        /* ctx+0xc = ctx+4 >> 2 (512 @ 16 kHz). */
        s->qmf_seg_m2_h1 = cap >> 2;
        s->lpc_order  = 16;
        s->num_lsf_cb = 4;
        s->num_bands  = 10;
    } else {
        int cap = 1024; /* ctx+4 for rate < 11026 */

        s->frame_size = LPEC_FRAME_8K;
        s->subfr_size = LPEC_SUBFR_8K;
        s->frame_out_size = LPEC_FRAME_8K / 4;
        /* ctx+0xc = ctx+4 >> 2 (256 @ 8 kHz). */
        s->qmf_seg_m2_h1 = cap >> 2;
        s->lpc_order  = 10;
        s->num_lsf_cb = 3;
        s->num_bands  = 8;
    }

    if (s->frame_size >= LPEC_FRAME_16K) {
        s->chunk_sz[0] = s->chunk_sz[3] = 128;
        s->chunk_sz[1] = 96;
        s->chunk_sz[2] = 160;
    } else {
        s->chunk_sz[0] = s->chunk_sz[3] = 48;
        s->chunk_sz[1] = 36;
        s->chunk_sz[2] = 60;
    }

    /* the reference decoder: pitch lag uses ctx+0x9c (same bumps as LPEC ctx+0xa0); ctx+0xa0
 * is LPC order (10 @ 8 kHz), not voiced-bit count.*/
    s->pitch_voiced_bits = 7;
    {
        int t = (avctx->sample_rate / 8000) * 0x78;

        if (t > 0x80)
            s->pitch_voiced_bits++;
        if (t > 0x100)
            s->pitch_voiced_bits++;
    }

    /* Default LSFs: lsf[0]=0, lines at lsf[1..order] (layout) */
    for (int i = 0; i < s->lpc_order; i++) {
        double v = (double)(i + 1) / (s->lpc_order + 1) * 0.5;
        s->prev_lsf[i + 1] = v;
        s->lsf1[i + 1]     = v;
        s->lsf2[i + 1]     = v;
    }
    s->prev_lsf[0] = s->lsf1[0] = s->lsf2[0] = 0.0;

    memset(s->synth_buf, 0, sizeof(s->synth_buf));
    memset(&s->excit, 0, sizeof(s->excit));
    lpec_excit_init(&s->excit, avctx->sample_rate, s->codec_rate_in,
                    s->num_bands, s->frame_size, s->subfr_size,
                    s->is_xaudio_lp);
    memset(s->frame_excit, 0, sizeof(s->frame_excit));
    memset(s->excit_src, 0, sizeof(s->excit_src));
    memset(s->excit_work, 0, sizeof(s->excit_work));
    memset(s->excit_ola_hist, 0, sizeof(s->excit_ola_hist));
    memset(s->frame_out,   0, sizeof(s->frame_out));
    s->noise_pos      = 0;
    s->half_lsf_cb0[0] = s->half_lsf_cb0[1] = 0;
    memset(s->half_lsf_idx, 0, sizeof(s->half_lsf_idx));
    memset(s->half_lsf_i16, 0, sizeof(s->half_lsf_i16));
    s->prev_pitch_lag = LPEC_PITCH_LAG_BASE;
    s->frame_count    = 0;
    s->lpec_frame_idx = 0;
    s->pending_frame_start = -1;
    s->pending_frame_mode  = 0;
    s->pad_pending_start   = -1;
    s->pad_pending_mode    = 0;
    s->synth_half_flag     = 0;
    s->chunk_carry_len     = 0;
    s->stream_len          = 0;
    memset(s->carry_buf, 0, sizeof(s->carry_buf));
    memset(s->stream_buf, 0, sizeof(s->stream_buf));
    s->stream_bit_off      = 0;
    lpec_set_frame_bit_targets(s);
    lpec_init_pitch_windows(s);
    lpec_init_f640_shape_m3(s);
    memset(s->pitch_cb_aux, 0, sizeof(s->pitch_cb_aux));
    memset(s->mode0_shaped_tail, 0, sizeof(s->mode0_shaped_tail));
    s->prev_half2_pitch_cb     = 0;
    s->prev_half2_pitch_cb_idx = -1;
    s->pitch_cb_flag_h1        = 0;
    s->pitch_cb_flag_h2        = 0;
    s->pitch_cb_half2_idx      = -1;
    s->pitch_cb_tbl_prev_idx   = -1;
    s->prev_lsf_cb_half2       = 0;
    s->prev_decode_mode        = -1;
    s->next_frame_mode_peek    = -1;
    s->eof_flush_done          = 0;
    s->eof_draining            = 0;
    s->pitch_win_prev          = 0;
    s->pitch_win_half1         = 0;
    s->pitch_win_half2         = 0;
    s->pitch_lag_off_half1     = 0;
    s->pitch_lag_off_half2     = 0;
    s->pitch_lag_off_prev      = 0;
    s->pitch_tbl_prev          = lpec_f430_unvoiced_tbl;
    s->pitch_tbl_half1         = lpec_f430_unvoiced_tbl;
    s->pitch_tbl_half2         = lpec_f430_unvoiced_tbl;
    lpec_qmf_init(&s->qmf);
    lpec_qmf_configure(&s->qmf, avctx->sample_rate, s->codec_rate_in);
    return 0;
}

static av_cold int lpec_decode_close(AVCodecContext *avctx)
{
    LPECContext *s = avctx->priv_data;

    if (getenv("LPEC_DEBUG"))
        fprintf(stderr, "LPEC_DEBUG close idx=%d chunk_carry=%d pad=%d\n",
                s->lpec_frame_idx, s->chunk_carry_len, s->pad_pending_start);
    return 0;
}

/* --------------------------------------------------------------- */
/* Main decode function */
/* --------------------------------------------------------------- */

/* the reference decoder 6000->8000:..db20 resampler gain after d800 synth (~8000/5376). */
static double lpec_xa_output_gain(const LPECContext *s)
{
    int rate_in  = s->codec_rate_in;
    int rate_out = s->avctx->sample_rate;

    if (!s->is_xaudio_lp || rate_in <= 0 || rate_in >= rate_out)
        return 1.0;
    /* 5376 = rate_in * 0.896 @ 6 kHz (the reference decoder measured hook/decode RMS ratio). */
    return (double) rate_out / ((double) rate_in * 5376.0 / 6000.0);
}

static void clamp_and_copy(int16_t *dst, const double *src, int len, double scale)
{
    /* sample + 0.5, then int16 limiter */
    for (int i = 0; i < len; i++) {
        double v = src[i];

        if (!isfinite(v))
            v = 0.0;
        v = v * scale + 0.5;
        if (v > 32767.0)
            dst[i] = 32767;
        else if (v < -32768.0)
            dst[i] = -32768;
        else
            dst[i] = (int16_t)lrint(v);
    }
}

/* --------------------------------------------------------------- */
/* Per-frame decode helper */
/* --------------------------------------------------------------- */
/**
 * Decode one LPEC frame from the bit reader.
 * Returns number of samples produced on success, AVERROR(EAGAIN) if
 * insufficient bits remain.
*/
static int decode_one_lpec_frame(LPECContext *s, GetBitContext *gb,
                                  double *output, int frame_size, int subfr_size)
{
    double cur_lsf1[LPEC_LSF_MAX];
    double cur_lsf2[LPEC_LSF_MAX];
    int lsf_idx[LPEC_NUM_LSF_CB_MAX];
    const int order = s->lpc_order;
    double *pcm = s->synth_buf + order;

    cur_lsf1[0] = cur_lsf2[0] = 0.0;
    lpec_decode_lsf_dump_seq = 0;
    if (get_bits_left(gb) < 2)
        return AVERROR(EAGAIN);

    if (s->pad_pending_start >= 0) {
        if (!lpec_frame_bits_complete(gb, s->pad_pending_mode, s->frame_bit_target,
                                      s->pad_pending_start) && !s->eof_draining)
            return AVERROR(EAGAIN);
        lpec_skip_to_frame_boundary(gb, s->pad_pending_mode, s->frame_bit_target,
                                    s->pad_pending_start);
        s->pad_pending_start = -1;
    }

    int frame_start = get_bits_count(gb);

    s->pending_frame_start = frame_start;

    int mode = get_bits(gb, 2);
    s->pending_frame_mode = mode;
    s->synth_half_flag = 0; /* clears ctx+0x128 each frame */
    {
        int bits_in_frame = s->frame_bit_target[mode];
        GetBitContext pk;

        s->next_frame_mode_peek = -1;
        if (bits_in_frame > 2 && get_bits_left(gb) >= bits_in_frame) {
            pk = *gb;
            skip_bits(&pk, bits_in_frame - 2);
            s->next_frame_mode_peek = show_bits(&pk, 2);
        }
    }

    if (getenv("LPEC_BITLOG") &&
        (!getenv("LPEC_BITLOG_FRAME") ||
         s->lpec_frame_idx == atoi(getenv("LPEC_BITLOG_FRAME")))) {
        fprintf(stderr, "LPEC_BITLOG enter f%d mode=%d bitpos=%d\n",
                s->lpec_frame_idx, mode, get_bits_count(gb) - 2);
    }

    if (getenv("LPEC_TRACE")) {
        static int fm;
        fprintf(stderr, "LPEC_TRACE frame=%d mode=%d bitpos=%d\n",
                fm++, mode, frame_start);
    }

    memset(pcm, 0, frame_size * sizeof(double));
    memset(output, 0, frame_size * sizeof(double));

    if (mode == 0) {
        if (get_bits_left(gb) < 1)
            return AVERROR(EAGAIN);
        int flag = get_bits(gb, 1);
        s->synth_half_flag = flag;

        int lsf_bits = s->num_lsf_cb * LPEC_LSF_BITS;

        if (flag == 0) {
            /* ctx+0x128==0 -> only (2); LSF-A stays prev */
            if (get_bits_left(gb) < lsf_bits)
                return AVERROR(EAGAIN);
            for (int i = 0; i < s->num_lsf_cb; i++)
                lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
            decode_lsf(s, lsf_idx, cur_lsf2);
            s->half_lsf_cb0[0] = -1; /* ctx+0x144 = -1 when flag==0 */
            s->half_lsf_cb0[1] = lsf_idx[0];
            memcpy(s->half_lsf_idx[2], lsf_idx, s->num_lsf_cb * sizeof(int));
            lpec_refresh_half_lsf_i16(s, 2);
            for (int i = 0; i <= order; i++)
                cur_lsf1[i] = (s->lsf1[i] + cur_lsf2[i]) * 0.5;
            lpec_avg_half_lsf_i16(s);
            if (getenv("LPEC_TRACE")) {
                static int lsf_tr;
                if (lsf_tr++ < 5) {
                    int ti;
                    fprintf(stderr, "LPEC_TRACE lsf frame=%d idx=%d,%d,%d\n",
                            lsf_tr - 1, lsf_idx[0], lsf_idx[1],
                            s->num_lsf_cb > 2 ? lsf_idx[2] : -1);
                    for (ti = 1; ti <= order; ti++)
                        fprintf(stderr, " lsf2[%d]=%.6f", ti, cur_lsf2[ti]);
                    fprintf(stderr, "\n");
                }
            }
        } else {
            if (get_bits_left(gb) < 2 * lsf_bits)
                return AVERROR(EAGAIN);
            for (int i = 0; i < s->num_lsf_cb; i++)
                lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
            decode_lsf(s, lsf_idx, cur_lsf1);
            s->half_lsf_cb0[0] = lsf_idx[0];
            memcpy(s->half_lsf_idx[1], lsf_idx, s->num_lsf_cb * sizeof(int));
            lpec_refresh_half_lsf_i16(s, 1);
            if (getenv("LPEC_BITLOG") && s->lpec_frame_idx == 0) {
                fprintf(stderr, "LPEC_BITLOG f%d lsf_h1 idx=%d,%d,%d,%d\n",
                        s->lpec_frame_idx, s->half_lsf_idx[1][0],
                        s->half_lsf_idx[1][1], s->half_lsf_idx[1][2],
                        s->half_lsf_idx[1][3]);
            }

            for (int i = 0; i < s->num_lsf_cb; i++)
                lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
            decode_lsf(s, lsf_idx, cur_lsf2);
            s->half_lsf_cb0[1] = lsf_idx[0];
            memcpy(s->half_lsf_idx[2], lsf_idx, s->num_lsf_cb * sizeof(int));
            lpec_refresh_half_lsf_i16(s, 2);
        }

        int voiced1, pitch1_lag = read_pitch(s, gb, &voiced1, &s->pitch_win_half1,
                                             &s->pitch_lag_off_half1);
        if (pitch1_lag < 0)
            return pitch1_lag;
        int voiced2, pitch2_lag = read_pitch(s, gb, &voiced2, &s->pitch_win_half2,
                                             &s->pitch_lag_off_half2);
        if (pitch2_lag < 0)
            return pitch2_lag;
        if (!voiced1)
            pitch1_lag = s->prev_pitch_lag;
        if (!voiced2)
            pitch2_lag = s->prev_pitch_lag;

        /* uVar4 = ctx+0x28 - bitpos after pitch, before pitch CB. */
        s->mode0_excit_rem = s->frame_bit_target[0] - (get_bits_count(gb) - frame_start);

        int pitch1_cb = read_pitch_cb_half(gb);
        int pitch2_cb;
        if (pitch1_cb < 0 && pitch1_cb != -1)
            return pitch1_cb;
        pitch2_cb = read_pitch_cb_half(gb);
        if (pitch2_cb < 0 && pitch2_cb != -1)
            return pitch2_cb;
        s->pitch_cb_half2_idx = pitch2_cb;
        s->pitch_cb_flag_h1   = pitch1_cb >= 0;
        s->pitch_cb_flag_h2   = pitch2_cb >= 0;

        if (getenv("LPEC_STAGE") && getenv("LPEC_STAGE_FRAME")) {
            int wf = atoi(getenv("LPEC_STAGE_FRAME"));
            if (s->lpec_frame_idx == wf) {
                fprintf(stderr, "LPEC_STAGE f%d pitch_cb h1=%d h2=%d prev=%d prev_idx=%d\n",
                        wf, pitch1_cb, pitch2_cb, s->prev_half2_pitch_cb,
                        s->pitch_cb_tbl_prev_idx);
            }
        }
        if (getenv("LPEC_BITLOG")) {
            fprintf(stderr,
                    "LPEC_BITLOG f%d mode0 flag=%d after_pitch_cb bitpos=%d rem=%d "
                    "p1=%d/%d p2=%d/%d cb=%d/%d\n",
                    s->lpec_frame_idx, flag, get_bits_count(gb),
                    s->mode0_excit_rem, pitch1_lag, s->pitch_win_half1,
                    pitch2_lag, s->pitch_win_half2, pitch1_cb, pitch2_cb);
        }

        {
            if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 1, pitch1_lag,
                                    s->pitch_lag_off_half1, s->pitch_win_half1,
                                    voiced1, s->pitch_tbl_half1, frame_start, mode,
                                    frame_size, order, pitch1_cb >= 0 ? 8 : 1,
                                    s->half_lsf_i16[1]) < 0)
                return AVERROR(EAGAIN);
            lpec_trace_stage(s, mode, "excit_h1_pre_qmf", s->excit_src, 0, subfr_size);
            if (getenv("LPEC_DUMP_CTX")) {
                char path[256];
                FILE *df;
                int qmf_dump = mode == 0 || mode == 2 ? s->qmf.desc512.len : subfr_size;

                snprintf(path, sizeof(path), "/tmp/lpec_f%d_preqmf_h1.bin",
                         s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(s->excit_src, sizeof(double), qmf_dump, df);
                    fclose(df);
                }
            }
            lpec_postprocess_excit(s, mode, s->excit_src, 0, subfr_size, 1);
            lpec_trace_stage(s, mode, "excit_h1_post_qmf", s->excit_src, 0, subfr_size);
            if (getenv("LPEC_DUMP_CTX")) {
                char path[256];
                FILE *df;
                int qmf_dump = mode == 0 || mode == 2 ? s->qmf.desc512.len : subfr_size;

                snprintf(path, sizeof(path), "/tmp/lpec_f%d_postqmf_h1.bin",
                         s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(s->excit_src, sizeof(double), qmf_dump, df);
                    fclose(df);
                }
            }
            {
                const char *inj = getenv("LPEC_INJECT_OLA");
                int inj_frame = -1;
                const char *ijf = getenv("LPEC_INJECT_FRAME");

                if (ijf)
                    inj_frame = atoi(ijf);
                if (inj && (inj_frame < 0 || s->lpec_frame_idx == inj_frame))
                    lpec_load_doubles(inj, lpec_ola_hist(s), subfr_size);
                if (getenv("LPEC_DUMP_CTX")) {
                    char path[256];
                    FILE *df;
                    snprintf(path, sizeof(path), "/tmp/lpec_f%d_pre04960_h1_ola.bin",
                             s->lpec_frame_idx);
                    df = fopen(path, "wb");
                    if (df) {
                        fwrite(lpec_ola_hist(s), sizeof(double), subfr_size, df);
                        fclose(df);
                    }
                    snprintf(path, sizeof(path), "/tmp/lpec_f%d_pre04960_h1_src.bin",
                             s->lpec_frame_idx);
                    df = fopen(path, "wb");
                    if (df) {
                        fwrite(s->excit_src, sizeof(double), subfr_size, df);
                        fclose(df);
                    }
                    snprintf(path, sizeof(path), "/tmp/lpec_f%d_mode0_tail.bin",
                             s->lpec_frame_idx);
                    df = fopen(path, "wb");
                    if (df) {
                        fwrite(s->mode0_shaped_tail, sizeof(double), subfr_size, df);
                        fclose(df);
                    }
                }
            }
            /* half 0: flags /0x13c, tbl /0x360 */
            lpec_apply_pitch_cb_mode0(s, s->frame_excit, s->excit_src, s->excit_src,
                                      subfr_size, NULL, s->prev_half2_pitch_cb,
                                      s->pitch_cb_flag_h1, s->pitch_cb_tbl_prev_idx,
                                      pitch1_cb, 1, 1);
            if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 2, pitch2_lag,
                                    s->pitch_lag_off_half2, s->pitch_win_half2,
                                    voiced2, s->pitch_tbl_half2, frame_start, mode,
                                    frame_size, order, pitch2_cb >= 0 ? 8 : 1,
                                    s->half_lsf_i16[2]) < 0)
                return AVERROR(EAGAIN);
            lpec_trace_stage(s, mode, "excit_h2_pre_qmf", s->excit_src, 0, subfr_size);
            if (getenv("LPEC_DUMP_CTX")) {
                char path[256];
                FILE *df;
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_preqmf_h2.bin",
                         s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(s->excit_src, sizeof(double), subfr_size, df);
                    fclose(df);
                }
            }
            lpec_postprocess_excit(s, mode, s->excit_src, 0, subfr_size, 2);
            lpec_trace_stage(s, mode, "excit_h2_post_qmf", s->excit_src, 0, subfr_size);
            if (getenv("LPEC_DUMP_CTX")) {
                char path[256];
                FILE *df;
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_postqmf_h2.bin",
                         s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(s->excit_src, sizeof(double), subfr_size, df);
                    fclose(df);
                }
            }
            if (getenv("LPEC_DUMP_CTX")) {
                char path[256];
                FILE *df;

                snprintf(path, sizeof(path), "/tmp/lpec_f%d_pre04960_h2_ola.bin",
                         s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(lpec_ola_hist(s), sizeof(double), subfr_size, df);
                    fclose(df);
                }
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_pre04960_h2_src.bin",
                         s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(s->excit_src, sizeof(double), subfr_size, df);
                    fclose(df);
                }
            }
            lpec_apply_pitch_cb_mode0(s, s->frame_excit + subfr_size,
                                      s->excit_src, s->excit_src,
                                      subfr_size, NULL, s->pitch_cb_flag_h1,
                                      s->pitch_cb_flag_h2, pitch1_cb, pitch2_cb,
                                      1, 2);
            lpec_trace_stage(s, mode, "frame_excit", s->frame_excit, 0, frame_size);
        }

        s->prev_pitch_lag = pitch2_lag;
        s->prev_lsf_cb_half2       = s->half_lsf_cb0[1];

        const double *synth_excit = lpec_prepare_synth_excit(s, mode, s->frame_excit,
                                                             s->frame_excit);

        if (lpec_dump_wants_frame(s))
            lpec_dump_lpc_frame = s->lpec_frame_idx;

        lpec_trace_stage(s, mode, "synth_excit", synth_excit, 0, frame_size);
        if (lpec_dump_wants_frame(s)) {
            char path[256];
            FILE *df;

            snprintf(path, sizeof(path), "/tmp/lpec_f%d_lpc_hist.bin", s->lpec_frame_idx);
            df = fopen(path, "wb");
            if (df) {
                fwrite(s->synth_buf, sizeof(double), order, df);
                fclose(df);
            }
            snprintf(path, sizeof(path), "/tmp/lpec_f%d_lsf1.bin", s->lpec_frame_idx);
            df = fopen(path, "wb");
            if (df) {
                fwrite(s->lsf1, sizeof(double), order + 1, df);
                fclose(df);
            }
            snprintf(path, sizeof(path), "/tmp/lpec_f%d_cur_lsf1.bin", s->lpec_frame_idx);
            df = fopen(path, "wb");
            if (df) {
                fwrite(cur_lsf1, sizeof(double), order + 1, df);
                fclose(df);
            }
            snprintf(path, sizeof(path), "/tmp/lpec_f%d_cur_lsf2.bin", s->lpec_frame_idx);
            df = fopen(path, "wb");
            if (df) {
                fwrite(cur_lsf2, sizeof(double), order + 1, df);
                fclose(df);
            }
        }
        lpec_dump_ctx_bufs(s, "excit_src");
        lpec_dump_ctx_bufs(s, "frame_excit");
        lpec_dump_ctx_bufs(s, "synth_work");
        lpec_dump_ctx_bufs(s, "excit_ola");

        if (getenv("LPEC_TRACE")) {
            static int n;
            if (n++ < 5) {
                double mx = 0, rms = 0;
                int ti;

                for (ti = 0; ti < frame_size; ti++) {
                    mx = fmax(mx, fabs(synth_excit[ti]));
                    rms += synth_excit[ti] * synth_excit[ti];
                }
                fprintf(stderr, "LPEC_TRACE synth_excit frame=%d max=%.6f rms=%.6f mode=%d flag=%d\n",
                        n - 1, mx, sqrt(rms / frame_size), mode, flag);
            }
        }

        if (s->synth_half_flag == 0) {
            /* (..., 0x10,...): lsf1 @ ctx+0x150, lsf2 @ ctx+0x260 */
            interpolated_lpc_synthesis(s->lsf1, cur_lsf2,
                                       synth_excit, pcm, 0, s->lpc_order,
                                       frame_size, lpec_lpc_segments_full(s));
            lpec_dump_f1c0_pass(s, 0, pcm, 0, frame_size);
        } else {
            int nseg = lpec_lpc_segments_half(s);

            /* flag==1: f1c0 lsf1->half1, half1->lsf2. */
            interpolated_lpc_synthesis(s->lsf1, cur_lsf1,
                                       synth_excit, pcm, 0, s->lpc_order,
                                       subfr_size, nseg);
            lpec_dump_f1c0_pass(s, 0, pcm, 0, subfr_size);
            if (!getenv("LPEC_SKIP_HALF2")) {
                interpolated_lpc_synthesis(cur_lsf1, cur_lsf2,
                                           synth_excit + subfr_size, pcm,
                                           subfr_size, s->lpc_order, subfr_size,
                                           nseg);
                lpec_dump_f1c0_pass(s, 1, pcm, subfr_size, subfr_size);
            }
        }

        memcpy(output, pcm, frame_size * sizeof(double));
        lpec_trace_stage(s, mode, "pcm", output, 0, frame_size);
        lpec_dump_f1c0_pass(s, 9, pcm, 0, frame_size);

        memcpy(s->prev_lsf, cur_lsf2, (order + 1) * sizeof(double));
        memcpy(s->lsf2, cur_lsf2, (order + 1) * sizeof(double));
        lpec_end_frame(s);
        if (getenv("LPEC_TRACE_LSF") && s->lpec_frame_idx == 6) {
            int ti;
            fprintf(stderr, "LPEC_TRACE_LSF f6 end lsf2 rolled lsf1[1]=%.6f\n", s->lsf1[1]);
            for (ti = 1; ti <= order; ti++)
                fprintf(stderr, " lsf2[%d]=%.6f\n", ti, cur_lsf2[ti]);
        }
        lpec_save_frame(s, output, s->frame_excit, frame_size, mode);
        if (getenv("LPEC_TRACE")) {
            static int tr;
            if (tr++ < 5) {
                double mx = 0, rms = 0;
                int ti;

                for (ti = 0; ti < frame_size; ti++) {
                    mx = fmax(mx, fabs(output[ti]));
                    rms += output[ti] * output[ti];
                }
                fprintf(stderr, "LPEC_TRACE pcm frame=%d max=%.6f rms=%.6f first=%.6f\n",
                        tr - 1, mx, sqrt(rms / frame_size), output[0]);
            }
        }
        return lpec_finish_frame(s, gb, mode, frame_start, frame_size);

    } else if (mode == 2) {
        /*
 * Mode 2: (2), pitchx2, avg LSF, 05d50+ab80+f640x2.
 * No flag bit, no pitch CB; f640 uses ctx+0x84 shape.
 *: clears ctx+0x13c/0x140 pitch CB flags before ab80.
*/
        int lsf_bits = s->num_lsf_cb * LPEC_LSF_BITS;

        s->pitch_cb_flag_h1 = 0;
        s->pitch_cb_flag_h2 = 0;

        if (get_bits_left(gb) < lsf_bits)
            return AVERROR(EAGAIN);
        for (int i = 0; i < s->num_lsf_cb; i++)
            lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
        decode_lsf(s, lsf_idx, cur_lsf2);
        s->half_lsf_cb0[1] = lsf_idx[0];
        memcpy(s->half_lsf_idx[2], lsf_idx, s->num_lsf_cb * sizeof(int));
        lpec_refresh_half_lsf_i16(s, 2);
        for (int i = 0; i <= order; i++)
            cur_lsf1[i] = (s->lsf1[i] + cur_lsf2[i]) * 0.5;
        lpec_avg_half_lsf_i16(s);
        s->half_lsf_cb0[0] = -1;

        int voiced1, pitch1_lag = read_pitch(s, gb, &voiced1, &s->pitch_win_half1,
                                             &s->pitch_lag_off_half1);
        if (pitch1_lag < 0)
            return pitch1_lag;
        int voiced2, pitch2_lag = read_pitch(s, gb, &voiced2, &s->pitch_win_half2,
                                             &s->pitch_lag_off_half2);
        if (pitch2_lag < 0)
            return pitch2_lag;
        if (!voiced1)
            pitch1_lag = s->prev_pitch_lag;
        if (!voiced2)
            pitch2_lag = s->prev_pitch_lag;

        if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 1, pitch1_lag,
                                s->pitch_lag_off_half1, s->pitch_win_half1,
                                voiced1, s->pitch_tbl_half1, frame_start, mode,
                                frame_size, order, 0, s->half_lsf_i16[1]) < 0)
            return AVERROR(EAGAIN);
        lpec_postprocess_excit(s, mode, s->excit_src, 0, s->qmf_seg_m2_h1, 1);
        /* mode-2 h1: f640 overlap/copy = ctx+0xc (384 @ 8 kHz). */
        lpec_excit_window_f640(s, 1, s->frame_excit, s->excit_src, lpec_ola_hist(s),
                               lpec_f640_window(s, mode), s->qmf_seg_m2_h1,
                               s->qmf_seg_m2_h1);
        if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 2, pitch2_lag,
                                s->pitch_lag_off_half2, s->pitch_win_half2,
                                voiced2, s->pitch_tbl_half2, frame_start, mode,
                                frame_size, order, 0, s->half_lsf_i16[2]) < 0)
            return AVERROR(EAGAIN);
        lpec_postprocess_excit(s, mode, s->excit_src, 0, frame_size, 2);
        /* Mode-2 h2: dst+ctx+0xc, overlap=ctx+0xc, copy=ctx+0x8 (512). */
        {
            const char *inj = getenv("LPEC_INJECT_OLA");
            const char *ijf = getenv("LPEC_INJECT_FRAME");
            int inj_frame = ijf ? atoi(ijf) : -1;

            if (inj && (inj_frame < 0 || s->lpec_frame_idx == inj_frame))
                lpec_load_doubles(inj, lpec_ola_hist(s), frame_size);
        }
        lpec_excit_window_f640(s, 2, s->frame_excit + s->qmf_seg_m2_h1, s->excit_src,
                               lpec_ola_hist(s), lpec_f640_window(s, mode),
                               s->qmf_seg_m2_h1, frame_size);
        if (getenv("LPEC_DUMP_HIST_CHAIN") && s->next_frame_mode_peek == 1) {
            char path[256];
            FILE *df;

            snprintf(path, sizeof(path), "/tmp/lpec_f%d_hist_chain_post_m2.bin",
                     s->lpec_frame_idx);
            df = fopen(path, "wb");
            if (df) {
                fwrite(lpec_ola_hist(s), sizeof(double), frame_size, df);
                fclose(df);
            }
        }
        s->prev_pitch_lag = pitch2_lag;

        lpec_maybe_inject_frame_excit(s);
        memcpy(output, s->frame_out, frame_size * sizeof(double));
        {
            const double *synth_excit = lpec_prepare_synth_excit(s, mode, s->frame_excit,
                                                                 s->frame_excit);

            if (lpec_dump_wants_frame(s)) {
                char path[256];
                FILE *df;

                lpec_dump_lpc_frame = s->lpec_frame_idx;
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_lpc_hist.bin",
                         s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(s->synth_buf, sizeof(double), order, df);
                    fclose(df);
                }
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_lsf1.bin", s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(s->lsf1, sizeof(double), order + 1, df);
                    fclose(df);
                }
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_cur_lsf2.bin", s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(cur_lsf2, sizeof(double), order + 1, df);
                    fclose(df);
                }
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_synth_excit.bin",
                         s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(synth_excit, sizeof(double), frame_size, df);
                    fclose(df);
                }
            }

            /* mode 2: prev=ctx+0x150 (lsf1), curr=ctx+0x260 (lsf2); not avg. */
            interpolated_lpc_synthesis(s->lsf1, cur_lsf2,
                                       synth_excit, pcm, 0, s->lpc_order,
                                       frame_size, lpec_lpc_segments_full(s));
            lpec_dump_f1c0_pass(s, 0, pcm, 0, frame_size);
        }
        memcpy(output, pcm, frame_size * sizeof(double));
        memcpy(s->prev_lsf, cur_lsf2, (order + 1) * sizeof(double));
        memcpy(s->lsf2, cur_lsf2, (order + 1) * sizeof(double));
        lpec_end_frame(s);
        lpec_save_frame(s, output, s->frame_excit, frame_size, mode);
        lpec_dump_ctx_bufs(s, "excit_src");
        lpec_dump_ctx_bufs(s, "frame_excit");
        lpec_dump_ctx_bufs(s, "synth_work");
        lpec_dump_ctx_bufs(s, "excit_ola");
        return lpec_finish_frame(s, gb, mode, frame_start, frame_size);

    } else if (mode == 1) {
        /*
 * Mode 1: second subframe only.
 * Reads new LSF-B; LSF-A is ctx lsf1 (rolled from prior frame lsf2 @ ).
*/
        if (get_bits_left(gb) < s->num_lsf_cb * LPEC_LSF_BITS)
            return AVERROR(EAGAIN);
        for (int i = 0; i < s->num_lsf_cb; i++)
            lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
        decode_lsf(s, lsf_idx, cur_lsf2);
        s->half_lsf_cb0[1] = lsf_idx[0];
        memcpy(s->half_lsf_idx[2], lsf_idx, s->num_lsf_cb * sizeof(int));
        lpec_refresh_half_lsf_i16(s, 2);
        memcpy(cur_lsf1, s->lsf1, (order + 1) * sizeof(double));

        if (getenv("LPEC_TRACE_LSF") && s->lpec_frame_idx == 7) {
            int ti;
            fprintf(stderr, "LPEC_TRACE_LSF f7 mode1 idx=%d,%d,%d,%d\n",
                    lsf_idx[0], lsf_idx[1], lsf_idx[2], lsf_idx[3]);
            for (ti = 1; ti <= order; ti++)
                fprintf(stderr, " lsf1[%d]=%.6f lsf2[%d]=%.6f\n",
                        ti, cur_lsf1[ti], ti, cur_lsf2[ti]);
        }

        int voiced, pitch_lag = read_pitch(s, gb, &voiced, &s->pitch_win_half2,
                                           &s->pitch_lag_off_half2);
        if (pitch_lag < 0)
            return pitch_lag;
        if (!voiced)
            pitch_lag = s->prev_pitch_lag;
        if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 2, pitch_lag,
                                s->pitch_lag_off_half2, s->pitch_win_half2,
                                voiced, s->pitch_tbl_half2, frame_start, mode,
                                frame_size, order, 0, s->half_lsf_i16[2]) < 0)
            return AVERROR(EAGAIN);
        s->prev_pitch_lag = pitch_lag;
        if (lpec_dump_wants_frame(s)) {
            char path[256];
            FILE *df;

            snprintf(path, sizeof(path), "/tmp/lpec_f%d_preqmf_m1.bin",
                     s->lpec_frame_idx);
            df = fopen(path, "wb");
            if (df) {
                fwrite(s->excit_src, sizeof(double), 768, df);
                fclose(df);
            }
        }
        {
            const char *inj = getenv("LPEC_INJECT_EXCIT_SRC");
            const char *ijf = getenv("LPEC_INJECT_FRAME");

            if (inj && ijf && s->lpec_frame_idx == atoi(ijf)) {
                int nload = frame_size;

                if (s->is_xaudio_lp && s->frame_size < LPEC_FRAME_16K)
                    nload = s->qmf.desc768.len;
                lpec_load_doubles(inj, s->excit_src, nload);
            }
        }
        lpec_postprocess_excit(s, mode, s->excit_src, 0, lpec_mode1_ab80_len(s), 2);
        if (lpec_dump_wants_frame(s)) {
            char path[256];
            FILE *df;

            snprintf(path, sizeof(path), "/tmp/lpec_f%d_postqmf_m1.bin",
                     s->lpec_frame_idx);
            df = fopen(path, "wb");
            if (df) {
                fwrite(s->excit_src, sizeof(double), 768, df);
                fclose(df);
            }
        }
        if (getenv("LPEC_STAGE") && s->lpec_frame_idx == 7) {
            const double *h2 = s->excit_src + subfr_size;
            fprintf(stderr, "LPEC_STAGE f7 raw_excit_h2 first4=%.1f,%.1f,%.1f,%.1f pitch_win=%d lag_off=%d\n",
                    h2[0], h2[1], h2[2], h2[3],
                    s->pitch_win_half2, s->pitch_lag_off_half2);
        }

        {
            const char *inj = getenv("LPEC_INJECT_OLA");
            const char *ijf = getenv("LPEC_INJECT_FRAME");
            int inj_frame = -1;

            if (ijf)
                inj_frame = atoi(ijf);
            if (inj && (inj_frame < 0 || s->lpec_frame_idx == inj_frame))
                lpec_load_doubles(inj, lpec_ola_hist(s), frame_size);
        }

        {
            const double *synth_excit = lpec_prepare_synth_excit(s, mode, s->excit_src,
                                                                 s->frame_excit);
            int nseg = lpec_lpc_segments_half(s);

            if (lpec_dump_wants_frame(s)) {
                char path[256];
                FILE *df;

                lpec_dump_lpc_frame = s->lpec_frame_idx;
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_lpc_hist.bin",
                         s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(s->synth_buf, sizeof(double), order, df);
                    fclose(df);
                }
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_lsf1.bin", s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(s->lsf1, sizeof(double), order + 1, df);
                    fclose(df);
                }
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_cur_lsf1.bin", s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(cur_lsf1, sizeof(double), order + 1, df);
                    fclose(df);
                }
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_cur_lsf2.bin", s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(cur_lsf2, sizeof(double), order + 1, df);
                    fclose(df);
                }
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_synth_excit.bin",
                         s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(synth_excit, sizeof(double), frame_size, df);
                    fclose(df);
                }
            }

            /* the reference: ctx+0x128==0 -> full-frame f1c0 for modes 1/3. */
            if (getenv("LPEC_MODE1_H2ISO")) {
                memcpy(pcm, s->frame_out, subfr_size * sizeof(double));
                lpec_mode1_half2_isolated(cur_lsf1, cur_lsf2, synth_excit,
                                          s->synth_buf, pcm, order,
                                          frame_size, subfr_size, nseg);
                memcpy(output, s->frame_out, frame_size * sizeof(double));
                memcpy(output + subfr_size, pcm + subfr_size,
                       subfr_size * sizeof(double));
            } else if (getenv("LPEC_MODE1_INPLACE")) {
                memcpy(output, s->frame_out, frame_size * sizeof(double));
                memcpy(pcm, output, subfr_size * sizeof(double));
                lpec_interpolated_lpc_inplace(cur_lsf1, cur_lsf2, synth_excit,
                                              s->synth_buf, pcm, order,
                                              frame_size, nseg);
                memcpy(output + subfr_size, pcm + subfr_size,
                       subfr_size * sizeof(double));
            } else if (getenv("LPEC_MODE1_HALF2") &&
                       (!getenv("LPEC_MODE1_HALF2_FROM") ||
                        s->lpec_frame_idx >= atoi(getenv("LPEC_MODE1_HALF2_FROM")))) {
                memcpy(output, s->frame_out, frame_size * sizeof(double));
                memcpy(pcm, output, subfr_size * sizeof(double));
                lpec_mode1_half2_synthesis(cur_lsf1, cur_lsf2, synth_excit,
                                           s->synth_buf, pcm, s->frame_out,
                                           order, frame_size, subfr_size, nseg);
                memcpy(output + subfr_size, pcm + subfr_size,
                       subfr_size * sizeof(double));
            } else {
                /* ctx+0x128==0 -> full-frame f1c0 (all rates). */
                interpolated_lpc_synthesis(cur_lsf1, cur_lsf2, synth_excit, pcm,
                                           0, s->lpc_order, frame_size,
                                           lpec_lpc_segments_full(s));
                memcpy(output, pcm, frame_size * sizeof(double));
            }
            lpec_dump_f1c0_pass(s, 0, output, 0, frame_size);
            lpec_trace_stage(s, mode, "pcm", output, 0, frame_size);
        }
        memcpy(s->prev_lsf, cur_lsf2, (order + 1) * sizeof(double));
        memcpy(s->lsf2, cur_lsf2, (order + 1) * sizeof(double));
        lpec_end_frame(s);
        lpec_save_frame(s, output,
                        s->is_xaudio_lp ? s->excit_work + LPEC_WORK_SLACK
                                        : s->frame_excit,
                        frame_size, mode);
        lpec_dump_ctx_bufs(s, "excit_src");
        lpec_dump_ctx_bufs(s, "frame_excit");
        lpec_dump_ctx_bufs(s, "synth_work");
        lpec_dump_ctx_bufs(s, "excit_ola");
        return lpec_finish_frame(s, gb, mode, frame_start, frame_size);

    } else if (mode == 3) {
        /*
 * Mode 3: second subframe only (route 2/3).
 * reference decoder: (2) before.
*/
        if (get_bits_left(gb) < s->num_lsf_cb * LPEC_LSF_BITS)
            return AVERROR(EAGAIN);
        for (int i = 0; i < s->num_lsf_cb; i++)
            lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
        decode_lsf(s, lsf_idx, cur_lsf2);
        s->half_lsf_cb0[1] = lsf_idx[0];
        memcpy(s->half_lsf_idx[2], lsf_idx, s->num_lsf_cb * sizeof(int));
        lpec_refresh_half_lsf_i16(s, 2);
        memcpy(cur_lsf1, s->lsf1, (order + 1) * sizeof(double));

        int voiced, pitch_lag = read_pitch(s, gb, &voiced, &s->pitch_win_half2,
                                           &s->pitch_lag_off_half2);
        if (pitch_lag < 0)
            return pitch_lag;
        if (!voiced)
            pitch_lag = s->prev_pitch_lag;
        if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 2, pitch_lag,
                                s->pitch_lag_off_half2, s->pitch_win_half2,
                                voiced, s->pitch_tbl_half2, frame_start, mode,
                                frame_size, order, 0, s->half_lsf_i16[2]) < 0)
            return AVERROR(EAGAIN);
        s->prev_pitch_lag = pitch_lag;
        lpec_postprocess_excit(s, mode, s->excit_src, 0, frame_size, 2);

        {
            const char *inj = getenv("LPEC_INJECT_OLA");
            const char *ijf = getenv("LPEC_INJECT_FRAME");
            int inj_frame = -1;

            if (ijf)
                inj_frame = atoi(ijf);
            if (inj && (inj_frame < 0 || s->lpec_frame_idx == inj_frame))
                lpec_load_doubles(inj, lpec_ola_hist(s), frame_size);

            const double *synth_excit = lpec_prepare_synth_excit(s, mode, s->excit_src,
                                                                 s->frame_excit);

            if (lpec_dump_wants_frame(s)) {
                char path[256];
                FILE *df;

                lpec_dump_lpc_frame = s->lpec_frame_idx;
                snprintf(path, sizeof(path), "/tmp/lpec_f%d_lpc_hist.bin",
                         s->lpec_frame_idx);
                df = fopen(path, "wb");
                if (df) {
                    fwrite(s->synth_buf, sizeof(double), order, df);
                    fclose(df);
                }
            }

            /* the reference: ctx+0x128==0 -> full-frame f1c0 for modes 1/3. */
            interpolated_lpc_synthesis(cur_lsf1, cur_lsf2, synth_excit, pcm, 0,
                                       s->lpc_order, frame_size,
                                       lpec_lpc_segments_full(s));
            memcpy(output, pcm, frame_size * sizeof(double));
            lpec_dump_f1c0_pass(s, 0, output, 0, frame_size);
        }
        memcpy(s->prev_lsf, cur_lsf2, (order + 1) * sizeof(double));
        memcpy(s->lsf2, cur_lsf2, (order + 1) * sizeof(double));
        lpec_end_frame(s);
        lpec_save_frame(s, output,
                        s->is_xaudio_lp ? s->excit_work + LPEC_WORK_SLACK
                                        : s->frame_excit,
                        frame_size, mode);
        lpec_dump_ctx_bufs(s, "excit_src");
        lpec_dump_ctx_bufs(s, "frame_excit");
        lpec_dump_ctx_bufs(s, "synth_work");
        lpec_dump_ctx_bufs(s, "excit_ola");
        return lpec_finish_frame(s, gb, mode, frame_start, frame_size);

    } else {
        /* Unknown mode - stop this packet */
        return AVERROR(EAGAIN);
    }
}


/* --------------------------------------------------------------- */
/* Main decode function */
/* --------------------------------------------------------------- */

static int lpec_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                             int *got_frame_ptr, AVPacket *avpkt)
{
    LPECContext *s   = avctx->priv_data;
    int buf_size     = avpkt->size;
    int ret;
    int draining = 0;

    int frame_size = s->frame_size;
    int subfr_size = s->subfr_size;

    if (buf_size == 0) {
        draining = 1;
        s->eof_draining = 1;
        if (s->eof_flush_done) {
            *got_frame_ptr = 0;
            return 0;
        }
        lpec_feed_chunks(s);
    } else {
        s->eof_draining = 0;
    }

    const uint8_t *src = avpkt->data;
    int payload_size = buf_size;
    int total_samples = 0;
    int num_frames = 0;

    if (!draining && getenv("LPEC_TRACE") && s->frame_count < 3) {
        int n = payload_size < 16 ? payload_size : 16;
        fprintf(stderr, "LPEC_TRACE pkt%d size=%d payload:",
                s->frame_count, payload_size);
        for (int i = 0; i < n; i++)
            fprintf(stderr, " %02x", src[i]);
        fprintf(stderr, " stream_len=%d\n", s->stream_len);
    }

    if (!draining && payload_size > (int)sizeof(s->carry_buf) - s->chunk_carry_len)
        return AVERROR(EINVAL);

    total_samples = lpec_decode_from_stream(s, s->scratch, LPEC_SCRATCH_SAMPLES,
                                            frame_size, subfr_size,
                                            LPEC_MAX_FRAMES_PKT, &num_frames);

    if (!draining) {
        memcpy(s->carry_buf + s->chunk_carry_len, src, payload_size);
        s->chunk_carry_len += payload_size;
        lpec_feed_chunks(s);
    }

    {
        int nf2 = 0;
        int n2 = lpec_decode_from_stream(s, s->scratch + total_samples,
                                         LPEC_SCRATCH_SAMPLES - total_samples,
                                         frame_size, subfr_size,
                                         LPEC_MAX_FRAMES_PKT - num_frames, &nf2);

        total_samples += n2;
        num_frames    += nf2;
    }

    if (total_samples <= 0) {
        if (draining) {
            s->stream_len      = 0;
            s->stream_bit_off  = 0;
            s->chunk_carry_len = 0;
            s->eof_flush_done  = 1;
            s->eof_draining    = 0;
        }
        *got_frame_ptr = 0;
        return draining ? 0 : buf_size;
    }

    frame->nb_samples = total_samples;
    if (getenv("LPEC_TRACE") && total_samples > 0) {
        static int out_tr;
        if (out_tr++ < 8)
            fprintf(stderr, "LPEC_OUT #%d idx=%d nb=%d nf=%d\n",
                    out_tr - 1, s->lpec_frame_idx, total_samples, num_frames);
    }
    int16_t *dst;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    dst = (int16_t *)frame->data[0];

    clamp_and_copy(dst, s->scratch, total_samples, lpec_xa_output_gain(s));

    s->frame_count++;
    if (draining && s->stream_len == 0 && s->chunk_carry_len == 0)
        s->eof_flush_done = 1;
    *got_frame_ptr = 1;
    return draining ? 0 : buf_size;
}

/* --------------------------------------------------------------- */
/* Codec registration */
/* --------------------------------------------------------------- */

const FFCodec ff_lpec_decoder = {
    .p.name      = "lpec",
    CODEC_LONG_NAME("Sony LPEC"),
    .p.type      = AVMEDIA_TYPE_AUDIO,
    .p.id        = AV_CODEC_ID_LPEC,
    .priv_data_size = sizeof(LPECContext),
    .init        = lpec_decode_init,
    .close       = lpec_decode_close,
    FF_CODEC_DECODE_CB(lpec_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_VARIABLE_FRAME_SIZE |
                       AV_CODEC_CAP_DELAY,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16),
};


/*
 * LPEC LPC synthesis helpers (mode-1 half2 path).
 * lsf_to_lpc(), LPEC_LSF_MAX and LPEC_LPC_ORDER_MAX are defined earlier.
 */
void lpec_mode1_half2_synthesis(const double *prev_lsf, const double *curr_lsf,
                                const double *excit, double *synth_buf,
                                double *output, const double *frame_out,
                                int order, int frame_size, int subfr_size,
                                int num_seg)
{
    double interp_lsf[LPEC_LSF_MAX];
    double lpc[LPEC_LPC_ORDER_MAX + 2];
    const double step = num_seg > 0 ? 1.0 / (double)num_seg : 1.0;
    int out_pos = 0;
    int i;

    memcpy(output + subfr_size, frame_out + subfr_size, subfr_size * sizeof(double));
    for (i = 0; i < subfr_size; i++)
        synth_buf[order + subfr_size + i] = output[subfr_size + i];

    if (getenv("LPEC_STAGE") && getenv("LPEC_SYNTH_DBG")) {
        double e = 0, ex = 0;
        for (i = 0; i < subfr_size; i++) {
            e += synth_buf[order + i] * synth_buf[order + i];
            ex += excit[subfr_size + i] * excit[subfr_size + i];
        }
        fprintf(stderr, "LPEC_STAGE half2 prefill synth rms=%.1f excit_h2 rms=%.1f\n",
                sqrt(e / subfr_size), sqrt(ex / subfr_size));
    }

    for (int seg = 0; seg <= num_seg; seg++) {
        double w_curr = (double)seg * step;
        double w_prev = 1.0 - w_curr;
        int div = (seg == 0 || seg == num_seg) ? num_seg * 2 : num_seg;
        int seg_len = frame_size / div;

        for (i = 0; i <= order; i++)
            interp_lsf[i] = prev_lsf[i] * w_prev + curr_lsf[i] * w_curr;

        lsf_to_lpc(interp_lsf, lpc, order);

        for (int n = 0; n < seg_len; n++) {
            int pos = out_pos + n;
            double s;

            if (pos < subfr_size)
                continue;

            s = excit[pos];
            for (int k = 1; k <= order; k++)
                s -= lpc[k] * synth_buf[order + pos - k];

            if (!isfinite(s))
                s = 0.0;

            synth_buf[order + pos] = s;
        }
        out_pos += seg_len;
    }

    memcpy(output + subfr_size, synth_buf + order + subfr_size,
           subfr_size * sizeof(double));
    memcpy(synth_buf, synth_buf + order + frame_size - order,
           order * sizeof(double));
}

void lpec_interpolated_lpc_inplace(const double *prev_lsf, const double *curr_lsf,
                                   const double *excit, double *synth_buf,
                                   double *output, int order, int frame_size,
                                   int num_seg)
{
    double interp_lsf[LPEC_LSF_MAX];
    double lpc[LPEC_LPC_ORDER_MAX + 2];
    const double step = num_seg > 0 ? 1.0 / (double)num_seg : 1.0;
    int out_pos = 0;

    for (int seg = 0; seg <= num_seg; seg++) {
        double w_curr = (double)seg * step;
        double w_prev = 1.0 - w_curr;
        int div = (seg == 0 || seg == num_seg) ? num_seg * 2 : num_seg;
        int seg_len = frame_size / div;

        for (int i = 0; i <= order; i++)
            interp_lsf[i] = prev_lsf[i] * w_prev + curr_lsf[i] * w_curr;

        lsf_to_lpc(interp_lsf, lpc, order);

        for (int n = 0; n < seg_len; n++) {
            int pos = out_pos + n;
            double s = excit[pos];

            for (int k = 1; k <= order; k++) {
                if (pos >= k)
                    s -= lpc[k] * output[pos - k];
                else
                    s -= lpc[k] * synth_buf[order + pos - k];
            }

            if (!isfinite(s))
                s = 0.0;

            output[pos] = s;
            synth_buf[order + pos] = s;
        }
        out_pos += seg_len;
    }

    memcpy(synth_buf, synth_buf + order + frame_size - order,
           order * sizeof(double));
}

void lpec_mode1_half2_isolated(const double *prev_lsf, const double *curr_lsf,
                               const double *excit, double *synth_buf,
                               double *output, int order, int frame_size,
                               int subfr_size, int num_seg)
{
    double interp_lsf[LPEC_LSF_MAX];
    double lpc[LPEC_LPC_ORDER_MAX + 2];
    const double step = num_seg > 0 ? 1.0 / (double)num_seg : 1.0;
    int out_pos = 0;

    memset(synth_buf, 0, (order + frame_size) * sizeof(double));

    for (int seg = 0; seg <= num_seg; seg++) {
        double w_curr = (double)seg * step;
        double w_prev = 1.0 - w_curr;
        int div = (seg == 0 || seg == num_seg) ? num_seg * 2 : num_seg;
        int seg_len = frame_size / div;

        for (int i = 0; i <= order; i++)
            interp_lsf[i] = prev_lsf[i] * w_prev + curr_lsf[i] * w_curr;

        lsf_to_lpc(interp_lsf, lpc, order);

        for (int n = 0; n < seg_len; n++) {
            int pos = out_pos + n;
            double s;

            if (pos < subfr_size)
                continue;

            s = excit[pos];
            for (int k = 1; k <= order; k++)
                s -= lpc[k] * synth_buf[order + pos - k];

            if (!isfinite(s))
                s = 0.0;

            synth_buf[order + pos] = s;
        }
        out_pos += seg_len;
    }

    memcpy(output + subfr_size, synth_buf + order + subfr_size,
           subfr_size * sizeof(double));
    memcpy(synth_buf, synth_buf + order + frame_size - order,
           order * sizeof(double));
}
