/*
 * Sony TRC audio decoder
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
 * Sony TRC audio decoder.
 *
 * TRC is a CELP speech codec used by Sony IC recorders inside the MSV/DVF
 * container.  A frame always produces 160 samples; the packet's first byte
 * carries a rate prefix selecting packet size, subframe geometry and frame
 * type (voiced CELP, comfort noise, or silence).
 */

#include <string.h>

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "trc_unpack.h"
#include "trcdata.h"
#include "trc_dsp.h"
#include "trc_lsp.h"
#include "trc_excit.h"
#include "trc_synth.h"

#define TRC_EXC_SIZE  0x90
#define TRC_ORDER     10

typedef struct TRCContext {
    int16_t  gain_state;   /* fixed-codebook gain MA state */
    int16_t  agc_mem;      /* AGC / de-emphasis gain memory */
    uint32_t noise_state;  /* noise generator LCG state */
    int16_t  noise_gain_idx;
    int16_t  noise_gain_val;
    int      prev_type;    /* previous frame TYPE */

    int16_t  lsf_hist[TRC_ORDER];                    /* previous-frame LSF */
    int16_t  lsf_cur[TRC_ORDER];                     /* current-frame LSF */
    int16_t  exc_hist[TRC_EXC_SIZE];                 /* adaptive-codebook history */
    int16_t  exc_buf[TRC_EXC_SIZE + TRC_FRAME_SAMPLES];

    int16_t  synth_hist[TRC_ORDER];                  /* LPC synthesis filter state */
    int16_t  formant_hist[TRC_ORDER];                /* postfilter formant feedback */

    TRCFrameParams cur;
} TRCContext;

static av_cold int trc_decode_init(AVCodecContext *avctx)
{
    TRCContext *s = avctx->priv_data;

    av_channel_layout_uninit(&avctx->ch_layout);
    avctx->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
    avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    if (avctx->sample_rate <= 0)
        avctx->sample_rate = 8000;

    memcpy(s->lsf_hist, trc_lsf_mean, sizeof(s->lsf_hist));
    memcpy(s->lsf_cur,  trc_lsf_mean, sizeof(s->lsf_cur));
    memset(s->exc_hist, 0, sizeof(s->exc_hist));
    memset(s->exc_buf, 0, sizeof(s->exc_buf));
    memset(s->synth_hist, 0, sizeof(s->synth_hist));
    memset(s->formant_hist, 0, sizeof(s->formant_hist));

    s->gain_state     = 0;
    s->agc_mem        = 0x1000;
    s->noise_state    = 0;
    s->noise_gain_idx = 0;
    s->noise_gain_val = trc_gain_quant[0];
    s->prev_type      = 1;

    return 0;
}

/* TYPE 0/2: fill the excitation with scaled comfort noise. */
static void trc_fill_noise_excitation(TRCContext *s)
{
    int16_t *cur = s->exc_buf + TRC_EXC_SIZE;
    int k;

    for (k = 0; k < TRC_FRAME_SAMPLES / 40; k++)
        trc_noise_fill(cur, k * 40, &s->noise_gain_idx, &s->noise_gain_val,
                       &s->noise_state);

    memset(s->exc_hist, 0, sizeof(s->exc_hist));
}

static void trc_build_excitation(TRCContext *s, const TRCFrameParams *fp)
{
    int16_t *cur = s->exc_buf + TRC_EXC_SIZE;
    int k, off = 0;

    memcpy(s->exc_buf, s->exc_hist, sizeof(s->exc_hist));

    if (fp->type == 0) {
        memcpy(s->lsf_cur, s->lsf_hist, sizeof(s->lsf_cur));
        trc_fill_noise_excitation(s);
        return;
    }

    trc_lsf_decode(fp->lsf_idx, s->lsf_hist, s->lsf_cur);

    if (fp->type == 2) {
        if (s->prev_type == 1) {
            s->noise_gain_idx = fp->aux_gain;
            s->noise_state    = 0x5ba0;
            s->noise_gain_val = trc_gain_quant[fp->aux_gain];
        }
        trc_fill_noise_excitation(s);
        return;
    }

    /* TYPE 1: voiced CELP, adaptive + fixed codebook per subframe */
    for (k = 0; k < fp->nsub; k++) {
        trc_adaptive_excit(cur + off, &fp->sf[k], s->exc_buf, fp->sublen);
        trc_fixed_excit(cur + off, &fp->sf[k], fp->rate, fp->sublen,
                        &s->gain_state);
        off += fp->sublen;
    }

    memcpy(s->exc_hist, cur + off - TRC_EXC_SIZE, sizeof(s->exc_hist));

    /*
     * Long-term pitch postfilter: applied to the excitation used for
     * synthesis, AFTER the adaptive-codebook history has been saved (the
     * saved history stays pre-filter, matching the reference decoder).
     */
    trc_lt_pitch_filter(s->exc_buf, fp);
}

/* LPC-synthesise the built excitation and post-filter to PCM. */
static void trc_synthesize_frame(TRCContext *s, const TRCFrameParams *fp, int16_t *out)
{
    const int16_t *exc = s->exc_buf + TRC_EXC_SIZE;
    int16_t syn[TRC_ORDER + TRC_FRAME_SAMPLES];
    int k;

    memcpy(syn, s->synth_hist, TRC_ORDER * sizeof(int16_t));

    for (k = 0; k < fp->nsub; k++) {
        int16_t lpc[TRC_ORDER];
        int off = k * fp->sublen;
        int16_t *sub = syn + TRC_ORDER + off;

        trc_lsf_interpolate(lpc, s->lsf_cur, s->lsf_hist, fp->nsub, k);
        trc_lsf_to_lpc(lpc);
        memcpy(sub, exc + off, fp->sublen * sizeof(int16_t));
        trc_lpc_synth(sub, lpc, fp->sublen);
        trc_postfilter(out + off, sub, lpc, s->formant_hist,
                       &s->agc_mem, fp->rate, fp->sublen);
    }

    memcpy(s->synth_hist,
           syn + TRC_ORDER + fp->nsub * fp->sublen - TRC_ORDER,
           TRC_ORDER * sizeof(int16_t));
    memcpy(s->lsf_hist, s->lsf_cur, sizeof(s->lsf_hist));
}

static int trc_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                            int *got_frame_ptr, AVPacket *avpkt)
{
    TRCContext *s = avctx->priv_data;
    int16_t *dst;
    int i, ret, consumed;

    consumed = trc_unpack_frame(avpkt->data, avpkt->size, &s->cur);
    if (consumed < 0)
        return consumed;

    trc_build_excitation(s, &s->cur);

    frame->nb_samples = TRC_FRAME_SAMPLES;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    dst = (int16_t *)frame->data[0];
    trc_synthesize_frame(s, &s->cur, dst);

    /* Output scaling used by the reference decoder / WAV output. */
    for (i = 0; i < TRC_FRAME_SAMPLES; i++)
        dst[i] = trc_clip_int16((int32_t)dst[i] * 16);

    s->prev_type = s->cur.type;

    *got_frame_ptr = 1;
    return avpkt->size;
}

const FFCodec ff_trc_decoder = {
    .p.name         = "trc",
    CODEC_LONG_NAME("Sony TRC"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_TRC,
    .priv_data_size = sizeof(TRCContext),
    .init           = trc_decode_init,
    FF_CODEC_DECODE_CB(trc_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
