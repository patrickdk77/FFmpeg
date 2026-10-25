/*
 * Sony Memory Stick Voice ADPCM decoder
 * Reverse-engineered from the reference decoder (3-bit symbols, 48-byte frames).
 * Not PlayStation/XA ADPCM.
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

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "msv_adpcm_sony.h"

#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"

enum MSVADPCMVariant {
    MSV_ADPCM_VARIANT_DVE,
    MSV_ADPCM_VARIANT_PLUGIN,
};

typedef struct MSVADPCMContext {
    const AVClass *class;
    int need_init;
    int mode;
    int variant;
    MSVADPCMState state;
    MSVADPCMBitstream bs;
} MSVADPCMContext;

static av_cold int msv_adpcm_decode_init(AVCodecContext *avctx)
{
    MSVADPCMContext *s = avctx->priv_data;

    if (avctx->ch_layout.nb_channels != 1)
        return AVERROR_PATCHWELCOME;

    avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    if (!avctx->block_align)
        avctx->block_align = MSV_ADPCM_FRAME_SIZE;

    /*
 * bits/sample selects the ADPCM mode: 2-bit (LP) or 3-bit (SP), the only
 * modes Sony's writer has; anything else falls back to 3-bit.
 * The demuxer passes the mode via bits_per_coded_sample.
*/
    s->mode = avctx->bits_per_coded_sample;
    if (s->mode < 2 || s->mode > 3)
        s->mode = MSV_ADPCM_MODE_3BIT;

    s->need_init = 1;
    return 0;
}

static int msv_adpcm_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                                  int *got_frame_ptr, AVPacket *avpkt)
{
    MSVADPCMContext *s = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    int block_align = avctx->block_align;
    int num_blocks, total_samples = 0, ret, b, n;
    int16_t *out;

    if (block_align < MSV_ADPCM_FRAME_SIZE)
        return AVERROR_INVALIDDATA;

    /* final LP block of whole 16-byte units; the plug-in drops it */
    if (buf_size < block_align) {
        if (s->mode != 2 || buf_size % 16 || s->variant != MSV_ADPCM_VARIANT_DVE)
            return buf_size;
        frame->nb_samples = buf_size * 8 / s->mode;
        if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
            return ret;
        frame->nb_samples = ff_msv_adpcm_decode_block(buf, buf_size,
                                                   (int16_t *)frame->data[0],
                                                   s->mode, &s->need_init, 1,
                                                   &s->state, &s->bs);
        *got_frame_ptr = 1;
        return buf_size;
    }

    num_blocks = buf_size / block_align;

    frame->nb_samples = num_blocks * ff_msv_adpcm_samples_per_frame(s->mode);
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    out = (int16_t *)frame->data[0];
    total_samples = 0;
    for (b = 0; b < num_blocks; b++) {
        n = ff_msv_adpcm_decode_block(buf + b * block_align, MSV_ADPCM_FRAME_SIZE,
                                  out + total_samples, s->mode,
                                  &s->need_init,
                                  s->variant == MSV_ADPCM_VARIANT_DVE,
                                  &s->state, &s->bs);
        total_samples += n;
    }
    frame->nb_samples = total_samples;

    *got_frame_ptr = 1;
    return buf_size;
}

static av_cold void msv_adpcm_flush(AVCodecContext *avctx)
{
    MSVADPCMContext *s = avctx->priv_data;

    s->need_init = 1;
}

#define OFFSET(x) offsetof(MSVADPCMContext, x)
#define PARAM AV_OPT_FLAG_DECODING_PARAM | AV_OPT_FLAG_AUDIO_PARAM
static const AVOption msv_adpcm_options[] = {
    { "variant", "Sony decoder whose output to reproduce", OFFSET(variant),
        AV_OPT_TYPE_INT, { .i64 = MSV_ADPCM_VARIANT_DVE }, 0, 1, PARAM, .unit = "variant" },
      { "dve",    "Digital Voice Editor 3 reader; the first symbol is decoded",
        0, AV_OPT_TYPE_CONST, { .i64 = MSV_ADPCM_VARIANT_DVE }, .flags = PARAM, .unit = "variant" },
      { "plugin", "Windows Media Player plug-in; the first symbol only resets the decoder",
        0, AV_OPT_TYPE_CONST, { .i64 = MSV_ADPCM_VARIANT_PLUGIN }, .flags = PARAM, .unit = "variant" },
    { NULL },
};

static const AVClass msv_adpcm_class = {
    .class_name = "msv_adpcm decoder",
    .item_name  = av_default_item_name,
    .option     = msv_adpcm_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_msv_adpcm_decoder = {
    .p.name         = "msv_adpcm",
    CODEC_LONG_NAME("Sony Memory Stick Voice ADPCM"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_MSV_ADPCM,
    .p.priv_class   = &msv_adpcm_class,
    .priv_data_size = sizeof(MSVADPCMContext),
    .init           = msv_adpcm_decode_init,
    FF_CODEC_DECODE_CB(msv_adpcm_decode_frame),
    .flush          = msv_adpcm_flush,
    .p.capabilities = AV_CODEC_CAP_DR1,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16),
};
