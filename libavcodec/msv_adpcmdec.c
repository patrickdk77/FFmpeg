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

typedef struct MSVADPCMContext {
    int need_init;
    int mode;
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
 * bits/sample selects the ADPCM mode. Only 2-bit (LP) and 3-bit (SP) are
 * verified against reference decodes; a 4-bit (HQ) mode is NOT implemented
 * yet (no HQ sample is available to reverse-engineer/verify against), so
 * anything outside 2..3 falls back to 3-bit.
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

    if (block_align < MSV_ADPCM_FRAME_SIZE || buf_size < block_align)
        return AVERROR_INVALIDDATA;

    num_blocks = buf_size / block_align;
    if (!num_blocks)
        return AVERROR_INVALIDDATA;

    frame->nb_samples = num_blocks * msv_adpcm_samples_per_frame(s->mode);
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    out = (int16_t *)frame->data[0];
    total_samples = 0;
    for (b = 0; b < num_blocks; b++) {
        n = msv_adpcm_decode_block(buf + b * block_align,
                                  out + total_samples, s->mode,
                                  &s->need_init, &s->bs);
        total_samples += n;
    }
    frame->nb_samples = total_samples;

    *got_frame_ptr = 1;
    return buf_size;
}

const FFCodec ff_msv_adpcm_decoder = {
    .p.name         = "msv_adpcm",
    CODEC_LONG_NAME("Sony Memory Stick Voice ADPCM"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_MSV_ADPCM,
    .priv_data_size = sizeof(MSVADPCMContext),
    .init           = msv_adpcm_decode_init,
    FF_CODEC_DECODE_CB(msv_adpcm_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16),
};
