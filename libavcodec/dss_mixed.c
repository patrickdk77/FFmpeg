/*
 * DSS and DS2 recordings whose block formats change
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Each packet leads with the format byte of its block; the frames come out
 * at the sample rate of that format.
 */

#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"

static const struct {
    int format;
    enum AVCodecID codec_id;
    int sample_rate;
} dss_mixed_formats[] = {
    { 0, AV_CODEC_ID_GRUNDIG_SP, 12000 }, /* SP */
    { 1, AV_CODEC_ID_GRUNDIG_SP, 12000 }, /* variable rate SP */
    { 2, AV_CODEC_ID_G723_1,      8000 }, /* LP */
    { 6, AV_CODEC_ID_DS2,        16000 }, /* QP */
    { 7, AV_CODEC_ID_DS2,        16000 }, /* QP with noise frames */
};

#define DSS_MIXED_NB_FORMATS FF_ARRAY_ELEMS(dss_mixed_formats)

typedef struct DSSMixedContext {
    AVCodecContext *sub[DSS_MIXED_NB_FORMATS];
    AVPacket *pkt;
    AVFrame *frame;
    int last;
} DSSMixedContext;

static av_cold int dss_mixed_init(AVCodecContext *avctx)
{
    DSSMixedContext *s = avctx->priv_data;

    av_channel_layout_uninit(&avctx->ch_layout);
    avctx->ch_layout  = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
    avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    s->last = -1;

    s->pkt   = av_packet_alloc();
    s->frame = av_frame_alloc();
    if (!s->pkt || !s->frame)
        return AVERROR(ENOMEM);
    return 0;
}

static int dss_mixed_open(AVCodecContext *avctx, int i)
{
    DSSMixedContext *s = avctx->priv_data;
    const AVCodec *codec = avcodec_find_decoder(dss_mixed_formats[i].codec_id);
    AVCodecContext *sub;
    int ret;

    if (!codec)
        return AVERROR_DECODER_NOT_FOUND;
    sub = s->sub[i] = avcodec_alloc_context3(codec);
    if (!sub)
        return AVERROR(ENOMEM);

    sub->ch_layout   = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
    sub->sample_rate = dss_mixed_formats[i].sample_rate;
    /* SP reads the format from extradata, QP from codec_tag */
    sub->codec_tag   = dss_mixed_formats[i].format;
    sub->extradata   = av_mallocz(1 + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!sub->extradata)
        return AVERROR(ENOMEM);
    sub->extradata[0]  = dss_mixed_formats[i].format;
    sub->extradata_size = 1;

    ret = avcodec_open2(sub, codec, NULL);
    if (ret < 0)
        return ret;
    if (sub->ch_layout.nb_channels != 1 ||
        av_get_bytes_per_sample(sub->sample_fmt) != 2)
        return AVERROR_BUG;
    return 0;
}

static int dss_mixed_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                                  int *got_frame, AVPacket *avpkt)
{
    DSSMixedContext *s = avctx->priv_data;
    AVCodecContext *sub;
    int i, ret, format;

    if (avpkt->size < 1)
        return AVERROR_INVALIDDATA;
    /* format 4 is LP */
    format = avpkt->data[0] == 4 ? 2 : avpkt->data[0];
    for (i = 0; i < DSS_MIXED_NB_FORMATS; i++)
        if (dss_mixed_formats[i].format == format)
            break;
    if (i == DSS_MIXED_NB_FORMATS) {
        av_log(avctx, AV_LOG_ERROR, "Unknown block format %d\n", format);
        return AVERROR_INVALIDDATA;
    }

    if (!s->sub[i] && (ret = dss_mixed_open(avctx, i)) < 0)
        return ret;
    sub = s->sub[i];
    if (s->last >= 0 && s->last != i)
        avcodec_flush_buffers(sub);
    s->last = i;

    av_packet_unref(s->pkt);
    if ((ret = av_packet_ref(s->pkt, avpkt)) < 0)
        return ret;
    s->pkt->data++;
    s->pkt->size--;

    ret = avcodec_send_packet(sub, s->pkt);
    if (ret < 0)
        return ret;
    ret = avcodec_receive_frame(sub, s->frame);
    if (ret == AVERROR(EAGAIN))
        return avpkt->size;
    if (ret < 0)
        return ret;

    avctx->sample_rate = sub->sample_rate;
    frame->nb_samples  = s->frame->nb_samples;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    memcpy(frame->data[0], s->frame->extended_data[0],
           frame->nb_samples * sizeof(int16_t));
    av_frame_unref(s->frame);

    *got_frame = 1;
    return avpkt->size;
}

static av_cold void dss_mixed_flush(AVCodecContext *avctx)
{
    DSSMixedContext *s = avctx->priv_data;

    for (int i = 0; i < DSS_MIXED_NB_FORMATS; i++)
        if (s->sub[i])
            avcodec_flush_buffers(s->sub[i]);
    s->last = -1;
}

static av_cold int dss_mixed_close(AVCodecContext *avctx)
{
    DSSMixedContext *s = avctx->priv_data;

    for (int i = 0; i < DSS_MIXED_NB_FORMATS; i++)
        avcodec_free_context(&s->sub[i]);
    av_packet_free(&s->pkt);
    av_frame_free(&s->frame);
    return 0;
}

const FFCodec ff_dss_mixed_decoder = {
    .p.name         = "dss_mixed",
    CODEC_LONG_NAME("DSS/DS2 with changing block formats"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_DSS_MIXED,
    .priv_data_size = sizeof(DSSMixedContext),
    .init           = dss_mixed_init,
    FF_CODEC_DECODE_CB(dss_mixed_decode_frame),
    .flush          = dss_mixed_flush,
    .close          = dss_mixed_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
