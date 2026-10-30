/*
 * ATRAC3+ compatible decoder
 *
 * Copyright (c) 2010-2013 Maxim Poliakovski
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
 * Sony ATRAC3+ compatible decoder.
 *
 * Container formats used to store its data:
 * RIFF WAV (.at3), Sony OpenMG (.oma, .aa3) and Sony IC recorder MSV/DVF.
 *
 * Technical description of this codec can be found here:
 * http://wiki.multimedia.cx/index.php?title=ATRAC3plus
 *
 * Kudos to Benjamin Larsson and Michael Karcher
 * for their precious technical help!
 */

#include <stdint.h>
#include <string.h>

#include "libavutil/channel_layout.h"
#include "libavutil/float_dsp.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/mem_internal.h"
#include "libavutil/opt.h"
#include "libavutil/thread.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "get_bits.h"
#include "atrac.h"
#include "atrac3plus.h"
#include "atrac3plus_hfc.h"

static const uint8_t channel_map[8][8] = {
    { 0, },
    { 0, 1, },
    { 0, 1, 2, },
    { 0, 1, 2, 3, },
    { 0, },
    { 0, 1, 2, 4, 5, 3, },
    { 0, 1, 2, 4, 5, 6, 3, },
    { 0, 1, 2, 4, 5, 6, 7, 3, },
};

typedef struct ATRAC3PContext {
    const AVClass *class;
    GetBitContext gb;
    AVFloatDSPContext *fdsp;

    DECLARE_ALIGNED(32, float, samples)[2][ATRAC3P_FRAME_SAMPLES];  ///< quantized MDCT spectrum
    DECLARE_ALIGNED(32, float, time_buf)[2][ATRAC3P_FRAME_SAMPLES]; ///< output of the gain compensation
    DECLARE_ALIGNED(32, float, outp_buf)[2][ATRAC3P_FRAME_SAMPLES];

    Atrac3pChanUnitCtx *ch_units;   ///< global channel units

    int num_channel_blocks;     ///< number of channel blocks
    uint8_t channel_blocks[5];  ///< channel configuration descriptor
    const uint8_t *channel_map; ///< channel layout map

    int hfc_opt;                ///< apply AT-X HFC when the stream has it
    Atrac3pHfcContext *hfc;     ///< HFC state per channel block, NULL if unused
    uint8_t hfc_running[5];     ///< HFC data was seen on this channel block
    uint8_t hfc_parsed[5];      ///< HFC data was parsed in the current frame
} ATRAC3PContext;

static av_cold int atrac3p_decode_close(AVCodecContext *avctx)
{
    ATRAC3PContext *ctx = avctx->priv_data;

    av_freep(&ctx->ch_units);
    av_freep(&ctx->fdsp);
    av_freep(&ctx->hfc);

    return 0;
}

static av_cold int set_channel_params(ATRAC3PContext *ctx,
                                      AVCodecContext *avctx)
{
    int channels = avctx->ch_layout.nb_channels;
    memset(ctx->channel_blocks, 0, sizeof(ctx->channel_blocks));

    av_channel_layout_uninit(&avctx->ch_layout);
    switch (channels) {
    case 1:
        avctx->ch_layout        = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
        ctx->num_channel_blocks = 1;
        ctx->channel_blocks[0]  = CH_UNIT_MONO;
        break;
    case 2:
        avctx->ch_layout        = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
        ctx->num_channel_blocks = 1;
        ctx->channel_blocks[0]  = CH_UNIT_STEREO;
        break;
    case 3:
        avctx->ch_layout        = (AVChannelLayout)AV_CHANNEL_LAYOUT_SURROUND;
        ctx->num_channel_blocks = 2;
        ctx->channel_blocks[0]  = CH_UNIT_STEREO;
        ctx->channel_blocks[1]  = CH_UNIT_MONO;
        break;
    case 4:
        avctx->ch_layout        = (AVChannelLayout)AV_CHANNEL_LAYOUT_4POINT0;
        ctx->num_channel_blocks = 3;
        ctx->channel_blocks[0]  = CH_UNIT_STEREO;
        ctx->channel_blocks[1]  = CH_UNIT_MONO;
        ctx->channel_blocks[2]  = CH_UNIT_MONO;
        break;
    case 6:
        avctx->ch_layout        = (AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT1_BACK;
        ctx->num_channel_blocks = 4;
        ctx->channel_blocks[0]  = CH_UNIT_STEREO;
        ctx->channel_blocks[1]  = CH_UNIT_MONO;
        ctx->channel_blocks[2]  = CH_UNIT_STEREO;
        ctx->channel_blocks[3]  = CH_UNIT_MONO;
        break;
    case 7:
        avctx->ch_layout        = (AVChannelLayout)AV_CHANNEL_LAYOUT_6POINT1_BACK;
        ctx->num_channel_blocks = 5;
        ctx->channel_blocks[0]  = CH_UNIT_STEREO;
        ctx->channel_blocks[1]  = CH_UNIT_MONO;
        ctx->channel_blocks[2]  = CH_UNIT_STEREO;
        ctx->channel_blocks[3]  = CH_UNIT_MONO;
        ctx->channel_blocks[4]  = CH_UNIT_MONO;
        break;
    case 8:
        avctx->ch_layout        = (AVChannelLayout)AV_CHANNEL_LAYOUT_7POINT1;
        ctx->num_channel_blocks = 5;
        ctx->channel_blocks[0]  = CH_UNIT_STEREO;
        ctx->channel_blocks[1]  = CH_UNIT_MONO;
        ctx->channel_blocks[2]  = CH_UNIT_STEREO;
        ctx->channel_blocks[3]  = CH_UNIT_STEREO;
        ctx->channel_blocks[4]  = CH_UNIT_MONO;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR,
               "Unsupported channel count: %d!\n", channels);
        return AVERROR_INVALIDDATA;
    }

    ctx->channel_map = channel_map[channels - 1];

    return 0;
}

/**
 * Set up AT-X high frequency compensation for Sony IC recorder streams. The
 * MSV demuxer exports two of the reference decoder's format parameters as
 * extradata, the quality code as 16-bit LE and the engine version.
 */
static av_cold int hfc_init(ATRAC3PContext *ctx, AVCodecContext *avctx)
{
    int i, ret, base, start;

    if (!ctx->hfc_opt || avctx->extradata_size != 3 || avctx->extradata[2] != 3)
        return 0;

    switch (AV_RL16(avctx->extradata)) {
    case 0x20:
        base  = 3;
        start = 11;
        break;
    case 0x24:
        base  = 2;
        start = 6;
        break;
    default:
        return 0;
    }

    ctx->hfc = av_calloc(ctx->num_channel_blocks, sizeof(*ctx->hfc));
    if (!ctx->hfc)
        return AVERROR(ENOMEM);

    for (i = 0; i < ctx->num_channel_blocks; i++)
        if ((ret = ff_atrac3p_hfc_init(&ctx->hfc[i], ctx->channel_blocks[i] + 1,
                                       base, start, 11, 0)) < 0)
            return ret;

    return 0;
}

static void ch_unit_init_ptrs(Atrac3pChanUnitCtx *u)
{
    int ch;

    for (ch = 0; ch < 2; ch++) {
        u->channels[ch].ch_num          = ch;
        u->channels[ch].wnd_shape       = &u->channels[ch].wnd_shape_hist[0][0];
        u->channels[ch].wnd_shape_prev  = &u->channels[ch].wnd_shape_hist[1][0];
        u->channels[ch].gain_data       = &u->channels[ch].gain_data_hist[0][0];
        u->channels[ch].gain_data_prev  = &u->channels[ch].gain_data_hist[1][0];
        u->channels[ch].tones_info      = &u->channels[ch].tones_info_hist[0][0];
        u->channels[ch].tones_info_prev = &u->channels[ch].tones_info_hist[1][0];
    }

    u->waves_info      = &u->wave_synth_hist[0];
    u->waves_info_prev = &u->wave_synth_hist[1];
}

static av_cold void atrac3p_init_static(void)
{
    ff_atrac3p_init_vlcs();
    ff_atrac3p_init_dsp_static();
    ff_atrac3p_hfc_init_static();
}

static av_cold int atrac3p_decode_init(AVCodecContext *avctx)
{
    static AVOnce init_static_once = AV_ONCE_INIT;
    ATRAC3PContext *ctx = avctx->priv_data;
    int i, ret;

    if (!avctx->block_align) {
        av_log(avctx, AV_LOG_ERROR, "block_align is not set\n");
        return AVERROR(EINVAL);
    }

    if ((ret = set_channel_params(ctx, avctx)) < 0)
        return ret;

    ctx->ch_units = av_calloc(ctx->num_channel_blocks, sizeof(*ctx->ch_units));
    ctx->fdsp = avpriv_float_dsp_alloc(avctx->flags & AV_CODEC_FLAG_BITEXACT);

    if (!ctx->ch_units || !ctx->fdsp) {
        return AVERROR(ENOMEM);
    }

    if ((ret = hfc_init(ctx, avctx)) < 0)
        return ret;

    for (i = 0; i < ctx->num_channel_blocks; i++)
        ch_unit_init_ptrs(&ctx->ch_units[i]);

    avctx->sample_fmt = AV_SAMPLE_FMT_FLTP;

    ff_thread_once(&init_static_once, atrac3p_init_static);

    return 0;
}

static void decode_residual_spectrum(ATRAC3PContext *ctx, Atrac3pChanUnitCtx *ch_unit,
                                     float out[2][ATRAC3P_FRAME_SAMPLES],
                                     int num_channels,
                                     AVCodecContext *avctx)
{
    int i, sb, ch, qu, nspeclines, RNG_index;
    float *dst, q;
    int16_t *src;
    /* calculate RNG table index for each subband */
    int sb_RNG_index[ATRAC3P_SUBBANDS] = { 0 };

    if (ch_unit->mute_flag) {
        for (ch = 0; ch < num_channels; ch++)
            memset(out[ch], 0, ATRAC3P_FRAME_SAMPLES * sizeof(*out[ch]));
        return;
    }

    for (qu = 0, RNG_index = 0; qu < ch_unit->used_quant_units; qu++)
        RNG_index += ch_unit->channels[0].qu_sf_idx[qu] +
                     ch_unit->channels[1].qu_sf_idx[qu];

    for (sb = 0; sb < ch_unit->num_coded_subbands; sb++, RNG_index += 128)
        sb_RNG_index[sb] = RNG_index & 0x3FC;

    /* inverse quant and power compensation */
    for (ch = 0; ch < num_channels; ch++) {
        /* clear channel's residual spectrum */
        memset(out[ch], 0, ATRAC3P_FRAME_SAMPLES * sizeof(*out[ch]));

        for (qu = 0; qu < ch_unit->used_quant_units; qu++) {
            src        = &ch_unit->channels[ch].spectrum[ff_atrac3p_qu_to_spec_pos[qu]];
            dst        = &out[ch][ff_atrac3p_qu_to_spec_pos[qu]];
            nspeclines = ff_atrac3p_qu_to_spec_pos[qu + 1] -
                         ff_atrac3p_qu_to_spec_pos[qu];

            if (ch_unit->channels[ch].qu_wordlen[qu] > 0) {
                q = ff_atrac3p_sf_tab[ch_unit->channels[ch].qu_sf_idx[qu]] *
                    ff_atrac3p_mant_tab[ch_unit->channels[ch].qu_wordlen[qu]];
                for (i = 0; i < nspeclines; i++)
                    dst[i] = src[i] * q;
            }
        }

        for (sb = 0; sb < ch_unit->num_coded_subbands; sb++)
            ff_atrac3p_power_compensation(ch_unit, ctx->fdsp, ch, &out[ch][0],
                                          sb_RNG_index[sb], sb);
    }

    if (ch_unit->unit_type == CH_UNIT_STEREO) {
        for (sb = 0; sb < ch_unit->num_coded_subbands; sb++) {
            if (ch_unit->swap_channels[sb]) {
                for (i = 0; i < ATRAC3P_SUBBAND_SAMPLES; i++)
                    FFSWAP(float, out[0][sb * ATRAC3P_SUBBAND_SAMPLES + i],
                                  out[1][sb * ATRAC3P_SUBBAND_SAMPLES + i]);
            }

            /* flip coefficients' sign if requested */
            if (ch_unit->negate_coeffs[sb])
                for (i = 0; i < ATRAC3P_SUBBAND_SAMPLES; i++)
                    out[1][sb * ATRAC3P_SUBBAND_SAMPLES + i] = -(out[1][sb * ATRAC3P_SUBBAND_SAMPLES + i]);
        }
    }
}

static void reconstruct_frame(ATRAC3PContext *ctx, Atrac3pChanUnitCtx *ch_unit,
                              int num_channels, AVCodecContext *avctx)
{
    float mdct_buf[2 * ATRAC3P_SUBBAND_SAMPLES];
    int ch, sb;

    for (ch = 0; ch < num_channels; ch++) {
        for (sb = 0; sb < ch_unit->num_subbands; sb++) {
            /* inverse transform and windowing */
            ff_atrac3p_imdct(&ctx->samples[ch][sb * ATRAC3P_SUBBAND_SAMPLES],
                             mdct_buf,
                             (ch_unit->channels[ch].wnd_shape_prev[sb] << 1) +
                             ch_unit->channels[ch].wnd_shape[sb], sb);

            /* gain compensation and overlapping */
            ff_atrac3p_gain_compensation(mdct_buf,
                                         &ch_unit->prev_buf[ch][sb * ATRAC3P_SUBBAND_SAMPLES],
                                         &ch_unit->channels[ch].gain_data_prev[sb],
                                         &ch_unit->channels[ch].gain_data[sb],
                                         &ctx->time_buf[ch][sb * ATRAC3P_SUBBAND_SAMPLES]);
        }

        /* zero unused subbands in both output and overlapping buffers */
        memset(&ch_unit->prev_buf[ch][ch_unit->num_subbands * ATRAC3P_SUBBAND_SAMPLES],
               0,
               (ATRAC3P_SUBBANDS - ch_unit->num_subbands) *
               ATRAC3P_SUBBAND_SAMPLES *
               sizeof(ch_unit->prev_buf[ch][ch_unit->num_subbands * ATRAC3P_SUBBAND_SAMPLES]));
        memset(&ctx->time_buf[ch][ch_unit->num_subbands * ATRAC3P_SUBBAND_SAMPLES],
               0,
               (ATRAC3P_SUBBANDS - ch_unit->num_subbands) *
               ATRAC3P_SUBBAND_SAMPLES *
               sizeof(ctx->time_buf[ch][ch_unit->num_subbands * ATRAC3P_SUBBAND_SAMPLES]));

        /* resynthesize and add tonal signal */
        if (ch_unit->waves_info->tones_present ||
            ch_unit->waves_info_prev->tones_present) {
            for (sb = 0; sb < ch_unit->num_subbands; sb++)
                if (ch_unit->channels[ch].tones_info[sb].num_wavs ||
                    ch_unit->channels[ch].tones_info_prev[sb].num_wavs) {
                    ff_atrac3p_generate_tones(ch_unit, ctx->fdsp, ch, sb,
                                              &ctx->time_buf[ch][sb * 128]);
                }
        }

        if (ch_unit->noise_present)
            ff_atrac3p_add_noise(ch_unit, ch, ctx->time_buf[ch]);
    }

    /* swap window shape and gain control buffers. */
    for (ch = 0; ch < num_channels; ch++) {
        FFSWAP(uint8_t *, ch_unit->channels[ch].wnd_shape,
               ch_unit->channels[ch].wnd_shape_prev);
        FFSWAP(AtracGainInfo *, ch_unit->channels[ch].gain_data,
               ch_unit->channels[ch].gain_data_prev);
        FFSWAP(Atrac3pWavesData *, ch_unit->channels[ch].tones_info,
               ch_unit->channels[ch].tones_info_prev);
    }

    FFSWAP(Atrac3pWaveSynthParams *, ch_unit->waves_info, ch_unit->waves_info_prev);
}

/**
 * Apply HFC if enabled, then run subband synthesis and output a channel block.
 */
static void output_block(ATRAC3PContext *ctx, AVCodecContext *avctx,
                         int block, float **samples_p)
{
    Atrac3pChanUnitCtx *ch_unit = &ctx->ch_units[block];
    int i, ch, sb, num_channels = ch_unit->unit_type + 1, out_ch_index = 0;

    for (i = 0; i < block; i++)
        out_ch_index += ctx->channel_blocks[i] + 1;

    if (ctx->hfc) {
        if (ctx->hfc_parsed[block] && !ctx->hfc_running[block]) {
            /* the reference decoder starts HFC on a silent frame */
            for (ch = 0; ch < num_channels; ch++)
                memset(ctx->time_buf[ch], 0, sizeof(ctx->time_buf[ch]));
            ctx->hfc_running[block] = 1;
        } else if (ctx->hfc_running[block] && !ctx->hfc_parsed[block]) {
            av_log(avctx, AV_LOG_WARNING, "HFC data missing\n");
        }

        if (ctx->hfc_running[block]) {
            float *bands[2 * ATRAC3P_SUBBANDS];

            for (ch = 0; ch < num_channels; ch++)
                for (sb = 0; sb < ATRAC3P_SUBBANDS; sb++)
                    bands[ch * ATRAC3P_SUBBANDS + sb] = &ctx->time_buf[ch][sb * ATRAC3P_SUBBAND_SAMPLES];
            ff_atrac3p_hfc_synth(&ctx->hfc[block], bands);
        }
    }

    for (ch = 0; ch < num_channels; ch++) {
        /* subband synthesis and acoustic signal output */
        ff_atrac3p_ipqf(&ch_unit->ipqf_ctx[ch], &ctx->time_buf[ch][0],
                        &ctx->outp_buf[ch][0]);
        memcpy(samples_p[ctx->channel_map[out_ch_index + ch]], ctx->outp_buf[ch],
               ATRAC3P_FRAME_SAMPLES * sizeof(**samples_p));
    }
}

/**
 * Decode an extension unit. Type 0 carries the HFC data of the preceding
 * channel block, other types are skipped.
 */
static int decode_extension_unit(ATRAC3PContext *ctx, AVCodecContext *avctx,
                                 int block)
{
    uint8_t buf[0x7FE + AV_INPUT_BUFFER_PADDING_SIZE];
    int i, ret, type, len;

    type = get_bits(&ctx->gb, 5);
    len  = get_bits(&ctx->gb, 11);
    if (len > 0x7FE || get_bits_left(&ctx->gb) < len * 8) {
        av_log(avctx, AV_LOG_ERROR, "Invalid extension unit!\n");
        return AVERROR_INVALIDDATA;
    }

    if (type || block < 0) {
        skip_bits_long(&ctx->gb, len * 8);
        return 0;
    }

    if (ctx->hfc_parsed[block]) {
        av_log(avctx, AV_LOG_ERROR, "Repeated HFC data!\n");
        return AVERROR_INVALIDDATA;
    }

    for (i = 0; i < len; i++)
        buf[i] = get_bits(&ctx->gb, 8);
    memset(&buf[len], 0, AV_INPUT_BUFFER_PADDING_SIZE);

    if ((ret = ff_atrac3p_hfc_parse(&ctx->hfc[block], buf, len)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Invalid HFC data!\n");
        return ret;
    }
    ctx->hfc_parsed[block] = 1;

    return 0;
}

static int atrac3p_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                                int *got_frame_ptr, AVPacket *avpkt)
{
    ATRAC3PContext *ctx = avctx->priv_data;
    int ret, ch_unit_id, ch_block = 0, channels_to_process;
    float **samples_p = (float **)frame->extended_data;

    frame->nb_samples = ATRAC3P_FRAME_SAMPLES;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    if ((ret = init_get_bits8(&ctx->gb, avpkt->data, avpkt->size)) < 0)
        return ret;

    if (get_bits1(&ctx->gb)) {
        av_log(avctx, AV_LOG_ERROR, "Invalid start bit!\n");
        return AVERROR_INVALIDDATA;
    }

    while (get_bits_left(&ctx->gb) >= 2 &&
           (ch_unit_id = get_bits(&ctx->gb, 2)) != CH_UNIT_TERMINATOR) {
        if (ch_unit_id == CH_UNIT_EXTENSION) {
            /* Without HFC, stop here and keep the base band. Decoders with
             * high frequency reconstruction turned off do the same. */
            if (!ctx->hfc)
                break;
            if ((ret = decode_extension_unit(ctx, avctx, ch_block - 1)) < 0)
                return ret;
            continue;
        }
        /* the previous block's HFC data, if any, has been read by now */
        if (ctx->hfc && ch_block > 0)
            output_block(ctx, avctx, ch_block - 1, samples_p);
        if (ch_block >= ctx->num_channel_blocks ||
            ctx->channel_blocks[ch_block] != ch_unit_id) {
            av_log(avctx, AV_LOG_ERROR,
                   "Frame data doesn't match channel configuration!\n");
            return AVERROR_INVALIDDATA;
        }

        ctx->ch_units[ch_block].unit_type = ch_unit_id;
        channels_to_process               = ch_unit_id + 1;

        if ((ret = ff_atrac3p_decode_channel_unit(&ctx->gb,
                                                  &ctx->ch_units[ch_block],
                                                  channels_to_process,
                                                  avctx)) < 0)
            return ret;

        decode_residual_spectrum(ctx, &ctx->ch_units[ch_block], ctx->samples,
                                 channels_to_process, avctx);
        reconstruct_frame(ctx, &ctx->ch_units[ch_block],
                          channels_to_process, avctx);

        if (ctx->hfc)
            ctx->hfc_parsed[ch_block] = 0;
        else
            output_block(ctx, avctx, ch_block, samples_p);

        ch_block++;
    }

    if (ctx->hfc && ch_block > 0)
        output_block(ctx, avctx, ch_block - 1, samples_p);

    *got_frame_ptr = 1;

    return avctx->codec_id == AV_CODEC_ID_ATRAC3P ? FFMIN(avctx->block_align, avpkt->size) : avpkt->size;
}

static av_cold void atrac3p_flush(AVCodecContext *avctx)
{
    ATRAC3PContext *ctx = avctx->priv_data;
    int i;

    for (i = 0; i < ctx->num_channel_blocks; i++) {
        memset(&ctx->ch_units[i], 0, sizeof(ctx->ch_units[i]));
        ch_unit_init_ptrs(&ctx->ch_units[i]);
        if (ctx->hfc)
            ff_atrac3p_hfc_reset(&ctx->hfc[i]);
        ctx->hfc_running[i] = 0;
        ctx->hfc_parsed[i]  = 0;
    }
}

#define OFFSET(x) offsetof(ATRAC3PContext, x)
#define AD AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_DECODING_PARAM
static const AVOption atrac3p_options[] = {
    { "hfc", "Apply AT-X high frequency compensation (Sony IC recorder files)",
      OFFSET(hfc_opt), AV_OPT_TYPE_BOOL, { .i64 = 1 }, 0, 1, AD },
    { NULL },
};

static const AVClass atrac3p_class = {
    .class_name = "ATRAC3+ decoder",
    .item_name  = av_default_item_name,
    .option     = atrac3p_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_atrac3p_decoder = {
    .p.name         = "atrac3plus",
    CODEC_LONG_NAME("ATRAC3+ (Adaptive TRansform Acoustic Coding 3+)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ATRAC3P,
    .p.priv_class   = &atrac3p_class,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .priv_data_size = sizeof(ATRAC3PContext),
    .init           = atrac3p_decode_init,
    .close          = atrac3p_decode_close,
    .flush          = atrac3p_flush,
    FF_CODEC_DECODE_CB(atrac3p_decode_frame),
};

const FFCodec ff_atrac3pal_decoder = {
    .p.name         = "atrac3plusal",
    CODEC_LONG_NAME("ATRAC3+ AL (Adaptive TRansform Acoustic Coding 3+ Advanced Lossless)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ATRAC3PAL,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .priv_data_size = sizeof(ATRAC3PContext),
    .init           = atrac3p_decode_init,
    .close          = atrac3p_decode_close,
    .flush          = atrac3p_flush,
    FF_CODEC_DECODE_CB(atrac3p_decode_frame),
};
