/*
 * Digital Speech Standard (DSS) demuxer
 * Copyright (c) 2014 Oleksij Rempel <linux@rempel-privat.de>
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

#include <string.h>

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"
#include "ds2crypto.h"
#include "dss_metadata.h"

#define DSS_HEAD_OFFSET_AUTHOR        0xc
#define DSS_AUTHOR_SIZE               16

#define DSS_HEAD_OFFSET_START_TIME    0x26
#define DSS_HEAD_OFFSET_END_TIME      0x32
#define DSS_TIME_SIZE                 12

#define DSS_ACODEC_DSS_SP             0x0    /* SP mode */
#define DSS_ACODEC_DSS_SP_VR          0x1    /* SP mode, variable rate */
#define DSS_ACODEC_G723_1             0x2    /* LP mode */
#define DSS_ACODEC_G723_1_ALT         0x4    /* LP mode */
#define DSS_ACODEC_QP                 0x6    /* QP mode */
#define DSS_ACODEC_QP7                0x7    /* QP mode with noise frames */

/* Native CELP codec at 12 kHz, 41-byte (328-bit) frames packed as a
 * continuous bitstream across 512-byte audio blocks. */
#define DSS_GRUNDIG_FRAME_SIZE        41
#define DSS_GRUNDIG_FRAME_BITS        328
#define DSS_GRUNDIG_SAMPLE_RATE       12000
#define DSS_GRUNDIG_SAMPLES_PER_FRAME 288

#define DSS_QP_FRAME_SIZE             56
#define DSS_QP7_NOISE_FRAME_SIZE      12
#define DSS_QP_SAMPLE_RATE            16000
#define DSS_QP_SAMPLES_PER_FRAME      256

/* SP, LP and QP frames all last a whole number of ticks */
#define DSS_MIXED_TIME_BASE           48000

/* The next block that holds frames is in another format */
#define DSS_PART_END                  FFERRTAG('D', 'S', 'P', 'E')

#define DSS_HEAD_OFFSET_COMMENT       0x31e
#define DSS_COMMENT_SIZE              64

#define DSS_BLOCK_SIZE                512
#define DSS_AUDIO_BLOCK_HEADER_SIZE   6
#define DSS_BLOCK_KEEP_BEFORE         6
#define DSS_FORMAT_SCAN_BLOCKS        64
#define DSS_FRAME_SIZE                42
#define DSS_SP_SHORT_FRAME_SIZE       9

static const uint8_t frame_size[4] = { 24, 20, 4, 1 };

typedef struct DSSDemuxContext {
    AVClass *class;
    unsigned int audio_codec;
    int swap;
    int64_t block_pos;      /* offset of the current audio block header */
    int frames_left;        /* frames still to come from that block */
    int dss_sp_swap_byte;
    int64_t frames_read;    /* index of the next frame */
    int64_t redo_frame;     /* index of a frame dropped for its swap byte, or -1 */

    int dss_header_size;

    /* Grundig variant: the frames run through the block payloads as one bit
     * stream of little-endian 16-bit words, read MSB first. */
    int      grundig;
    unsigned grundig_word;  /* payload word being read */
    int      grundig_bits;  /* bits of it still to read */

    AVIOContext *pb;        /* s->pb, or decrypt_pb in an encrypted DS2 file */
    AVIOContext *decrypt_pb;
    char *password;

    int extract;            /* DS2ExtractMode */
    DS2Annotations annotations;
    int64_t total_frames;   /* frames of the recording, returned or not */
    int64_t frames_out;     /* frames returned so far */
    int skipped;            /* frames were left out since the last returned */

    int     mixed;          /* packets lead with the format byte */
    int64_t pts;
    int64_t stated_pts;
    int     first_format;
    int     format_warned;

    int qp_left;
    int qp_frame_size;
    int qp_block_format;
    int qp_frame_format;
} DSSDemuxContext;

/* Frames that start in an audio block: byte 2 when it is not 0, otherwise
 * byte 5 unless that is 0xff. A recorder writes the second form where it
 * pauses or stops, and Olympus's decoder plays those frames. */
static int dss_block_frames(const uint8_t *header)
{
    if (header[2] || header[5] == 0xff)
        return header[2];
    return header[5];
}

static int dss_format_known(const uint8_t *header)
{
    if (header[3] != 0xff)
        return 0;
    switch (header[4]) {
    case 0: case 1: case 2: case 4: case 6: case 7:
        return 1;
    }
    return 0;
}

static int dss_frame_samples(const DSSDemuxContext *ctx)
{
    if (ctx->grundig)
        return DSS_GRUNDIG_SAMPLES_PER_FRAME;
    if (ctx->audio_codec == DSS_ACODEC_QP)
        return DSS_QP_SAMPLES_PER_FRAME;
    return ctx->audio_codec == DSS_ACODEC_G723_1 ? 240 : 264;
}

static int dss_part_format(int format)
{
    switch (format) {
    case DSS_ACODEC_DSS_SP:
    case DSS_ACODEC_DSS_SP_VR:
    case DSS_ACODEC_G723_1:
    case DSS_ACODEC_QP:
        return format;
    case DSS_ACODEC_G723_1_ALT:
        return DSS_ACODEC_G723_1;
    case DSS_ACODEC_QP7:
        return DSS_ACODEC_QP;
    }
    return -1;
}

static int dss_block_part(const uint8_t *header)
{
    return dss_format_known(header) ? dss_part_format(header[4]) : -1;
}

static int dss_part_rate(int format)
{
    if (format == DSS_ACODEC_G723_1)
        return 8000;
    if (format == DSS_ACODEC_QP)
        return DSS_QP_SAMPLE_RATE;
    return DSS_GRUNDIG_SAMPLE_RATE;
}

static int dss_part_ticks(int format)
{
    int samples = DSS_GRUNDIG_SAMPLES_PER_FRAME;

    if (format == DSS_ACODEC_G723_1)
        samples = 240;
    else if (format == DSS_ACODEC_QP)
        samples = DSS_QP_SAMPLES_PER_FRAME;
    return samples * (DSS_MIXED_TIME_BASE / dss_part_rate(format));
}

static void dss_set_part(DSSDemuxContext *ctx, int format)
{
    ctx->audio_codec      = format;
    ctx->grundig          = format == DSS_ACODEC_DSS_SP ||
                            format == DSS_ACODEC_DSS_SP_VR;
    ctx->frames_left      = 0;
    ctx->swap             = 0;
    ctx->dss_sp_swap_byte = -1;
}

static void dss_start_part(DSSDemuxContext *ctx, int format)
{
    dss_set_part(ctx, dss_part_format(format));
    if (ctx->audio_codec == DSS_ACODEC_QP)
        ctx->qp_block_format = format;
}

/* Header size of a DS2 file from the first byte, in 512-byte blocks. Philips
 * SpeechExec writes 1 in the DS2 files it splits out of a recording, over the
 * usual 3 blocks; Olympus writes 1 over 1 block, followed by an audio block. */
static int dss_ds2_header_size(const uint8_t *buf, int size)
{
    if (buf[0] == 1 && (size < DSS_BLOCK_SIZE + 4 ||
                        buf[DSS_BLOCK_SIZE + 3] != 0xff))
        return 3 * DSS_BLOCK_SIZE;
    return buf[0] * DSS_BLOCK_SIZE;
}

/* Olympus DssWriter can leave a copy of the file header in the audio area */
static int dss_header_copy_blocks(const uint8_t *header)
{
    if ((header[1] == 'd' && header[2] == 's' &&
         (header[3] == 's' || header[3] == '2')) ||
        (header[1] == 'e' && header[2] == 'n' && header[3] == 'c'))
        return FFMAX(header[0], 1);
    return 0;
}

static int dss_other_format(const DSSDemuxContext *ctx, const uint8_t *header)
{
    int format = dss_block_part(header);

    return !dss_header_copy_blocks(header) && dss_block_frames(header) &&
           format >= 0 && format != ctx->audio_codec;
}

static int dss_block_frames_read(AVFormatContext *s, const uint8_t *header)
{
    DSSDemuxContext *ctx = s->priv_data;

    if (!dss_other_format(ctx, header))
        return dss_block_frames(header);
    if (ctx->mixed)
        return DSS_PART_END;
    if (!ctx->format_warned) {
        av_log(s, AV_LOG_WARNING, "Skipping blocks of format %d in a file "
               "of format %d\n", header[4], ctx->audio_codec);
        ctx->format_warned = 1;
    }
    return 0;
}

static int dss_probe_format(const uint8_t *buf, int size, int pos)
{
    int copy;

    while (pos + DSS_AUDIO_BLOCK_HEADER_SIZE <= size) {
        copy = dss_header_copy_blocks(buf + pos);
        if (copy) {
            pos += copy * DSS_BLOCK_SIZE;
            continue;
        }
        if (dss_block_frames(buf + pos) && dss_format_known(buf + pos))
            return buf[pos + 4];
        pos += DSS_BLOCK_SIZE;
    }
    return -1;
}

static int dss_probe(const AVProbeData *p)
{
    int format;

    if (p->buf[1] == 'd' && p->buf[2] == 's' && p->buf[3] == 's')
        return AVPROBE_SCORE_MAX;

    /* DS2 files in SP, variable rate SP or LP mode carry exactly the DSS
     * payload; only the header is bigger. The block headers of an encrypted
     * DS2 file are in the clear. */
    if (p->buf[0] &&
        ((p->buf[1] == 'd' && p->buf[2] == 's' && p->buf[3] == '2') ||
         AV_RL32(p->buf) == DS2_ENCRYPTED_MAGIC)) {
        format = dss_probe_format(p->buf, p->buf_size,
                                  dss_ds2_header_size(p->buf, p->buf_size));
        if (format == DSS_ACODEC_DSS_SP || format == DSS_ACODEC_DSS_SP_VR ||
            format == DSS_ACODEC_G723_1 || format == DSS_ACODEC_G723_1_ALT)
            return AVPROBE_SCORE_MAX;
    }

    return 0;
}

static int dss_read_metadata_date(AVFormatContext *s, unsigned int offset,
                                  const char *key)
{
    AVIOContext *pb = s->pb;
    char datetime[64], string[DSS_TIME_SIZE + 1] = { 0 };
    int y, month, d, h, minute, sec;
    int ret;

    avio_seek(pb, offset, SEEK_SET);

    ret = avio_read(s->pb, string, DSS_TIME_SIZE);
    if (ret < DSS_TIME_SIZE)
        return ret < 0 ? ret : AVERROR_EOF;

    if (sscanf(string, "%2d%2d%2d%2d%2d%2d", &y, &month, &d, &h, &minute, &sec) != 6)
        return 0;
    /* We deal with a two-digit year here, so set the default date to 2000
     * and hope it will never be used in the next century. */
    snprintf(datetime, sizeof(datetime), "%.4d-%.2d-%.2dT%.2d:%.2d:%.2d",
             y + 2000, month, d, h, minute, sec);
    return av_dict_set(&s->metadata, key, datetime, 0);
}

static int dss_read_metadata_string(AVFormatContext *s, unsigned int offset,
                                    unsigned int size, const char *key)
{
    AVIOContext *pb = s->pb;
    char *value;
    int ret;

    avio_seek(pb, offset, SEEK_SET);

    value = av_mallocz(size + 1);
    if (!value)
        return AVERROR(ENOMEM);

    ret = avio_read(s->pb, value, size);
    if (ret != size) {
        av_free(value);
        return ret < 0 ? ret : AVERROR_EOF;
    }

    ff_dss_trim_string(value, size);
    if (!value[0]) {
        av_free(value);
        return 0;
    }

    return av_dict_set(&s->metadata, key, value, AV_DICT_DONT_STRDUP_VAL);
}

static int dss_read_packet(AVFormatContext *s, AVPacket *pkt);

/* Frames in the file, or 0 if the input cannot seek. Each block header states
 * how many frames start in the block, but the reader takes them from where the
 * first one starts, on into the next blocks when the block states more than it
 * holds, and stops where the file ends or a frame does not parse. So the
 * frames are counted by reading them all. */
static int64_t dss_count_frames(AVFormatContext *s)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = ctx->pb;
    int64_t frames = 0;
    int ret;
    AVPacket *pkt;

    if (!(pb->seekable & AVIO_SEEKABLE_NORMAL))
        return 0;

    pkt = av_packet_alloc();
    if (!pkt)
        return AVERROR(ENOMEM);

    /* dss_next_block steps forward first, so start one block short. */
    ctx->block_pos        = ctx->dss_header_size - DSS_BLOCK_SIZE;
    ctx->frames_left      = 0;
    ctx->dss_sp_swap_byte = -1;
    while ((ret = dss_read_packet(s, pkt)) >= 0 || ret == FFERROR_REDO) {
        if (ret >= 0)
            frames++;
        av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
    ctx->total_frames = ctx->frames_read;

    return frames;
}

static int dss_first_format(AVFormatContext *s)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = ctx->pb;
    uint8_t header[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int64_t pos = ctx->dss_header_size, end = INT64_MAX;
    int copy, ret, first = -1, held = 0;

    if (!(pb->seekable & AVIO_SEEKABLE_NORMAL)) {
        end = pos + FFMAX(FFMIN(s->probesize, INT_MAX / 2),
                          DSS_FORMAT_SCAN_BLOCKS * DSS_BLOCK_SIZE);
        if (avio_seek(pb, 0, SEEK_SET) < 0)
            return DSS_ACODEC_DSS_SP;
        if ((ret = ffio_ensure_seekback(pb, end)) < 0)
            return ret;
    }

    for (; pos < end; pos += DSS_BLOCK_SIZE) {
        if (avio_seek(pb, pos, SEEK_SET) < 0)
            break;
        ret = avio_read(pb, header, DSS_AUDIO_BLOCK_HEADER_SIZE);
        if (ret < 0 && ret != AVERROR_EOF)
            return ret;
        if (ret != DSS_AUDIO_BLOCK_HEADER_SIZE)
            break;
        copy = dss_header_copy_blocks(header);
        if (copy)
            pos += (copy - 1) * DSS_BLOCK_SIZE;
        else if (dss_block_frames(header)) {
            if (dss_format_known(header))
                return header[4];
            if (first < 0 && header[3] == 0xff)
                first = header[4];
            held = 1;
        }
    }
    if (first >= 0)
        return first;
    return held ? AVERROR_INVALIDDATA : DSS_ACODEC_DSS_SP;
}

static int dss_formats_mixed(AVFormatContext *s, int first)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = ctx->pb;
    uint8_t header[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int64_t pos;
    int copy, format;

    if (!(pb->seekable & AVIO_SEEKABLE_NORMAL))
        return 1;

    for (pos = ctx->dss_header_size; ; pos += DSS_BLOCK_SIZE) {
        if (avio_seek(pb, pos, SEEK_SET) < 0 ||
            avio_read(pb, header, sizeof(header)) != sizeof(header))
            return 0;
        if ((copy = dss_header_copy_blocks(header))) {
            pos += (copy - 1) * DSS_BLOCK_SIZE;
            continue;
        }
        format = dss_block_part(header);
        if (dss_block_frames(header) && format >= 0 && format != first)
            return 1;
    }
}

static int dss_read_header(AVFormatContext *s)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st;
    uint8_t buf[DSS_BLOCK_SIZE + 4];
    int64_t ret64, frames;
    unsigned int magic;
    int ret, version, is_ds2, format;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    ret = avio_read(pb, buf, sizeof(buf));
    if (ret < 4)
        return ret < 0 ? ret : AVERROR_INVALIDDATA;
    magic   = AV_RL32(buf);
    if (memcmp(buf + 1, "dss", 3) && memcmp(buf + 1, "ds2", 3) &&
        magic != DS2_ENCRYPTED_MAGIC)
        return AVERROR_INVALIDDATA;
    version = buf[0];
    is_ds2  = magic >> 24 == '2' || magic == DS2_ENCRYPTED_MAGIC;
    ctx->dss_header_size = is_ds2 ? dss_ds2_header_size(buf, ret)
                                  : version * DSS_BLOCK_SIZE;

    ctx->pb = s->pb;
    if (magic == DS2_ENCRYPTED_MAGIC) {
        ret = ff_ds2_decrypt_open(s, ctx->password, ctx->dss_header_size,
                                  &ctx->decrypt_pb);
        if (ret < 0)
            return ret;
        ctx->pb = ctx->decrypt_pb;
    }

    /* A DS2 header can leave the recording date zeroed, which does not
     * parse. The audio does not depend on these fields, so in a DS2 file a
     * field that does not parse is left out. */
    if (is_ds2) {
        ret = ff_dss_read_ds2_metadata(s);
        if (ret < 0)
            av_log(s, AV_LOG_WARNING, "Failed to read DS2 metadata\n");
    } else {
        ret = dss_read_metadata_string(s, DSS_HEAD_OFFSET_AUTHOR,
                                       DSS_AUTHOR_SIZE, "author");
        if (ret)
            return ret;

        ret = dss_read_metadata_date(s, DSS_HEAD_OFFSET_END_TIME, "date");
        if (ret)
            return ret;

        ret = dss_read_metadata_string(s, DSS_HEAD_OFFSET_COMMENT,
                                       DSS_COMMENT_SIZE, "comment");
        if (ret)
            return ret;
    }

    format = dss_first_format(s);
    if (format < 0)
        return format;

    ctx->mixed = dss_formats_mixed(s, dss_part_format(format));

    if (ctx->mixed && dss_part_format(format) >= 0) {
        if (ctx->extract != DS2_EXTRACT_ALL &&
            (ctx->pb->seekable & AVIO_SEEKABLE_NORMAL)) {
            avpriv_report_missing_feature(s, "extract in a file of more "
                                          "than one format");
            return AVERROR_PATCHWELCOME;
        }
        ctx->first_format = format;
        dss_start_part(ctx, format);
        st->codecpar->codec_id    = AV_CODEC_ID_DSS_MIXED;
        st->codecpar->sample_rate = dss_part_rate(ctx->audio_codec);
    } else if (format == DSS_ACODEC_DSS_SP || format == DSS_ACODEC_DSS_SP_VR) {
        ctx->audio_codec = format;
        ctx->grundig = 1;
        st->codecpar->codec_id    = AV_CODEC_ID_GRUNDIG_SP;
        st->codecpar->sample_rate = DSS_GRUNDIG_SAMPLE_RATE;
        /* The decoder reads the format to tell variable rate SP apart. */
        if ((ret = ff_alloc_extradata(st->codecpar, 1)) < 0)
            return ret;
        st->codecpar->extradata[0] = ctx->audio_codec;
    } else if (format == DSS_ACODEC_G723_1 ||
               format == DSS_ACODEC_G723_1_ALT) {
        ctx->audio_codec = DSS_ACODEC_G723_1;
        st->codecpar->codec_id    = AV_CODEC_ID_G723_1;
        st->codecpar->sample_rate = 8000;
    } else {
        avpriv_request_sample(s, "Support for codec %x in DSS", format);
        return AVERROR_PATCHWELCOME;
    }

    st->codecpar->codec_type     = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout      = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;

    avpriv_set_pts_info(st, 64, 1, ctx->mixed ? DSS_MIXED_TIME_BASE
                                              : st->codecpar->sample_rate);
    st->start_time = 0;

    if (is_ds2) {
        ret = ff_dss_read_ds2_annotations(s, ctx->dss_header_size,
                                          dss_frame_samples(ctx) * 1000 /
                                          st->codecpar->sample_rate,
                                          &ctx->annotations);
        if (ret < 0)
            return ret;
    }
    if (ctx->extract == DS2_EXTRACT_ANNOTATIONS && !ctx->annotations.nb)
        av_log(s, AV_LOG_WARNING, "No annotations to extract\n");

    ctx->redo_frame = -1;
    ctx->pts        = 0;
    ctx->stated_pts = 0;
    frames = dss_count_frames(s);
    if (frames < 0)
        return (int)frames;
    if (frames > 0) {
        st->nb_frames = frames;
        st->duration  = ctx->mixed ? ctx->pts
                                   : frames * dss_frame_samples(ctx);
    }
    if (ctx->mixed)
        dss_start_part(ctx, format);
    ctx->pts        = 0;
    ctx->stated_pts = 0;

    /* Jump over header */

    if ((ret64 = avio_seek(pb, ctx->dss_header_size, SEEK_SET)) < 0)
        return (int)ret64;

    ctx->swap        = 0;
    ctx->frames_left = 0;
    ctx->frames_read = 0;
    ctx->frames_out  = 0;
    ctx->skipped     = 0;
    ctx->dss_sp_swap_byte = -1;
    /* dss_next_block steps forward first, so start one block short. */
    ctx->block_pos   = ctx->dss_header_size - DSS_BLOCK_SIZE;

    return 0;
}

static int dss_seek_block(AVFormatContext *s, int64_t pos)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = ctx->pb;
    int64_t keep = FFMAX(pos - DSS_BLOCK_KEEP_BEFORE, ctx->dss_header_size);
    int ret;

    if (!(pb->seekable & AVIO_SEEKABLE_NORMAL)) {
        if (avio_seek(pb, keep, SEEK_SET) < 0)
            return AVERROR_EOF;
        ret = ffio_ensure_seekback(pb, pos - keep + 2 * DSS_BLOCK_SIZE);
        if (ret < 0)
            return ret;
    }
    return avio_seek(pb, pos, SEEK_SET) < 0 ? AVERROR_EOF : 0;
}

/* Read `size` bytes of frame payload, stepping over any block header met on
 * the way. Block payloads are contiguous as far as the frames are concerned,
 * and a frame is short enough to straddle at most one boundary. */
static int dss_read_payload(AVFormatContext *s, uint8_t *buf, int size)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = ctx->pb;
    int ret;

    while (size > 0) {
        int64_t off = (avio_tell(pb) - ctx->dss_header_size) % DSS_BLOCK_SIZE;
        int chunk;

        if (!off) {
            uint8_t header[DSS_AUDIO_BLOCK_HEADER_SIZE];

            /* A frame does not go on into a block in another format. */
            if (ffio_read_size(pb, header, sizeof(header)) < 0)
                return AVERROR_EOF;
            if (dss_other_format(ctx, header)) {
                ctx->block_pos = avio_tell(pb) - sizeof(header) -
                                 DSS_BLOCK_SIZE;
                return DSS_PART_END;
            }
            continue;
        }
        if (off < DSS_AUDIO_BLOCK_HEADER_SIZE) {
            if (avio_skip(pb, DSS_AUDIO_BLOCK_HEADER_SIZE - off) < 0)
                return AVERROR_EOF;
            continue;
        }
        chunk = FFMIN(size, (int)(DSS_BLOCK_SIZE - off));
        ret = ffio_read_size(pb, buf, chunk);
        if (ret < 0)
            return ret;
        buf  += chunk;
        size -= chunk;
    }
    return 0;
}

/* Move to the next audio block and take its framing from its header.
 *
 * Each block states the length of the frame fragment carried over from the
 * previous block, the byte-swap parity of its first whole frame, and how many
 * frames it holds. Deriving the framing by running on from the previous frame
 * instead gives the same answer for as long as nothing disturbs the recording,
 * which is why most files decode; but a paused or edited recording restates
 * them, and a running walk then reads every remaining frame a byte out of
 * phase. Believing the block keeps the walk in step, and costs nothing when
 * the two agree. */
static int dss_next_block(AVFormatContext *s)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = ctx->pb;
    uint8_t header[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int64_t block_pos = ctx->block_pos, stated_pts = ctx->stated_pts;
    int cont, copy, ret;

    for (;;) {
        ctx->block_pos += DSS_BLOCK_SIZE;
        if ((ret = dss_seek_block(s, ctx->block_pos)) < 0)
            goto fail;
        ret = avio_read(pb, header, DSS_AUDIO_BLOCK_HEADER_SIZE);
        if (ret < 0 && ret != AVERROR_EOF)
            goto fail;
        if (ret != DSS_AUDIO_BLOCK_HEADER_SIZE) {
            ret = AVERROR_EOF;
            goto fail;
        }
        if ((copy = dss_header_copy_blocks(header))) {
            ctx->block_pos += (copy - 1) * DSS_BLOCK_SIZE;
            continue;
        }
        if ((ret = dss_block_frames_read(s, header)) < 0) {
            ctx->block_pos -= DSS_BLOCK_SIZE;
            return ret;
        }

        ctx->swap = !!(header[0] & 0x80);
        cont      = 2 * header[1] + 2 * ctx->swap - DSS_AUDIO_BLOCK_HEADER_SIZE;
        if (cont < 0 || cont > DSS_BLOCK_SIZE - DSS_AUDIO_BLOCK_HEADER_SIZE)
            cont = 0;

        ctx->frames_left = ret;
        ctx->stated_pts += (int64_t)ret * dss_part_ticks(ctx->audio_codec);
        if (!ctx->frames_left)
            continue;

        /* A swapped frame takes one byte from the frame before it, two bytes
         * before its own start, in this block or at the end of the previous
         * one. Read it from there rather than carry it over: after a seek, at
         * the start of a cut file, or where a block restarts the framing, the
         * frame before has not been read. */
        if (ctx->swap) {
            int64_t at = ctx->block_pos + cont - 2;

            if (cont >= 2)
                at += DSS_AUDIO_BLOCK_HEADER_SIZE;
            if (at >= ctx->dss_header_size + DSS_AUDIO_BLOCK_HEADER_SIZE &&
                avio_seek(pb, at, SEEK_SET) >= 0)
                ctx->dss_sp_swap_byte = avio_r8(pb);
        }

        if (avio_seek(pb, ctx->block_pos + DSS_AUDIO_BLOCK_HEADER_SIZE + cont,
                      SEEK_SET) < 0) {
            ret = AVERROR_EOF;
            goto fail;
        }
        return 0;
    }

fail:
    ctx->block_pos   = block_pos;
    ctx->stated_pts  = stated_pts;
    ctx->frames_left = 0;
    return ret;
}

static int dss_frame_cut(int ret)
{
    return ret == DSS_PART_END ? ret : AVERROR_EOF;
}

/* Offset of the next frame. A frame can start where a block does, after the
 * block header that dss_read_payload() steps over. */
static int64_t dss_frame_pos(AVFormatContext *s)
{
    DSSDemuxContext *ctx = s->priv_data;
    int64_t pos = avio_tell(ctx->pb);
    int64_t off = (pos - ctx->dss_header_size) % DSS_BLOCK_SIZE;

    if (off < DSS_AUDIO_BLOCK_HEADER_SIZE)
        pos += DSS_AUDIO_BLOCK_HEADER_SIZE - off;
    return pos;
}

/* size is the frame length in bytes, 41 or 9; both are odd, so the parity
 * flips after every frame. */
static void dss_sp_byte_swap(DSSDemuxContext *ctx, uint8_t *data, int size)
{
    int i;

    if (ctx->swap) {
        for (i = 0; i < size - 1; i += 2)
            data[i] = data[i + 4];

        data[1] = ctx->dss_sp_swap_byte;
    } else {
        ctx->dss_sp_swap_byte = data[size - 1];
    }

    /* Zero the padding, and make sure the byte past the frame is 0. */
    memset(data + size + 1, 0, DSS_FRAME_SIZE - size);
    data[size - 1] = 0;
    ctx->swap     ^= 1;
}

static int dss_sp_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;
    int size = DSS_FRAME_SIZE - 1;
    int ret, done = 0;
    int64_t pos;

    if (ctx->frames_left <= 0) {
        ret = dss_next_block(s);
        if (ret < 0)
            return ret;
    }
    pos = dss_frame_pos(s);

    ret = av_new_packet(pkt, DSS_FRAME_SIZE);
    if (ret < 0)
        return ret;

    pkt->duration     = 264;
    pkt->pts          = ctx->frames_read++ * 264;
    pkt->pos = pos;
    pkt->stream_index = 0;

    /* In variable rate SP the top bit of the frame's first byte, in 16-bit
     * word order, is clear for a short frame. A swapped frame starts with the
     * byte carried over from the previous frame; otherwise it is the second
     * byte read. */
    if (ctx->audio_codec == DSS_ACODEC_DSS_SP_VR) {
        if (ctx->swap) {
            if (ctx->dss_sp_swap_byte >= 0 && !(ctx->dss_sp_swap_byte & 0x80))
                size = DSS_SP_SHORT_FRAME_SIZE;
        } else {
            ret = dss_read_payload(s, pkt->data, 2);
            if (ret < 0)
                return dss_frame_cut(ret); /* no whole frame left */
            done = 2;
            if (!(pkt->data[1] & 0x80))
                size = DSS_SP_SHORT_FRAME_SIZE;
        }
    }

    /* A swapped frame reads one byte less than its size, starting at offset
     * 3, which writes one byte into pkt's padding for a long frame; an
     * unswapped one reads one byte more. */
    if (ctx->swap)
        ret = dss_read_payload(s, pkt->data + 3, size - 1);
    else
        ret = dss_read_payload(s, pkt->data + done, size + 1 - done);
    if (ret < 0)
        return dss_frame_cut(ret);

    ctx->frames_left--;

    dss_sp_byte_swap(ctx, pkt->data, size);

    if (ctx->dss_sp_swap_byte < 0) {
        ctx->redo_frame = ctx->frames_read - 1;
        return FFERROR_REDO;
    }

    return 0;
}

static int dss_723_1_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;
    uint8_t byte;
    int size, ret;
    int64_t pos;

    if (ctx->frames_left <= 0) {
        ret = dss_next_block(s);
        if (ret < 0)
            return ret;
    }
    pos = dss_frame_pos(s);

    ret = dss_read_payload(s, &byte, 1);
    if (ret < 0)
        return dss_frame_cut(ret); /* no whole frame left */
    if (byte == 0xff)
        return AVERROR_INVALIDDATA;

    size = frame_size[byte & 3];

    ret = av_new_packet(pkt, size);
    if (ret < 0)
        return ret;
    pkt->pos = pos;

    pkt->data[0]  = byte;
    pkt->duration = 240;
    pkt->pts      = ctx->frames_read++ * 240;

    pkt->stream_index = 0;

    ret = dss_read_payload(s, pkt->data + 1, size - 1);
    if (ret < 0)
        return dss_frame_cut(ret); /* no whole frame left */

    ctx->frames_left--;

    return 0;
}

/* Move to the next audio block that has frames, and set the bit reader to the
 * first of them. Byte 1 of the block header is its offset from the start of
 * the block in 16-bit words, and the top 4 bits of byte 0 the bit in that
 * word. An offset into the block header is taken as 0, as dss_next_block()
 * takes it. */
static int dss_grundig_next_block(AVFormatContext *s)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = ctx->pb;
    uint8_t header[DSS_AUDIO_BLOCK_HEADER_SIZE], word[2];
    int64_t at, block_pos = ctx->block_pos, stated_pts = ctx->stated_pts;
    int64_t frames_read = ctx->frames_read, pts = ctx->pts;
    int start, copy, ret;

    for (;;) {
        ctx->block_pos += DSS_BLOCK_SIZE;
        if ((ret = dss_seek_block(s, ctx->block_pos)) < 0)
            goto fail;
        ret = avio_read(pb, header, DSS_AUDIO_BLOCK_HEADER_SIZE);
        if (ret < 0 && ret != AVERROR_EOF)
            goto fail;
        if (ret != DSS_AUDIO_BLOCK_HEADER_SIZE) {
            ret = AVERROR_EOF;
            goto fail;
        }
        if ((copy = dss_header_copy_blocks(header))) {
            ctx->block_pos += (copy - 1) * DSS_BLOCK_SIZE;
            continue;
        }
        if ((ret = dss_block_frames_read(s, header)) < 0) {
            ctx->block_pos -= DSS_BLOCK_SIZE;
            return ret;
        }

        ctx->frames_left = ret;
        ctx->stated_pts += (int64_t)ret * dss_part_ticks(ctx->audio_codec);
        if (!ctx->frames_left)
            continue;

        start = (header[1] - DSS_AUDIO_BLOCK_HEADER_SIZE / 2) * 16 +
                (header[0] >> 4);
        if (start < 0 && header[0] & 0x80) {
            at = ctx->block_pos - (15 - start) / 16 * 2;
            if (at >= ctx->dss_header_size + DSS_AUDIO_BLOCK_HEADER_SIZE) {
                if (avio_seek(pb, at, SEEK_SET) < 0 ||
                    ffio_read_size(pb, word, 2) < 0) {
                    ret = AVERROR_EOF;
                    goto fail;
                }
                ctx->grundig_word = AV_RL16(word);
                ctx->grundig_bits = (15 - start) % 16 + 1;
                return 0;
            }
            while (start < 0 && ctx->frames_left) {
                start += DSS_GRUNDIG_FRAME_BITS;
                ctx->frames_left--;
                ctx->frames_read++;
                if (ctx->mixed)
                    ctx->pts += dss_part_ticks(ctx->audio_codec);
            }
            if (!ctx->frames_left)
                continue;
        }
        if (start < 0)
            start = 0;
        if (avio_seek(pb, ctx->block_pos + DSS_AUDIO_BLOCK_HEADER_SIZE +
                      start / 16 * 2, SEEK_SET) < 0 ||
            ffio_read_size(pb, word, 2) < 0) {
            ret = AVERROR_EOF;
            goto fail;
        }
        ctx->grundig_word = AV_RL16(word);
        ctx->grundig_bits = 16 - start % 16;
        return 0;
    }

fail:
    ctx->block_pos   = block_pos;
    ctx->stated_pts  = stated_pts;
    ctx->frames_read = frames_read;
    ctx->pts         = pts;
    ctx->frames_left = 0;
    return ret;
}

static int dss_grundig_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;
    uint8_t word[2];
    int64_t pos;
    int i, ret, bits = DSS_GRUNDIG_FRAME_BITS;

    if (ctx->frames_left <= 0) {
        ret = dss_grundig_next_block(s);
        if (ret < 0)
            return ret;
    }
    /* The offset of the word that holds the first bit of the frame. */
    pos = ctx->grundig_bits ? avio_tell(ctx->pb) - 2 : dss_frame_pos(s);

    if ((ret = av_new_packet(pkt, DSS_GRUNDIG_FRAME_SIZE)) < 0)
        return ret;
    memset(pkt->data, 0, DSS_GRUNDIG_FRAME_SIZE);

    for (i = 0; i < bits; i++) {
        if (!ctx->grundig_bits) {
            if ((ret = dss_read_payload(s, word, 2)) < 0)
                return dss_frame_cut(ret); /* no whole frame left */
            ctx->grundig_word = AV_RL16(word);
            ctx->grundig_bits = 16;
        }
        ctx->grundig_bits--;
        if ((ctx->grundig_word >> ctx->grundig_bits) & 1)
            pkt->data[i >> 3] |= 0x80 >> (i & 7);
        else if (!i && ctx->audio_codec == DSS_ACODEC_DSS_SP_VR)
            bits = 72;
    }
    if (bits != DSS_GRUNDIG_FRAME_BITS)
        av_shrink_packet(pkt, bits / 8);

    ctx->frames_left--;

    pkt->stream_index = 0;
    pkt->pts          = ctx->frames_read++ * DSS_GRUNDIG_SAMPLES_PER_FRAME;
    pkt->duration     = DSS_GRUNDIG_SAMPLES_PER_FRAME;
    pkt->pos          = pos;
    pkt->flags       |= AV_PKT_FLAG_KEY;

    return 0;
}

static int dss_qp_anchor(const uint8_t *header)
{
    return av_clip(2 * header[1] + 2 * (header[0] >> 7) -
                   DSS_AUDIO_BLOCK_HEADER_SIZE,
                   0, DSS_BLOCK_SIZE - DSS_AUDIO_BLOCK_HEADER_SIZE);
}

/* 1 for a re-sync block, whose anchor is not where the frame in flight ends */
static int dss_qp_next_block(AVFormatContext *s, int in_frame)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = ctx->pb;
    uint8_t header[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int64_t block_pos = ctx->block_pos;
    int anchor, copy, ret;

    for (;;) {
        ctx->block_pos += DSS_BLOCK_SIZE;
        if ((ret = dss_seek_block(s, ctx->block_pos)) < 0)
            break;
        ret = avio_read(pb, header, DSS_AUDIO_BLOCK_HEADER_SIZE);
        if (ret >= 0 && ret != DSS_AUDIO_BLOCK_HEADER_SIZE)
            ret = AVERROR_EOF;
        if (ret < 0)
            break;
        if (!(copy = dss_header_copy_blocks(header)))
            break;
        ctx->block_pos += (copy - 1) * DSS_BLOCK_SIZE;
    }
    if (ret < 0) {
        ctx->block_pos   = block_pos;
        ctx->frames_left = 0;
        ctx->qp_left     = 0;
        return ret;
    }
    if ((ret = dss_block_frames_read(s, header)) < 0) {
        ctx->block_pos -= DSS_BLOCK_SIZE;
        return ret;
    }

    ctx->frames_left = ret;
    if (ret && dss_block_part(header) == DSS_ACODEC_QP)
        ctx->qp_block_format = header[4];
    anchor = dss_qp_anchor(header);

    if (!ctx->frames_left) {
        ctx->qp_left = anchor;
        return 0;
    }
    if (in_frame ? in_frame + anchor != ctx->qp_frame_size : anchor) {
        ctx->qp_left = DSS_BLOCK_SIZE - DSS_AUDIO_BLOCK_HEADER_SIZE - anchor;
        if (avio_skip(pb, anchor) < 0)
            ctx->qp_left = 0;
        return 1;
    }
    ctx->qp_left = DSS_BLOCK_SIZE - DSS_AUDIO_BLOCK_HEADER_SIZE;
    return 0;
}

static int dss_qp_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;
    int64_t pos;
    int ret, size, offset = 0;

    while (ctx->frames_left <= 0 || !ctx->qp_left)
        if ((ret = dss_qp_next_block(s, 0)) < 0)
            return ret;
    pos = avio_tell(ctx->pb);

    if ((ret = av_new_packet(pkt, DSS_QP_FRAME_SIZE)) < 0)
        return ret;

    ctx->qp_frame_size = DSS_QP_FRAME_SIZE;
    while (offset < ctx->qp_frame_size) {
        size = FFMIN(ctx->qp_frame_size - offset, ctx->qp_left);
        if (size <= 0) {
            ret = dss_qp_next_block(s, offset);
            if (ret == AVERROR_EOF)
                goto truncated;
            if (ret == DSS_PART_END)
                ctx->stated_pts += dss_part_ticks(ctx->audio_codec);
            if (ret < 0)
                return ret;
            if (ret) {
                offset = 0;
                ctx->qp_frame_size = DSS_QP_FRAME_SIZE;
                pos = avio_tell(ctx->pb);
            }
            continue;
        }

        if (!offset) {
            ctx->qp_frame_format = ctx->qp_block_format;
            /* QP7: the type bit in the first 16-bit word gives the size */
            if (ctx->qp_frame_format == DSS_ACODEC_QP7)
                size = FFMIN(size, 2);
        }
        ret = avio_read(ctx->pb, pkt->data + offset, size);
        if (ret < size) {
            if (ret < 0 && ret != AVERROR_EOF)
                return ret;
            if (ret > 0)
                offset += ret;
            goto truncated;
        }
        if (!offset) {
            ctx->frames_left--;
            if (ctx->qp_frame_format == DSS_ACODEC_QP7 &&
                !(pkt->data[1] & 0x80))
                ctx->qp_frame_size = DSS_QP7_NOISE_FRAME_SIZE;
        }
        offset += size;
        ctx->qp_left -= size;
    }
    goto done;

truncated:
    /* zero-filled, as in the ds2 demuxer */
    if (!offset)
        return AVERROR_EOF;
    memset(pkt->data + offset, 0, ctx->qp_frame_size - offset);
    ctx->qp_left = 0;
done:
    if (ctx->qp_frame_size != DSS_QP_FRAME_SIZE)
        av_shrink_packet(pkt, ctx->qp_frame_size);
    ctx->stated_pts += dss_part_ticks(ctx->audio_codec);

    pkt->stream_index = 0;
    pkt->pts          = ctx->frames_read++ * DSS_QP_SAMPLES_PER_FRAME;
    pkt->duration     = DSS_QP_SAMPLES_PER_FRAME;
    pkt->pos          = pos;
    pkt->flags       |= AV_PKT_FLAG_KEY;

    return 0;
}

static int dss_read_frame(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;
    uint8_t header[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int64_t frames_read;
    int ret, format;

    for (;;) {
        frames_read = ctx->frames_read;
        if (ctx->audio_codec == DSS_ACODEC_QP)
            ret = dss_qp_read_packet(s, pkt);
        else if (ctx->grundig)
            ret = dss_grundig_read_packet(s, pkt);
        else if (ctx->audio_codec != DSS_ACODEC_G723_1)
            ret = dss_sp_read_packet(s, pkt);
        else
            ret = dss_723_1_read_packet(s, pkt);
        if (ret != DSS_PART_END)
            break;

        av_packet_unref(pkt);
        ctx->frames_read = frames_read;
        format = ctx->audio_codec;
        if (ctx->mixed) {
            if (ctx->extract != DS2_EXTRACT_ALL) {
                avpriv_report_missing_feature(s, "extract in a file of more "
                                              "than one format");
                return AVERROR_PATCHWELCOME;
            }
            if (avio_seek(ctx->pb, ctx->block_pos + DSS_BLOCK_SIZE,
                          SEEK_SET) < 0 ||
                ffio_read_size(ctx->pb, header, sizeof(header)) < 0)
                return AVERROR_EOF;
            if (dss_block_part(header) < 0)
                return AVERROR_BUG;
            format = header[4];
        }
        dss_start_part(ctx, format);
        if (ctx->mixed)
            ctx->pts = ctx->stated_pts;
    }
    if (ret < 0 || !ctx->mixed)
        return ret;

    pkt->duration = dss_part_ticks(ctx->audio_codec);
    pkt->pts      = ctx->pts;
    ctx->pts     += pkt->duration;
    if ((ret = av_grow_packet(pkt, 1)) < 0)
        return ret;
    memmove(pkt->data + 1, pkt->data, pkt->size - 1);
    pkt->data[0] = ctx->audio_codec == DSS_ACODEC_QP ? ctx->qp_frame_format
                                                     : ctx->audio_codec;
    return 0;
}

/* Return the next frame that the extract option keeps. The first one after
 * frames left out tells the decoder to start afresh. */
static int dss_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;
    int ret;

    for (;;) {
        ret = dss_read_frame(s, pkt);
        if (ret < 0)
            return ret;
        if (ff_dss_ds2_frame_included(&ctx->annotations, ctx->extract,
                                      ctx->frames_read - 1))
            break;
        av_packet_unref(pkt);
        ctx->skipped = 1;
    }

    if (ctx->extract != DS2_EXTRACT_ALL)
        pkt->pts = ctx->frames_out++ * (ctx->mixed ?
                                        dss_part_ticks(ctx->audio_codec) :
                                        dss_frame_samples(ctx));
    if (ctx->skipped) {
        ctx->skipped = 0;
        if (!av_packet_new_side_data(pkt, AV_PKT_DATA_DECODER_RESET, 0))
            return AVERROR(ENOMEM);
    }
    return 0;
}

static int dss_qp_block_frames(AVFormatContext *s, int64_t pos,
                               const uint8_t *header, int qp7, int count)
{
    AVIOContext *pb = ((DSSDemuxContext *)s->priv_data)->pb;
    uint8_t buf[DSS_BLOCK_SIZE];
    int64_t size = FFMIN(avio_size(pb) - pos, DSS_BLOCK_SIZE);
    int off = DSS_AUDIO_BLOCK_HEADER_SIZE + dss_qp_anchor(header);
    int n = 0, need, copy, ret;

    memcpy(buf, header, DSS_AUDIO_BLOCK_HEADER_SIZE);
    if (qp7 && size > DSS_AUDIO_BLOCK_HEADER_SIZE &&
        (ret = ffio_read_size(pb, buf + DSS_AUDIO_BLOCK_HEADER_SIZE,
                              size - DSS_AUDIO_BLOCK_HEADER_SIZE)) < 0)
        return ret;
    while (n < count && off < size) {
        n++;
        if (qp7 && off + 1 < size && !(buf[off + 1] & 0x80))
            off += DSS_QP7_NOISE_FRAME_SIZE;
        else
            off += DSS_QP_FRAME_SIZE;
    }
    need = size == DSS_BLOCK_SIZE ? off - DSS_BLOCK_SIZE : 0;
    while (need > 0) {
        pos += DSS_BLOCK_SIZE;
        if (avio_seek(pb, pos, SEEK_SET) < 0 ||
            avio_read(pb, buf, DSS_AUDIO_BLOCK_HEADER_SIZE) !=
                DSS_AUDIO_BLOCK_HEADER_SIZE)
            break;
        if ((copy = dss_header_copy_blocks(buf))) {
            pos += (copy - 1) * DSS_BLOCK_SIZE;
            continue;
        }
        if (dss_block_frames(buf)) {
            if ((dss_block_part(buf) < 0 ||
                 dss_block_part(buf) == DSS_ACODEC_QP) &&
                dss_qp_anchor(buf) != need)
                n--;
            break;
        }
        need -= dss_qp_anchor(buf);
    }
    return n;
}

static int dss_mixed_seek(AVFormatContext *s, int64_t timestamp)
{
    DSSDemuxContext *ctx = s->priv_data;
    uint8_t header[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int64_t pos, pts = 0, frames = 0, length;
    int64_t last = -1, last_pts = 0, last_frames = 0;
    int copy, count, format = dss_part_format(ctx->first_format);
    int last_format = format;
    int qp_format = ctx->first_format, last_qp_format = qp_format;

    for (pos = ctx->dss_header_size; ; pos += DSS_BLOCK_SIZE) {
        if (avio_seek(ctx->pb, pos, SEEK_SET) < 0 ||
            ffio_read_size(ctx->pb, header, sizeof(header)) < 0) {
            if (last < 0)
                return AVERROR_EOF;
            pos    = last;
            pts    = last_pts;
            frames = last_frames;
            format = last_format;
            qp_format = last_qp_format;
            break;
        }
        if ((copy = dss_header_copy_blocks(header))) {
            pos += (copy - 1) * DSS_BLOCK_SIZE;
            continue;
        }
        if (!(count = dss_block_frames(header)))
            continue;
        /* an unread format byte leaves the block to the format before it */
        if (dss_block_part(header) >= 0)
            format = dss_block_part(header);
        if (dss_block_part(header) == DSS_ACODEC_QP)
            qp_format = header[4];
        if (format == DSS_ACODEC_QP) {
            count = dss_qp_block_frames(s, pos, header,
                                        qp_format == DSS_ACODEC_QP7, count);
            if (count < 0)
                return count;
            if (!count)
                continue;
        }
        length = (int64_t)count * dss_part_ticks(format);
        if (pts + length > timestamp)
            break;
        last        = pos;
        last_pts    = pts;
        last_frames = frames;
        last_format = format;
        last_qp_format = qp_format;
        pts        += length;
        frames     += count;
    }

    dss_set_part(ctx, format);
    ctx->qp_block_format = qp_format;
    ctx->pts         = pts;
    ctx->stated_pts  = pts;
    ctx->frames_read = frames;
    ctx->frames_out  = frames;
    ctx->skipped     = 0;
    ctx->block_pos   = pos - DSS_BLOCK_SIZE;
    return 0;
}

static int dss_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags)
{
    DSSDemuxContext *ctx = s->priv_data;
    int samples = dss_frame_samples(ctx);
    int64_t ret, seekto, target = timestamp / samples, frames = 0;
    uint8_t header[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int copy, count;

    if (ctx->mixed)
        return dss_mixed_seek(s, timestamp);

    /* With extract, timestamps count the frames returned: find the frame of
     * the file that the target is, or past the end the last one returned. */
    if (ctx->extract != DS2_EXTRACT_ALL) {
        int64_t out = target, f;

        for (f = target = 0; f < ctx->total_frames; f++) {
            if (f == ctx->redo_frame ||
                !ff_dss_ds2_frame_included(&ctx->annotations, ctx->extract, f))
                continue;
            target = f;
            if (!out--)
                break;
        }
    }

    /* Frame sizes vary, and a block can hold fewer frames than would fit, so
     * find the block from the per-block frame counts, taken as in
     * dss_next_block(). */
    for (seekto = ctx->dss_header_size; ; seekto += DSS_BLOCK_SIZE) {
        ret = avio_seek(ctx->pb, seekto, SEEK_SET);
        if (ret >= 0)
            ret = ffio_read_size(ctx->pb, header, DSS_AUDIO_BLOCK_HEADER_SIZE);
        if (ret < 0) {
            /* Past the end: scan again for the block with the last frame. */
            if (!frames)
                return ret;
            target = frames - 1;
            frames = 0;
            seekto = ctx->dss_header_size - DSS_BLOCK_SIZE;
            continue;
        }
        if ((copy = dss_header_copy_blocks(header))) {
            seekto += (copy - 1) * DSS_BLOCK_SIZE;
            continue;
        }
        count = dss_block_frames(header);
        if (frames + count > target)
            break;
        frames += count;
    }

    ctx->frames_read = frames;
    ctx->frames_out  = 0;
    for (frames = 0; frames < ctx->frames_read; frames++)
        ctx->frames_out += frames != ctx->redo_frame &&
                           ff_dss_ds2_frame_included(&ctx->annotations,
                                                     ctx->extract, frames);
    ctx->skipped     = 0;
    ctx->swap        = !!(header[0] & 0x80);
    ctx->frames_left = 0;
    ctx->block_pos   = seekto - DSS_BLOCK_SIZE;
    ctx->dss_sp_swap_byte = -1;
    return 0;
}

static int dss_read_close(AVFormatContext *s)
{
    DSSDemuxContext *ctx = s->priv_data;

    ff_ds2_decrypt_close(&ctx->decrypt_pb);
    return 0;
}

#define OFFSET(x) offsetof(DSSDemuxContext, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM
static const AVOption dss_options[] = {
    { "password", "Decryption password for encrypted DS2 (max 16 bytes)",
      OFFSET(password), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, DEC },
    { "extract", "select DS2 recording content to demux",
      OFFSET(extract), AV_OPT_TYPE_INT,
      { .i64 = DS2_EXTRACT_ALL }, DS2_EXTRACT_ALL, DS2_EXTRACT_ANNOTATIONS,
      DEC, .unit = "extract" },
    { "all", "full recording in original order", 0, AV_OPT_TYPE_CONST,
      { .i64 = DS2_EXTRACT_ALL }, 0, 0, DEC, .unit = "extract" },
    { "main", "main dictation only (skip annotations)", 0, AV_OPT_TYPE_CONST,
      { .i64 = DS2_EXTRACT_MAIN }, 0, 0, DEC, .unit = "extract" },
    { "annotations", "annotation segments only", 0, AV_OPT_TYPE_CONST,
      { .i64 = DS2_EXTRACT_ANNOTATIONS }, 0, 0, DEC, .unit = "extract" },
    { NULL },
};

static const AVClass dss_demuxer_class = {
    .class_name = "dss demuxer",
    .item_name  = av_default_item_name,
    .option     = dss_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFInputFormat ff_dss_demuxer = {
    .p.name         = "dss",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Digital Speech Standard (DSS)"),
    .p.extensions   = "dss",
    .p.flags        = AVFMT_NO_BYTE_SEEK,
    .p.priv_class   = &dss_demuxer_class,
    .priv_data_size = sizeof(DSSDemuxContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = dss_probe,
    .read_header    = dss_read_header,
    .read_packet    = dss_read_packet,
    .read_seek      = dss_read_seek,
    .read_close     = dss_read_close,
};
