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

/* Native CELP codec at 12 kHz, 41-byte (328-bit) frames packed as a
 * continuous bitstream across 512-byte audio blocks. */
#define DSS_GRUNDIG_FRAME_SIZE        41
#define DSS_GRUNDIG_FRAME_BITS        328
#define DSS_GRUNDIG_SAMPLE_RATE       12000
#define DSS_GRUNDIG_SAMPLES_PER_FRAME 288

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
    return ctx->audio_codec == DSS_ACODEC_G723_1 ? 240 : 264;
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

static int dss_probe_format(const uint8_t *buf, int size, int pos)
{
    int copy;

    while (pos + DSS_AUDIO_BLOCK_HEADER_SIZE <= size) {
        copy = dss_header_copy_blocks(buf + pos);
        if (copy) {
            pos += copy * DSS_BLOCK_SIZE;
            continue;
        }
        if (dss_block_frames(buf + pos))
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
    ctx->audio_codec = format;

    if (ctx->audio_codec == DSS_ACODEC_DSS_SP ||
        ctx->audio_codec == DSS_ACODEC_DSS_SP_VR) {
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
        avpriv_request_sample(s, "Support for codec %x in DSS",
                              ctx->audio_codec);
        return AVERROR_PATCHWELCOME;
    }

    st->codecpar->codec_type     = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout      = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
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
    frames = dss_count_frames(s);
    if (frames < 0)
        return (int)frames;
    if (frames > 0) {
        st->nb_frames = frames;
        st->duration  = frames * dss_frame_samples(ctx);
    }

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
    int64_t block_pos = ctx->block_pos;
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

        ctx->swap = !!(header[0] & 0x80);
        cont      = 2 * header[1] + 2 * ctx->swap - DSS_AUDIO_BLOCK_HEADER_SIZE;
        if (cont < 0 || cont > DSS_BLOCK_SIZE - DSS_AUDIO_BLOCK_HEADER_SIZE)
            cont = 0;

        ctx->frames_left = dss_block_frames(header);
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
    ctx->frames_left = 0;
    return ret;
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
                return AVERROR_EOF; /* no whole frame left */
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
        return ret == AVERROR_EOF ? ret : AVERROR_EOF;

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
        return AVERROR_EOF; /* no whole frame left */
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
        return AVERROR_EOF; /* no whole frame left */

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
    int64_t at, block_pos = ctx->block_pos;
    int64_t frames_read = ctx->frames_read;
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

        ctx->frames_left = dss_block_frames(header);
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
    ctx->frames_read = frames_read;
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
            if (dss_read_payload(s, word, 2) < 0)
                return AVERROR_EOF; /* no whole frame left */
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

static int dss_read_frame(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;

    if (ctx->grundig)
        return dss_grundig_read_packet(s, pkt);
    else if (ctx->audio_codec != DSS_ACODEC_G723_1)
        return dss_sp_read_packet(s, pkt);
    else
        return dss_723_1_read_packet(s, pkt);
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
        pkt->pts = ctx->frames_out++ * dss_frame_samples(ctx);
    if (ctx->skipped) {
        ctx->skipped = 0;
        if (!av_packet_new_side_data(pkt, AV_PKT_DATA_DECODER_RESET, 0))
            return AVERROR(ENOMEM);
    }
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
