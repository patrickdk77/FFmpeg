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

#include <limits.h>
#include <string.h>

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"

#define DSS_HEAD_OFFSET_AUTHOR        0xc
#define DSS_AUTHOR_SIZE               16

#define DSS_HEAD_OFFSET_START_TIME    0x26
#define DSS_HEAD_OFFSET_END_TIME      0x32
#define DSS_TIME_SIZE                 12

#define DSS_HEAD_OFFSET_ACODEC        0x2a4
#define DSS_ACODEC_DSS_SP             0x0    /* SP mode */
#define DSS_ACODEC_G723_1             0x2    /* LP mode */

/* Grundig DSS-SP (Digta) variant: magic byte 6, fixed 6 x 512-byte header.
 * Native CELP codec at 12 kHz, 41-byte (328-bit) frames packed as a
 * continuous bitstream across 512-byte audio blocks. */
#define DSS_GRUNDIG_VERSION           6
#define DSS_GRUNDIG_FRAME_SIZE        41
#define DSS_GRUNDIG_FRAME_BITS        328
#define DSS_GRUNDIG_SAMPLE_RATE       12000
#define DSS_GRUNDIG_SAMPLES_PER_FRAME 288

#define DSS_HEAD_OFFSET_COMMENT       0x31e
#define DSS_COMMENT_SIZE              64

#define DSS_BLOCK_SIZE                512
#define DSS_AUDIO_BLOCK_HEADER_SIZE   6
#define DSS_FRAME_SIZE                42

static const uint8_t frame_size[4] = { 24, 20, 4, 1 };

typedef struct DSSDemuxContext {
    unsigned int audio_codec;
    int counter;
    int swap;
    int dss_sp_swap_byte;

    int packet_size;
    int dss_header_size;
    int resync_pending;    /* VOX-pause (empty block) re-anchor pending (v4) */
    int total_frames;
    int frames_read;
    int span_continues;    /* set when a frame spans into the next block */
    int block_frames_left; /* frames remaining in compact block (fc fits in payload) */
    int compact_fc;
    int compact_poff;
    int compact_next_idx;  /* frames read in current compact block */
    int compact_stream_off;/* payload cursor (40/42-byte steps, NCH stream) */
    int in_compact;        /* do not let linear path touch block_frames_left */
    int compact_swap;      /* swap bit at start of current compact block */
    int pending_block_fc;
    int pending_compact_poff;
    int pending_compact_swap;
    int64_t block_start_pos;
    int64_t pending_block_start;

    /* Grundig variant: pre-extracted continuous 41-byte frame stream */
    int      grundig;
    uint8_t *grundig_frames;
    int      grundig_nb_frames;
    int      grundig_pos;
} DSSDemuxContext;

#define DSS_BLOCK_PAYLOAD_SIZE (DSS_BLOCK_SIZE - DSS_AUDIO_BLOCK_HEADER_SIZE)

static int dss_block_payload_fits(const uint8_t *hdr)
{
    int poff = FFMAX(0, 2 * hdr[1] + 2 * (hdr[0] >> 7) -
                         DSS_AUDIO_BLOCK_HEADER_SIZE);
    int payload = DSS_BLOCK_SIZE - DSS_AUDIO_BLOCK_HEADER_SIZE;
    int fc = hdr[2];

    return fc > 0 && fc * DSS_FRAME_SIZE + poff <= payload;
}

static int dss_align_after_compact_block(AVFormatContext *s, int64_t block_start)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    uint8_t hdr[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int64_t next = block_start + DSS_BLOCK_SIZE;
    int poff, ret;

    if (avio_seek(pb, next, SEEK_SET) < 0)
        return AVERROR(EIO);

    ret = avio_read(pb, hdr, sizeof(hdr));
    if (ret < (int)sizeof(hdr))
        return ret < 0 ? ret : AVERROR_EOF;

    poff = FFMAX(0, 2 * hdr[1] + 2 * (hdr[0] >> 7) -
                     DSS_AUDIO_BLOCK_HEADER_SIZE);

    ctx->block_start_pos = next;
    ctx->swap              = hdr[0] >> 7;
    if (dss_block_payload_fits(hdr)) {
        ctx->block_frames_left = hdr[2];
        ctx->compact_fc        = hdr[2];
        ctx->compact_poff      = poff;
        ctx->compact_next_idx  = 0;
        ctx->compact_stream_off = 0;
        ctx->in_compact        = 1;
        ctx->compact_swap      = ctx->swap;
    } else {
        ctx->block_frames_left = 0;
        ctx->in_compact        = 0;
    }

    if (avio_seek(pb, next + DSS_AUDIO_BLOCK_HEADER_SIZE + poff, SEEK_SET) < 0)
        return AVERROR(EIO);

    ctx->counter = DSS_BLOCK_PAYLOAD_SIZE - poff;

    return 0;
}

#define DSS_SP_SAMPLES_PER_FRAME 264

static int dss_probe(const AVProbeData *p)
{
    if (   AV_RL32(p->buf) != MKTAG(0x2, 'd', 's', 's')
        && AV_RL32(p->buf) != MKTAG(0x3, 'd', 's', 's')
        && AV_RL32(p->buf) != MKTAG(0x6, 'd', 's', 's'))
        return 0;

    return AVPROBE_SCORE_MAX;
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
        return AVERROR_INVALIDDATA;
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
    if (ret < size) {
        av_free(value);
        return ret < 0 ? ret : AVERROR_EOF;
    }

    return av_dict_set(&s->metadata, key, value, AV_DICT_DONT_STRDUP_VAL);
}

static int dss_count_total_frames(AVFormatContext *s)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t fsize = avio_size(pb);
    int blocks, i, total = 0;

    if (fsize < ctx->dss_header_size)
        return AVERROR_INVALIDDATA;

    blocks = (fsize - ctx->dss_header_size) / DSS_BLOCK_SIZE;
    for (i = 0; i < blocks; i++) {
        uint8_t fc;
        avio_seek(pb, ctx->dss_header_size + (int64_t)i * DSS_BLOCK_SIZE + 2,
                  SEEK_SET);
        if (avio_read(pb, &fc, 1) < 1)
            return AVERROR_EOF;
        total += fc;
    }

    return total;
}

/* Read one bit from a 506-byte block payload that is stored as a sequence of
 * little-endian 16-bit words, MSB first within each word. */
static inline int grundig_get_bit(const uint8_t *payload, int payload_size,
                                  int bit)
{
    int wi  = bit >> 4;
    int sub = bit & 15;
    int idx = wi * 2;
    unsigned w;

    if (idx + 1 >= payload_size)
        return 0;
    w = (payload[idx + 1] << 8) | payload[idx];
    return (w >> (15 - sub)) & 1;
}

/* Extract the continuous 41-byte CELP frame stream from the Grundig audio
 * blocks.  Each 512-byte block starts with a 6-byte header
 *   [b0][b1][b2] ff 00 ff
 * where b2 = number of frames that start in this block and
 * (b1<<8|b0)>>4 encodes the bit offset of the first whole frame in the block.
 * Frames are a continuous 328-bit stream that spans block boundaries; on a
 * boundary the next block's payload continues at bit 0 (after its 6-byte
 * header).  Each extracted frame is repacked as 41 plain MSB-first bytes for
 * the decoder. */
static int grundig_extract_frames(AVFormatContext *s, const uint8_t *audio,
                                  int audio_size)
{
    DSSDemuxContext *ctx = s->priv_data;
    int nb_blocks = audio_size / DSS_BLOCK_SIZE;
    int blockbits = (DSS_BLOCK_SIZE - DSS_AUDIO_BLOCK_HEADER_SIZE) * 8; /* 506*8 */
    const int payload_size = DSS_BLOCK_SIZE - DSS_AUDIO_BLOCK_HEADER_SIZE;
    int total_frames = 0;
    int bi, cap, count;
    uint8_t *out;

    for (bi = 0; bi < nb_blocks; bi++)
        total_frames += audio[bi * DSS_BLOCK_SIZE + 2];

    if (total_frames <= 0)
        return AVERROR_INVALIDDATA;

    out = av_malloc_array(total_frames, DSS_GRUNDIG_FRAME_SIZE);
    if (!out)
        return AVERROR(ENOMEM);

    cap   = total_frames;
    count = 0;
    for (bi = 0; bi < nb_blocks; bi++) {
        const uint8_t *blk = audio + bi * DSS_BLOCK_SIZE;
        int w0       = (blk[1] << 8) | blk[0];
        int f421     = w0 >> 4;
        int fc       = blk[2];
        int word_off = (f421 >> 4) - 3;
        int avail    = 0x10 - (f421 & 0xf);
        int pos      = word_off * 16 + (16 - avail);
        int fi;

        for (fi = 0; fi < fc && count < cap; fi++) {
            uint8_t *dst = out + count * DSS_GRUNDIG_FRAME_SIZE;
            int need = DSS_GRUNDIG_FRAME_BITS;
            int p    = pos;
            int cb   = bi;
            int bitn = 0;

            memset(dst, 0, DSS_GRUNDIG_FRAME_SIZE);
            while (need > 0) {
                const uint8_t *cpl = audio + cb * DSS_BLOCK_SIZE +
                                     DSS_AUDIO_BLOCK_HEADER_SIZE;
                int take = FFMIN(need, blockbits - p);
                int k;

                for (k = 0; k < take; k++) {
                    int bit = grundig_get_bit(cpl, payload_size, p + k);
                    if (bit)
                        dst[bitn >> 3] |= 0x80 >> (bitn & 7);
                    bitn++;
                }
                need -= take;
                p    += take;
                if (need > 0) {
                    cb++;
                    if (cb >= nb_blocks)
                        break;
                    p = 0;
                }
            }
            count++;
            pos = p;
        }
    }

    ctx->grundig_frames    = out;
    ctx->grundig_nb_frames = count;
    ctx->grundig_pos       = 0;
    return 0;
}

static int dss_read_header(AVFormatContext *s)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st;
    int64_t ret64;
    int ret, version;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    version = avio_r8(pb);
    ctx->dss_header_size = version * DSS_BLOCK_SIZE;

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

    if (version == DSS_GRUNDIG_VERSION) {
        uint8_t *audio;
        int64_t total_size, audio_size;

        ctx->grundig = 1;
        st->codecpar->codec_type     = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id       = AV_CODEC_ID_GRUNDIG_SP;
        st->codecpar->ch_layout      = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
        st->codecpar->sample_rate    = DSS_GRUNDIG_SAMPLE_RATE;
        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
        st->start_time = 0;

        total_size = avio_size(pb);
        if (total_size < 0)
            return total_size;
        if (total_size <= ctx->dss_header_size)
            return AVERROR_INVALIDDATA;
        audio_size = total_size - ctx->dss_header_size;
        audio_size -= audio_size % DSS_BLOCK_SIZE;
        if (audio_size <= 0 || audio_size > INT_MAX)
            return AVERROR_INVALIDDATA;

        if ((ret64 = avio_seek(pb, ctx->dss_header_size, SEEK_SET)) < 0)
            return (int)ret64;

        audio = av_malloc(audio_size);
        if (!audio)
            return AVERROR(ENOMEM);
        ret = avio_read(pb, audio, audio_size);
        if (ret < audio_size) {
            av_free(audio);
            return ret < 0 ? ret : AVERROR_EOF;
        }

        ret = grundig_extract_frames(s, audio, audio_size);
        av_free(audio);
        if (ret < 0)
            return ret;

        s->bit_rate = 8LL * DSS_GRUNDIG_FRAME_SIZE * st->codecpar->sample_rate
                          / DSS_GRUNDIG_SAMPLES_PER_FRAME;
        return 0;
    }

    avio_seek(pb, DSS_HEAD_OFFSET_ACODEC, SEEK_SET);
    ctx->audio_codec = avio_r8(pb);

    if (ctx->audio_codec == DSS_ACODEC_DSS_SP) {
        st->codecpar->codec_id    = AV_CODEC_ID_DSS_SP;
        st->codecpar->sample_rate = 11025;
        s->bit_rate = 8 * (DSS_FRAME_SIZE - 1) * st->codecpar->sample_rate
                        * 512 / (506 * 264);
    } else if (ctx->audio_codec == DSS_ACODEC_G723_1) {
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

    ret = dss_count_total_frames(s);
    if (ret < 0)
        return ret;
    ctx->total_frames = ret;

    if (ctx->audio_codec == DSS_ACODEC_DSS_SP && ctx->total_frames > 0) {
        int64_t nb_samples = (int64_t)ctx->total_frames * DSS_SP_SAMPLES_PER_FRAME;

        st->duration = nb_samples;
        s->duration  = av_rescale_q(nb_samples, (AVRational){1, st->codecpar->sample_rate},
                                    AV_TIME_BASE_Q);
    }

    /* Jump over file header; first audio block header loaded below. */
    if ((ret64 = avio_seek(pb, ctx->dss_header_size, SEEK_SET)) < 0)
        return (int)ret64;

    {
        uint8_t first_hdr[DSS_AUDIO_BLOCK_HEADER_SIZE];
        ret = avio_read(pb, first_hdr, sizeof(first_hdr));
        if (ret < (int)sizeof(first_hdr))
            return ret < 0 ? ret : AVERROR_EOF;
        ctx->swap = first_hdr[0] >> 7;
        ctx->counter = DSS_BLOCK_SIZE - DSS_AUDIO_BLOCK_HEADER_SIZE;
    }
    ctx->resync_pending     = 0;
    ctx->frames_read        = 0;
    ctx->span_continues     = 0;
    ctx->block_frames_left    = 0;
    ctx->compact_fc           = 0;
    ctx->compact_poff         = 0;
    ctx->compact_next_idx     = 0;
    ctx->compact_stream_off   = 0;
    ctx->in_compact           = 0;
    ctx->compact_swap         = 0;
    ctx->pending_block_fc     = 0;
    ctx->pending_compact_poff = 0;
    ctx->pending_compact_swap = 0;
    ctx->block_start_pos      = 0;
    ctx->pending_block_start  = 0;
    return 0;
}

static void dss_skip_audio_header(AVFormatContext *s, int mid_frame)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    uint8_t hdr[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int64_t block_pos;
    int frame_count, cont_size, offset;

    /* Second half of the VOX-pause handling (upstream v4 DSS-SP resync): the
     * frame that straddled into the empty block has now been completed from
     * its leading bytes. Discard the rest of that block (padding) by aligning
     * to the next 512-byte boundary, then re-sync the frame grid at the
     * following block's anchor 2*byte1 (+2 when byte-swapped), restarting the
     * byte-swap parity from that block and flagging a decoder reset. */
    if (ctx->resync_pending) {
        int64_t rel = avio_tell(pb) - ctx->dss_header_size;
        int pad = (DSS_BLOCK_SIZE - (int)(rel % DSS_BLOCK_SIZE)) % DSS_BLOCK_SIZE;

        ctx->resync_pending = 0;
        avio_skip(pb, pad);
        if (avio_read(pb, hdr, sizeof(hdr)) < (int)sizeof(hdr)) {
            ctx->counter = 0;
            return;
        }
        ctx->swap = !!(hdr[0] & 0x80);
        offset    = 2 * hdr[1] + 2 * ctx->swap;
        if (offset < DSS_AUDIO_BLOCK_HEADER_SIZE)
            offset = DSS_AUDIO_BLOCK_HEADER_SIZE;
        if (offset > DSS_BLOCK_SIZE)
            offset = DSS_BLOCK_SIZE;
        avio_skip(pb, offset - DSS_AUDIO_BLOCK_HEADER_SIZE);
        ctx->counter           = DSS_BLOCK_SIZE - offset;
        ctx->dss_sp_swap_byte  = -1;
        ctx->block_frames_left = 0;
        ctx->in_compact        = 0;
        return;
    }

    block_pos = avio_tell(pb);
    if (avio_read(pb, hdr, sizeof(hdr)) < (int)sizeof(hdr))
        return;

    frame_count = hdr[2];
    cont_size   = FFMAX(0, 2 * hdr[1] + 2 * (hdr[0] >> 7) -
                           DSS_AUDIO_BLOCK_HEADER_SIZE);

    /* VOX pause: an empty block (frame_count == 0) carries no fresh frames.
     * Its leading bytes complete the frame straddling into it (read by the
     * caller); the next call re-syncs at the following block's anchor. */
    if (ctx->audio_codec == DSS_ACODEC_DSS_SP && frame_count == 0) {
        ctx->counter        = ctx->swap ? DSS_FRAME_SIZE - 2 : DSS_FRAME_SIZE;
        ctx->resync_pending = 1;
        return;
    }

    if (!mid_frame) {
        ctx->block_start_pos = block_pos;
        if (dss_block_payload_fits(hdr)) {
            ctx->block_frames_left = frame_count;
            ctx->compact_fc        = frame_count;
            ctx->compact_poff      = cont_size;
            ctx->compact_next_idx  = 0;
            ctx->compact_stream_off = 0;
            ctx->in_compact        = 1;
            ctx->compact_swap      = hdr[0] >> 7;
            ctx->counter           = 1;
            return;
        }
        ctx->block_frames_left = 0;
        ctx->in_compact        = 0;
    } else if (dss_block_payload_fits(hdr)) {
        ctx->pending_block_fc     = frame_count;
        ctx->pending_block_start  = block_pos;
        ctx->pending_compact_poff = cont_size;
        ctx->pending_compact_swap = hdr[0] >> 7;
        return;
    }

    ctx->counter += DSS_BLOCK_PAYLOAD_SIZE;
}

static void dss_sp_byte_swap(DSSDemuxContext *ctx, uint8_t *data)
{
    int i;

    if (ctx->swap) {
        for (i = 0; i < DSS_FRAME_SIZE - 2; i += 2)
            data[i] = data[i + 4];

        /* Zero the padding. */
        data[DSS_FRAME_SIZE] = 0;
        data[1] = ctx->dss_sp_swap_byte;
    } else {
        ctx->dss_sp_swap_byte = data[DSS_FRAME_SIZE - 2];
    }

    /* make sure byte 40 is always 0 */
    data[DSS_FRAME_SIZE - 2] = 0;
    ctx->swap             ^= 1;
}

/* If linear demux drifted into compact-block padding, skip to next block. */
static void dss_skip_compact_padding(AVFormatContext *s)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t tell = avio_tell(pb);
    int64_t bstart, audio_end;
    uint8_t hdr[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int poff, fc;

    if (tell < ctx->dss_header_size || ctx->block_frames_left > 0)
        return;

    bstart = ctx->dss_header_size +
             (tell - ctx->dss_header_size) / DSS_BLOCK_SIZE * DSS_BLOCK_SIZE;

    avio_seek(pb, bstart, SEEK_SET);
    if (avio_read(pb, hdr, sizeof(hdr)) < (int)sizeof(hdr)) {
        avio_seek(pb, tell, SEEK_SET);
        return;
    }

    if (!dss_block_payload_fits(hdr)) {
        avio_seek(pb, tell, SEEK_SET);
        return;
    }

    fc = hdr[2];
    poff = FFMAX(0, 2 * hdr[1] + 2 * (hdr[0] >> 7) -
                     DSS_AUDIO_BLOCK_HEADER_SIZE);
    audio_end = bstart + DSS_AUDIO_BLOCK_HEADER_SIZE + poff +
                  (int64_t)fc * DSS_FRAME_SIZE;

    if (tell < audio_end) {
        avio_seek(pb, tell, SEEK_SET);
        return;
    }

    {
        int b;

        avio_seek(pb, tell, SEEK_SET);
        b = avio_r8(pb);
        if (b != 0xff) {
            avio_seek(pb, tell, SEEK_SET);
            return;
        }
    }

    /* Last file block: normal compact exit handles EOF; do not seek past end. */
    if (avio_size(pb) > 0 && bstart + DSS_BLOCK_SIZE >= avio_size(pb)) {
        avio_seek(pb, tell, SEEK_SET);
        return;
    }

    ctx->block_start_pos = bstart;
    dss_align_after_compact_block(s, bstart);
}

static int dss_sp_read_compact_packet(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    uint8_t hdr[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int idx, read_size, buff_offset, ret;
    int64_t pos;

    avio_seek(pb, ctx->block_start_pos, SEEK_SET);
    if (avio_read(pb, hdr, sizeof(hdr)) < (int)sizeof(hdr))
        return AVERROR_EOF;
    ctx->compact_fc       = hdr[2];
    ctx->compact_poff     = FFMAX(0, 2 * hdr[1] + 2 * (hdr[0] >> 7) -
                                      DSS_AUDIO_BLOCK_HEADER_SIZE);
    ctx->compact_swap     = hdr[0] >> 7;

    idx = ctx->compact_next_idx;
    pos = ctx->block_start_pos + DSS_AUDIO_BLOCK_HEADER_SIZE +
          ctx->compact_poff + (int64_t)ctx->compact_stream_off;

    avio_seek(pb, pos, SEEK_SET);
    ctx->swap = ctx->compact_swap ^ (idx & 1);

    if (ctx->swap) {
        read_size   = DSS_FRAME_SIZE - 2;
        buff_offset = 3;
    } else {
        read_size   = DSS_FRAME_SIZE;
        buff_offset = 0;
    }

    ret = av_new_packet(pkt, DSS_FRAME_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);
    if (ret < 0)
        return ret;
    pkt->size         = DSS_FRAME_SIZE;
    pkt->duration     = DSS_SP_SAMPLES_PER_FRAME;
    pkt->pos          = pos;
    pkt->stream_index = 0;
    pkt->flags        = 0;
    memset(pkt->data, 0, DSS_FRAME_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);

    ret = avio_read(pb, pkt->data + buff_offset, read_size);
    if (ret < read_size)
        return ret < 0 ? ret : AVERROR_EOF;

    dss_sp_byte_swap(ctx, pkt->data);

    if (ctx->dss_sp_swap_byte < 0)
        return AVERROR(EAGAIN);

    ctx->compact_stream_off += read_size;

    ctx->compact_next_idx++;
    ctx->block_frames_left = ctx->compact_fc - ctx->compact_next_idx;
    ctx->counter           = 1;

    if (ctx->compact_next_idx >= ctx->compact_fc) {
        int64_t fsize = avio_size(pb);

        ctx->in_compact = 0;
        if (fsize <= 0 || ctx->block_start_pos + DSS_BLOCK_SIZE < fsize)
            dss_align_after_compact_block(s, ctx->block_start_pos);
    }

    ctx->frames_read++;

    return 0;
}

static int dss_sp_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;
    int read_size, ret, offset = 0, buff_offset = 0;
    int spanned_start = 0, pending_new = 0;
    int64_t pos = avio_tell(s->pb);

    if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
        return AVERROR_EOF;

    dss_skip_compact_padding(s);

    if (ctx->block_frames_left > 0 && !ctx->span_continues)
        return dss_sp_read_compact_packet(s, pkt);

    while (ctx->counter == 0) {
        if (avio_feof(s->pb))
            return AVERROR_EOF;
        dss_skip_audio_header(s, 0);
        if (ctx->counter == 0 && avio_feof(s->pb))
            return AVERROR_EOF;
    }

    if (ctx->swap) {
        read_size   = DSS_FRAME_SIZE - 2;
        buff_offset = 3;
    } else
        read_size = DSS_FRAME_SIZE;

    ret = av_new_packet(pkt, DSS_FRAME_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);
    if (ret < 0)
        return ret;
    pkt->size = DSS_FRAME_SIZE;

    pkt->duration     = DSS_SP_SAMPLES_PER_FRAME;
    pkt->pos          = pos;
    pkt->stream_index = 0;
    pkt->flags        = 0;

    if (ctx->block_frames_left == 0 && ctx->counter < read_size) {
        ret = avio_read(s->pb, pkt->data + buff_offset, ctx->counter);
        if (ret < ctx->counter)
            goto error_eof;

        offset = ctx->counter;
        spanned_start = 1;
        ctx->span_continues = 1;
        dss_skip_audio_header(s, 1);
    }
    ctx->counter -= read_size;

    ret = avio_read(s->pb, pkt->data + offset + buff_offset,
                    read_size - offset);
    if (ret < read_size - offset)
        goto error_eof;

    dss_sp_byte_swap(ctx, pkt->data);

    if (ctx->dss_sp_swap_byte < 0)
        return AVERROR(EAGAIN);

    {
        int spanned = ctx->span_continues;

        ctx->span_continues = 0;

        pending_new = ctx->pending_block_fc > 0;

        if (pending_new) {
            if (!ctx->block_frames_left) {
                ctx->block_frames_left    = ctx->pending_block_fc;
                ctx->block_start_pos      = ctx->pending_block_start;
                ctx->compact_fc           = ctx->pending_block_fc;
                ctx->compact_poff         = ctx->pending_compact_poff;
                ctx->compact_swap         = ctx->pending_compact_swap;
                ctx->compact_next_idx     = 0;
                ctx->compact_stream_off   = 0;
                ctx->in_compact           = 1;
            }
            ctx->pending_block_fc     = 0;
            ctx->pending_block_start  = 0;
            ctx->pending_compact_poff = 0;
            ctx->pending_compact_swap = 0;
        }

        if (ctx->block_frames_left > 0 && !ctx->in_compact) {
            if (!(pending_new && spanned))
                ctx->block_frames_left--;
            if (ctx->block_frames_left == 0)
                dss_align_after_compact_block(s, ctx->block_start_pos);
        }
    }

    if (spanned_start && pending_new && ctx->in_compact)
        ctx->counter = 1;

    ctx->frames_read++;

    return 0;

error_eof:
    return ret < 0 ? ret : AVERROR_EOF;
}

static int dss_723_1_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;
    AVStream *st = s->streams[0];
    int size, byte, ret, offset;
    int64_t pos = avio_tell(s->pb);

    if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
        return AVERROR_EOF;

    if (ctx->block_frames_left == 0 && ctx->counter > 0) {
        avio_skip(s->pb, ctx->counter);
        ctx->counter = 0;
    }

    while (ctx->counter == 0) {
        if (avio_feof(s->pb))
            return AVERROR_EOF;
        dss_skip_audio_header(s, 0);
        if (ctx->counter == 0 && avio_feof(s->pb))
            return AVERROR_EOF;
    }

    /* We make one byte-step here. Don't forget to add offset. */
    byte = avio_r8(s->pb);
    if (byte == 0xff)
        return AVERROR_INVALIDDATA;

    size = frame_size[byte & 3];

    ctx->packet_size = size;
    ctx->counter--;

    ret = av_new_packet(pkt, size);
    if (ret < 0)
        return ret;
    pkt->pos = pos;

    pkt->data[0]  = byte;
    offset        = 1;
    pkt->duration = 240;
    s->bit_rate = 8LL * size-- * st->codecpar->sample_rate * 512 / (506 * pkt->duration);

    pkt->stream_index = 0;

    if (ctx->counter < size) {
        ret = avio_read(s->pb, pkt->data + offset,
                        ctx->counter);
        if (ret < ctx->counter)
            return ret < 0 ? ret : AVERROR_EOF;

        offset += ctx->counter;
        size   -= ctx->counter;
        ctx->counter = 0;
        dss_skip_audio_header(s, 1);
    }
    ctx->counter -= size;

    ret = avio_read(s->pb, pkt->data + offset, size);
    if (ret < size)
        return ret < 0 ? ret : AVERROR_EOF;

    ctx->frames_read++;

    return 0;
}

static int dss_grundig_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;
    int ret;

    if (ctx->grundig_pos >= ctx->grundig_nb_frames)
        return AVERROR_EOF;

    if ((ret = av_new_packet(pkt, DSS_GRUNDIG_FRAME_SIZE)) < 0)
        return ret;

    memcpy(pkt->data,
           ctx->grundig_frames + ctx->grundig_pos * DSS_GRUNDIG_FRAME_SIZE,
           DSS_GRUNDIG_FRAME_SIZE);

    pkt->stream_index = 0;
    pkt->pts          = (int64_t)ctx->grundig_pos * DSS_GRUNDIG_SAMPLES_PER_FRAME;
    pkt->duration     = DSS_GRUNDIG_SAMPLES_PER_FRAME;
    pkt->flags       |= AV_PKT_FLAG_KEY;
    ctx->grundig_pos++;

    return 0;
}

static int dss_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DSSDemuxContext *ctx = s->priv_data;

    if (ctx->grundig)
        return dss_grundig_read_packet(s, pkt);
    else if (ctx->audio_codec == DSS_ACODEC_DSS_SP)
        return dss_sp_read_packet(s, pkt);
    else
        return dss_723_1_read_packet(s, pkt);
}

static int dss_read_close(AVFormatContext *s)
{
    DSSDemuxContext *ctx = s->priv_data;

    av_freep(&ctx->grundig_frames);
    return 0;
}

static int dss_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags)
{
    DSSDemuxContext *ctx = s->priv_data;
    int64_t ret, seekto;
    uint8_t header[DSS_AUDIO_BLOCK_HEADER_SIZE];
    int offset;

    if (ctx->grundig) {
        int64_t idx = timestamp / DSS_GRUNDIG_SAMPLES_PER_FRAME;

        if (idx < 0)
            idx = 0;
        if (idx > ctx->grundig_nb_frames)
            idx = ctx->grundig_nb_frames;
        ctx->grundig_pos = idx;
        return 0;
    }

    if (ctx->audio_codec == DSS_ACODEC_DSS_SP)
        seekto = timestamp / 264 * 41 / 506 * 512;
    else
        seekto = timestamp / 240 * ctx->packet_size / 506 * 512;

    if (seekto < 0)
        seekto = 0;

    seekto += ctx->dss_header_size;

    ret = avio_seek(s->pb, seekto, SEEK_SET);
    if (ret < 0)
        return ret;

    ret = ffio_read_size(s->pb, header, DSS_AUDIO_BLOCK_HEADER_SIZE);
    if (ret < 0)
        return ret;
    ctx->swap = !!(header[0] & 0x80);
    offset = 2*header[1] + 2*ctx->swap;
    if (offset < DSS_AUDIO_BLOCK_HEADER_SIZE)
        return AVERROR_INVALIDDATA;
    if (offset == DSS_AUDIO_BLOCK_HEADER_SIZE) {
        ctx->counter = 0;
        offset = avio_skip(s->pb, -DSS_AUDIO_BLOCK_HEADER_SIZE);
    } else {
        ctx->counter = DSS_BLOCK_SIZE - offset;
        offset = avio_skip(s->pb, offset - DSS_AUDIO_BLOCK_HEADER_SIZE);
    }
    ctx->dss_sp_swap_byte     = -1;
    ctx->resync_pending       = 0;
    ctx->frames_read          = 0;
    ctx->span_continues       = 0;
    ctx->block_frames_left    = 0;
    ctx->compact_fc           = 0;
    ctx->compact_poff         = 0;
    ctx->compact_swap         = 0;
    ctx->compact_next_idx     = 0;
    ctx->compact_stream_off   = 0;
    ctx->pending_block_fc     = 0;
    ctx->pending_compact_poff = 0;
    ctx->pending_compact_swap = 0;
    ctx->block_start_pos      = 0;
    ctx->pending_block_start  = 0;
    return 0;
}


const FFInputFormat ff_dss_demuxer = {
    .p.name         = "dss",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Digital Speech Standard (DSS)"),
    .p.extensions   = "dss",
    .priv_data_size = sizeof(DSSDemuxContext),
    .read_probe     = dss_probe,
    .read_header    = dss_read_header,
    .read_packet    = dss_read_packet,
    .read_close     = dss_read_close,
    .read_seek      = dss_read_seek,
};
