/*
 * Digital Speech Standard Pro (DS2) demuxer
 */

/**
 * @file
 * DS2 (DSS Pro) demuxer
 *
 * DS2 is an evolution of the Digital Speech Standard (DSS) format used by
 * Olympus dictation recorders. It supports two codec modes:
 *   - SP (Standard Play): 12000 Hz, CELP with C(72,7) codebook
 *   - QP (Quality Play):  16000 Hz, CELP with C(64,11) codebook
 *
 * Block layout and demuxing follow hirparak/dss-codec CODEC_SPECIFICATION.md.
 */

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"

#define DS2_HEADER_SIZE 0x600 /* 1536 bytes */
#define DS2_BLOCK_SIZE 512
#define DS2_AUDIO_BLOCK_HEADER_SIZE 6
#define DS2_BLOCK_PAYLOAD_SIZE (DS2_BLOCK_SIZE - DS2_AUDIO_BLOCK_HEADER_SIZE)

#define DS2_FORMAT_SP 0 /* Standard Play, 12000 Hz */
#define DS2_FORMAT_QP 6 /* Quality Play, 16000 Hz */

#define DS2_SP_FRAME_SIZE 42 /* SP: 328 bits ~ 41 bytes, padded to 42 */
#define DS2_SP_FRAME_BITS 328
#define DS2_SP_SAMPLES_PER_FRAME 288 /* 72 samples * 4 subframes */
#define DS2_SP_SAMPLE_RATE 12000

#define DS2_QP_FRAME_BITS 448
#define DS2_QP_FRAME_SIZE 56         /* 448 bits = 56 bytes */
#define DS2_QP_SAMPLES_PER_FRAME 256 /* 64 samples * 4 subframes */
#define DS2_QP_SAMPLE_RATE 16000

/* Metadata offsets (same layout as DSS) */
#define DS2_HEAD_OFFSET_AUTHOR 0xc
#define DS2_AUTHOR_SIZE 16
#define DS2_HEAD_OFFSET_START_TIME 0x26
#define DS2_HEAD_OFFSET_END_TIME 0x32
#define DS2_TIME_SIZE 12
#define DS2_HEAD_OFFSET_COMMENT 0x31e
#define DS2_COMMENT_SIZE 64

typedef struct DS2DemuxContext {
  int format_type;      /* DS2_FORMAT_SP or DS2_FORMAT_QP */
  int counter;          /* bytes remaining in current block payload */
  int swap;             /* SP byte-swap state */
  int ds2_sp_swap_byte; /* saved swap byte for SP mode */
  int total_frames;     /* total frame count from block headers */
  int frames_read;      /* frames read so far */
  int swap_reset_pending;
  int next_blk_swap;    /* swap state from next non-empty block */
} DS2DemuxContext;

static int ds2_probe(const AVProbeData *p) {
  if (AV_RL32(p->buf) != MKTAG(0x3, 'd', 's', '2'))
    return 0;

  return AVPROBE_SCORE_MAX;
}

static int ds2_read_metadata_date(AVFormatContext *s, unsigned int offset,
                                  const char *key) {
  AVIOContext *pb = s->pb;
  char datetime[64], string[DS2_TIME_SIZE + 1] = {0};
  int y, month, d, h, minute, sec;
  int ret;

  avio_seek(pb, offset, SEEK_SET);

  ret = avio_read(pb, string, DS2_TIME_SIZE);
  if (ret < DS2_TIME_SIZE)
    return ret < 0 ? ret : AVERROR_EOF;

  if (sscanf(string, "%2d%2d%2d%2d%2d%2d", &y, &month, &d, &h, &minute, &sec) !=
      6)
    return AVERROR_INVALIDDATA;

  snprintf(datetime, sizeof(datetime), "%.4d-%.2d-%.2dT%.2d:%.2d:%.2d",
           y + 2000, month, d, h, minute, sec);
  return av_dict_set(&s->metadata, key, datetime, 0);
}

static int ds2_read_metadata_string(AVFormatContext *s, unsigned int offset,
                                    unsigned int size, const char *key) {
  AVIOContext *pb = s->pb;
  char *value;
  int ret;

  avio_seek(pb, offset, SEEK_SET);

  value = av_mallocz(size + 1);
  if (!value)
    return AVERROR(ENOMEM);

  ret = avio_read(pb, value, size);
  if (ret < size) {
    av_free(value);
    return ret < 0 ? ret : AVERROR_EOF;
  }

  return av_dict_set(&s->metadata, key, value, AV_DICT_DONT_STRDUP_VAL);
}

static int ds2_count_total_frames(AVFormatContext *s) {
  AVIOContext *pb = s->pb;
  int64_t size = avio_size(pb);
  int blocks, i, total = 0;

  if (size < DS2_HEADER_SIZE)
    return AVERROR_INVALIDDATA;

  blocks = (size - DS2_HEADER_SIZE) / DS2_BLOCK_SIZE;
  for (i = 0; i < blocks; i++) {
    avio_seek(pb, DS2_HEADER_SIZE + (int64_t)i * DS2_BLOCK_SIZE + 2, SEEK_SET);
    total += avio_r8(pb);
  }

  return total;
}

static int ds2_find_next_nonempty_swap(AVFormatContext *s, int block_idx) {
  AVIOContext *pb = s->pb;
  int64_t fsize = avio_size(pb);
  int64_t pos = avio_tell(pb);
  int bi;

  for (bi = block_idx + 1; ; bi++) {
    int64_t bstart = DS2_HEADER_SIZE + (int64_t)bi * DS2_BLOCK_SIZE;
    uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];
    int ret;

    if (bstart + DS2_AUDIO_BLOCK_HEADER_SIZE > fsize)
      break;

    avio_seek(pb, bstart, SEEK_SET);
    ret = avio_read(pb, hdr, sizeof(hdr));
    if (ret < sizeof(hdr))
      break;
    if (hdr[2] > 0) {
      avio_seek(pb, pos, SEEK_SET);
      return hdr[0] >> 7;
    }
  }

  avio_seek(pb, pos, SEEK_SET);
  return 0;
}

static int ds2_load_block(AVFormatContext *s) {
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];
  int64_t block_pos;
  int ret, frame_count, cont_size, blk_swap, block_idx;

  if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
    return AVERROR_EOF;

  block_pos = avio_tell(pb);
  ret = avio_read(pb, hdr, sizeof(hdr));
  if (ret < sizeof(hdr))
    return ret < 0 ? ret : AVERROR_EOF;

  blk_swap    = hdr[0] >> 7;
  frame_count = hdr[2];
  cont_size   = FFMAX(0, 2 * hdr[1] + 2 * blk_swap - DS2_AUDIO_BLOCK_HEADER_SIZE);
  block_idx   = (block_pos - DS2_HEADER_SIZE) / DS2_BLOCK_SIZE;

  if (frame_count == 0) {
    ctx->counter = cont_size;
    if (ctx->format_type == DS2_FORMAT_SP) {
      ctx->swap_reset_pending = 1;
      ctx->next_blk_swap = ds2_find_next_nonempty_swap(s, block_idx);
    }
  } else {
    if (ctx->format_type == DS2_FORMAT_SP && ctx->swap_reset_pending) {
      ctx->swap = ctx->next_blk_swap;
      ctx->ds2_sp_swap_byte = -1;
      ctx->swap_reset_pending = 0;
    }
    ctx->counter = DS2_BLOCK_PAYLOAD_SIZE;
  }

  return 0;
}

static int ds2_read_header(AVFormatContext *s) {
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  AVStream *st;
  uint8_t block_header[DS2_AUDIO_BLOCK_HEADER_SIZE];
  int ret, frame_count, cont_size, blk_swap, samples_per_frame;
  int64_t ret64;

  st = avformat_new_stream(s, NULL);
  if (!st)
    return AVERROR(ENOMEM);

  ret = ds2_read_metadata_string(s, DS2_HEAD_OFFSET_AUTHOR, DS2_AUTHOR_SIZE,
                                 "author");
  if (ret < 0)
    av_log(s, AV_LOG_WARNING, "Failed to read author metadata\n");

  ret = ds2_read_metadata_date(s, DS2_HEAD_OFFSET_END_TIME, "date");
  if (ret < 0)
    av_log(s, AV_LOG_WARNING, "Failed to read date metadata\n");

  ret = ds2_read_metadata_string(s, DS2_HEAD_OFFSET_COMMENT, DS2_COMMENT_SIZE,
                                 "comment");
  if (ret < 0)
    av_log(s, AV_LOG_WARNING, "Failed to read comment metadata\n");

  ret = ds2_count_total_frames(s);
  if (ret < 0)
    return ret;
  ctx->total_frames = ret;

  if ((ret64 = avio_seek(pb, DS2_HEADER_SIZE, SEEK_SET)) < 0)
    return (int)ret64;

  ret = avio_read(pb, block_header, DS2_AUDIO_BLOCK_HEADER_SIZE);
  if (ret < DS2_AUDIO_BLOCK_HEADER_SIZE)
    return ret < 0 ? ret : AVERROR_EOF;

  ctx->format_type = block_header[4];

  st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
  st->codecpar->codec_id = AV_CODEC_ID_DS2;
  st->codecpar->codec_tag = ctx->format_type;
  st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;

  if (ctx->format_type == DS2_FORMAT_SP) {
    st->codecpar->sample_rate = DS2_SP_SAMPLE_RATE;
    samples_per_frame = DS2_SP_SAMPLES_PER_FRAME;
  } else if (ctx->format_type == DS2_FORMAT_QP) {
    st->codecpar->sample_rate = DS2_QP_SAMPLE_RATE;
    samples_per_frame = DS2_QP_SAMPLES_PER_FRAME;
  } else {
    avpriv_request_sample(s, "DS2 format type %d", ctx->format_type);
    return AVERROR_PATCHWELCOME;
  }

  avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
  st->start_time = 0;
  if (ctx->total_frames > 0)
    st->duration = (int64_t)ctx->total_frames * samples_per_frame;

  blk_swap    = block_header[0] >> 7;
  frame_count = block_header[2];
  cont_size   = FFMAX(0, 2 * block_header[1] + 2 * blk_swap -
                           DS2_AUDIO_BLOCK_HEADER_SIZE);

  ctx->swap = blk_swap;
  ctx->ds2_sp_swap_byte = -1;
  ctx->swap_reset_pending = 0;
  ctx->frames_read = 0;

  if (frame_count == 0)
    ctx->counter = cont_size;
  else
    ctx->counter = DS2_BLOCK_PAYLOAD_SIZE;

  return 0;
}

/**
 * SP byte-swap logic.
 *
 * DS2 SP uses an alternating byte-swap scheme similar to DSS:
 * - No-swap frame: read 42 bytes, save byte[40] as swap_byte, zero byte[40]
 * - Swap frame: read 40 bytes into pkt[3..], shift even-indexed bytes:
 *   pkt[i] = pkt[i+4] for i in 0..40 step 2, set pkt[1] = swap_byte
 * - Zero pkt[40] before decoding, toggle swap state
 */
static void ds2_sp_byte_swap(DS2DemuxContext *ctx, uint8_t *data) {
  int i;

  if (ctx->swap) {
    for (i = 0; i < DS2_SP_FRAME_SIZE - 2; i += 2)
      data[i] = data[i + 4];

    data[DS2_SP_FRAME_SIZE] = 0;
    data[1] = ctx->ds2_sp_swap_byte;
  } else {
    ctx->ds2_sp_swap_byte = data[DS2_SP_FRAME_SIZE - 2];
  }

  /* Ensure byte 40 is always 0 */
  data[DS2_SP_FRAME_SIZE - 2] = 0;
  ctx->swap ^= 1;
}

static int ds2_sp_read_packet(AVFormatContext *s, AVPacket *pkt) {
  DS2DemuxContext *ctx = s->priv_data;
  int read_size, ret, offset = 0, buff_offset = 0;
  int64_t pos = avio_tell(s->pb);

  if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
    return AVERROR_EOF;

  if (ctx->counter == 0) {
    ret = ds2_load_block(s);
    if (ret < 0)
      return ret;
  }

  if (ctx->swap) {
    read_size = DS2_SP_FRAME_SIZE - 2; /* 40 bytes */
    buff_offset = 3;
  } else {
    read_size = DS2_SP_FRAME_SIZE; /* 42 bytes */
  }

  ret = av_new_packet(pkt, DS2_SP_FRAME_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);
  if (ret < 0)
    return ret;
  pkt->size = DS2_SP_FRAME_SIZE;

  pkt->duration = DS2_SP_SAMPLES_PER_FRAME;
  pkt->pos = pos;
  pkt->stream_index = 0;

  if (ctx->counter < read_size) {
    ret = avio_read(s->pb, pkt->data + buff_offset, ctx->counter);
    if (ret < ctx->counter)
      goto error_eof;

    offset = ctx->counter;
    ret = ds2_load_block(s);
    if (ret < 0)
      goto error_eof;
  }
  ctx->counter -= read_size;

  ret = avio_read(s->pb, pkt->data + offset + buff_offset, read_size - offset);
  if (ret < read_size - offset)
    goto error_eof;

  ds2_sp_byte_swap(ctx, pkt->data);

  if (ctx->ds2_sp_swap_byte < 0)
    return AVERROR(EAGAIN);

  ctx->frames_read++;
  return 0;

error_eof:
  return ret < 0 ? ret : AVERROR_EOF;
}

static int ds2_qp_read_packet(AVFormatContext *s, AVPacket *pkt) {
  DS2DemuxContext *ctx = s->priv_data;
  int ret, offset = 0;
  int64_t pos = avio_tell(s->pb);

  if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
    return AVERROR_EOF;

  if (ctx->counter == 0) {
    ret = ds2_load_block(s);
    if (ret < 0)
      return ret;
  }

  ret = av_new_packet(pkt, DS2_QP_FRAME_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);
  if (ret < 0)
    return ret;
  pkt->size = DS2_QP_FRAME_SIZE;

  pkt->duration = DS2_QP_SAMPLES_PER_FRAME;
  pkt->pos = pos;
  pkt->stream_index = 0;

  /*
   * QP frames are 56 bytes in a continuous bitstream across blocks.
   * Frames can span block boundaries, so we read in pieces if needed.
   */
  while (offset < DS2_QP_FRAME_SIZE) {
    int to_read = FFMIN(DS2_QP_FRAME_SIZE - offset, ctx->counter);
    if (to_read <= 0) {
      ret = ds2_load_block(s);
      if (ret < 0)
        return ret;
      to_read = FFMIN(DS2_QP_FRAME_SIZE - offset, ctx->counter);
    }

    ret = avio_read(s->pb, pkt->data + offset, to_read);
    if (ret < to_read) {
      return ret < 0 ? ret : AVERROR_EOF;
    }

    offset += to_read;
    ctx->counter -= to_read;
  }

  ctx->frames_read++;
  return 0;
}

static int ds2_read_packet(AVFormatContext *s, AVPacket *pkt) {
  DS2DemuxContext *ctx = s->priv_data;

  if (ctx->format_type == DS2_FORMAT_SP)
    return ds2_sp_read_packet(s, pkt);
  else
    return ds2_qp_read_packet(s, pkt);
}

static int ds2_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags) {
  DS2DemuxContext *ctx = s->priv_data;
  int64_t ret, seekto;
  uint8_t header[DS2_AUDIO_BLOCK_HEADER_SIZE];
  int offset, frame_count, cont_size, blk_swap;

  if (ctx->format_type == DS2_FORMAT_SP) {
    /* SP: 42-byte frames (avg 41 bytes with swap interleaving) */
    seekto = timestamp / DS2_SP_SAMPLES_PER_FRAME * 41 /
             DS2_BLOCK_PAYLOAD_SIZE * DS2_BLOCK_SIZE;
  } else {
    /* QP: 56-byte frames */
    seekto = timestamp / DS2_QP_SAMPLES_PER_FRAME * DS2_QP_FRAME_SIZE /
             DS2_BLOCK_PAYLOAD_SIZE * DS2_BLOCK_SIZE;
  }

  if (seekto < 0)
    seekto = 0;

  seekto += DS2_HEADER_SIZE;

  ret = avio_seek(s->pb, seekto, SEEK_SET);
  if (ret < 0)
    return ret;

  ret = avio_read(s->pb, header, DS2_AUDIO_BLOCK_HEADER_SIZE);
  if (ret < DS2_AUDIO_BLOCK_HEADER_SIZE)
    return ret < 0 ? ret : AVERROR_EOF;

  blk_swap    = header[0] >> 7;
  frame_count = header[2];
  cont_size   = FFMAX(0, 2 * header[1] + 2 * blk_swap -
                           DS2_AUDIO_BLOCK_HEADER_SIZE);

  ctx->frames_read = timestamp /
      (ctx->format_type == DS2_FORMAT_SP ? DS2_SP_SAMPLES_PER_FRAME
                                         : DS2_QP_SAMPLES_PER_FRAME);
  ctx->swap_reset_pending = 0;

  if (ctx->format_type == DS2_FORMAT_SP) {
    ctx->swap = blk_swap;
    offset = 2 * header[1] + 2 * ctx->swap;
    if (offset < DS2_AUDIO_BLOCK_HEADER_SIZE)
      return AVERROR_INVALIDDATA;
    if (offset == DS2_AUDIO_BLOCK_HEADER_SIZE) {
      ctx->counter = 0;
      avio_skip(s->pb, -DS2_AUDIO_BLOCK_HEADER_SIZE);
    } else {
      ctx->counter = DS2_BLOCK_SIZE - offset;
      avio_skip(s->pb, offset - DS2_AUDIO_BLOCK_HEADER_SIZE);
    }
    ctx->ds2_sp_swap_byte = -1;
  } else {
    if (frame_count == 0)
      ctx->counter = cont_size;
    else
      ctx->counter = DS2_BLOCK_PAYLOAD_SIZE;
  }

  return 0;
}

const FFInputFormat ff_ds2_demuxer = {
    .p.name = "ds2",
    .p.long_name = NULL_IF_CONFIG_SMALL("Digital Speech Standard Pro (DS2)"),
    .p.extensions = "ds2",
    .priv_data_size = sizeof(DS2DemuxContext),
    .read_probe = ds2_probe,
    .read_header = ds2_read_header,
    .read_packet = ds2_read_packet,
    .read_seek = ds2_read_seek,
};
