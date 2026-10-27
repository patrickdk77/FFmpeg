/*
 * Digital Speech Standard Pro (DS2) demuxer
 */

/**
 * @file
 * DS2 (DSS Pro) demuxer
 *
 * DS2 is an evolution of the Digital Speech Standard (DSS) format used by
 * Olympus dictation recorders. This demuxer reads QP (Quality Play, 16000 Hz,
 * CELP with a C(64,11) codebook). SP and LP files carry exactly the DSS
 * payload behind a bigger header, so the DSS demuxer reads those.
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

#define DS2_FORMAT_SP 0 /* Standard Play, read by the DSS demuxer */
#define DS2_FORMAT_LP 2 /* Long Play (G.723.1), read by the DSS demuxer */
#define DS2_FORMAT_QP 6 /* Quality Play, 16000 Hz */

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
  int format_type;      /* block format byte, DS2_FORMAT_QP */
  int counter;          /* bytes remaining in current block payload */
  int total_frames;     /* total frame count from block headers */
  int frames_read;      /* frames read so far */
  int resync_drop;      /* QP: partial frame orphaned at a re-sync block */
} DS2DemuxContext;

static int ds2_probe(const AVProbeData *p) {
  int format;

  if (AV_RL32(p->buf) != MKTAG(0x3, 'd', 's', '2'))
    return 0;

  /* SP and LP files belong to the DSS demuxer. */
  if (p->buf_size >= DS2_HEADER_SIZE + DS2_AUDIO_BLOCK_HEADER_SIZE) {
    format = p->buf[DS2_HEADER_SIZE + 4];
    if (format == DS2_FORMAT_SP || format == DS2_FORMAT_LP)
      return 0;
  }

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

  if (sscanf(string, "%2d%2d%2d%2d%2d%2d", &y, &month, &d, &h, &minute, &sec) != 6)
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

/*
 * Frames that start in the part of a block a cut file still holds, from the
 * block's header and payload, size bytes of them. Only frames whose first
 * byte is there count; the reader returns the last one zero-filled.
 */
static int ds2_partial_block_frames(const uint8_t *buf, int size) {
  int frame_count = buf[2], n = 0;
  int off = DS2_AUDIO_BLOCK_HEADER_SIZE +
            av_clip(2 * buf[1] + 2 * (buf[0] >> 7) -
                    DS2_AUDIO_BLOCK_HEADER_SIZE, 0, DS2_BLOCK_PAYLOAD_SIZE);

  while (n < frame_count && off < size) {
    n++;
    off += DS2_QP_FRAME_SIZE;
  }
  return n;
}

static int ds2_count_total_frames(AVFormatContext *s) {
  AVIOContext *pb = s->pb;
  int64_t size = avio_size(pb);
  uint8_t buf[DS2_BLOCK_SIZE];
  int blocks, i, tail, total = 0;

  if (size < DS2_HEADER_SIZE)
    return AVERROR_INVALIDDATA;

  blocks = (size - DS2_HEADER_SIZE) / DS2_BLOCK_SIZE;
  for (i = 0; i < blocks; i++) {
    avio_seek(pb, DS2_HEADER_SIZE + (int64_t)i * DS2_BLOCK_SIZE + 2, SEEK_SET);
    total += avio_r8(pb);
  }

  tail = (size - DS2_HEADER_SIZE) % DS2_BLOCK_SIZE;
  if (i == blocks && tail >= DS2_AUDIO_BLOCK_HEADER_SIZE &&
      avio_seek(pb, DS2_HEADER_SIZE + (int64_t)blocks * DS2_BLOCK_SIZE, SEEK_SET) >= 0 &&
      avio_read(pb, buf, tail) == tail)
    total += ds2_partial_block_frames(buf, tail);

  return total;
}

static int ds2_load_block(AVFormatContext *s, int in_frame_offset) {
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];
  int ret, frame_count, cont_size, blk_swap, anchor, aligned;

  if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
    return AVERROR_EOF;

  ret = avio_read(pb, hdr, sizeof(hdr));
  if (ret != DS2_AUDIO_BLOCK_HEADER_SIZE)
    return ret < 0 ? ret : AVERROR_EOF;

  blk_swap    = hdr[0] >> 7;
  frame_count = hdr[2];
  cont_size   = FFMAX(0, 2 * hdr[1] + 2 * blk_swap - DS2_AUDIO_BLOCK_HEADER_SIZE);

  ctx->resync_drop = 0;

  if (frame_count == 0) {
    ctx->counter = cont_size;
  } else {
    /*
     * QP frames re-anchor at payload offset 2*byte1 - 6 at every block
     * (blk_swap == 0 in QP, so this equals cont_size). On gap-free audio the
     * anchor coincides with the straddle carried from the previous block, so a
     * flat concatenation is correct. At a segment boundary the recorder emits
     * a re-sync block whose anchor jumps: the leading bytes are an orphaned
     * straddle tail, the partial frame in flight is stale, and framing must
     * restart at the anchor. frame_count over-counts on these blocks (observed
     * 19), so it is not trusted to drive framing; the block end is the limit.
     * Confirmed byte-for-byte against the Olympus DssParser.dll.
     */
    anchor  = FFMIN(cont_size, DS2_BLOCK_PAYLOAD_SIZE);
    aligned = (!in_frame_offset && !anchor) ||
              (in_frame_offset && in_frame_offset + anchor == DS2_QP_FRAME_SIZE);
    if (!aligned) {
      if (anchor > 0)
        avio_skip(pb, anchor);
      ctx->counter     = DS2_BLOCK_PAYLOAD_SIZE - anchor;
      ctx->resync_drop = 1;
      return 0;
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

  if (ctx->format_type == DS2_FORMAT_SP || ctx->format_type == DS2_FORMAT_LP) {
    av_log(s, AV_LOG_ERROR, "DS2 SP and LP files are read by the dss demuxer\n");
    return AVERROR_INVALIDDATA;
  } else if (ctx->format_type == DS2_FORMAT_QP) {
    st->codecpar->sample_rate = DS2_QP_SAMPLE_RATE;
    samples_per_frame = DS2_QP_SAMPLES_PER_FRAME;
  } else {
    avpriv_request_sample(s, "DS2 format type %d", ctx->format_type);
    return AVERROR_PATCHWELCOME;
  }

  avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
  st->start_time = 0;
  if (ctx->total_frames > 0) {
    int64_t nb_samples = (int64_t)ctx->total_frames * samples_per_frame;
    int frame_bytes = DS2_QP_FRAME_SIZE;
    int64_t file_size = avio_size(pb);

    st->duration = nb_samples;
    s->duration  = av_rescale_q(nb_samples, (AVRational){1, st->codecpar->sample_rate},
                                AV_TIME_BASE_Q);

    if (file_size > DS2_HEADER_SIZE && s->duration > 0)
      s->bit_rate = (file_size - DS2_HEADER_SIZE) * 8LL * AV_TIME_BASE / s->duration;
    else
      s->bit_rate = 8LL * frame_bytes * st->codecpar->sample_rate / samples_per_frame;

    st->codecpar->bit_rate = (int)s->bit_rate;
  }

  blk_swap    = block_header[0] >> 7;
  frame_count = block_header[2];
  cont_size   = FFMAX(0, 2 * block_header[1] + 2 * blk_swap -
                           DS2_AUDIO_BLOCK_HEADER_SIZE);

  ctx->frames_read = 0;

  if (frame_count == 0)
    ctx->counter = cont_size;
  else
    ctx->counter = DS2_BLOCK_PAYLOAD_SIZE;

  return 0;
}

static int ds2_qp_read_packet(AVFormatContext *s, AVPacket *pkt) {
  DS2DemuxContext *ctx = s->priv_data;
  int ret, offset = 0;
  int64_t pos = avio_tell(s->pb);

  if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
    return AVERROR_EOF;

  if (ctx->counter == 0) {
    ret = ds2_load_block(s, 0);
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
      ret = ds2_load_block(s, offset);
      if (ret == AVERROR_EOF && offset > 0)
        goto truncated;
      if (ret < 0)
        return ret;
      if (ctx->resync_drop)
        offset = 0;
      to_read = FFMIN(DS2_QP_FRAME_SIZE - offset, ctx->counter);
    }

    ret = avio_read(s->pb, pkt->data + offset, to_read);
    if (ret < to_read) {
      if (ret < 0 && ret != AVERROR_EOF)
        return ret;
      if (ret > 0)
        offset += ret;
      if (!offset)
        return AVERROR_EOF;
      goto truncated;
    }

    offset += to_read;
    ctx->counter -= to_read;
  }

  ctx->frames_read++;
  return 0;

truncated:
  /* The file ends inside a frame its block still counts. Olympus decodes
   * that frame too, so keep it, with the missing bytes zeroed. */
  memset(pkt->data + offset, 0, DS2_QP_FRAME_SIZE - offset);
  ctx->counter = 0;
  ctx->frames_read++;
  return 0;
}

static int ds2_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags) {
  DS2DemuxContext *ctx = s->priv_data;
  int64_t ret, seekto;
  uint8_t header[DS2_AUDIO_BLOCK_HEADER_SIZE];
  int frame_count, cont_size, blk_swap;

  seekto = timestamp / DS2_QP_SAMPLES_PER_FRAME * DS2_QP_FRAME_SIZE /
           DS2_BLOCK_PAYLOAD_SIZE * DS2_BLOCK_SIZE;

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

  ctx->frames_read = timestamp / DS2_QP_SAMPLES_PER_FRAME;

  if (frame_count == 0)
    ctx->counter = cont_size;
  else
    ctx->counter = DS2_BLOCK_PAYLOAD_SIZE;

  return 0;
}

const FFInputFormat ff_ds2_demuxer = {
    .p.name = "ds2",
    .p.long_name = NULL_IF_CONFIG_SMALL("Digital Speech Standard Pro (DS2)"),
    .p.extensions = "ds2",
    .priv_data_size = sizeof(DS2DemuxContext),
    .read_probe = ds2_probe,
    .read_header = ds2_read_header,
    .read_packet = ds2_qp_read_packet,
    .read_seek = ds2_read_seek,
};
