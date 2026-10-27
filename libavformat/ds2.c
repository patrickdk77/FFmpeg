/*
 * Digital Speech Standard Pro (DS2) demuxer
 */

/**
 * @file
 * DS2 (DSS Pro) demuxer
 *
 * DS2 is an evolution of the Digital Speech Standard (DSS) format used by
 * Olympus dictation recorders. This demuxer reads QP (Quality Play, 16000 Hz,
 * CELP with a C(64,11) codebook). SP, variable rate SP and LP files carry
 * exactly the DSS payload behind a bigger header, so the DSS demuxer reads
 * those.
 *
 * Block layout and demuxing follow hirparak/dss-codec CODEC_SPECIFICATION.md.
 */

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"

#define DS2_BLOCK_SIZE 512
#define DS2_AUDIO_BLOCK_HEADER_SIZE 6
#define DS2_BLOCK_PAYLOAD_SIZE (DS2_BLOCK_SIZE - DS2_AUDIO_BLOCK_HEADER_SIZE)

#define DS2_FORMAT_SP 0 /* Standard Play, read by the DSS demuxer */
#define DS2_FORMAT_SP_VR 1 /* variable rate SP, read by the DSS demuxer */
#define DS2_FORMAT_LP 2 /* Long Play (G.723.1), read by the DSS demuxer */
#define DS2_FORMAT_QP 6 /* Quality Play, 16000 Hz */
#define DS2_FORMAT_QP7 7 /* QP with 12-byte noise frames, 16000 Hz */

#define DS2_QP_FRAME_SIZE 56         /* 448 bits = 56 bytes */
#define DS2_QP_SAMPLES_PER_FRAME 256 /* 64 samples * 4 subframes */
#define DS2_QP_SAMPLE_RATE 16000
#define DS2_QP7_NOISE_FRAME_SIZE 12

/* Metadata offsets (same layout as DSS) */
#define DS2_HEAD_OFFSET_AUTHOR 0xc
#define DS2_AUTHOR_SIZE 16
#define DS2_HEAD_OFFSET_START_TIME 0x26
#define DS2_HEAD_OFFSET_END_TIME 0x32
#define DS2_TIME_SIZE 12
#define DS2_HEAD_OFFSET_COMMENT 0x31e
#define DS2_COMMENT_SIZE 64

typedef struct DS2DemuxContext {
  int header_size;      /* in bytes, from ds2_header_size() */
  int format_type;      /* block format byte, DS2_FORMAT_QP or DS2_FORMAT_QP7 */
  int counter;          /* bytes remaining in current block payload */
  int64_t total_frames; /* total frame count from block headers */
  int64_t frames_read;  /* frames read so far */
  int resync_drop;      /* QP: partial frame orphaned at a re-sync block */
  int frame_size;       /* QP: size of the frame being read */
  int blk_frames;       /* frames still to start in the current block */
} DS2DemuxContext;

/*
 * The first byte is the header size in 512-byte blocks (Olympus 2/3,
 * Grundig/Philips 6/7, newer Olympus firmware 8). Philips SpeechExec
 * writes 1 in the files it splits out of a recording, over the usual
 * 3-block header.
 */
static int ds2_header_size(int first_byte) {
  return (first_byte == 1 ? 3 : first_byte) * DS2_BLOCK_SIZE;
}

static int ds2_probe(const AVProbeData *p) {
  int format, header_size;

  /* bytes 1..3 are the "ds2" tag */
  if (p->buf_size < 4 || p->buf[1] != 'd' || p->buf[2] != 's' || p->buf[3] != '2')
    return 0;
  if (p->buf[0] < 1 || p->buf[0] > 16)
    return 0;

  /* SP, variable rate SP and LP files belong to the DSS demuxer. A long
   * header can put the first block past the probe buffer; ask for more. */
  header_size = ds2_header_size(p->buf[0]);
  if (p->buf_size < header_size + DS2_AUDIO_BLOCK_HEADER_SIZE)
    return AVPROBE_SCORE_RETRY;
  format = p->buf[header_size + 4];
  if (format == DS2_FORMAT_SP || format == DS2_FORMAT_SP_VR ||
      format == DS2_FORMAT_LP)
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
 * Frames that start in a block: byte 2 when it is not 0, otherwise byte 5
 * unless that is 0xff. A recorder writes the second form where it pauses or
 * stops, and Olympus's decoder plays those frames.
 */
static int ds2_block_frames(const uint8_t *hdr) {
  if (hdr[2] || hdr[5] == 0xff)
    return hdr[2];
  return hdr[5];
}

/*
 * Payload offset of the first frame that starts in a block, past the tail of
 * a frame carried over from the previous block.
 */
static int ds2_block_anchor(const uint8_t *hdr) {
  return av_clip(2 * hdr[1] + 2 * (hdr[0] >> 7) - DS2_AUDIO_BLOCK_HEADER_SIZE,
                 0, DS2_BLOCK_PAYLOAD_SIZE);
}

/*
 * Frames that start in a block, from its header and the size bytes of it the
 * file holds, as the reader takes them: from the anchor on, up to the stated
 * count and while they start in the block. A re-sync block states more than
 * that. In QP7 the type bit gives each frame's size. In a cut file only frames
 * whose first byte is there count; the reader returns the last one
 * zero-filled. *need is set to the bytes the last frame of a whole block takes
 * from the blocks after it.
 */
static int ds2_block_frames_held(const uint8_t *buf, int size, int qp7,
                                 int *need) {
  int frame_count = ds2_block_frames(buf), n = 0;
  int off = DS2_AUDIO_BLOCK_HEADER_SIZE + ds2_block_anchor(buf);

  while (n < frame_count && off < size) {
    n++;
    if (qp7 && off + 1 < size && !(buf[off + 1] & 0x80))
      off += DS2_QP7_NOISE_FRAME_SIZE;
    else
      off += DS2_QP_FRAME_SIZE;
  }
  *need = size == DS2_BLOCK_SIZE ? FFMAX(off - DS2_BLOCK_SIZE, 0) : 0;
  return n;
}

/*
 * Whether the reader drops a frame that the block at pos ends need bytes into.
 * It takes the rest from the next block if that block's anchor is where the
 * rest ends, and drops the frame at a re-sync block, whose anchor is elsewhere.
 * A block that states no frames passes up to its anchor's worth on to the
 * block after it. Where the file ends, the reader keeps the frame.
 */
static int ds2_frame_dropped(AVFormatContext *s, int64_t pos, int need) {
  uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];

  while (need > 0) {
    pos += DS2_BLOCK_SIZE;
    if (avio_seek(s->pb, pos, SEEK_SET) < 0 ||
        avio_read(s->pb, hdr, sizeof(hdr)) != sizeof(hdr))
      return 0;
    if (ds2_block_frames(hdr))
      return ds2_block_anchor(hdr) != need;
    need -= ds2_block_anchor(hdr);
  }
  return 0;
}

/*
 * Read the header of the block at pos, and in QP7, where the frame sizes are
 * in the payload, the rest of its len bytes. *qp7 below 0 is set from this
 * block's format.
 */
static int ds2_read_block(AVFormatContext *s, int64_t pos, uint8_t *buf,
                          int len, int *qp7) {
  if (len < DS2_AUDIO_BLOCK_HEADER_SIZE ||
      avio_seek(s->pb, pos, SEEK_SET) < 0 ||
      avio_read(s->pb, buf, DS2_AUDIO_BLOCK_HEADER_SIZE) !=
          DS2_AUDIO_BLOCK_HEADER_SIZE)
    return AVERROR_EOF;
  if (*qp7 < 0)
    *qp7 = buf[4] == DS2_FORMAT_QP7;
  len -= DS2_AUDIO_BLOCK_HEADER_SIZE;
  if (*qp7 && avio_read(s->pb, buf + DS2_AUDIO_BLOCK_HEADER_SIZE, len) != len)
    return AVERROR_EOF;
  return 0;
}

/*
 * Frames the reader returns that start in the block at pos, of which buf holds
 * the size bytes the file has.
 */
static int ds2_block_frames_read(AVFormatContext *s, int64_t pos,
                                 const uint8_t *buf, int size, int qp7) {
  int need, n = ds2_block_frames_held(buf, size, qp7, &need);

  if (need && ds2_frame_dropped(s, pos, need))
    n--;
  return n;
}

static int64_t ds2_count_total_frames(AVFormatContext *s) {
  AVIOContext *pb = s->pb;
  int header_size = ((DS2DemuxContext *)s->priv_data)->header_size;
  int64_t size = avio_size(pb), total = 0, pos;
  uint8_t buf[DS2_BLOCK_SIZE];
  int len, qp7 = -1;

  if (!(pb->seekable & AVIO_SEEKABLE_NORMAL))
    return 0;
  if (size < header_size)
    return AVERROR_INVALIDDATA;

  for (pos = header_size; pos < size; pos += DS2_BLOCK_SIZE) {
    len = FFMIN(size - pos, DS2_BLOCK_SIZE);
    if (ds2_read_block(s, pos, buf, len, &qp7) < 0)
      break;
    total += ds2_block_frames_read(s, pos, buf, len, qp7);
  }

  return total;
}

static int ds2_load_block(AVFormatContext *s, int in_frame_offset) {
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];
  int ret, frame_count, anchor, aligned;

  if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
    return AVERROR_EOF;

  ret = avio_read(pb, hdr, sizeof(hdr));
  if (ret != DS2_AUDIO_BLOCK_HEADER_SIZE)
    return ret < 0 ? ret : AVERROR_EOF;

  frame_count = ds2_block_frames(hdr);
  anchor      = ds2_block_anchor(hdr);

  ctx->blk_frames  = frame_count;

  if (frame_count == 0) {
    ctx->counter = anchor;
  } else {
    /*
     * QP frames re-anchor at payload offset 2*byte1 - 6 at every block (the
     * swap bit is 0 in QP). On gap-free audio the anchor coincides with the
     * straddle carried from the previous block, so a flat concatenation is
     * correct. At a segment boundary the recorder emits a re-sync block whose
     * anchor jumps: the leading bytes are an orphaned straddle tail, the
     * partial frame in flight is stale, and framing must restart at the anchor.
     * frame_count over-counts on these blocks (observed 19), so there the block
     * end is the limit.
     * Confirmed byte-for-byte against the Olympus DssParser.dll.
     */
    aligned = (!in_frame_offset && !anchor) ||
              (in_frame_offset && in_frame_offset + anchor == ctx->frame_size);
    if (!aligned) {
      ctx->resync_drop = 1;
      if (anchor > 0 && avio_skip(pb, anchor) < 0)
        return AVERROR_EOF;
      ctx->counter = DS2_BLOCK_PAYLOAD_SIZE - anchor;
      return 0;
    }

    ctx->counter = DS2_BLOCK_PAYLOAD_SIZE;
  }

  return 0;
}

/*
 * Skip the rest of the current block and load the next one. A pause block
 * holds padding after its last frame. The file header is a whole number of
 * blocks, so blocks are aligned to their size in the file.
 */
static int ds2_next_block(AVFormatContext *s, int in_frame_offset) {
  DS2DemuxContext *ctx = s->priv_data;
  int64_t in_block = avio_tell(s->pb) % DS2_BLOCK_SIZE;

  ctx->counter = 0;
  ctx->resync_drop = 0;
  if (in_block > 0 && avio_skip(s->pb, DS2_BLOCK_SIZE - in_block) < 0)
    return AVERROR_EOF;

  return ds2_load_block(s, in_frame_offset);
}

static int ds2_read_header(AVFormatContext *s) {
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  AVStream *st;
  uint8_t block_header[DS2_AUDIO_BLOCK_HEADER_SIZE];
  int ret, samples_per_frame;
  int64_t ret64;
  int version;

  if ((ret64 = avio_seek(pb, 0, SEEK_SET)) < 0)
    return (int)ret64;
  version = avio_r8(pb);
  if (version < 1 || version > 16)
    return AVERROR_INVALIDDATA;
  ctx->header_size = ds2_header_size(version);

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

  ret64 = ds2_count_total_frames(s);
  if (ret64 < 0)
    return (int)ret64;
  ctx->total_frames = ret64;

  if ((ret64 = avio_seek(pb, ctx->header_size, SEEK_SET)) < 0)
    return (int)ret64;

  ret = avio_read(pb, block_header, DS2_AUDIO_BLOCK_HEADER_SIZE);
  if (ret < DS2_AUDIO_BLOCK_HEADER_SIZE)
    return ret < 0 ? ret : AVERROR_EOF;

  ctx->format_type = block_header[4];

  st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
  st->codecpar->codec_id = AV_CODEC_ID_DS2;
  st->codecpar->codec_tag = ctx->format_type;
  st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;

  if (ctx->format_type == DS2_FORMAT_SP ||
      ctx->format_type == DS2_FORMAT_SP_VR ||
      ctx->format_type == DS2_FORMAT_LP) {
    av_log(s, AV_LOG_ERROR, "DS2 SP and LP files are read by the dss demuxer\n");
    return AVERROR_INVALIDDATA;
  } else if (ctx->format_type == DS2_FORMAT_QP ||
             ctx->format_type == DS2_FORMAT_QP7) {
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

    if (file_size > ctx->header_size && s->duration > 0)
      s->bit_rate = (file_size - ctx->header_size) * 8LL * AV_TIME_BASE / s->duration;
    else
      s->bit_rate = 8LL * frame_bytes * st->codecpar->sample_rate / samples_per_frame;

    st->codecpar->bit_rate = s->bit_rate;
  }

  /* Let the first read load the block: a file cut out of a longer recording
   * can start with the tail of a frame, before the block's anchor. */
  if ((ret64 = avio_seek(pb, ctx->header_size, SEEK_SET)) < 0)
    return (int)ret64;
  ctx->frames_read = 0;
  ctx->counter     = 0;
  ctx->blk_frames  = 0;

  return 0;
}

static int ds2_qp_read_packet(AVFormatContext *s, AVPacket *pkt) {
  DS2DemuxContext *ctx = s->priv_data;
  int ret, offset = 0;
  int64_t pos;

  if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
    return AVERROR_EOF;

  while (ctx->blk_frames <= 0 || ctx->counter == 0) {
    ret = ds2_next_block(s, 0);
    if (ret < 0)
      return ret;
  }
  pos = avio_tell(s->pb);

  ret = av_new_packet(pkt, DS2_QP_FRAME_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);
  if (ret < 0)
    return ret;
  pkt->size = DS2_QP_FRAME_SIZE;

  pkt->duration = DS2_QP_SAMPLES_PER_FRAME;
  pkt->pts = ctx->frames_read * DS2_QP_SAMPLES_PER_FRAME;
  pkt->pos = pos;
  pkt->stream_index = 0;

  /*
   * Frames are 56 bytes, or 12 for a QP7 noise frame, in a continuous
   * bitstream across blocks. Frames can span block boundaries, so we read
   * in pieces if needed.
   */
  ctx->frame_size = DS2_QP_FRAME_SIZE;
  while (offset < ctx->frame_size) {
    int to_read = FFMIN(ctx->frame_size - offset, ctx->counter);
    if (to_read <= 0) {
      ret = ds2_next_block(s, offset);
      if (ret == AVERROR_EOF && offset > 0 && !ctx->resync_drop)
        goto truncated;
      if (ret < 0)
        return ret;
      if (ctx->resync_drop) {
        offset = 0;
        ctx->frame_size = DS2_QP_FRAME_SIZE;
      }
      to_read = FFMIN(ctx->frame_size - offset, ctx->counter);
      if (to_read <= 0)
        continue;
    }

    /* QP7: the type bit in the first 16-bit word gives the frame size */
    if (ctx->format_type == DS2_FORMAT_QP7 && !offset)
      to_read = FFMIN(to_read, 2);

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

    if (!offset) {
      ctx->blk_frames--;
      if (ctx->format_type == DS2_FORMAT_QP7 && !(pkt->data[1] & 0x80))
        ctx->frame_size = DS2_QP7_NOISE_FRAME_SIZE;
    }
    offset += to_read;
    ctx->counter -= to_read;
  }

  if (ctx->frame_size != DS2_QP_FRAME_SIZE)
    av_shrink_packet(pkt, ctx->frame_size);

  ctx->frames_read++;
  return 0;

truncated:
  /* The file ends inside a frame its block still counts. Olympus decodes
   * that frame too, so keep it, with the missing bytes zeroed. */
  memset(pkt->data + offset, 0, ctx->frame_size - offset);
  if (ctx->frame_size != DS2_QP_FRAME_SIZE)
    av_shrink_packet(pkt, ctx->frame_size);
  ctx->counter = 0;
  ctx->frames_read++;
  return 0;
}

static int ds2_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags) {
  DS2DemuxContext *ctx = s->priv_data;
  int64_t ret, seekto, target = timestamp / DS2_QP_SAMPLES_PER_FRAME, frames = 0;
  int64_t last = -1, last_frames = 0, size = avio_size(s->pb);
  uint8_t buf[DS2_BLOCK_SIZE];
  int frame_count, len, qp7 = ctx->format_type == DS2_FORMAT_QP7;

  if (target < 0)
    target = 0;

  /* Find the block in which the target frame starts, or past the end the
   * last block that holds frames. */
  for (seekto = ctx->header_size; ; seekto += DS2_BLOCK_SIZE) {
    len = FFMIN(size - seekto, DS2_BLOCK_SIZE);
    ret = ds2_read_block(s, seekto, buf, len, &qp7);
    if (ret < 0) {
      if (last < 0)
        return ret;
      seekto = last;
      frames = last_frames;
      break;
    }
    frame_count = ds2_block_frames_read(s, seekto, buf, len, qp7);
    if (frames + frame_count > target)
      break;
    if (frame_count) {
      last        = seekto;
      last_frames = frames;
    }
    frames += frame_count;
  }

  /* The next read loads this block and starts at its anchor, skipping the
   * tail of a frame that began in the previous block. */
  ret = avio_seek(s->pb, seekto, SEEK_SET);
  if (ret < 0)
    return ret;
  ctx->frames_read = frames;
  ctx->counter     = 0;
  ctx->blk_frames  = 0;

  return 0;
}

const FFInputFormat ff_ds2_demuxer = {
    .p.name = "ds2",
    .p.long_name = NULL_IF_CONFIG_SMALL("Digital Speech Standard Pro (DS2)"),
    .p.extensions = "ds2",
    .p.flags = AVFMT_NO_BYTE_SEEK,
    .priv_data_size = sizeof(DS2DemuxContext),
    .read_probe = ds2_probe,
    .read_header = ds2_read_header,
    .read_packet = ds2_qp_read_packet,
    .read_seek = ds2_read_seek,
};
