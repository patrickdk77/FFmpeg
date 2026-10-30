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
#include "libavutil/opt.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "ds2crypto.h"
#include "dss_metadata.h"

#define DS2_BLOCK_SIZE 512
#define DS2_AUDIO_BLOCK_HEADER_SIZE 6
#define DS2_BLOCK_PAYLOAD_SIZE (DS2_BLOCK_SIZE - DS2_AUDIO_BLOCK_HEADER_SIZE)

#define DS2_FORMAT_SP 0 /* Standard Play, read by the DSS demuxer */
#define DS2_FORMAT_SP_VR 1 /* variable rate SP, read by the DSS demuxer */
#define DS2_FORMAT_LP 2 /* Long Play (G.723.1), read by the DSS demuxer */
#define DS2_FORMAT_LP_ALT 4 /* LP, read by the DSS demuxer */
#define DS2_FORMAT_QP 6 /* Quality Play, 16000 Hz */
#define DS2_FORMAT_QP7 7 /* QP with 12-byte noise frames, 16000 Hz */

#define DS2_QP_FRAME_SIZE 56         /* 448 bits = 56 bytes */
#define DS2_QP_SAMPLES_PER_FRAME 256 /* 64 samples * 4 subframes */
#define DS2_QP_SAMPLE_RATE 16000
#define DS2_QP7_NOISE_FRAME_SIZE 12

typedef struct DS2DemuxContext {
  AVClass *class;
  int header_size;      /* in bytes, from ds2_header_size() */
  int format_type;      /* block format byte, DS2_FORMAT_QP or DS2_FORMAT_QP7 */
  int counter;          /* bytes remaining in current block payload */
  int64_t total_frames; /* total frame count from block headers */
  int64_t frames_read;  /* frames read so far */
  int resync_drop;      /* QP: partial frame orphaned at a re-sync block */
  int frame_size;       /* QP: size of the frame being read */
  int blk_frames;       /* frames still to start in the current block */
  AVIOContext *pb;      /* s->pb, or decrypt_pb in an encrypted file */
  AVIOContext *decrypt_pb;
  char *password;
  int extract;          /* DS2ExtractMode */
  int64_t frames_out;   /* frames returned so far */
  DS2Annotations annotations;
} DS2DemuxContext;

#define OFFSET(x) offsetof(DS2DemuxContext, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM

/*
 * The first byte is the header size in 512-byte blocks (Olympus 2/3,
 * Grundig/Philips 6/7, newer Olympus firmware 8). Philips SpeechExec
 * writes 1 in the files it splits out of a recording, over the usual
 * 3-block header; Olympus writes 1 over 1 block, followed by an audio block.
 */
static int ds2_header_size(const uint8_t *buf, int size) {
  if (buf[0] == 1 && (size < DS2_BLOCK_SIZE + 4 ||
                      buf[DS2_BLOCK_SIZE + 3] != 0xff))
    return 3 * DS2_BLOCK_SIZE;
  return buf[0] * DS2_BLOCK_SIZE;
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

static int ds2_probe_format(const uint8_t *buf, int size, int pos) {
  while (pos + DS2_AUDIO_BLOCK_HEADER_SIZE <= size) {
    if (ds2_block_frames(buf + pos))
      return buf[pos + 4];
    pos += DS2_BLOCK_SIZE;
  }
  return -1;
}

static int ds2_probe(const AVProbeData *p) {
  int format;

  /* bytes 1..3 are the "ds2" tag, or "enc" in an encrypted file, whose block
   * headers are in the clear */
  if (p->buf_size < 4 || (AV_RL32(p->buf) != DS2_ENCRYPTED_MAGIC &&
      (p->buf[1] != 'd' || p->buf[2] != 's' || p->buf[3] != '2')))
    return 0;
  if (!p->buf[0])
    return 0;

  /* SP, variable rate SP and LP files belong to the DSS demuxer. A long
   * header can put the first block past the probe buffer; ask for more. */
  format = ds2_probe_format(p->buf, p->buf_size,
                            ds2_header_size(p->buf, p->buf_size));
  if (format < 0)
    return AVPROBE_SCORE_RETRY;
  if (format == DS2_FORMAT_SP || format == DS2_FORMAT_SP_VR ||
      format == DS2_FORMAT_LP || format == DS2_FORMAT_LP_ALT)
    return 0;

  return AVPROBE_SCORE_MAX;
}

static int64_t ds2_qp_count_output_frames(const DS2DemuxContext *ctx) {
  int64_t f, n = 0;

  for (f = 0; f < ctx->total_frames; f++) {
    if (ff_dss_ds2_frame_included(&ctx->annotations, ctx->extract, f))
      n++;
  }
  return n;
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
  AVIOContext *pb = ((DS2DemuxContext *)s->priv_data)->pb;
  uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];

  while (need > 0) {
    pos += DS2_BLOCK_SIZE;
    if (avio_seek(pb, pos, SEEK_SET) < 0 ||
        avio_read(pb, hdr, sizeof(hdr)) != sizeof(hdr))
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
  AVIOContext *pb = ((DS2DemuxContext *)s->priv_data)->pb;

  if (len < DS2_AUDIO_BLOCK_HEADER_SIZE ||
      avio_seek(pb, pos, SEEK_SET) < 0 ||
      avio_read(pb, buf, DS2_AUDIO_BLOCK_HEADER_SIZE) !=
          DS2_AUDIO_BLOCK_HEADER_SIZE)
    return AVERROR_EOF;
  if (*qp7 < 0)
    *qp7 = buf[4] == DS2_FORMAT_QP7;
  len -= DS2_AUDIO_BLOCK_HEADER_SIZE;
  if (*qp7 && avio_read(pb, buf + DS2_AUDIO_BLOCK_HEADER_SIZE, len) != len)
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
  AVIOContext *pb = ((DS2DemuxContext *)s->priv_data)->pb;
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
  AVIOContext *pb = ctx->pb;
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
  int64_t in_block = avio_tell(ctx->pb) % DS2_BLOCK_SIZE;

  ctx->counter = 0;
  ctx->resync_drop = 0;
  if (in_block > 0 && avio_skip(ctx->pb, DS2_BLOCK_SIZE - in_block) < 0)
    return AVERROR_EOF;

  return ds2_load_block(s, in_frame_offset);
}

static int ds2_read_header(AVFormatContext *s) {
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  AVStream *st;
  uint8_t block_header[DS2_AUDIO_BLOCK_HEADER_SIZE];
  uint8_t file_magic[DS2_BLOCK_SIZE + 4];
  int ret, samples_per_frame;
  int64_t ret64, out_frames;

  if ((ret64 = avio_seek(pb, 0, SEEK_SET)) < 0)
    return (int)ret64;
  ret = avio_read(pb, file_magic, sizeof(file_magic));
  if (ret < 4)
    return ret < 0 ? ret : AVERROR_EOF;
  if (!file_magic[0] || (memcmp(file_magic + 1, "ds2", 3) &&
                          AV_RL32(file_magic) != DS2_ENCRYPTED_MAGIC))
    return AVERROR_INVALIDDATA;
  ctx->header_size = ds2_header_size(file_magic, ret);

  ctx->pb = s->pb;
  if (AV_RL32(file_magic) == DS2_ENCRYPTED_MAGIC) {
    ret = ff_ds2_decrypt_open(s, ctx->password, ctx->header_size,
                              &ctx->decrypt_pb);
    if (ret < 0)
      return ret;
    ctx->pb = ctx->decrypt_pb;
  }
  pb = ctx->pb;

  st = avformat_new_stream(s, NULL);
  if (!st)
    return AVERROR(ENOMEM);

  ret = ff_dss_read_ds2_metadata(s);
  if (ret < 0)
    av_log(s, AV_LOG_WARNING, "Failed to read DS2 metadata\n");

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
      ctx->format_type == DS2_FORMAT_LP ||
      ctx->format_type == DS2_FORMAT_LP_ALT) {
    av_log(s, AV_LOG_ERROR, "DS2 SP and LP files are read by the dss demuxer\n");
    return AVERROR_INVALIDDATA;
  } else if (ctx->format_type == DS2_FORMAT_QP ||
             ctx->format_type == DS2_FORMAT_QP7) {
    st->codecpar->sample_rate = DS2_QP_SAMPLE_RATE;
    samples_per_frame = DS2_QP_SAMPLES_PER_FRAME;

    ret = ff_dss_read_ds2_annotations(s, ctx->header_size,
                                      DS2_QP_SAMPLES_PER_FRAME * 1000 /
                                      DS2_QP_SAMPLE_RATE, &ctx->annotations);
    if (ret < 0)
      return ret;
    if (ctx->extract == DS2_EXTRACT_ANNOTATIONS && !ctx->annotations.nb)
      av_log(s, AV_LOG_WARNING, "No annotations to extract\n");
  } else {
    avpriv_request_sample(s, "DS2 format type %d", ctx->format_type);
    return AVERROR_PATCHWELCOME;
  }

  out_frames = ctx->extract == DS2_EXTRACT_ALL ? ctx->total_frames
                                               : ds2_qp_count_output_frames(ctx);

  avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
  st->start_time = 0;
  if (out_frames > 0) {
    int64_t nb_samples = (int64_t)out_frames * samples_per_frame;
    int64_t total_duration = av_rescale(ctx->total_frames * samples_per_frame,
                                        AV_TIME_BASE, st->codecpar->sample_rate);
    int frame_bytes = DS2_QP_FRAME_SIZE;
    int64_t file_size = avio_size(pb);

    st->duration = nb_samples;
    s->duration  = av_rescale_q(nb_samples, (AVRational){1, st->codecpar->sample_rate},
                                AV_TIME_BASE_Q);

    if (file_size > ctx->header_size && total_duration > 0)
      s->bit_rate = (file_size - ctx->header_size) * 8LL * AV_TIME_BASE / total_duration;
    else
      s->bit_rate = 8LL * frame_bytes * st->codecpar->sample_rate / samples_per_frame;

    st->codecpar->bit_rate = s->bit_rate;
  }

  /* Let the first read load the block: a file cut out of a longer recording
   * can start with the tail of a frame, before the block's anchor. */
  if ((ret64 = avio_seek(pb, ctx->header_size, SEEK_SET)) < 0)
    return (int)ret64;
  ctx->frames_read = 0;
  ctx->frames_out  = 0;
  ctx->counter     = 0;
  ctx->blk_frames  = 0;

  return 0;
}

static int ds2_qp_read_frame(AVFormatContext *s, AVPacket *pkt) {
  DS2DemuxContext *ctx = s->priv_data;
  int ret, offset = 0;

  if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
    return AVERROR_EOF;

  while (ctx->blk_frames <= 0 || ctx->counter == 0) {
    ret = ds2_next_block(s, 0);
    if (ret < 0)
      return ret;
  }

  ret = av_new_packet(pkt, DS2_QP_FRAME_SIZE);
  if (ret < 0)
    return ret;

  pkt->duration = DS2_QP_SAMPLES_PER_FRAME;
  pkt->pts = ctx->frames_read * DS2_QP_SAMPLES_PER_FRAME;
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

    if (!offset)
      pkt->pos = avio_tell(ctx->pb);
    ret = avio_read(ctx->pb, pkt->data + offset, to_read);
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

static int ds2_qp_read_packet(AVFormatContext *s, AVPacket *pkt) {
  DS2DemuxContext *ctx = s->priv_data;
  int ret, skipped = 0;

  for (;;) {
    ret = ds2_qp_read_frame(s, pkt);
    if (ret < 0)
      return ret;

    if (ff_dss_ds2_frame_included(&ctx->annotations, ctx->extract,
                                  ctx->frames_read - 1)) {
      pkt->pts = ctx->frames_out++ * DS2_QP_SAMPLES_PER_FRAME;
      if (skipped &&
          !av_packet_new_side_data(pkt, AV_PKT_DATA_DECODER_RESET, 0))
        return AVERROR(ENOMEM);
      return 0;
    }

    av_packet_unref(pkt);
    skipped = 1;
  }
}

static int ds2_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags) {
  DS2DemuxContext *ctx = s->priv_data;
  int64_t ret, seekto, target = timestamp / DS2_QP_SAMPLES_PER_FRAME, frames = 0;
  int64_t last = -1, last_frames = 0, size = avio_size(ctx->pb);
  uint8_t buf[DS2_BLOCK_SIZE];
  int frame_count, len, qp7 = ctx->format_type == DS2_FORMAT_QP7;

  if (target < 0)
    target = 0;

  /* With extract, timestamps count the frames returned: find the frame of
   * the file that the target is, or past the end the last one returned. */
  if (ctx->extract != DS2_EXTRACT_ALL) {
    int64_t out = target, f;

    for (f = target = 0; f < ctx->total_frames; f++) {
      if (!ff_dss_ds2_frame_included(&ctx->annotations, ctx->extract, f))
        continue;
      target = f;
      if (!out--)
        break;
    }
  }

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
  ret = avio_seek(ctx->pb, seekto, SEEK_SET);
  if (ret < 0)
    return ret;
  ctx->frames_read = frames;
  ctx->frames_out  = 0;
  for (frames = 0; frames < ctx->frames_read; frames++)
    ctx->frames_out += ff_dss_ds2_frame_included(&ctx->annotations,
                                                 ctx->extract, frames);
  ctx->counter     = 0;
  ctx->blk_frames  = 0;

  return 0;
}

static int ds2_read_close(AVFormatContext *s) {
  DS2DemuxContext *ctx = s->priv_data;

  ff_ds2_decrypt_close(&ctx->decrypt_pb);
  return 0;
}

static const AVOption ds2_options[] = {
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

static const AVClass ds2_demuxer_class = {
    .class_name = "ds2 demuxer",
    .item_name  = av_default_item_name,
    .option     = ds2_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFInputFormat ff_ds2_demuxer = {
    .p.name = "ds2",
    .p.long_name = NULL_IF_CONFIG_SMALL("Digital Speech Standard Pro (DS2)"),
    .p.extensions = "ds2",
    .p.flags = AVFMT_NO_BYTE_SEEK,
    .p.priv_class = &ds2_demuxer_class,
    .priv_data_size = sizeof(DS2DemuxContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe = ds2_probe,
    .read_header = ds2_read_header,
    .read_packet = ds2_qp_read_packet,
    .read_seek = ds2_read_seek,
    .read_close = ds2_read_close,
};
