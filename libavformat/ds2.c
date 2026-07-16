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
 *
 * Demuxer options (private, set before -i):
 *   -password <string>   encrypted DS2 passphrase
 *   -extract <mode>      QP content selection: all (default), main, annotations
 *
 * Example:
 *   ffmpeg -extract main -i recording.ds2 -c:a pcm_s16le out.wav
 */

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/avstring.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "ds2crypto.h"

#define DS2_HEADER_SIZE 0x600 /* 1536 bytes */
#define DS2_BLOCK_SIZE 512
#define DS2_AUDIO_BLOCK_HEADER_SIZE 6
#define DS2_BLOCK_PAYLOAD_SIZE (DS2_BLOCK_SIZE - DS2_AUDIO_BLOCK_HEADER_SIZE)

#define DS2_FORMAT_SP 0 /* Standard Play, 12000 Hz */
#define DS2_FORMAT_QP 6 /* Quality Play, 16000 Hz */
#define DS2_FORMAT_QP7 7 /* Grundig/Philips QP: variable-length records */
#define DS2_QP7_SHORT_SIZE 12 /* bytes: unvoiced noise-fill record */
#define DS2_QP7_LONG_SIZE 56  /* bytes: full CELP record */

#define DS2_SP_FRAME_SIZE 42 /* SP: 328 bits ~ 41 bytes, padded to 42 */
#define DS2_SP_SAMPLES_PER_FRAME 288 /* 72 samples * 4 subframes */
#define DS2_SP_SAMPLE_RATE 12000

#define DS2_QP_FRAME_SIZE 56         /* 448 bits = 56 bytes */
#define DS2_QP_SAMPLES_PER_FRAME 256 /* 64 samples * 4 subframes */
#define DS2_QP_SAMPLE_RATE 16000

/* Metadata offsets (DSS-compatible; DS2 adds a typed table at 0x240) */
#define DS2_HEAD_OFFSET_ENCODER     0xc
#define DS2_ENCODER_SIZE            16
#define DS2_HEAD_OFFSET_START_TIME  0x26
#define DS2_HEAD_OFFSET_END_TIME    0x32
#define DS2_TIME_SIZE               12
#define DS2_HEAD_OFFSET_COMMENT     0x31e
#define DS2_COMMENT_SIZE            64
#define DS2_HEAD_OFFSET_EXT_META    0x240
#define DS2_META_ENTRY_SIZE         60
#define DS2_META_KEY_SIZE           30
#define DS2_META_VAL_SIZE           30
#define DS2_HEAD_EXT_META_END       0x31e

#define DS2_HEAD_OFFSET_ANNOTATIONS 0x400
#define DS2_HEAD_ANNOTATIONS_END      0x500
#define DS2_MAX_ANNOTATIONS           32
#define DS2_MS_PER_QP_FRAME           16

enum DS2ExtractMode {
  DS2_EXTRACT_ALL = 0,
  DS2_EXTRACT_MAIN,
  DS2_EXTRACT_ANNOTATIONS,
};

typedef struct DS2AnnotationRange {
  int first_frame; /* inclusive */
  int last_frame;  /* exclusive */
} DS2AnnotationRange;

typedef struct DS2Qp7Record {
  int off;   /* byte offset into the concatenated payload stream */
  int size;  /* 12 (short) or 56 (long) */
  int reset; /* reset the decoder before this record (segment boundary) */
} DS2Qp7Record;

typedef struct DS2DemuxContext {
  AVClass *class;
  int extract;          /* DS2ExtractMode; QP only */
  int format_type;      /* DS2_FORMAT_SP or DS2_FORMAT_QP */
  int counter;          /* bytes remaining in current block payload */
  int swap;             /* SP byte-swap state */
  int ds2_sp_swap_byte; /* saved swap byte for SP mode */
  int total_frames;     /* total frame count from block headers */
  int frames_read;      /* frames read so far */
  int swap_reset_pending;
  int next_blk_swap;    /* swap state from next non-empty block */
  int encrypted;
  int key_mode;
  int header_size;      /* first_byte * 512 (0x600 Olympus, 0xe00 Grundig/Philips);
                         * magic 0x01 annotation slices keep the 0x600 layout */
  uint8_t file_magic_byte; /* byte 0 of file header (0x01 = annotation slice) */
  char *password;
  DS2DecryptState decrypt_state;
  uint8_t cur_block[DS2_BLOCK_SIZE];
  int64_t cur_block_start;
  int cur_block_valid;
  /* QP block-quantized demux state */
  int abs_frame;           /* frames emitted (or skipped) so far */
  int qp_blk_frames_left;  /* fresh frames remaining in the current block */
  int qp_reset_next;       /* flag decoder reset on the next emitted frame */
  /* QP7 (format 7): variable-length records precomputed at open */
  int is_qp7;
  uint8_t *qp7_raw;        /* concatenated block payloads */
  int qp7_raw_len;
  DS2Qp7Record *qp7_records;
  int qp7_nb_records;
  int qp7_rec_idx;
  DS2AnnotationRange annotations[DS2_MAX_ANNOTATIONS];
  int nb_annotations;
} DS2DemuxContext;

#define OFFSET(x) offsetof(DS2DemuxContext, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM

static void ds2_invalidate_block_cache(DS2DemuxContext *ctx)
{
  ctx->cur_block_valid = 0;
  ctx->cur_block_start = -1;
}

static int64_t ds2_block_start_for_pos(int64_t pos)
{
  if (pos < DS2_HEADER_SIZE)
    return -1;
  return DS2_HEADER_SIZE +
         ((pos - DS2_HEADER_SIZE) / DS2_BLOCK_SIZE) * DS2_BLOCK_SIZE;
}

static int ds2_ensure_decrypted_block(AVFormatContext *s, int64_t pos)
{
  DS2DemuxContext *ctx = s->priv_data;
  int64_t bstart = ds2_block_start_for_pos(pos);
  int ret;

  if (!ctx->encrypted || bstart < 0)
    return 0;
  if (ctx->cur_block_valid && ctx->cur_block_start == bstart)
    return 0;

  avio_seek(s->pb, bstart, SEEK_SET);
  ret = avio_read(s->pb, ctx->cur_block, DS2_BLOCK_SIZE);
  if (ret < DS2_BLOCK_SIZE)
    return ret < 0 ? ret : AVERROR_EOF;

  ds2_decrypt_record(&ctx->decrypt_state, ctx->key_mode, ctx->cur_block);
  ctx->cur_block_start = bstart;
  ctx->cur_block_valid = 1;
  return 0;
}

static int ds2_io_read(AVFormatContext *s, uint8_t *buf, int size)
{
  DS2DemuxContext *ctx = s->priv_data;
  int total = 0;

  while (size > 0) {
    int64_t pos = avio_tell(s->pb);
    int ret, n;

    if (!ctx->encrypted || pos < DS2_HEADER_SIZE) {
      n = avio_read(s->pb, buf, size);
      if (n < 0)
        return n < 0 ? n : (total ? total : AVERROR_EOF);
      if (n == 0)
        return total ? total : AVERROR_EOF;
      buf  += n;
      size -= n;
      total += n;
      continue;
    }

    ret = ds2_ensure_decrypted_block(s, pos);
    if (ret < 0)
      return total ? total : ret;

    n = FFMIN(size, DS2_BLOCK_SIZE - (int)(pos - ctx->cur_block_start));
    memcpy(buf, ctx->cur_block + (pos - ctx->cur_block_start), n);
    avio_seek(s->pb, pos + n, SEEK_SET);
    buf  += n;
    size -= n;
    total += n;
  }

  return total;
}

static int ds2_probe(const AVProbeData *p) {
  if (p->buf_size < 4)
    return 0;
  if (AV_RL32(p->buf) == DS2_ENCRYPTED_MAGIC)
    return AVPROBE_SCORE_MAX;
  /* Byte 0 is the header size in 512-byte blocks (Olympus 3 -> 0x600,
   * Grundig/Philips 6/7 -> 0xc00/0xe00); magic 0x01 is an annotation slice.
   * Bytes 1..3 are the "ds2" tag. */
  if (p->buf[1] == 'd' && p->buf[2] == 's' && p->buf[3] == '2' &&
      p->buf[0] >= 1 && p->buf[0] <= 16)
    return AVPROBE_SCORE_MAX;

  return 0;
}

static void ds2_trim_string(char *s, int size)
{
    int i;

    for (i = 0; i < size && s[i]; i++) {
        if ((unsigned char)s[i] < 0x20 || (unsigned char)s[i] == 0xff) {
            s[i] = 0;
            break;
        }
    }
    while (i > 0 && ((unsigned char)s[i - 1] <= 0x20 ||
                     (unsigned char)s[i - 1] == 0xff))
        s[--i] = 0;
}

static int ds2_field_is_blank(const char *s, int size)
{
    int i;

    for (i = 0; i < size; i++) {
        if (s[i] && (unsigned char)s[i] != 0xff)
            return 0;
    }
    return 1;
}

static int ds2_parse_ascii_time(const uint8_t *buf, char *datetime, size_t dsize)
{
    char string[DS2_TIME_SIZE + 1];
    int y, month, d, h, minute, sec;
    int i;

    memcpy(string, buf, DS2_TIME_SIZE);
    string[DS2_TIME_SIZE] = 0;
    for (i = 0; i < DS2_TIME_SIZE; i++) {
        if (string[i] < '0' || string[i] > '9')
            return AVERROR_INVALIDDATA;
    }

    if (sscanf(string, "%2d%2d%2d%2d%2d%2d", &y, &month, &d, &h, &minute, &sec) != 6)
        return AVERROR_INVALIDDATA;
    if (month < 1 || month > 12 || d < 1 || d > 31 ||
        h < 0 || h > 23 || minute < 0 || minute > 59 || sec < 0 || sec > 59)
        return AVERROR_INVALIDDATA;

    snprintf(datetime, dsize, "%.4d-%.2d-%.2dT%.2d:%.2d:%.2d",
             y + 2000, month, d, h, minute, sec);
    return 0;
}

static int ds2_set_metadata_time(AVFormatContext *s, const uint8_t *buf,
                                 const char *key)
{
    char datetime[64];
    int ret;

    ret = ds2_parse_ascii_time(buf, datetime, sizeof(datetime));
    if (ret < 0)
        return ret;

    return av_dict_set(&s->metadata, key, datetime, 0);
}

static int ds2_set_metadata_copy(AVFormatContext *s, const char *src, int size,
                                 const char *key)
{
    char *value;

    if (ds2_field_is_blank(src, size))
        return 0;

    value = av_malloc(size + 1);
    if (!value)
        return AVERROR(ENOMEM);
    memcpy(value, src, size);
    value[size] = 0;
    ds2_trim_string(value, size);
    if (!value[0]) {
        av_free(value);
        return 0;
    }

    return av_dict_set(&s->metadata, key, value, AV_DICT_DONT_STRDUP_VAL);
}

static int ds2_read_extended_metadata(AVFormatContext *s, const uint8_t *hdr)
{
    int off, ret = 0;

    if (hdr[DS2_HEAD_OFFSET_EXT_META] == 0xff)
        return 0;

    for (off = DS2_HEAD_OFFSET_EXT_META;
         off + DS2_META_ENTRY_SIZE <= DS2_HEAD_EXT_META_END;
         off += DS2_META_ENTRY_SIZE) {
        const char *key = (const char *)(hdr + off);
        const char *val = (const char *)(hdr + off + DS2_META_KEY_SIZE);
        char keybuf[DS2_META_KEY_SIZE + 1];
        char valbuf[DS2_META_VAL_SIZE + 1];

        if (ds2_field_is_blank(key, DS2_META_KEY_SIZE))
            break;

        av_strlcpy(keybuf, key, sizeof(keybuf));
        av_strlcpy(valbuf, val, sizeof(valbuf));
        ds2_trim_string(keybuf, DS2_META_KEY_SIZE);
        ds2_trim_string(valbuf, DS2_META_VAL_SIZE);
        if (!keybuf[0])
            break;

        if (!av_strcasecmp(keybuf, "Author")) {
            if (valbuf[0])
                ret = av_dict_set(&s->metadata, "author", valbuf, 0);
        } else if (!av_strcasecmp(keybuf, "Memo") ||
                   !av_strcasecmp(keybuf, "Comment")) {
            if (valbuf[0])
                ret = av_dict_set(&s->metadata, "comment", valbuf, 0);
        } else if (valbuf[0]) {
            ret = av_dict_set(&s->metadata, keybuf, valbuf, 0);
        }
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int ds2_read_metadata(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    uint8_t hdr[DS2_HEADER_SIZE];
    int ret;

    avio_seek(pb, 0, SEEK_SET);
    ret = avio_read(pb, hdr, sizeof(hdr));
    if (ret < sizeof(hdr))
        return ret < 0 ? ret : AVERROR_EOF;

    ret = ds2_set_metadata_copy(s, (const char *)(hdr + DS2_HEAD_OFFSET_ENCODER),
                                DS2_ENCODER_SIZE, "encoder");
    if (ret < 0)
        return ret;

    ret = ds2_set_metadata_time(s, hdr + DS2_HEAD_OFFSET_START_TIME,
                                "creation_time");
    if (ret < 0)
        av_log(s, AV_LOG_DEBUG, "DS2 start time invalid or missing\n");

    ret = ds2_set_metadata_time(s, hdr + DS2_HEAD_OFFSET_END_TIME, "date");
    if (ret < 0)
        av_log(s, AV_LOG_DEBUG, "DS2 end time invalid or missing\n");

    ret = ds2_set_metadata_copy(s, (const char *)(hdr + DS2_HEAD_OFFSET_COMMENT),
                                DS2_COMMENT_SIZE, "comment");
    if (ret < 0)
        return ret;

    return ds2_read_extended_metadata(s, hdr);
}

static int ds2_count_total_frames(AVFormatContext *s) {
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  int64_t size = avio_size(pb);
  int blocks, i, total = 0, ret;

  if (size < ctx->header_size)
    return AVERROR_INVALIDDATA;

  blocks = (size - ctx->header_size) / DS2_BLOCK_SIZE;
  for (i = 0; i < blocks; i++) {
    uint8_t fc;
    avio_seek(pb, ctx->header_size + (int64_t)i * DS2_BLOCK_SIZE + 2, SEEK_SET);
    ret = ds2_io_read(s, &fc, 1);
    if (ret < 1)
      return ret < 0 ? ret : AVERROR_EOF;
    total += fc;
  }

  return total;
}

static void ds2_read_annotations(DS2DemuxContext *ctx, const uint8_t *hdr,
                                   int hdr_size)
{
  int off;

  ctx->nb_annotations = 0;
  if (hdr_size < DS2_HEAD_ANNOTATIONS_END)
    return;

  for (off = DS2_HEAD_OFFSET_ANNOTATIONS;
       off + 8 <= DS2_HEAD_ANNOTATIONS_END &&
       ctx->nb_annotations < DS2_MAX_ANNOTATIONS;
       off += 8) {
    uint32_t start_ms = AV_RL32(hdr + off);
    uint32_t end_ms   = AV_RL32(hdr + off + 4);

    if (start_ms == 0xFFFFFFFFu)
      break;

    ctx->annotations[ctx->nb_annotations].first_frame =
        start_ms / DS2_MS_PER_QP_FRAME;
    ctx->annotations[ctx->nb_annotations].last_frame =
        end_ms / DS2_MS_PER_QP_FRAME;
    ctx->nb_annotations++;
  }
}

static int ds2_qp_is_annotation(const DS2DemuxContext *ctx, int abs_frame)
{
  int i;

  for (i = 0; i < ctx->nb_annotations; i++) {
    if (abs_frame >= ctx->annotations[i].first_frame &&
        abs_frame < ctx->annotations[i].last_frame)
      return 1;
  }
  return 0;
}

static int ds2_qp_frame_included(const DS2DemuxContext *ctx, int abs_frame)
{
  int ann = ds2_qp_is_annotation(ctx, abs_frame);

  switch (ctx->extract) {
  case DS2_EXTRACT_MAIN:
    return !ann;
  case DS2_EXTRACT_ANNOTATIONS:
    return ann;
  default:
    return 1;
  }
}

static int ds2_qp_count_output_frames(const DS2DemuxContext *ctx)
{
  int f, n = 0;

  for (f = 0; f < ctx->total_frames; f++) {
    if (ds2_qp_frame_included(ctx, f))
      n++;
  }
  return n;
}

static int ds2_qp_cont_size(const uint8_t *hdr)
{
  return FFMAX(0, 2 * hdr[1] + 2 * (hdr[0] >> 7) -
                   DS2_AUDIO_BLOCK_HEADER_SIZE);
}

/**
 * Fresh frames that can start in a block entered with entry_off payload
 * bytes already consumed (straddle tail of the previous block's last frame).
 * Caps recorders that over-declare the header frame count (Grundig).
 */
static int ds2_qp_entry_cap(int entry_off)
{
  return (DS2_BLOCK_PAYLOAD_SIZE - entry_off + DS2_QP_FRAME_SIZE - 1) /
         DS2_QP_FRAME_SIZE;
}

/**
 * Walk QP blocks simulating the block-quantized read used by the reference
 * players: per block, an inbound straddling frame is finished first, then up
 * to min(header frame count, geometric cap) fresh frames start back-to-back
 * at the current offset; the last fresh frame may straddle into the next
 * block; when a block's fresh frames end before the payload does, the tail
 * bytes are dead and the next block starts a new frame at payload offset 0.
 * The byte-1 anchor is not consulted during linear reads - NCH
 * selective-extraction cut files rely on this (verified sample-exact
 * against NCH decodes of such files).
 *
 * With target_frame >= 0, stops at the block in which that frame starts and
 * fills *out; otherwise walks to end of file and returns the total number of
 * frames.
 */
typedef struct DS2QPWalk {
  int64_t block_start;  /* file offset of the block holding target_frame */
  int entry_off;        /* payload bytes consumed by the inbound straddle */
  int frames_before;    /* frames starting in earlier blocks */
} DS2QPWalk;

static int ds2_qp_walk_blocks(AVFormatContext *s, int target_frame,
                              DS2QPWalk *out)
{
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  int64_t fsize = avio_size(pb);
  int64_t saved_pos = avio_tell(pb);
  int64_t bstart;
  int total = 0, entry = 0, ret;

  for (bstart = ctx->header_size;
       bstart + DS2_AUDIO_BLOCK_HEADER_SIZE <= fsize;
       bstart += DS2_BLOCK_SIZE) {
    uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];
    int fc, take, end;

    avio_seek(pb, bstart, SEEK_SET);
    ret = ds2_io_read(s, hdr, sizeof(hdr));
    if (ret < (int)sizeof(hdr)) {
      ret = ret < 0 ? ret : AVERROR_EOF;
      goto done;
    }
    fc   = hdr[2];
    take = FFMIN(fc, ds2_qp_entry_cap(entry));
    if (out && target_frame >= 0 && target_frame < total + take) {
      out->block_start   = bstart;
      out->entry_off     = entry;
      out->frames_before = total;
      ret = 0;
      goto done;
    }
    if (take > 0) {
      end   = entry + take * DS2_QP_FRAME_SIZE;
      entry = end > DS2_BLOCK_PAYLOAD_SIZE ? end - DS2_BLOCK_PAYLOAD_SIZE : 0;
    } else {
      entry = 0; /* pause/end block: any straddle tail ends inside it */
    }
    total += take;
  }
  ret = (out && target_frame >= 0) ? AVERROR_EOF : total;

done:
  avio_seek(pb, saved_pos, SEEK_SET);
  return ret;
}

static int ds2_qp_count_frames(AVFormatContext *s)
{
  return ds2_qp_walk_blocks(s, -1, NULL);
}

/*
 * Precompute the QP7 (format 7) record list. QP7 packs the payload as a
 * continuous stream of byte-aligned records; each record's second byte's top
 * bit selects a 12-byte "short" or 56-byte "long" record. Per block, the
 * byte-1 anchor gives the first record's stream offset (never rewinding), and
 * the header frame count is the number of records that start there. A frame
 * count of 0 is a pause: it forces a decoder reset before the next record.
 * The whole payload is read into ctx->qp7_raw and the records are described in
 * ctx->qp7_records (mirrors the dss-codec Rust reference demuxer).
 */
static int ds2_qp7_build_records(AVFormatContext *s)
{
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  int64_t fsize = avio_size(pb);
  int64_t saved = avio_tell(pb);
  const int payload = DS2_BLOCK_PAYLOAD_SIZE;
  int num_blocks, bi, cap, raw_read_pos = 0, reset_next = 0, seg_has = 0, ret = 0;

  if (fsize < ctx->header_size)
    return AVERROR_INVALIDDATA;
  num_blocks = (fsize - ctx->header_size) / DS2_BLOCK_SIZE;

  ctx->qp7_raw_len = num_blocks * payload;
  ctx->qp7_raw = av_malloc(FFMAX(ctx->qp7_raw_len, 1));
  if (!ctx->qp7_raw)
    return AVERROR(ENOMEM);

  for (bi = 0; bi < num_blocks; bi++) {
    avio_seek(pb, ctx->header_size + (int64_t)bi * DS2_BLOCK_SIZE +
                      DS2_AUDIO_BLOCK_HEADER_SIZE, SEEK_SET);
    if (ds2_io_read(s, ctx->qp7_raw + bi * payload, payload) < payload) {
      ret = AVERROR_EOF;
      goto done;
    }
  }

  cap = ctx->qp7_raw_len / DS2_QP7_SHORT_SIZE + 1;
  ctx->qp7_records = av_malloc_array(cap, sizeof(*ctx->qp7_records));
  if (!ctx->qp7_records) {
    ret = AVERROR(ENOMEM);
    goto done;
  }
  ctx->qp7_nb_records = 0;

  for (bi = 0; bi < num_blocks; bi++) {
    int fc, payload_off, frames_raw_start, f;
    uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];

    avio_seek(pb, ctx->header_size + (int64_t)bi * DS2_BLOCK_SIZE, SEEK_SET);
    if (ds2_io_read(s, hdr, sizeof(hdr)) < (int)sizeof(hdr)) {
      ret = AVERROR_EOF;
      goto done;
    }
    fc = hdr[2];

    if (fc == 0) {
      int zero_end = (bi + 1) * payload;

      if (seg_has) {
        reset_next = 1;
        seg_has = 0;
      }
      if (zero_end > raw_read_pos)
        raw_read_pos = zero_end;
      continue;
    }

    payload_off = FFMAX(0, hdr[1] * 2 - DS2_AUDIO_BLOCK_HEADER_SIZE);
    frames_raw_start = bi * payload + payload_off;
    if (frames_raw_start > raw_read_pos)
      raw_read_pos = frames_raw_start;

    for (f = 0; f < fc; f++) {
      int size;

      if (raw_read_pos + 2 > ctx->qp7_raw_len)
        goto finish; /* trailing partial record: stop (lenient) */
      size = (ctx->qp7_raw[raw_read_pos + 1] & 0x80) ? DS2_QP7_LONG_SIZE
                                                     : DS2_QP7_SHORT_SIZE;
      if (raw_read_pos + size > ctx->qp7_raw_len)
        goto finish;

      ctx->qp7_records[ctx->qp7_nb_records].off   = raw_read_pos;
      ctx->qp7_records[ctx->qp7_nb_records].size  = size;
      ctx->qp7_records[ctx->qp7_nb_records].reset = reset_next;
      ctx->qp7_nb_records++;
      reset_next = 0;
      seg_has = 1;
      raw_read_pos += size;
    }
  }

finish:
  avio_seek(pb, saved, SEEK_SET);
  return 0;

done:
  avio_seek(pb, saved, SEEK_SET);
  return ret;
}

static int ds2_find_next_nonempty_swap(AVFormatContext *s, int block_idx) {
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  int64_t fsize = avio_size(pb);
  int64_t pos = avio_tell(pb);
  int bi;

  for (bi = block_idx + 1; ; bi++) {
    int64_t bstart = ctx->header_size + (int64_t)bi * DS2_BLOCK_SIZE;
    uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];
    int ret;

    if (bstart + DS2_AUDIO_BLOCK_HEADER_SIZE > fsize)
      break;

    avio_seek(pb, bstart, SEEK_SET);
    ret = ds2_io_read(s, hdr, sizeof(hdr));
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

static int ds2_load_block(AVFormatContext *s, int align_check) {
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];
  int64_t block_pos;
  int ret, frame_count, cont_size, block_idx;

  if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
    return AVERROR_EOF;

  block_pos = avio_tell(pb);
  block_idx = (block_pos - ctx->header_size) / DS2_BLOCK_SIZE;

  ret = ds2_io_read(s, hdr, sizeof(hdr));
  if (ret < sizeof(hdr))
    return ret < 0 ? ret : AVERROR_EOF;

  frame_count = hdr[2];
  cont_size   = ds2_qp_cont_size(hdr);

  if (frame_count == 0) {
    if (cont_size > 0)
      avio_skip(pb, cont_size);
    avio_skip(pb, DS2_BLOCK_PAYLOAD_SIZE - cont_size);
    ctx->counter = 0;
    ctx->swap_reset_pending = 1;
    ctx->next_blk_swap = ds2_find_next_nonempty_swap(s, block_idx);
  } else {
    if (ctx->swap_reset_pending) {
      ctx->swap = ctx->next_blk_swap;
      ctx->ds2_sp_swap_byte = -1;
      ctx->swap_reset_pending = 0;
    } else {
      ctx->counter = DS2_BLOCK_PAYLOAD_SIZE;
    }
  }

  return 0;
}

/**
 * Position the stream at the payload of the next block that declares fresh
 * frames, skipping dead tail bytes and whole pause blocks (frame count 0).
 * Sets counter and qp_blk_frames_left.
 */
static int ds2_qp_enter_next_block(AVFormatContext *s)
{
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  int64_t fsize = avio_size(pb);

  for (;;) {
    int64_t pos = avio_tell(pb);
    int64_t rel = pos - ctx->header_size;
    int64_t bstart;
    uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];
    int ret, fc;

    if (rel < 0)
      rel = 0;
    bstart = ctx->header_size +
             ((rel + DS2_BLOCK_SIZE - 1) / DS2_BLOCK_SIZE) * DS2_BLOCK_SIZE;
    if (bstart + DS2_AUDIO_BLOCK_HEADER_SIZE > fsize)
      return AVERROR_EOF;

    avio_seek(pb, bstart, SEEK_SET);
    ret = ds2_io_read(s, hdr, sizeof(hdr));
    if (ret < (int)sizeof(hdr))
      return ret < 0 ? ret : AVERROR_EOF;

    fc = hdr[2];
    if (fc > 0) {
      ctx->qp_blk_frames_left = FFMIN(fc, ds2_qp_entry_cap(0));
      ctx->counter = DS2_BLOCK_PAYLOAD_SIZE;
      return 0;
    }

    /* Pause/segment marker: skip the whole block, reset the decoder at the
     * next emitted frame. */
    avio_skip(pb, DS2_BLOCK_PAYLOAD_SIZE);
    ctx->qp_reset_next = 1;
  }
}

/**
 * Read the next 56-byte QP frame from the block-quantized stream into dst.
 * A frame that runs past the current payload straddles into the next block,
 * whose header is consumed in passing and whose (capped) frame count
 * becomes current.
 */
static int ds2_qp_next_frame_bytes(AVFormatContext *s, uint8_t *dst)
{
  DS2DemuxContext *ctx = s->priv_data;
  int ret, offset = 0;

  if (ctx->qp_blk_frames_left <= 0) {
    ret = ds2_qp_enter_next_block(s);
    if (ret < 0)
      return ret;
  }
  ctx->qp_blk_frames_left--;

  while (offset < DS2_QP_FRAME_SIZE) {
    int to_read;

    if (ctx->counter == 0) {
      uint8_t hdr[DS2_AUDIO_BLOCK_HEADER_SIZE];
      int rem = DS2_QP_FRAME_SIZE - offset;

      ret = ds2_io_read(s, hdr, sizeof(hdr));
      if (ret < (int)sizeof(hdr))
        return ret < 0 ? ret : AVERROR_EOF;
      ctx->qp_blk_frames_left = FFMIN(hdr[2], ds2_qp_entry_cap(rem));
      ctx->counter = DS2_BLOCK_PAYLOAD_SIZE;
    }

    to_read = FFMIN(DS2_QP_FRAME_SIZE - offset, ctx->counter);
    ret = ds2_io_read(s, dst + offset, to_read);
    if (ret < to_read)
      return ret < 0 ? ret : AVERROR_EOF;

    offset       += to_read;
    ctx->counter -= to_read;
  }

  return 0;
}

/* Heuristic check for a valid DS2 audio block header (byte 0 = 0x0f, the two
 * 0xff markers, a nonzero frame count, and a known format type). */
static int ds2_is_audio_block_header(const uint8_t *h)
{
  int fmt = h[4];

  return h[0] == 0x0f && h[3] == 0xff && h[5] == 0xff && h[2] > 0 &&
         (fmt == 0 || fmt == 1 || fmt == 2 || fmt == 3 || fmt == 6 || fmt == 7);
}

/*
 * Locate the first audio block in a Grundig format-7 ("\x07ds2") file. Its
 * header size is not byte0*512: scan the 512-byte grid for the first run of at
 * least four consecutive valid audio-block headers (mirrors the dss-codec
 * reference detect_ds2_audio_start); fall back to the best partial run, or
 * 0x1000. Only used for byte0 == 0x07; other layouts use byte0*512.
 */
static int64_t ds2_detect_audio_start(AVFormatContext *s)
{
  AVIOContext *pb = s->pb;
  int64_t fsize = avio_size(pb);
  int64_t scan_end = FFMIN(fsize - DS2_AUDIO_BLOCK_HEADER_SIZE, 0x10000);
  int64_t off, best_start = 0x1000;
  int best_score = 0;

  for (off = DS2_BLOCK_SIZE; off <= scan_end; off += DS2_BLOCK_SIZE) {
    uint8_t h[DS2_AUDIO_BLOCK_HEADER_SIZE];
    int valid = 0, i;

    avio_seek(pb, off, SEEK_SET);
    if (ds2_io_read(s, h, sizeof(h)) < (int)sizeof(h))
      break;
    if (!ds2_is_audio_block_header(h))
      continue;

    for (i = 0; i < 16; i++) {
      int64_t bs = off + (int64_t)i * DS2_BLOCK_SIZE;
      uint8_t bh[DS2_AUDIO_BLOCK_HEADER_SIZE];

      if (bs + DS2_AUDIO_BLOCK_HEADER_SIZE > fsize)
        break;
      avio_seek(pb, bs, SEEK_SET);
      if (ds2_io_read(s, bh, sizeof(bh)) < (int)sizeof(bh))
        break;
      if (!ds2_is_audio_block_header(bh))
        break;
      valid++;
    }

    if (valid >= 4)
      return off;
    if (valid > best_score) {
      best_score = valid;
      best_start = off;
    }
  }

  return best_start;
}

static int ds2_read_header(AVFormatContext *s) {
  DS2DemuxContext *ctx = s->priv_data;
  AVIOContext *pb = s->pb;
  AVStream *st;
  uint8_t block_header[DS2_AUDIO_BLOCK_HEADER_SIZE];
  uint8_t file_magic[4];
  int ret, frame_count, cont_size, blk_swap, samples_per_frame;
  int64_t ret64;

  avio_seek(pb, 0, SEEK_SET);
  ret = avio_read(pb, file_magic, sizeof(file_magic));
  if (ret < sizeof(file_magic))
    return ret < 0 ? ret : AVERROR_EOF;

  ctx->file_magic_byte = file_magic[0];

  /* Byte 0 is the header size in 512-byte blocks. Olympus DS2 uses 3 (0x600);
   * Grundig/Philips 6 recorders use 6 (0xc00), reserving the extra blocks for
   * GR___ device-id records before the audio. Magic 0x01 annotation slices
   * keep the standard 0x600 layout (byte 0 is a type tag there, not a size).
   * Encrypted files (magic "\x03enc") are byte 0 == 3 -> 0x600 as well. The
   * format-7 magic "\x07ds2" is a type marker, not a block count, so its audio
   * start is found by scanning (mirrors the dss-codec reference). */
  if (ctx->file_magic_byte == 0x01)
    ctx->header_size = DS2_HEADER_SIZE;
  else if (ctx->file_magic_byte == 0x07)
    ctx->header_size = ds2_detect_audio_start(s);
  else
    ctx->header_size = ctx->file_magic_byte * DS2_BLOCK_SIZE;

  if (AV_RL32(file_magic) == DS2_ENCRYPTED_MAGIC) {
    uint8_t hdr[DS2_HEADER_SIZE];
    DS2DecryptDescriptor desc;
    int pwd_len;

    if (!ctx->password || !ctx->password[0]) {
      av_log(s, AV_LOG_ERROR,
             "Encrypted DS2 file requires the -password option\n");
      return AVERROR(EINVAL);
    }

    avio_seek(pb, 0, SEEK_SET);
    ret = avio_read(pb, hdr, sizeof(hdr));
    if (ret < sizeof(hdr))
      return ret < 0 ? ret : AVERROR_EOF;

    ret = ds2_parse_decrypt_descriptor(hdr, sizeof(hdr), &desc);
    if (ret < 0) {
      av_log(s, AV_LOG_ERROR, "Invalid DS2 encryption descriptor\n");
      return ret;
    }

    pwd_len = strlen(ctx->password);
    if (pwd_len > DS2_MAX_PASSWORD_BYTES) {
      av_log(s, AV_LOG_ERROR,
             "DS2 password longer than %d bytes is not supported\n",
             DS2_MAX_PASSWORD_BYTES);
      return AVERROR(EINVAL);
    }

    ret = ds2_decrypt_init(&ctx->decrypt_state, &desc,
                           (const uint8_t *)ctx->password, pwd_len);
    if (ret < 0) {
      av_log(s, AV_LOG_ERROR, "DS2 password rejected\n");
      return ret;
    }

    ctx->encrypted = 1;
    ctx->key_mode  = desc.key_mode;
    ds2_invalidate_block_cache(ctx);
    av_dict_set(&s->metadata, "encryption", "1", 0);
  }

  st = avformat_new_stream(s, NULL);
  if (!st)
    return AVERROR(ENOMEM);

  ret = ds2_read_metadata(s);
  if (ret < 0)
    av_log(s, AV_LOG_WARNING, "Failed to read DS2 metadata\n");

  ret = ds2_count_total_frames(s);
  if (ret < 0)
    return ret;
  ctx->total_frames = ret;

  if ((ret64 = avio_seek(pb, ctx->header_size, SEEK_SET)) < 0)
    return (int)ret64;

  ret = ds2_io_read(s, block_header, DS2_AUDIO_BLOCK_HEADER_SIZE);
  if (ret < DS2_AUDIO_BLOCK_HEADER_SIZE)
    return ret < 0 ? ret : AVERROR_EOF;

  ctx->format_type = block_header[4];
  ctx->is_qp7      = ctx->format_type == DS2_FORMAT_QP7;

  st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
  st->codecpar->codec_id = AV_CODEC_ID_DS2;
  st->codecpar->codec_tag = ctx->format_type;
  st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;

  if (ctx->format_type < DS2_FORMAT_QP) {
    st->codecpar->sample_rate = DS2_SP_SAMPLE_RATE;
    samples_per_frame = DS2_SP_SAMPLES_PER_FRAME;
    if (ctx->format_type != DS2_FORMAT_SP)
      av_log(s, AV_LOG_WARNING, "DS2 SP variant format type %d\n",
             ctx->format_type);
  } else {
    st->codecpar->sample_rate = DS2_QP_SAMPLE_RATE;
    samples_per_frame = DS2_QP_SAMPLES_PER_FRAME;
    if (ctx->format_type != DS2_FORMAT_QP)
      av_log(s, AV_LOG_WARNING, "DS2 QP variant format type %d\n",
             ctx->format_type);
    ctx->format_type = DS2_FORMAT_QP;
    {
      uint8_t file_hdr[DS2_HEADER_SIZE];
      int out_frames;

      avio_seek(pb, 0, SEEK_SET);
      ret = ds2_io_read(s, file_hdr, sizeof(file_hdr));
      if (ret < (int)sizeof(file_hdr))
        return ret < 0 ? ret : AVERROR_EOF;

      ds2_read_annotations(ctx, file_hdr, sizeof(file_hdr));
      if (ctx->is_qp7) {
        ret = ds2_qp7_build_records(s);
        if (ret < 0)
          return ret;
        ctx->total_frames = ctx->qp7_nb_records;
        ctx->qp7_rec_idx  = 0;
      } else {
        ret = ds2_qp_count_frames(s);
        if (ret < 0)
          return ret;
        ctx->total_frames = ret;
      }
      ctx->abs_frame          = 0;
      ctx->qp_blk_frames_left = 0;
      ctx->qp_reset_next      = 0;

      out_frames = ctx->extract == DS2_EXTRACT_ALL ? ctx->total_frames
                                                   : ds2_qp_count_output_frames(ctx);
      if (out_frames > 0 && ctx->extract != DS2_EXTRACT_ALL) {
        int64_t nb_samples = (int64_t)out_frames * samples_per_frame;
        int64_t file_size  = avio_size(pb);

        st->duration = nb_samples;
        s->duration  = av_rescale_q(nb_samples,
                                    (AVRational){1, st->codecpar->sample_rate},
                                    AV_TIME_BASE_Q);
        if (file_size > ctx->header_size && s->duration > 0)
          s->bit_rate = (file_size - ctx->header_size) * 8LL * AV_TIME_BASE /
                        s->duration;
        else
          s->bit_rate = 8LL * DS2_QP_FRAME_SIZE * st->codecpar->sample_rate /
                        samples_per_frame;
        st->codecpar->bit_rate = (int)s->bit_rate;
      }

      /* QP6 reads the file lazily from the first block; QP7 emits from its
       * precomputed record list, so no stream positioning is needed. */
      if (!ctx->is_qp7) {
        if ((ret64 = avio_seek(pb, ctx->header_size, SEEK_SET)) < 0)
          return (int)ret64;
        ctx->counter = 0;
      }
    }
  }

  avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
  st->start_time = 0;
  if (ctx->total_frames > 0 &&
      !(ctx->format_type == DS2_FORMAT_QP && ctx->extract != DS2_EXTRACT_ALL)) {
    int64_t nb_samples = (int64_t)ctx->total_frames * samples_per_frame;
    int frame_bytes = (ctx->format_type == DS2_FORMAT_SP) ? DS2_SP_FRAME_SIZE
                                                          : DS2_QP_FRAME_SIZE;
    int64_t file_size = avio_size(pb);

    st->duration = nb_samples;
    s->duration  = av_rescale_q(nb_samples, (AVRational){1, st->codecpar->sample_rate},
                                AV_TIME_BASE_Q);

    if (file_size > ctx->header_size && s->duration > 0)
      s->bit_rate = (file_size - ctx->header_size) * 8LL * AV_TIME_BASE / s->duration;
    else
      s->bit_rate = 8LL * frame_bytes * st->codecpar->sample_rate / samples_per_frame;

    st->codecpar->bit_rate = (int)s->bit_rate;
  }

  blk_swap    = block_header[0] >> 7;
  frame_count = block_header[2];
  cont_size   = ds2_qp_cont_size(block_header);

  ctx->swap = blk_swap;
  ctx->ds2_sp_swap_byte = -1;
  ctx->swap_reset_pending = 0;
  ctx->frames_read = 0;

  if (ctx->format_type != DS2_FORMAT_QP) {
    if (frame_count == 0)
      ctx->counter = cont_size;
    else
      ctx->counter = DS2_BLOCK_PAYLOAD_SIZE;
  }

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

  while (ctx->counter == 0) {
    if (ctx->total_frames > 0 && ctx->frames_read >= ctx->total_frames)
      return AVERROR_EOF;
    ret = ds2_load_block(s, 1);
    if (ret < 0)
      return ret;
    if (ctx->counter == 0 && avio_feof(s->pb))
      return AVERROR_EOF;
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
    ret = ds2_io_read(s, pkt->data + buff_offset, ctx->counter);
    if (ret < ctx->counter)
      goto error_eof;

    offset = ctx->counter;
    ret = ds2_load_block(s, 0);
    if (ret < 0)
      goto error_eof;
  }
  ctx->counter -= read_size;

  ret = ds2_io_read(s, pkt->data + offset + buff_offset, read_size - offset);
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

static int ds2_qp_read_packet(AVFormatContext *s, AVPacket *pkt)
{
  DS2DemuxContext *ctx = s->priv_data;
  int ret;

  for (;;) {
    int64_t pos;

    if (ctx->total_frames > 0 && ctx->abs_frame >= ctx->total_frames)
      return AVERROR_EOF;

    /* Enter the next block up front (skipping dead tail bytes and pause
     * blocks) so pkt->pos records the frame's true start offset. */
    if (ctx->qp_blk_frames_left <= 0) {
      ret = ds2_qp_enter_next_block(s);
      if (ret < 0)
        return ret;
    }
    pos = avio_tell(s->pb);

    ret = av_new_packet(pkt, DS2_QP_FRAME_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);
    if (ret < 0)
      return ret;
    pkt->size         = DS2_QP_FRAME_SIZE;
    pkt->duration     = DS2_QP_SAMPLES_PER_FRAME;
    pkt->pos          = pos;
    pkt->stream_index = 0;
    pkt->flags        = 0;

    ret = ds2_qp_next_frame_bytes(s, pkt->data);
    if (ret < 0) {
      av_packet_unref(pkt);
      return ret;
    }

    if (ds2_qp_frame_included(ctx, ctx->abs_frame)) {
      if (ctx->qp_reset_next) {
        pkt->flags |= AV_PKT_FLAG_CORRUPT;
        ctx->qp_reset_next = 0;
      }
      ctx->abs_frame++;
      ctx->frames_read++;
      return 0;
    }

    av_packet_unref(pkt);
    ctx->abs_frame++;
  }
}

static int ds2_qp7_read_packet(AVFormatContext *s, AVPacket *pkt)
{
  DS2DemuxContext *ctx = s->priv_data;
  DS2Qp7Record *rec;
  int bi, within, ret;

  if (ctx->qp7_rec_idx >= ctx->qp7_nb_records)
    return AVERROR_EOF;

  rec = &ctx->qp7_records[ctx->qp7_rec_idx];
  ret = av_new_packet(pkt, rec->size);
  if (ret < 0)
    return ret;
  memcpy(pkt->data, ctx->qp7_raw + rec->off, rec->size);

  bi     = rec->off / DS2_BLOCK_PAYLOAD_SIZE;
  within = rec->off % DS2_BLOCK_PAYLOAD_SIZE;
  pkt->pos          = ctx->header_size + (int64_t)bi * DS2_BLOCK_SIZE +
                      DS2_AUDIO_BLOCK_HEADER_SIZE + within;
  pkt->duration     = DS2_QP_SAMPLES_PER_FRAME;
  pkt->stream_index = 0;
  pkt->flags        = rec->reset ? AV_PKT_FLAG_CORRUPT : 0;

  ctx->qp7_rec_idx++;
  return 0;
}

static int ds2_read_packet(AVFormatContext *s, AVPacket *pkt) {
  DS2DemuxContext *ctx = s->priv_data;

  if (ctx->format_type == DS2_FORMAT_SP)
    return ds2_sp_read_packet(s, pkt);
  else if (ctx->is_qp7)
    return ds2_qp7_read_packet(s, pkt);
  else
    return ds2_qp_read_packet(s, pkt);
}

static int ds2_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags) {
  DS2DemuxContext *ctx = s->priv_data;
  int64_t ret, seekto;
  uint8_t header[DS2_AUDIO_BLOCK_HEADER_SIZE];
  int offset, blk_swap;

  if (ctx->is_qp7) {
    /* One record == one output frame; seek to the record index. */
    int target = timestamp / DS2_QP_SAMPLES_PER_FRAME;

    if (target < 0)
      target = 0;
    if (ctx->qp7_nb_records > 0 && target >= ctx->qp7_nb_records)
      target = ctx->qp7_nb_records - 1;
    ctx->qp7_rec_idx = target;
    return 0;
  }

  if (ctx->format_type == DS2_FORMAT_QP) {
    DS2QPWalk walk;
    int target = timestamp / DS2_QP_SAMPLES_PER_FRAME;
    int i, skip, err;
    uint8_t scratch[DS2_QP_FRAME_SIZE];

    if (target < 0)
      target = 0;
    if (ctx->total_frames > 0 && target >= ctx->total_frames)
      target = ctx->total_frames - 1;

    err = ds2_qp_walk_blocks(s, target, &walk);
    if (err < 0)
      return err;

    ds2_invalidate_block_cache(ctx);
    avio_seek(s->pb, walk.block_start, SEEK_SET);
    err = ds2_io_read(s, header, DS2_AUDIO_BLOCK_HEADER_SIZE);
    if (err < DS2_AUDIO_BLOCK_HEADER_SIZE)
      return err < 0 ? err : AVERROR_EOF;

    avio_skip(s->pb, walk.entry_off);
    ctx->counter = DS2_BLOCK_PAYLOAD_SIZE - walk.entry_off;
    ctx->qp_blk_frames_left = FFMIN(header[2],
                                    ds2_qp_entry_cap(walk.entry_off));
    ctx->qp_reset_next = 0;
    ctx->abs_frame     = walk.frames_before;
    ctx->frames_read   = walk.frames_before;

    skip = target - walk.frames_before;
    for (i = 0; i < skip; i++) {
      err = ds2_qp_next_frame_bytes(s, scratch);
      if (err < 0)
        return err;
      ctx->abs_frame++;
      ctx->frames_read++;
    }
    return 0;
  }

  /* SP: 42-byte frames (avg 41 bytes with swap interleaving) */
  seekto = timestamp / DS2_SP_SAMPLES_PER_FRAME * 41 /
           DS2_BLOCK_PAYLOAD_SIZE * DS2_BLOCK_SIZE;

  if (seekto < 0)
    seekto = 0;

  seekto += ctx->header_size;

  ret = avio_seek(s->pb, seekto, SEEK_SET);
  if (ret < 0)
    return ret;

  ds2_invalidate_block_cache(ctx);

  ret = ds2_io_read(s, header, DS2_AUDIO_BLOCK_HEADER_SIZE);
  if (ret < DS2_AUDIO_BLOCK_HEADER_SIZE)
    return ret < 0 ? ret : AVERROR_EOF;

  blk_swap = header[0] >> 7;

  ctx->frames_read = timestamp / DS2_SP_SAMPLES_PER_FRAME;
  ctx->swap_reset_pending = 0;

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

  return 0;
}

static const AVOption ds2_options[] = {
    { "password", "Decryption password for encrypted DS2 (max 16 bytes)",
      OFFSET(password), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, DEC },
    { "extract", "select QP recording content to demux (ignored for SP)",
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

static int ds2_read_close(AVFormatContext *s)
{
  DS2DemuxContext *ctx = s->priv_data;

  av_freep(&ctx->qp7_raw);
  av_freep(&ctx->qp7_records);
  return 0;
}

const FFInputFormat ff_ds2_demuxer = {
    .p.name         = "ds2",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Digital Speech Standard Pro (DS2)"),
    .p.extensions   = "ds2",
    .p.priv_class   = &ds2_demuxer_class,
    .priv_data_size = sizeof(DS2DemuxContext),
    .read_probe     = ds2_probe,
    .read_header    = ds2_read_header,
    .read_packet    = ds2_read_packet,
    .read_seek      = ds2_read_seek,
    .read_close     = ds2_read_close,
};
