/*
 * Sony Memory Stick Voice (MSV) and Digital Voice File (DVF) demuxer
 * Container magic: "MS_VOICE" (see Tyler Thorsted's Sony IC recorder survey).
 * Audio codecs are selected from the driver SPI name and the quality byte at
 * file offset 0x3D (also reflected in the 32-bit tag at 0x3C).
 *
 * Copyright (c) 2026 Patrick Domack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"

#define MSV_VOICE_MAGIC     MKTAG('M', 'S', '_', 'V')
#define MSV_HEADER_MIN      0x100
#define MSV_SECTOR_SIZE     0x400
#define MSV_INDEX_SIZE      10

#define MSV_HEAD_OFFSET_AUTHOR  0x10
#define MSV_AUTHOR_SIZE         16
#define MSV_HEAD_OFFSET_TIME    0x34
#define MSV_ADPCM_SAMPLES_PER_FRAME 128
#define MSV_HEAD_OFFSET_SPI     0x20
#define MSV_SPI_SIZE            16
#define MSV_HEAD_OFFSET_TAG     0x3c
#define MSV_HEAD_OFFSET_QUALITY 0x3d
#define MSV_HEAD_OFFSET_RATE    0x42
#define MSV_HEAD_OFFSET_PERIOD  0x48
#define MSV_CHUNK_TABLE         0x50
#define MSV_DATA_BASE           0x200
enum MSVCodecType {
    MSV_CODEC_UNKNOWN = 0,
    MSV_CODEC_ADPCM,
    MSV_CODEC_LPEC,
    MSV_CODEC_LCST,
    MSV_CODEC_TRC,
};

typedef struct MSVDemuxContext {
    enum MSVCodecType codec;
    int quality;
    int sample_rate;
    int channels;
    int64_t data_offset;
    int64_t data_size;
    int adpcm_frame_size;
    int adpcm_mode;         /* bits/sample: 2 (LP), 3 (SP), 4 (HQ) */
    uint16_t frame_sizes[4];
} MSVDemuxContext;

/* MSV ADPCM samples per 48-byte frame, by bits/sample. */
static int msv_adpcm_spf(int mode)
{
    switch (mode) {
    case 2:  return 192;
    case 4:  return 96;
    case 3:
    default: return 128;
    }
}

/*
 * Map the quality byte (offset 0x3d) to ADPCM bits/sample.
 * Only LP (2-bit) and SP (3-bit) are supported; a 4-bit (HQ) mode is not
 * implemented yet (no HQ sample available), so unknown qualities use SP.
*/
static int msv_adpcm_mode_from_quality(int quality)
{
    switch (quality) {
    case 0x09: return 2;   /* LP */
    case 0x05:
    default:   return 3;   /* SP */
    }
}

static int msv_probe(const AVProbeData *p)
{
    if (p->buf_size < 8)
        return 0;
    if (AV_RL32(p->buf) != MSV_VOICE_MAGIC)
        return 0;
    if (memcmp(p->buf, "MS_VOICE", 8))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static enum MSVCodecType msv_guess_codec(const char *spi, int quality)
{
    if (strstr(spi, "apcm"))
        return MSV_CODEC_ADPCM;
    if (strstr(spi, "trc"))
        return MSV_CODEC_TRC;
    if (strstr(spi, "lcst"))
        return MSV_CODEC_LCST;
    if (strstr(spi, "lpec"))
        return MSV_CODEC_LPEC;

    switch (quality) {
    case 0x05:
    case 0x09:
        return MSV_CODEC_ADPCM;
    case 0x30:
    case 0x35:
    case 0x37:
        return MSV_CODEC_TRC;
    case 0x24:
    case 0x20:
    case 0x21:
    case 0x22:
    case 0x28:
    case 0x29:
        return MSV_CODEC_LCST;
    case 0x15:
    case 0x19:
    case 0x2a:
    case 0x2c:
    case 0x4a:
    case 0x4c:
        return MSV_CODEC_LPEC;
    default:
        return MSV_CODEC_UNKNOWN;
    }
}

static void msv_trim_string(char *s, int size)
{
    int i;

    for (i = 0; i < size && s[i]; i++) {
        if ((unsigned char)s[i] < 0x20) {
            s[i] = 0;
            break;
        }
    }
    while (i > 0 && ((unsigned char)s[i - 1] <= 0x20))
        s[--i] = 0;
}

static int msv_read_metadata_string(AVFormatContext *s, unsigned int offset,
                                    unsigned int size, const char *key)
{
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

    msv_trim_string(value, size);
    if (!value[0]) {
        av_free(value);
        return 0;
    }

    return av_dict_set(&s->metadata, key, value, AV_DICT_DONT_STRDUP_VAL);
}

static int msv_bcd_byte(uint8_t b)
{
    return (b >> 4) * 10 + (b & 0xf);
}

static int msv_read_metadata_date(AVFormatContext *s, unsigned int offset,
                                  const char *key)
{
    AVIOContext *pb = s->pb;
    char datetime[64];
    int year, month, day, hour, minute;
    uint8_t day_bcd;

    avio_seek(pb, offset, SEEK_SET);
    year   = avio_rb16(pb);
    month  = avio_r8(pb);
    day_bcd = avio_r8(pb);
    hour   = avio_r8(pb);
    minute = avio_r8(pb);

    if (year < 1900 || year > 2100 || month < 1 || month > 12)
        return AVERROR_INVALIDDATA;

    day = msv_bcd_byte(day_bcd);
    if (day < 1 || day > 31)
        return AVERROR_INVALIDDATA;

    snprintf(datetime, sizeof(datetime), "%.4d-%.2d-%.2dT%.2d:%.2d:00",
             year, month, day, hour, minute);
    return av_dict_set(&s->metadata, key, datetime, 0);
}

static int64_t msv_count_adpcm_samples(AVFormatContext *s, MSVDemuxContext *ctx)
{
    AVIOContext *pb = s->pb;
    int64_t end = ctx->data_offset + ctx->data_size;
    int64_t frames = 0;
    int fs = ctx->adpcm_frame_size;
    uint8_t *blk;
    int ret;

    if (fs <= 0)
        return 0;

    blk = av_malloc(fs);
    if (!blk)
        return AVERROR(ENOMEM);

    /* ADPCM payload is contiguous 48-byte frames (no per-sector index); every
 * frame produces output, so just count them.*/
    avio_seek(pb, ctx->data_offset, SEEK_SET);
    while (avio_tell(pb) + fs <= end) {
        ret = avio_read(pb, blk, fs);
        if (ret < fs)
            break;
        frames++;
    }
    av_free(blk);

    return frames * msv_adpcm_spf(ctx->adpcm_mode);
}

static int msv_payload_density(const uint8_t *buf, int len)
{
    int i, nz = 0;

    for (i = 0; i < len; i++) {
        if (buf[i] != 0 && buf[i] != 0xff)
            nz++;
    }
    return nz;
}

/*
 * Sector index at each 0x400 boundary.
 * Older DVF (e.g. ICD-U70) space-pads index fields: 20 0a 20 0a 04 20
 * instead of 00 0a 00 0a 04 00. Parse both layouts.
 */
static unsigned msv_index_u16(const uint8_t *p)
{
    /* Space-padded DVF indices use 0x20 only as a leading pad byte (20 0a...). */
    if (p[0] == 0x20)
        return p[1];
    if (p[0] == 0)
        return p[1];
    if (p[1] == 0)
        return p[0] << 8;
    return AV_RB16(p);
}

static int msv_sector_index_valid(const uint8_t *idx)
{
    unsigned o0 = msv_index_u16(idx);
    unsigned o1 = msv_index_u16(idx + 2);

    return o0 >= MSV_INDEX_SIZE && o0 < MSV_SECTOR_SIZE &&
           o1 >= MSV_INDEX_SIZE && o1 < MSV_SECTOR_SIZE;
}
static int64_t msv_find_data_offset(AVIOContext *pb, enum MSVCodecType codec)
{
    uint8_t buf[0x800];
    int64_t size, i, run, best_end = 0, pos;
    static const int candidates[] = { 0x400, 0x620 };

    size = avio_size(pb);
    if (size < MSV_HEADER_MIN)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, 0, SEEK_SET);
    if (avio_read(pb, buf, sizeof(buf)) < MSV_HEADER_MIN)
        return AVERROR_INVALIDDATA;

    if (codec != MSV_CODEC_ADPCM) {
        for (i = 0; i < FF_ARRAY_ELEMS(candidates); i++) {
            int off = candidates[i];
            if (off + MSV_INDEX_SIZE <= (int)sizeof(buf) &&
                msv_sector_index_valid(buf + off))
                return off;
        }
        /*
 * First 0x400-aligned sector with a valid index (e.g. BM/MX ).
 * Scan the file rather than the small probe buffer so sectors past the
 * first 0x800 bytes are reachable.
*/
        {
            uint8_t sidx[MSV_INDEX_SIZE];
            int64_t sec;

            for (sec = MSV_SECTOR_SIZE; sec + MSV_INDEX_SIZE <= size;
                 sec += MSV_SECTOR_SIZE) {
                avio_seek(pb, sec, SEEK_SET);
                if (avio_read(pb, sidx, MSV_INDEX_SIZE) < MSV_INDEX_SIZE)
                    break;
                if (msv_sector_index_valid(sidx))
                    return sec;
            }
        }
    }

    run = 0;
    for (i = 0x100; i < (int)sizeof(buf); i++) {
        if (buf[i] == 0xff)
            run++;
        else {
            if (run > 0x80)
                best_end = i;
            run = 0;
        }
    }

    pos = 0;
    for (i = 0x100; i < (int)sizeof(buf) - 80; i++) {
        int nz = msv_payload_density(buf + i, 80);

        if (nz >= 24 && nz > msv_payload_density(buf + pos, 80))
            pos = i;
    }
    if (pos > 0 && codec == MSV_CODEC_ADPCM) {
        pos = FFALIGN(pos, 16);
        if (pos < size)
            return pos;
    }

    pos = best_end > 0x200 ? best_end : 0x400;
    pos = FFALIGN(pos, 16);
    if (pos >= size)
        return AVERROR_INVALIDDATA;
    return pos;
}

/*
 * Locate the audio (type 4) chunk from the chunk table.  The audio data begins
 * at MSV_CHUNK_TABLE plus the accumulated size of every preceding chunk (a
 * logical offset that does not count the interleaved 10-byte sector indices).
 */
static int msv_chunk_data_offset(AVIOContext *pb, int64_t *offset)
{
    uint8_t ent[8];
    int64_t pc = MSV_CHUNK_TABLE;
    int64_t istack = 0;
    int i, typ;
    unsigned inc;

    for (i = 0; i < 16; i++) {
        avio_seek(pb, pc, SEEK_SET);
        if (avio_read(pb, ent, 8) < 8)
            return 0;
        typ = ent[0];
        inc = (ent[4] << 8) | ent[5] | (ent[6] << 8) | ent[7];
        if (typ == 4) {
            *offset = MSV_CHUNK_TABLE + istack;
            return 1;
        }
        istack += inc;
        if (typ == 0)
            break;
        pc += 8;
    }
    return 0;
}

/*
 * End of sector-based TRC audio: each 0x400 sector starts with a 10-byte
 * index whose BE16 field at +4 is the offset (within the sector) where valid
 * payload ends. Full sectors carry 0x400; the final data sector carries a
 * smaller value, and trailing sectors are padding with an invalid index.
 * TRC padding would decode as phantom pause frames, so the end bound matters
 * (unlike LPEC, whose reference decoder walks to EOF).
 */
static int64_t msv_sector_data_end(AVIOContext *pb, int64_t data_offset)
{
    int64_t fsize = avio_size(pb);
    int64_t sec;
    uint8_t idx[MSV_INDEX_SIZE];

    for (sec = data_offset; sec + MSV_INDEX_SIZE <= fsize; sec += MSV_SECTOR_SIZE) {
        unsigned end_off;

        avio_seek(pb, sec, SEEK_SET);
        if (avio_read(pb, idx, MSV_INDEX_SIZE) < MSV_INDEX_SIZE)
            break;
        if (!msv_sector_index_valid(idx))
            return sec;
        end_off = msv_index_u16(idx + 4);
        if (end_off >= MSV_INDEX_SIZE && end_off < MSV_SECTOR_SIZE)
            return sec + end_off;
    }
    return fsize;
}

/*
 * True end of sector-based audio (LPEC/LCST/TRC). Each 0x400 sector starts
 * with a 10-byte index whose BE16 field at +4 is the offset (within the
 * sector) where valid payload ends. Full sectors carry 0x400; the final data
 * sector carries a smaller value, and trailing sectors are padding with an
 * invalid index. Reading to EOF instead would decode that padding as audio
 * (harmless on long files, but several phantom frames on short clips).
*/
static void msv_set_frame_sizes(MSVDemuxContext *ctx)
{
    int fs;

    switch (ctx->codec) {
    case MSV_CODEC_LPEC:
        fs = 80;
        break;
    case MSV_CODEC_LCST:
        /* [3-byte MSV frame header][280-byte ATRAC3+ payload] = 283 bytes;
         * assembled across sector boundaries, then deframed in read_packet. */
        fs = 283;
        break;
    case MSV_CODEC_ADPCM:
        fs = ctx->adpcm_frame_size > 0 ? ctx->adpcm_frame_size : 48;
        break;
    case MSV_CODEC_TRC:
    default:
        fs = 0;
        break;
    }

    ctx->frame_sizes[0] = fs;
    ctx->frame_sizes[1] = fs;
    ctx->frame_sizes[2] = fs;
    ctx->frame_sizes[3] = fs;
}

/* TRC packet size from the first byte's 2/3-bit rate prefix. */
static int msv_trc_packet_size(uint8_t byte0)
{
    int top2 = byte0 >> 6;
    int rate = top2 != 3 ? top2 : ((byte0 & 0x20) ? 4 : 3);
    static const int sizes[] = { 11, 18, 24, 4, 2, 0 };

    if (rate < 0 || rate > 5)
        return 0;
    return sizes[rate];
}

/* TRC reference decoder output rate (WAV rate). */
static int msv_trc_output_rate(int quality)
{
    switch (quality) {
    case 0x30:
        return 16000;
    case 0x35:
    case 0x37:
        return 8000;
    default:
        return 0;
    }
}

/* the reference decoder output rate (WAV rate). */
static int msv_lpec_output_rate(int quality)
{
    switch (quality) {
    case 0x15:
    case 0x2a:
    case 0x4a:
        return 16000;
    case 0x19:
    case 0x2c:
    case 0x4c:
        return 8000;
    default:
        return 0;
    }
}

static int msv_guess_sample_rate(enum MSVCodecType type, int quality)
{
    switch (type) {
    case MSV_CODEC_ADPCM:
        return 11025;
    case MSV_CODEC_TRC:
        if (quality == 0x37)
            return 6000;
        if (quality == 0x35)
            return 7200;
        return 8000;
    case MSV_CODEC_LCST:
        return 44100;
    case MSV_CODEC_LPEC:
        return msv_lpec_output_rate(quality) ?: 8000;
    default:
        return 8000;
    }
}

static int msv_pick_codec_id(enum MSVCodecType type)
{
    switch (type) {
    case MSV_CODEC_ADPCM:
        return AV_CODEC_ID_MSV_ADPCM;
    case MSV_CODEC_TRC:
        return AV_CODEC_ID_TRC;
    case MSV_CODEC_LCST:
        /* LCST = Sony "AT-X" = ATRAC3+ (byte-identical tables) in an MSV
         * container; decode with FFmpeg's ATRAC3+ decoder. */
        return AV_CODEC_ID_ATRAC3P;
    case MSV_CODEC_LPEC:
        return AV_CODEC_ID_LPEC;
    default:
        return AV_CODEC_ID_NONE;
    }
}

static int msv_read_header(AVFormatContext *s)
{
    MSVDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st;
    char spi[MSV_SPI_SIZE + 1] = { 0 };
    uint8_t magic[8];
    int ret;
    int64_t pos, nb_samples = 0;

    avio_seek(pb, 0, SEEK_SET);
    ret = avio_read(pb, magic, sizeof(magic));
    if (ret < sizeof(magic))
        return ret < 0 ? ret : AVERROR_EOF;
    if (memcmp(magic, "MS_VOICE", 8))
        return AVERROR_INVALIDDATA;

    ret = msv_read_metadata_string(s, MSV_HEAD_OFFSET_AUTHOR,
                                 MSV_AUTHOR_SIZE, "author");
    if (ret < 0)
        av_log(s, AV_LOG_WARNING, "Failed to read author metadata\n");

    ret = msv_read_metadata_date(s, MSV_HEAD_OFFSET_TIME, "date");
    if (ret < 0)
        av_log(s, AV_LOG_WARNING, "Failed to read date metadata\n");

    avio_seek(pb, MSV_HEAD_OFFSET_SPI, SEEK_SET);
    ret = avio_read(pb, spi, MSV_SPI_SIZE);
    if (ret < MSV_SPI_SIZE)
        return ret < 0 ? ret : AVERROR_EOF;

    avio_seek(pb, MSV_HEAD_OFFSET_QUALITY, SEEK_SET);
    ctx->quality = avio_r8(pb);

    ctx->codec = msv_guess_codec(spi, ctx->quality);
    ctx->adpcm_mode = msv_adpcm_mode_from_quality(ctx->quality);
    if (ctx->codec == MSV_CODEC_UNKNOWN) {
        av_log(s, AV_LOG_WARNING,
               "Unknown MSV/DVF codec (spi=%s quality=0x%02x)\n",
               spi, ctx->quality);
    }

    if (ctx->codec == MSV_CODEC_LCST)
        ctx->channels = 2; /* AT-X/ATRAC3+ format 0x24 is stereo */
    else
        ctx->channels = 1;

    avio_seek(pb, MSV_HEAD_OFFSET_RATE, SEEK_SET);
    ctx->sample_rate = avio_rb16(pb);
    /* LCST/AT-X stores an internal rate (48234) in the header; the decoded
     * output is always 44100 Hz. */
    if (ctx->codec == MSV_CODEC_LCST)
        ctx->sample_rate = 44100;
    else if (ctx->sample_rate <= 0)
        ctx->sample_rate = msv_guess_sample_rate(ctx->codec, ctx->quality);
    else if (ctx->codec == MSV_CODEC_LPEC) {
        int out_rate = msv_lpec_output_rate(ctx->quality);

        if (out_rate > 0)
            ctx->sample_rate = out_rate;
    } else if (ctx->codec == MSV_CODEC_TRC) {
        int out_rate = msv_trc_output_rate(ctx->quality);

        if (out_rate > 0)
            ctx->sample_rate = out_rate;
    }

    avio_seek(pb, MSV_HEAD_OFFSET_PERIOD, SEEK_SET);
    ctx->adpcm_frame_size = avio_rb16(pb);
    /* MSV ADPCM uses 48-byte frames (the reference decoder); period 0x0010 on LP clips is not frame size */
    if (ctx->codec == MSV_CODEC_ADPCM)
        ctx->adpcm_frame_size = 48;
    else if (ctx->adpcm_frame_size < 16)
        ctx->adpcm_frame_size = 48;

    msv_set_frame_sizes(ctx);

    if (ctx->codec == MSV_CODEC_ADPCM) {
        /* ADPCM payload is one contiguous run of 48-byte frames from the fixed
 * 0x200 header to EOF; any trailing sector padding is dropped by the
 * padding check in read_packet.*/
        ctx->data_offset = MSV_DATA_BASE;
        ctx->data_size   = avio_size(pb) - MSV_DATA_BASE;
    } else if (ctx->codec == MSV_CODEC_TRC &&
               msv_chunk_data_offset(pb, &pos)) {
        /*
         * The chunk-table sum lands mid-frame (it ignores the interleaved
         * sector indices); the audio actually starts at the first data byte
         * of the sector containing that offset. Verified bit-exact against
         * the reference decoder. TRC sector geometry is absolute to the file.
         */
        int64_t base = (pos / MSV_SECTOR_SIZE) * MSV_SECTOR_SIZE;
        ctx->data_offset = base + MSV_INDEX_SIZE;
        ctx->data_size   = msv_sector_data_end(pb, base) - ctx->data_offset;
    } else {
        pos = msv_find_data_offset(pb, ctx->codec);
        if (pos < 0)
            return pos;
        ctx->data_offset = pos;
        /*
 * LPEC/LCST: the reference decoder walks packets to EOF. Sector index
 * end markers can sit above the last
 * 0xFF-padded frame(s) in the final 0x400 block; truncating there drops
 * tail frames (e.g. short 0x2a clips lose 2-3 packets vs the reference).
*/
        ctx->data_size   = avio_size(pb) - pos;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id    = msv_pick_codec_id(ctx->codec);
    st->codecpar->codec_tag   = ctx->quality;
    st->codecpar->sample_rate = ctx->sample_rate;
    if (ctx->channels == 2)
        st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
    else
        st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;

    if (st->codecpar->codec_id == AV_CODEC_ID_NONE) {
        avpriv_request_sample(s,
                              "MSV/DVF codec spi=%s quality=0x%02x",
                              spi, ctx->quality);
        return AVERROR_PATCHWELCOME;
    }

    if (ctx->codec == MSV_CODEC_ADPCM) {
        st->codecpar->block_align = ctx->adpcm_frame_size * ctx->channels;
        st->codecpar->bits_per_coded_sample = ctx->adpcm_mode;
    } else if (ctx->codec == MSV_CODEC_LCST) {
        /* ATRAC3+ decoder derives its layout from block_align (payload bytes
         * per frame, after the 3-byte MSV header is stripped in read_packet). */
        st->codecpar->block_align = 280;
    }

    msv_trim_string(spi, MSV_SPI_SIZE);
    if (spi[0])
        av_dict_set(&s->metadata, "encoder", spi, 0);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    st->start_time = 0;

    if (ctx->codec == MSV_CODEC_ADPCM) {
        nb_samples = msv_count_adpcm_samples(s, ctx);
        if (nb_samples < 0)
            return (int)nb_samples;
        if (nb_samples > 0) {
            int64_t file_size = avio_size(pb);

            st->duration = nb_samples;
            s->duration  = av_rescale_q(nb_samples,
                                          (AVRational){1, st->codecpar->sample_rate},
                                          AV_TIME_BASE_Q);
            if (file_size > ctx->data_offset && s->duration > 0)
                s->bit_rate = (file_size - ctx->data_offset) * 8LL * AV_TIME_BASE /
                              s->duration;
            else
                s->bit_rate = 8LL * ctx->adpcm_frame_size * st->codecpar->sample_rate /
                              MSV_ADPCM_SAMPLES_PER_FRAME;
            st->codecpar->bit_rate = (int)s->bit_rate;
        }
    }

    avio_seek(pb, ctx->data_offset, SEEK_SET);
    return 0;
}

static int msv_frame_size_from_byte(MSVDemuxContext *ctx, uint8_t b)
{
    int idx = b >> 6;

    if (ctx->frame_sizes[idx])
        return ctx->frame_sizes[idx];
    return ctx->frame_sizes[0];
}

static void msv_skip_sector_index(AVFormatContext *s, MSVDemuxContext *ctx)
{
    AVIOContext *pb = s->pb;

    /*
     * TRC data starts at an exact (non sector-aligned) chunk offset, so its
     * sector indices are at absolute file 0x400 boundaries.  LPEC/LCST keep
     * the original relative-to-data_offset geometry.
     */
    if (ctx->codec == MSV_CODEC_TRC) {
        int64_t tell = avio_tell(pb);
        int off = tell % MSV_SECTOR_SIZE;

        if (off < MSV_INDEX_SIZE)
            avio_seek(pb, (tell / MSV_SECTOR_SIZE) * MSV_SECTOR_SIZE +
                      MSV_INDEX_SIZE, SEEK_SET);
        return;
    }

    {
        int64_t pos = avio_tell(pb) - ctx->data_offset;
        int64_t sec = pos / MSV_SECTOR_SIZE;
        int off   = pos % MSV_SECTOR_SIZE;
        /*
         * LPEC/LCST: data_offset is the sector base (index at +0); skip it on
         * the first read too. MSV ADPCM chunk offset already lands past index.
         */
        int skip_at_start = ctx->codec != MSV_CODEC_ADPCM;

        if ((pos > 0 || skip_at_start) && off < MSV_INDEX_SIZE)
            avio_seek(pb, ctx->data_offset + sec * MSV_SECTOR_SIZE +
                      MSV_INDEX_SIZE, SEEK_SET);
    }
}

/*
 * LCST/AT-X (ATRAC3+) frames are stored as [3-byte MSV frame header][payload].
 * Header byte 2 selects an optional XOR descramble of the payload's first 8
 * bytes. Strip the header and descramble to recover a raw ATRAC3+
 * channel-unit frame.
 */
static const uint8_t msv_lcst_xor_key[64] = {
    0xa2,0x35,0x30,0x95,0x15,0x75,0xe7,0x43,0xc9,0x61,0x4a,0xeb,0xa2,0xa1,0x6e,0x19,
    0x90,0xb2,0xe9,0xd5,0x06,0x8b,0xea,0x6e,0xad,0x81,0x84,0x77,0x5b,0x68,0x45,0x0f,
    0x3f,0xfa,0x10,0x64,0x5e,0x7d,0x28,0x79,0x42,0xa1,0x8b,0x28,0xf0,0xa7,0x82,0x64,
    0x18,0x06,0x0e,0x10,0x00,0x00,0x00,0x00,0x2e,0x3f,0x41,0x56,0x43,0x43,0x6c,0x61,
};

static void msv_lcst_deframe(AVPacket *pkt)
{
    uint8_t *p = pkt->data;
    uint8_t sc;
    int i;

    if (pkt->size <= 3)
        return;
    sc = p[2];
    if ((sc >> 6) == 1) {
        unsigned lo   = sc & 0xf;
        unsigned base = ((sc >> 4) & 3) * 0x10;
        for (i = 0; i < 8 && 3 + i < pkt->size; i++)
            p[3 + i] ^= msv_lcst_xor_key[base + ((lo + i) & 0xf)];
    }
    memmove(p, p + 3, pkt->size - 3);
    pkt->size    -= 3;
    pkt->duration = 2048;   /* ATRAC3+ emits 2048 samples/channel per frame */
}

static int msv_read_raw_packet(AVFormatContext *s, AVPacket *pkt)
{
    MSVDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos, sec_end, size, ret, fsize, tell;
    uint8_t hdr;
    int use_sectors = ctx->codec == MSV_CODEC_LPEC ||
                      ctx->codec == MSV_CODEC_LCST ||
                      ctx->codec == MSV_CODEC_TRC;

    fsize = avio_size(pb);

    for (;;) {
        tell = avio_tell(pb);
        if (tell < ctx->data_offset || tell >= fsize)
            return AVERROR_EOF;
        /* TRC padding would decode as phantom pause frames; stop at the
         * sector-index end bound instead of walking to EOF. */
        if (ctx->codec == MSV_CODEC_TRC &&
            tell >= ctx->data_offset + ctx->data_size)
            return AVERROR_EOF;

        if (ctx->codec == MSV_CODEC_ADPCM) {
            int fs = ctx->adpcm_frame_size;
            uint8_t *blk;

            /* ADPCM payload is one contiguous run of 48-byte frames from
 * data_offset (no per-sector index). Every frame is fed to the
 * decoder, including all-0x00/0xff "quiet" frames, matching the
 * reference decoder's silent output for them.*/
            pos = avio_tell(pb);
            if (pos + fs > ctx->data_offset + ctx->data_size)
                return AVERROR_EOF;
            ret = av_new_packet(pkt, fs);
            if (ret < 0)
                return ret;
            blk = pkt->data;
            ret = avio_read(pb, blk, fs);
            if (ret < fs)
                return ret < 0 ? ret : AVERROR_EOF;
            return 0;
        }

        if (use_sectors)
            msv_skip_sector_index(s, ctx);
        tell = avio_tell(pb);
        /* TRC uses absolute file sector geometry; the others stay relative. */
        pos  = ctx->codec == MSV_CODEC_TRC ? tell : tell - ctx->data_offset;
        sec_end = use_sectors ? ((pos / MSV_SECTOR_SIZE) + 1) * MSV_SECTOR_SIZE
                              : ctx->data_size;

        if (ctx->codec == MSV_CODEC_TRC) {
            int64_t next_sec;

            ret = avio_read(pb, &hdr, 1);
            if (ret < 1)
                return ret < 0 ? ret : AVERROR_EOF;

            size = msv_trc_packet_size(hdr);
            if (size <= 0) {
                avio_seek(pb, -1, SEEK_CUR);
                return AVERROR_INVALIDDATA;
            }
            if (pos + size > sec_end) {
                int in_sec = sec_end - pos;

                ret = av_new_packet(pkt, size);
                if (ret < 0)
                    return ret;
                pkt->data[0] = hdr;
                ret = avio_read(pb, pkt->data + 1, in_sec - 1);
                if (ret < in_sec - 1) {
                    av_packet_unref(pkt);
                    return ret < 0 ? ret : AVERROR_EOF;
                }
                next_sec = (pos / MSV_SECTOR_SIZE + 1) * MSV_SECTOR_SIZE +
                           MSV_INDEX_SIZE;
                if (next_sec + (size - in_sec) > fsize) {
                    av_packet_unref(pkt);
                    return AVERROR_EOF;
                }
                avio_seek(pb, next_sec, SEEK_SET);
                ret = avio_read(pb, pkt->data + in_sec, size - in_sec);
                if (ret < size - in_sec) {
                    av_packet_unref(pkt);
                    return ret < 0 ? ret : AVERROR_EOF;
                }
                return 0;
            }

            ret = av_new_packet(pkt, size);
            if (ret < 0)
                return ret;
            pkt->data[0] = hdr;
            ret = avio_read(pb, pkt->data + 1, size - 1);
            if (ret < size - 1) {
                av_packet_unref(pkt);
                return ret < 0 ? ret : AVERROR_EOF;
            }
            return 0;
        }

        if (ctx->frame_sizes[0] == 0) {
            size = sec_end - pos;
            if (size <= 0) {
                if (!use_sectors)
                    return AVERROR_EOF;
                avio_seek(pb, ctx->data_offset +
                          (pos / MSV_SECTOR_SIZE + 1) * MSV_SECTOR_SIZE, SEEK_SET);
                continue;
            }
            return av_get_packet(pb, pkt, size);
        }

        ret = avio_read(pb, &hdr, 1);
        if (ret < 1)
            return ret < 0 ? ret : AVERROR_EOF;

        size = msv_frame_size_from_byte(ctx, hdr);
        if (size <= 1) {
            avio_seek(pb, -1, SEEK_CUR);
            return AVERROR_INVALIDDATA;
        }
        if (pos + size > sec_end) {
            int in_sec;
            int64_t next_sec;

            if (!use_sectors)
                return AVERROR_INVALIDDATA;
            /*
 * LPEC/LCST/TRC bitstream is continuous across 0x400 sectors; only
 * the 10-byte index at each sector start is outside the stream.
 * Read the frame spanning the sector boundary instead of dropping it.
*/
            in_sec = sec_end - pos;
            ret = av_new_packet(pkt, size);
            if (ret < 0)
                return ret;
            pkt->data[0] = hdr;
            ret = avio_read(pb, pkt->data + 1, in_sec - 1);
            if (ret < in_sec - 1) {
                av_packet_unref(pkt);
                return ret < 0 ? ret : AVERROR_EOF;
            }
            next_sec = ctx->data_offset +
                       (pos / MSV_SECTOR_SIZE + 1) * MSV_SECTOR_SIZE +
                       MSV_INDEX_SIZE;
            if (next_sec + (size - in_sec) > fsize)
                return AVERROR_EOF;
            avio_seek(pb, next_sec, SEEK_SET);
            ret = avio_read(pb, pkt->data + in_sec, size - in_sec);
            if (ret < size - in_sec) {
                av_packet_unref(pkt);
                return ret < 0 ? ret : AVERROR_EOF;
            }
            return 0;
        }
        if (tell + size > fsize)
            return AVERROR_EOF;
        break;
    }

    ret = av_new_packet(pkt, size);
    if (ret < 0)
        return ret;
    pkt->data[0] = hdr;
    ret = avio_read(pb, pkt->data + 1, size - 1);
    if (ret < size - 1) {
        av_packet_unref(pkt);
        return ret < 0 ? ret : AVERROR_EOF;
    }

    return 0;
}

static int msv_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    MSVDemuxContext *ctx = s->priv_data;
    int ret = msv_read_raw_packet(s, pkt);

    if (ret < 0)
        return ret;
    if (ctx->codec == MSV_CODEC_LCST)
        msv_lcst_deframe(pkt);
    return 0;
}

const FFInputFormat ff_msv_demuxer = {
    .p.name         = "msv",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony Memory Stick Voice (MSV/DVF)"),
    .p.extensions   = "msv,dvf",
    .priv_data_size = sizeof(MSVDemuxContext),
    .read_probe     = msv_probe,
    .read_header    = msv_read_header,
    .read_packet    = msv_read_packet,
};
