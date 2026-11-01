/*
 * DSS and DS2 file header metadata
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

#include "libavutil/avstring.h"
#include "libavutil/dict.h"
#include "libavutil/error.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"

#include "avformat.h"
#include "dss_metadata.h"

#define DS2_HEAD_OFFSET_AUTHOR      0xc
#define DS2_AUTHOR_SIZE             16
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

void ff_dss_trim_string(char *s, int size)
{
    int i;

    for (i = 0; i < size && s[i] && (unsigned char)s[i] != 0xff; i++)
        ;
    while (i > 0 && av_isspace(s[i - 1]))
        i--;
    s[i] = 0;
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
    ff_dss_trim_string(value, size);
    if (!value[0]) {
        av_free(value);
        return 0;
    }

    return av_dict_set(&s->metadata, key, value, AV_DICT_DONT_STRDUP_VAL);
}

/* A table of 60-byte entries, a 30-byte key and a 30-byte value, up to the
 * comment at 0x31e, which cuts the value of the fourth entry short. An entry
 * with a blank key is unused. */
static int ds2_read_extended_metadata(AVFormatContext *s, const uint8_t *hdr,
                                      int size)
{
    int off, len, end = FFMIN(size, DS2_HEAD_EXT_META_END), ret = 0;

    if (size <= DS2_HEAD_OFFSET_EXT_META || hdr[DS2_HEAD_OFFSET_EXT_META] == 0xff)
        return 0;

    for (off = DS2_HEAD_OFFSET_EXT_META; off + DS2_META_KEY_SIZE <= end;
         off += DS2_META_ENTRY_SIZE) {
        const char *key = (const char *)(hdr + off);
        const char *val = (const char *)(hdr + off + DS2_META_KEY_SIZE);
        char keybuf[DS2_META_KEY_SIZE + 1];
        char valbuf[DS2_META_VAL_SIZE + 1];

        if (ds2_field_is_blank(key, DS2_META_KEY_SIZE))
            continue;

        len = FFMIN(DS2_META_VAL_SIZE, end - off - DS2_META_KEY_SIZE);
        memcpy(keybuf, key, DS2_META_KEY_SIZE);
        keybuf[DS2_META_KEY_SIZE] = 0;
        memcpy(valbuf, val, len);
        valbuf[len] = 0;
        ff_dss_trim_string(keybuf, DS2_META_KEY_SIZE);
        ff_dss_trim_string(valbuf, len);
        if (!keybuf[0])
            continue;

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

int ff_dss_read_ds2_metadata(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    uint8_t hdr[DS2_HEADER_SIZE];
    int size, ret;

    /* A file can end before DS2_HEADER_SIZE when its header is smaller.
     * Fields past the end are left out. */
    avio_seek(pb, 0, SEEK_SET);
    size = avio_read(pb, hdr, sizeof(hdr));
    if (size < 0)
        return size;

    if (size >= DS2_HEAD_OFFSET_AUTHOR + DS2_AUTHOR_SIZE) {
        ret = ds2_set_metadata_copy(s, (const char *)(hdr + DS2_HEAD_OFFSET_AUTHOR),
                                    DS2_AUTHOR_SIZE, "author");
        if (ret < 0)
            return ret;
    }

    if (size < DS2_HEAD_OFFSET_START_TIME + DS2_TIME_SIZE ||
        ds2_set_metadata_time(s, hdr + DS2_HEAD_OFFSET_START_TIME,
                              "creation_time") < 0)
        av_log(s, AV_LOG_DEBUG, "DS2 start time invalid or missing\n");

    if (size < DS2_HEAD_OFFSET_END_TIME + DS2_TIME_SIZE ||
        ds2_set_metadata_time(s, hdr + DS2_HEAD_OFFSET_END_TIME, "date") < 0)
        av_log(s, AV_LOG_DEBUG, "DS2 end time invalid or missing\n");

    if (size >= DS2_HEAD_OFFSET_COMMENT + DS2_COMMENT_SIZE) {
        ret = ds2_set_metadata_copy(s, (const char *)(hdr + DS2_HEAD_OFFSET_COMMENT),
                                    DS2_COMMENT_SIZE, "comment");
        if (ret < 0)
            return ret;
    }

    return ds2_read_extended_metadata(s, hdr, size);
}
