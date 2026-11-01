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

#ifndef AVFORMAT_DSS_METADATA_H
#define AVFORMAT_DSS_METADATA_H

#include "avformat.h"

#define DS2_HEADER_SIZE 0x600 /* Olympus DS2 header, 3 blocks */

/**
 * End a header string of size bytes, in a buffer of size + 1, at its first
 * NUL or 0xff fill byte, and drop trailing white space.
 */
void ff_dss_trim_string(char *s, int size);

/**
 * Export the author, start and end time, comment and named fields of a DS2
 * file header. Fields that are blank or do not parse are left out.
 */
int ff_dss_read_ds2_metadata(AVFormatContext *s);

#define DS2_MAX_ANNOTATIONS 32

enum DS2ExtractMode {
    DS2_EXTRACT_ALL = 0,
    DS2_EXTRACT_MAIN,
    DS2_EXTRACT_ANNOTATIONS,
};

typedef struct DS2AnnotationRange {
    int first_frame; /* inclusive */
    int last_frame;  /* exclusive */
} DS2AnnotationRange;

typedef struct DS2Annotations {
    DS2AnnotationRange range[DS2_MAX_ANNOTATIONS];
    int nb;
} DS2Annotations;

/**
 * Read the annotations of a DS2 file header as ranges of frames of frame_ms
 * ms each. A header of header_size bytes that does not reach the table has
 * none.
 */
int ff_dss_read_ds2_annotations(AVFormatContext *s, int header_size,
                                int frame_ms, DS2Annotations *ann);

/**
 * Whether frame number frame of the recording is returned in the
 * DS2ExtractMode extract.
 */
int ff_dss_ds2_frame_included(const DS2Annotations *ann, int extract,
                              int64_t frame);

#endif /* AVFORMAT_DSS_METADATA_H */
