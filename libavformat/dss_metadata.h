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

#endif /* AVFORMAT_DSS_METADATA_H */
