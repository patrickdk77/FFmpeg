/*
 * DS2 encrypted file decryption (Olympus "\x03enc")
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

#ifndef AVFORMAT_DS2CRYPTO_H
#define AVFORMAT_DS2CRYPTO_H

#include "libavutil/macros.h"

#include "avformat.h"

#define DS2_ENCRYPTED_MAGIC MKTAG(0x3, 'e', 'n', 'c') /* "\3enc" */

/**
 * Check a password against the header of an encrypted DS2 file, and open a
 * read-only AVIOContext over s->pb that returns the file with its audio
 * blocks decrypted. The file header, header_size bytes, reads as it is. The
 * data ends after the last whole block, since a part of a block cannot be
 * decrypted. Exports the encryption tag.
 *
 * @return 0 or a negative error code
 */
int ff_ds2_decrypt_open(AVFormatContext *s, const char *password,
                        int header_size, AVIOContext **pb);

/**
 * Free an AVIOContext opened by ff_ds2_decrypt_open() and set *pb to NULL.
 */
void ff_ds2_decrypt_close(AVIOContext **pb);

#endif /* AVFORMAT_DS2CRYPTO_H */
