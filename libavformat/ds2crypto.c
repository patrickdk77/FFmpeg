/*
 * DS2 encrypted file decryption
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

#include "libavutil/aes.h"
#include "libavutil/dict.h"
#include "libavutil/error.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"
#include "libavutil/macros.h"
#include "libavutil/mem.h"
#include "libavutil/sha.h"
#include "libavutil/sha512.h"

#include "avformat.h"
#include "avio_internal.h"
#include "ds2crypto.h"

#define DS2_BLOCK_SIZE 512
#define DS2_BLOCK_HEADER_SIZE 6
#define DS2_TRANSFORMED_BODY_SIZE 0x1F0

#define DS2_DECRYPT_HEADER_SIZE 0x600
#define DS2_DECRYPT_DESC_OFFSET 0x146
#define DS2_DECRYPT_DESC_SIZE   22

#define DS2_KEY_MODE_UNSTATED 0
#define DS2_KEY_MODE_AES128 1
#define DS2_KEY_MODE_AES256 2

#define DS2_MAX_PASSWORD_BYTES 16

typedef struct DS2DecryptState {
    uint8_t key[32];
    struct AVAES *aes;
} DS2DecryptState;

typedef struct DS2DecryptDescriptor {
    int      key_mode;
    uint8_t  aux_16[16];
    uint16_t expected_check;
} DS2DecryptDescriptor;

typedef struct DS2DecryptContext {
    AVIOContext *src;
    DS2DecryptState state;
    int key_mode;
    int header_size;
    int64_t size;                   /* header and whole blocks, or -1 */
    int64_t pos;
    uint8_t block[DS2_BLOCK_SIZE];  /* decrypted block at block_start */
    int64_t block_start;
    uint8_t head[DS2_BLOCK_HEADER_SIZE];
    int64_t head_start;
    int head_len;
} DS2DecryptContext;

static const uint32_t ds2_aes_rcon[10] = {
    0x01000000, 0x02000000, 0x04000000, 0x08000000, 0x10000000,
    0x20000000, 0x40000000, 0x80000000, 0x1b000000, 0x36000000,
};

static const uint8_t ds2_aes_sbox[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b,
    0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0,
    0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26,
    0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2,
    0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
    0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed,
    0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f,
    0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
    0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec,
    0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14,
    0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c,
    0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d,
    0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f,
    0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
    0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11,
    0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f,
    0xb0, 0x54, 0xbb, 0x16,
};

static uint32_t ds2_rot_word(uint32_t x)
{
    return (x << 8) | (x >> 24);
}

static uint32_t ds2_sub_word(uint32_t x)
{
    return ((uint32_t)ds2_aes_sbox[(x >> 24) & 0xff] << 24) |
           (ds2_aes_sbox[(x >> 16) & 0xff] << 16) |
           (ds2_aes_sbox[(x >> 8) & 0xff] << 8) |
           ds2_aes_sbox[x & 0xff];
}

/* Replace key with the key of the next 16-byte block, which comes from the
 * last round key of key's AES key schedule. */
static void ds2_next_key(uint8_t *key, int key_len)
{
    uint32_t w[60];
    int nk = key_len / 4, nr = nk + 6, i, total_words = 4 * (nr + 1);
    const uint32_t *last = w + 4 * nr;

    for (i = 0; i < nk; i++)
        w[i] = AV_RB32(key + i * 4);

    for (i = nk; i < total_words; i++) {
        uint32_t temp = w[i - 1];
        if (i % nk == 0)
            temp = ds2_sub_word(ds2_rot_word(temp)) ^ ds2_aes_rcon[i / nk - 1];
        else if (nk > 6 && i % nk == 4)
            temp = ds2_sub_word(temp);
        w[i] = w[i - nk] ^ temp;
    }

    AV_WB32(key + 0,  last[0]);
    AV_WB32(key + 4,  last[1]);
    AV_WB32(key + 8,  last[2]);
    AV_WB32(key + 12, last[3]);
    if (key_len == 32) {
        AV_WB32(key + 16, last[1]);
        AV_WB32(key + 20, last[0]);
        AV_WB32(key + 24, last[3]);
        AV_WB32(key + 28, last[2]);
    }
}

static void ds2_swap_adjacent_bytes(uint8_t *buf, int size)
{
    int i;
    for (i = 0; i < size - 1; i += 2) {
        uint8_t t = buf[i];
        buf[i] = buf[i + 1];
        buf[i + 1] = t;
    }
}

static void ds2_mix_password(const uint8_t *password, int password_len,
                             const uint8_t aux_16[16], uint8_t mixed[16])
{
    int i;
    memset(mixed, 0, 16);
    if (password_len > DS2_MAX_PASSWORD_BYTES)
        password_len = DS2_MAX_PASSWORD_BYTES;
    memcpy(mixed, password, password_len);
    for (i = 0; i < 16; i++)
        mixed[i] ^= aux_16[i];
}

static int ds2_derive_key_128(const uint8_t *password, int password_len,
                              const uint8_t aux_16[16], uint8_t key[16],
                              uint16_t *check)
{
    struct AVSHA *sha = av_sha_alloc();
    uint8_t digest[20];
    uint8_t mixed[16];

    if (!sha)
        return AVERROR(ENOMEM);
    ds2_mix_password(password, password_len, aux_16, mixed);
    av_sha_init(sha, 160);
    av_sha_update(sha, mixed, 16);
    av_sha_final(sha, digest);
    av_free(sha);
    memcpy(key, digest, 16);
    *check = AV_RL16(digest + 16);
    return 0;
}

static int ds2_derive_key_256(const uint8_t *password, int password_len,
                              const uint8_t aux_16[16], uint8_t key[32],
                              uint16_t *check)
{
    struct AVSHA512 *sha = av_sha512_alloc();
    uint8_t digest[48];
    uint8_t mixed[16];

    if (!sha)
        return AVERROR(ENOMEM);
    ds2_mix_password(password, password_len, aux_16, mixed);
    av_sha512_init(sha, 384);
    av_sha512_update(sha, mixed, 16);
    av_sha512_final(sha, digest);
    av_free(sha);
    memcpy(key, digest, 32);
    *check = AV_RL16(digest + 32);
    return 0;
}

static int ds2_parse_decrypt_descriptor(const uint8_t *header, int header_size,
                                        DS2DecryptDescriptor *desc)
{
    const uint8_t *raw;

    if (header_size < DS2_DECRYPT_DESC_OFFSET + DS2_DECRYPT_DESC_SIZE)
        return AVERROR_INVALIDDATA;

    raw = header + DS2_DECRYPT_DESC_OFFSET;
    desc->key_mode = AV_RL16(raw);
    if (desc->key_mode != DS2_KEY_MODE_UNSTATED &&
        desc->key_mode != DS2_KEY_MODE_AES128 &&
        desc->key_mode != DS2_KEY_MODE_AES256)
        return AVERROR_INVALIDDATA;

    memcpy(desc->aux_16, raw + 2, 16);
    desc->expected_check = AV_RL16(raw + 18);
    return 0;
}

static int ds2_decrypt_init(DS2DecryptState *state,
                            DS2DecryptDescriptor *desc,
                            const uint8_t *password, int password_len)
{
    static const int modes[] = { DS2_KEY_MODE_AES128, DS2_KEY_MODE_AES256 };
    uint16_t check;
    int i, ret;

    if (!password || password_len <= 0)
        return AVERROR(EINVAL);
    if (password_len > DS2_MAX_PASSWORD_BYTES)
        return AVERROR(EINVAL);

    for (i = 0; i < FF_ARRAY_ELEMS(modes); i++) {
        if (desc->key_mode != DS2_KEY_MODE_UNSTATED &&
            desc->key_mode != modes[i])
            continue;
        if (modes[i] == DS2_KEY_MODE_AES128)
            ret = ds2_derive_key_128(password, password_len, desc->aux_16,
                                     state->key, &check);
        else
            ret = ds2_derive_key_256(password, password_len, desc->aux_16,
                                     state->key, &check);
        if (ret < 0)
            return ret;
        if (check == desc->expected_check)
            break;
    }
    if (i == FF_ARRAY_ELEMS(modes))
        return AVERROR_INVALIDDATA;
    desc->key_mode = modes[i];

    state->aes = av_aes_alloc();
    if (!state->aes)
        return AVERROR(ENOMEM);
    return 0;
}

/* The block header and the last 10 bytes are in the clear. The 0x1F0 bytes
 * between them, with each pair of bytes swapped, are 31 AES blocks in ECB
 * mode, each under its own key: the first from the password, each next one
 * from ds2_next_key(). */
static void ds2_decrypt_record(DS2DecryptState *state, int key_mode,
                               uint8_t *record512)
{
    uint8_t *body = record512 + DS2_BLOCK_HEADER_SIZE;
    int key_len = key_mode == DS2_KEY_MODE_AES128 ? 16 : 32;
    uint8_t key[32];
    int off;

    memcpy(key, state->key, key_len);
    ds2_swap_adjacent_bytes(body, DS2_TRANSFORMED_BODY_SIZE);
    for (off = 0; off < DS2_TRANSFORMED_BODY_SIZE; off += 16) {
        av_aes_init(state->aes, key, key_len * 8, 1);
        av_aes_crypt(state->aes, body + off, body + off, 1, NULL, 1);
        ds2_next_key(key, key_len);
    }
    ds2_swap_adjacent_bytes(body, DS2_TRANSFORMED_BODY_SIZE);
}

static void ds2_decrypt_uninit(DS2DecryptState *state)
{
    av_freep(&state->aes);
}

static int ds2_decrypt_read(void *opaque, uint8_t *buf, int buf_size)
{
    DS2DecryptContext *c = opaque;
    int total = 0, ret = AVERROR_EOF;

    while (total < buf_size && (c->size < 0 || c->pos < c->size)) {
        int64_t start;
        int n;

        if (c->pos < c->header_size) {
            if (avio_seek(c->src, c->pos, SEEK_SET) < 0)
                break;
            ret = avio_read(c->src, buf + total,
                            FFMIN(buf_size - total, c->header_size - c->pos));
            if (ret <= 0)
                break;
            n = ret;
        } else {
            start = c->pos - (c->pos - c->header_size) % DS2_BLOCK_SIZE;
            /* A block header is in the clear. Return it without decrypting
             * the block, which a scan of the block headers does not need. */
            if (c->block_start != start &&
                c->pos - start < DS2_BLOCK_HEADER_SIZE) {
                if (avio_seek(c->src, c->pos, SEEK_SET) < 0)
                    break;
                ret = avio_read(c->src, buf + total,
                                FFMIN(buf_size - total,
                                      start + DS2_BLOCK_HEADER_SIZE - c->pos));
                if (ret > 0) {
                    if (c->head_start != start ||
                        c->head_len != c->pos - start) {
                        c->head_start = start;
                        c->head_len   = c->pos == start ? 0 : -1;
                    }
                    if (c->head_len >= 0) {
                        memcpy(c->head + c->head_len, buf + total, ret);
                        c->head_len += ret;
                    }
                    c->pos += ret;
                    total  += ret;
                }
                break;
            }
            if (c->block_start != start) {
                int have = c->head_start == start ? FFMAX(c->head_len, 0) : 0;

                c->block_start = -1;
                if (have && avio_tell(c->src) == start + have) {
                    memcpy(c->block, c->head, have);
                } else {
                    have = 0;
                    if (avio_seek(c->src, start, SEEK_SET) < 0)
                        break;
                }
                ret = avio_read(c->src, c->block + have, DS2_BLOCK_SIZE - have);
                if (ret != DS2_BLOCK_SIZE - have) {
                    if (ret >= 0)
                        ret = AVERROR_EOF;
                    break;
                }
                ds2_decrypt_record(&c->state, c->key_mode, c->block);
                c->block_start = start;
            }
            n = FFMIN(buf_size - total, start + DS2_BLOCK_SIZE - c->pos);
            memcpy(buf + total, c->block + (c->pos - start), n);
        }
        c->pos += n;
        total  += n;
    }

    return total ? total : (ret < 0 ? ret : AVERROR_EOF);
}

static int64_t ds2_decrypt_seek(void *opaque, int64_t offset, int whence)
{
    DS2DecryptContext *c = opaque;

    if (whence == AVSEEK_SIZE)
        return c->size >= 0 ? c->size : AVERROR(ENOSYS);
    if ((whence & ~AVSEEK_FORCE) != SEEK_SET || offset < 0)
        return AVERROR(EINVAL);
    c->pos = offset;
    return offset;
}

int ff_ds2_decrypt_open(AVFormatContext *s, const char *password,
                        int header_size, AVIOContext **pb)
{
    uint8_t hdr[DS2_DECRYPT_HEADER_SIZE];
    DS2DecryptDescriptor desc;
    DS2DecryptContext *c;
    uint8_t *buf;
    int64_t size;
    int ret, pwd_len;

    if (!password || !password[0]) {
        av_log(s, AV_LOG_ERROR,
               "Encrypted DS2 file requires the -password option\n");
        return AVERROR(EINVAL);
    }

    avio_seek(s->pb, 0, SEEK_SET);
    ret = avio_read(s->pb, hdr, sizeof(hdr));
    if (ret != sizeof(hdr))
        return ret < 0 ? ret : AVERROR_EOF;

    ret = ds2_parse_decrypt_descriptor(hdr, sizeof(hdr), &desc);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "Invalid DS2 encryption descriptor\n");
        return ret;
    }

    pwd_len = strlen(password);
    if (pwd_len > DS2_MAX_PASSWORD_BYTES) {
        av_log(s, AV_LOG_ERROR,
               "DS2 password longer than %d bytes is not supported\n",
               DS2_MAX_PASSWORD_BYTES);
        return AVERROR(EINVAL);
    }

    c = av_mallocz(sizeof(*c));
    if (!c)
        return AVERROR(ENOMEM);

    ret = ds2_decrypt_init(&c->state, &desc, (const uint8_t *)password, pwd_len);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "DS2 password rejected\n");
        goto fail;
    }

    size = avio_size(s->pb);
    c->src         = s->pb;
    c->key_mode    = desc.key_mode;
    c->header_size = header_size;
    c->size        = size < header_size ? size :
                     size - (size - header_size) % DS2_BLOCK_SIZE;
    c->block_start = -1;
    c->head_start  = -1;

    buf = av_malloc(DS2_BLOCK_SIZE);
    if (!buf) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    *pb = avio_alloc_context(buf, DS2_BLOCK_SIZE, 0, c, ds2_decrypt_read,
                             NULL, ds2_decrypt_seek);
    if (!*pb) {
        av_free(buf);
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    (*pb)->seekable = s->pb->seekable;
    /* Seek past the payload of a block rather than read and decrypt it. */
    ffiocontext(*pb)->short_seek_threshold = 0;

    return av_dict_set(&s->metadata, "encryption", "1", 0);

fail:
    ds2_decrypt_uninit(&c->state);
    av_free(c);
    return ret;
}

void ff_ds2_decrypt_close(AVIOContext **pb)
{
    DS2DecryptContext *c;

    if (!*pb)
        return;
    c = (*pb)->opaque;
    ds2_decrypt_uninit(&c->state);
    av_freep(&c);
    av_freep(&(*pb)->buffer);
    avio_context_free(pb);
}
