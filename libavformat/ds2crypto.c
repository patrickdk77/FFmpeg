/*
 * DS2 encrypted file decryption
 */

#include <string.h>

#include "libavutil/error.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/sha.h"
#include "libavutil/sha512.h"

#include "ds2crypto.h"

#define DS2_BLOCK_SIZE 512
#define DS2_BLOCK_HEADER_SIZE 6
#define DS2_CURRENT_BLOCK_OFFSET 0x10
#define DS2_AES_STATE_OFFSET 0x20
#define DS2_AES_ROUND_COUNT_OFFSET 0x120
#define DS2_AES_FLAGS_OFFSET 0x124
#define DS2_BLOCK_BYTE_INDEX_OFFSET 0x128

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

static const uint8_t ds2_aes_inv_sbox[256] = {
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e,
    0x81, 0xf3, 0xd7, 0xfb, 0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87,
    0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb, 0x54, 0x7b, 0x94, 0x32,
    0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49,
    0x6d, 0x8b, 0xd1, 0x25, 0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16,
    0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92, 0x6c, 0x70, 0x48, 0x50,
    0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05,
    0xb8, 0xb3, 0x45, 0x06, 0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02,
    0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b, 0x3a, 0x91, 0x11, 0x41,
    0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8,
    0x1c, 0x75, 0xdf, 0x6e, 0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89,
    0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b, 0xfc, 0x56, 0x3e, 0x4b,
    0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59,
    0x27, 0x80, 0xec, 0x5f, 0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d,
    0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef, 0xa0, 0xe0, 0x3b, 0x4d,
    0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63,
    0x55, 0x21, 0x0c, 0x7d,
};

static uint32_t ds2_load_be32(const uint8_t *b)
{
    return AV_RB32(b);
}

static void ds2_store_be32(uint8_t *b, uint32_t w)
{
    AV_WB32(b, w);
}

static uint32_t ds2_round_key_word(const DS2DecryptState *state, int index)
{
    return ds2_load_be32(state->blob + DS2_AES_STATE_OFFSET + index * 4);
}

static void ds2_set_round_key_word(DS2DecryptState *state, int index, uint32_t w)
{
    ds2_store_be32(state->blob + DS2_AES_STATE_OFFSET + index * 4, w);
}

static int ds2_round_count(const DS2DecryptState *state)
{
    return AV_RL32(state->blob + DS2_AES_ROUND_COUNT_OFFSET);
}

static void ds2_set_round_count(DS2DecryptState *state, int rounds)
{
    AV_WL32(state->blob + DS2_AES_ROUND_COUNT_OFFSET, rounds);
}

static void ds2_set_flags(DS2DecryptState *state, uint32_t flags)
{
    AV_WL32(state->blob + DS2_AES_FLAGS_OFFSET, flags);
}

static int ds2_block_byte_index(const DS2DecryptState *state)
{
    return AV_RL32(state->blob + DS2_BLOCK_BYTE_INDEX_OFFSET);
}

static void ds2_set_block_byte_index(DS2DecryptState *state, int index)
{
    AV_WL32(state->blob + DS2_BLOCK_BYTE_INDEX_OFFSET, index);
}

static void ds2_get_current_block(const DS2DecryptState *state, uint8_t *out)
{
    memcpy(out, state->blob + DS2_CURRENT_BLOCK_OFFSET, 16);
}

static void ds2_set_current_block(DS2DecryptState *state, const uint8_t *block)
{
    memcpy(state->blob + DS2_CURRENT_BLOCK_OFFSET, block, 16);
}

static void ds2_rekey_source_128(const DS2DecryptState *state, uint8_t *out)
{
    int start = (ds2_round_count(state) + 2) * 0x10;
    memcpy(out, state->blob + start, 16);
}

static void ds2_rekey_source_256(const DS2DecryptState *state, uint8_t *out)
{
    int start = (ds2_round_count(state) + 2) * 0x10;
    const uint8_t *source = state->blob + start;
    uint32_t words[4];
    int i;

    for (i = 0; i < 4; i++)
        words[i] = ds2_load_be32(source + i * 4);
    ds2_store_be32(out + 0,  words[0]);
    ds2_store_be32(out + 4,  words[1]);
    ds2_store_be32(out + 8,  words[2]);
    ds2_store_be32(out + 12, words[3]);
    ds2_store_be32(out + 16, words[1]);
    ds2_store_be32(out + 20, words[0]);
    ds2_store_be32(out + 24, words[3]);
    ds2_store_be32(out + 28, words[2]);
}

static void ds2_state_clone(DS2DecryptState *dst, const DS2DecryptState *src)
{
    memcpy(dst->blob, src->blob, DS2_SAVED_STATE_SIZE);
}

static uint32_t ds2_rot_word(uint32_t x)
{
    return (x << 8) | (x >> 24);
}

static uint32_t ds2_sub_word(uint32_t x)
{
    return (ds2_aes_sbox[(x >> 24) & 0xff] << 24) |
           (ds2_aes_sbox[(x >> 16) & 0xff] << 16) |
           (ds2_aes_sbox[(x >> 8) & 0xff] << 8) |
           ds2_aes_sbox[x & 0xff];
}

static int ds2_gf_mul(int a, int b)
{
    int result = 0;
    while (b) {
        if (b & 1)
            result ^= a;
        if (a & 0x80)
            a = ((a << 1) ^ 0x1b) & 0xff;
        else
            a = (a << 1) & 0xff;
        b >>= 1;
    }
    return result;
}

static void ds2_aes_expand_key(DS2DecryptState *state, const uint8_t *key, int key_len)
{
    int nk, nr, i, total_words;

    if (key_len == 16) {
        nk = 4;
        nr = 10;
    } else if (key_len == 32) {
        nk = 8;
        nr = 14;
    } else {
        return;
    }

    memset(state->blob + DS2_AES_STATE_OFFSET, 0,
           DS2_BLOCK_BYTE_INDEX_OFFSET - DS2_AES_STATE_OFFSET);

    total_words = 4 * (nr + 1);
    for (i = 0; i < nk; i++)
        ds2_set_round_key_word(state, i, ds2_load_be32(key + i * 4));

    for (i = nk; i < total_words; i++) {
        uint32_t temp = ds2_round_key_word(state, i - 1);
        if (i % nk == 0)
            temp = ds2_sub_word(ds2_rot_word(temp)) ^ ds2_aes_rcon[i / nk - 1];
        else if (nk > 6 && i % nk == 4)
            temp = ds2_sub_word(temp);
        ds2_set_round_key_word(state, i,
                               ds2_round_key_word(state, i - nk) ^ temp);
    }

    ds2_set_round_count(state, nr);
    ds2_set_flags(state, 0x12);
}

static void ds2_add_round_key(uint8_t *block, const DS2DecryptState *state,
                              int word_index)
{
    int i;
    for (i = 0; i < 4; i++) {
        uint32_t word = ds2_round_key_word(state, word_index + i);
        block[4 * i]     ^= (word >> 24) & 0xff;
        block[4 * i + 1] ^= (word >> 16) & 0xff;
        block[4 * i + 2] ^= (word >> 8) & 0xff;
        block[4 * i + 3] ^= word & 0xff;
    }
}

static void ds2_inv_sub_bytes(uint8_t *block)
{
    int i;
    for (i = 0; i < 16; i++)
        block[i] = ds2_aes_inv_sbox[block[i]];
}

static void ds2_inv_shift_rows(uint8_t *block)
{
    uint8_t tmp[16];
    memcpy(tmp, block, 16);
    block[0]  = tmp[0];
    block[1]  = tmp[13];
    block[2]  = tmp[10];
    block[3]  = tmp[7];
    block[4]  = tmp[4];
    block[5]  = tmp[1];
    block[6]  = tmp[14];
    block[7]  = tmp[11];
    block[8]  = tmp[8];
    block[9]  = tmp[5];
    block[10] = tmp[2];
    block[11] = tmp[15];
    block[12] = tmp[12];
    block[13] = tmp[9];
    block[14] = tmp[6];
    block[15] = tmp[3];
}

static void ds2_inv_mix_columns(uint8_t *block)
{
    int col;
    for (col = 0; col < 4; col++) {
        int base = col * 4;
        int s0 = block[base];
        int s1 = block[base + 1];
        int s2 = block[base + 2];
        int s3 = block[base + 3];
        block[base]     = ds2_gf_mul(s0, 14) ^ ds2_gf_mul(s1, 11) ^
                          ds2_gf_mul(s2, 13) ^ ds2_gf_mul(s3, 9);
        block[base + 1] = ds2_gf_mul(s0, 9) ^ ds2_gf_mul(s1, 14) ^
                          ds2_gf_mul(s2, 11) ^ ds2_gf_mul(s3, 13);
        block[base + 2] = ds2_gf_mul(s0, 13) ^ ds2_gf_mul(s1, 9) ^
                          ds2_gf_mul(s2, 14) ^ ds2_gf_mul(s3, 11);
        block[base + 3] = ds2_gf_mul(s0, 11) ^ ds2_gf_mul(s1, 13) ^
                          ds2_gf_mul(s2, 9) ^ ds2_gf_mul(s3, 14);
    }
}

static void ds2_aes_decrypt_block(const DS2DecryptState *state,
                                  const uint8_t *ciphertext, uint8_t *plaintext)
{
    int round_count = ds2_round_count(state);
    int rnd;
    uint8_t block[16];

    memcpy(block, ciphertext, 16);
    ds2_add_round_key(block, state, 4 * round_count);
    for (rnd = round_count - 1; rnd > 0; rnd--) {
        ds2_inv_shift_rows(block);
        ds2_inv_sub_bytes(block);
        ds2_add_round_key(block, state, 4 * rnd);
        ds2_inv_mix_columns(block);
    }
    ds2_inv_shift_rows(block);
    ds2_inv_sub_bytes(block);
    ds2_add_round_key(block, state, 0);
    memcpy(plaintext, block, 16);
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

int ds2_parse_decrypt_descriptor(const uint8_t *header, int header_size,
                               DS2DecryptDescriptor *desc)
{
    const uint8_t *raw;

    if (header_size < DS2_DECRYPT_DESC_OFFSET + DS2_DECRYPT_DESC_SIZE)
        return AVERROR_INVALIDDATA;

    raw = header + DS2_DECRYPT_DESC_OFFSET;
    desc->key_mode = AV_RL16(raw);
    if (desc->key_mode != DS2_KEY_MODE_AES128 &&
        desc->key_mode != DS2_KEY_MODE_AES256)
        return AVERROR_INVALIDDATA;

    memcpy(desc->aux_16, raw + 2, 16);
    desc->expected_check = AV_RL16(raw + 18);
    return 0;
}

int ds2_decrypt_init(DS2DecryptState *state, const DS2DecryptDescriptor *desc,
                     const uint8_t *password, int password_len)
{
    uint8_t key[32];
    uint16_t check;
    int ret;

    if (!password || password_len <= 0)
        return AVERROR(EINVAL);
    if (password_len > DS2_MAX_PASSWORD_BYTES)
        return AVERROR(EINVAL);

    memset(state, 0, sizeof(*state));

    if (desc->key_mode == DS2_KEY_MODE_AES128) {
        ret = ds2_derive_key_128(password, password_len, desc->aux_16, key, &check);
        if (ret < 0)
            return ret;
        if (check != desc->expected_check)
            return AVERROR_INVALIDDATA;
        ds2_set_block_byte_index(state, 0x10);
        ds2_aes_expand_key(state, key, 16);
    } else {
        ret = ds2_derive_key_256(password, password_len, desc->aux_16, key, &check);
        if (ret < 0)
            return ret;
        if (check != desc->expected_check)
            return AVERROR_INVALIDDATA;
        ds2_set_block_byte_index(state, 0x10);
        ds2_aes_expand_key(state, key, 32);
    }

    return 0;
}

static void ds2_decrypt_body_self_rekey(DS2DecryptState *state, int key_mode,
                                        uint8_t *body, int body_size)
{
    int off;

    for (off = 0; off < body_size; off += 16) {
        uint8_t plaintext[16];
        uint8_t rekey[32];

        if (ds2_block_byte_index(state) == 0x10) {
            ds2_aes_decrypt_block(state, body + off, plaintext);
            ds2_set_current_block(state, plaintext);
            if (key_mode == DS2_KEY_MODE_AES128)
                ds2_rekey_source_128(state, rekey);
            else
                ds2_rekey_source_256(state, rekey);
            ds2_aes_expand_key(state, rekey,
                               key_mode == DS2_KEY_MODE_AES128 ? 16 : 32);
            ds2_set_block_byte_index(state, 0);
        }

        ds2_get_current_block(state, body + off);
        ds2_set_block_byte_index(state, 0x10);
    }
}

void ds2_decrypt_record(DS2DecryptState *state, int key_mode, uint8_t *record512)
{
    DS2DecryptState work;
    uint8_t *body = record512 + DS2_BLOCK_HEADER_SIZE;

    ds2_state_clone(&work, state);
    ds2_swap_adjacent_bytes(body, DS2_TRANSFORMED_BODY_SIZE);
    ds2_decrypt_body_self_rekey(&work, key_mode, body, DS2_TRANSFORMED_BODY_SIZE);
    ds2_swap_adjacent_bytes(body, DS2_TRANSFORMED_BODY_SIZE);
}
