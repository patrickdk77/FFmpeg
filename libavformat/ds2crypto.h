/*
 * DS2 encrypted file decryption (Olympus "\x03enc")
 *
 * Port of the hirparak/dss-codec DS2 decryption reference.
 */

#ifndef AVFORMAT_DS2CRYPTO_H
#define AVFORMAT_DS2CRYPTO_H

#include <stdint.h>

#include "libavutil/macros.h"

#define DS2_ENCRYPTED_MAGIC MKTAG(0x3, 'e', 'n', 'c') /* "\3enc" */
#define DS2_PLAIN_MAGIC     MKTAG(0x3, 'd', 's', '2') /* "\3ds2" */

#define DS2_DECRYPT_DESC_OFFSET 0x146
#define DS2_DECRYPT_DESC_SIZE   22
#define DS2_TRANSFORMED_BODY_SIZE 0x1F0
#define DS2_SAVED_STATE_SIZE 0x12C

#define DS2_KEY_MODE_AES128 1
#define DS2_KEY_MODE_AES256 2

#define DS2_MAX_PASSWORD_BYTES 16

typedef struct DS2DecryptState {
    uint8_t blob[DS2_SAVED_STATE_SIZE];
} DS2DecryptState;

typedef struct DS2DecryptDescriptor {
    int      key_mode;
    uint8_t  aux_16[16];
    uint16_t expected_check;
} DS2DecryptDescriptor;

int ds2_parse_decrypt_descriptor(const uint8_t *header, int header_size,
                               DS2DecryptDescriptor *desc);

int ds2_decrypt_init(DS2DecryptState *state, const DS2DecryptDescriptor *desc,
                     const uint8_t *password, int password_len);

void ds2_decrypt_record(DS2DecryptState *state, int key_mode,
                        uint8_t *record512);

#endif /* AVFORMAT_DS2CRYPTO_H */
