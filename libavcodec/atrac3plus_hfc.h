/*
 * Sony AT-X high frequency compensation (HFC) for ATRAC3+
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

/**
 * @file
 * HFC rebuilds the uncoded upper QMF subbands of a channel block from the
 * coded ones. Like SBR, it copies low subbands up, shapes them with a coded
 * envelope and adds noise and tones. Its parameters come in ATRAC3+
 * extension units of type 0. Sony IC recorders use it in LCST MSV/DVF files
 * with quality codes 0x20 and 0x24.
 */

#ifndef AVCODEC_ATRAC3PLUS_HFC_H
#define AVCODEC_ATRAC3PLUS_HFC_H

#include <stdint.h>

#define HFC_BANDS      16
#define HFC_BAND_LEN   128
#define HFC_MAX_SLOTS  16
#define HFC_TONE_WORDS 18

typedef union Atrac3pHfcWord {
    int32_t i;
    float   f;
} Atrac3pHfcWord;

typedef struct Atrac3pHfcContext {
    int nch;
    int num_slots;
    int prev_num_slots;
    int slot_len;
    int base_band;
    int num_patches;
    int patch_start[HFC_BANDS];
    int patch_end[HFC_BANDS];
    int patch_map[HFC_BANDS];
    int prev_end;
    int num_groups;
    int groups[HFC_BANDS];
    int num_groups13;
    int groups13[HFC_BANDS];
    int no_parse;           ///< set by synthesis, cleared by a parsed payload
    int limit_level;
    int smooth_mode;
    int coupled;
    int mirror;
    uint16_t seed;
    int noise_sel;
    int noise_idx[HFC_BANDS];
    int env_mode[2];
    int prev_env_mode[2];
    int env_idx[2][HFC_BANDS];
    int prev_env_idx[2][HFC_BANDS];
    float env_gain[2][20][HFC_BANDS];
    float env_db[2][HFC_MAX_SLOTS][HFC_BANDS];
    int env_q[2][HFC_MAX_SLOTS][HFC_BANDS];
    float slot_gain[HFC_MAX_SLOTS];
    float recip[HFC_BANDS][19];
    int time_group[2];
    int env_offset[2];
    int band_sel[2][HFC_BANDS];
    int sel_grouped[2];
    int time_flags[2][HFC_MAX_SLOTS];
    float hist[2][HFC_BANDS][32];
    int tones_present;
    int tones_coupled;
    int tone_code[2];
    int tone_flag[2];
    int tone_count[2];
    int tone_index[2][2];
    Atrac3pHfcWord tone_rec[4][HFC_TONE_WORDS];
    int tone_sel[4];        ///< tone record per channel: [2 * ch] previous, [2 * ch + 1] current
} Atrac3pHfcContext;

/** Build the static Huffman tables; call once before any parsing. */
void ff_atrac3p_hfc_init_static(void);

/**
 * Set up the state the way the reference decoder does for a stream.
 *
 * @param nch     number of channels in the block (1 or 2)
 * @param base    lowest source subband
 * @param start   first rebuilt subband
 * @param end     initial last rebuilt subband
 * @param mirror  patch mode: 0 translate, 1 mirror
 * @return 0 or a negative error code for an invalid configuration
 */
int ff_atrac3p_hfc_init(Atrac3pHfcContext *s, int nch, int base, int start, int end, int mirror);

void ff_atrac3p_hfc_reset(Atrac3pHfcContext *s);

/**
 * Parse one HFC payload (the bytes of a type 0 extension unit).
 *
 * @param buf  payload followed by AV_INPUT_BUFFER_PADDING_SIZE zero bytes
 * @return 0 or a negative error code; the state is then partially updated,
 *         as in the reference decoder
 */
int ff_atrac3p_hfc_parse(Atrac3pHfcContext *s, const uint8_t *buf, int len);

/**
 * Rebuild the upper subbands in place.
 *
 * @param sb  subband buffers, sb[16 * ch + band], HFC_BAND_LEN samples each,
 *            at 16-bit sample scale; the coded subbands are delayed in place
 *            by 16 samples
 */
void ff_atrac3p_hfc_synth(Atrac3pHfcContext *s, float *sb[]);

#endif /* AVCODEC_ATRAC3PLUS_HFC_H */
