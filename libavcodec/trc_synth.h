/*
 * Sony TRC synthesis / postfilter
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

#ifndef AVCODEC_TRC_SYNTH_H
#define AVCODEC_TRC_SYNTH_H

#include <stdint.h>

#include "trc_unpack.h"

void trc_lpc_synth(int16_t *exc, const int16_t lpc[10], int len);
void trc_postfilter(int16_t *pcm, int16_t *exc, const int16_t lpc[10],
                    int16_t *formant_hist, int16_t *agc_mem, int rate, int len);
void trc_noise_fill(int16_t *exc_block, int block_off, int16_t *gain_idx,
                    int16_t *gain_val, uint32_t *noise_state);

#endif /* AVCODEC_TRC_SYNTH_H */
