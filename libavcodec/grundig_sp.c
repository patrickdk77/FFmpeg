/*
 * Grundig DSS-SP (Digta CELP) audio decoder.
 * Copyright (c) 2026 Guillain d'Erceville <guillain@poulpe.us>
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

/*
 * Grundig DSS-SP ('\x06dss' / Digta SP) decoder, as found in Grundig DigtaSoft
 * .dss recordings and produced by the Grundig reference decoder.
 *
 * It is a CELP coder operating at a native 12 kHz sampling rate: each 41-byte
 * (328-bit) frame yields 288 samples (4 subframes x 72 samples).  Each frame
 * carries 14 reflection-coefficient indices (a lattice synthesis filter of
 * order 14), four differential pitch lags packed into a single 24-bit field,
 * and, per subframe, an adaptive (pitch) gain, a 31-bit fixed-codebook index
 * enumerating 7 pulses via cumulative binomial tables, a fixed-codebook gain
 * and 7 pulse signs.  Excitation = adaptive (pitch repeat x gain) + fixed
 * (7 signed pulses); an order-14 lattice synthesis filter and a de-emphasis
 * filter (y[n] = x[n] + 0.1*y[n-1]) follow.
 *
 * The Grundig device resamples this 12 kHz signal to 16 kHz with a 3:4
 * polyphase FIR for playback; that resampling is a device post-process and is
 * intentionally left to libswresample -- this decoder emits the codec's native
 * 12 kHz rate.
 *
 * The decoder uses f64 arithmetic and floor(x + 0.5) rounding to remain
 * bit-exact against the reference codec.
 */

#include <math.h>
#include <string.h>

#include "libavutil/channel_layout.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "get_bits.h"

#define GRUNDIG_FRAME_BYTES   41
#define GRUNDIG_FRAME_BITS    328
#define GRUNDIG_SUBFRAMES     4
#define GRUNDIG_SUBLEN        72
#define GRUNDIG_SAMPLES       (GRUNDIG_SUBFRAMES * GRUNDIG_SUBLEN) /* 288 */
#define GRUNDIG_ORDER         14
#define GRUNDIG_HIST          187

/* de-emphasis pole (y[n] = x[n] - DEEMPH_POLE*y[n-1], DEEMPH_POLE = -0.1) */
#define GRUNDIG_DEEMPH_POLE  (-0.1)

/* Auto-generated from gtables.json. Do not edit by hand. */

static const double grundig_lpc0[448] = {
    -0x1.fe33e3fffffffp-1, -0x1.fd2bce0000002p-1, -0x1.fc0b47fffffffp-1, -0x1.fad7340000001p-1,
    -0x1.f955420000000p-1, -0x1.f7602c0000001p-1, -0x1.f4f8520000000p-1, -0x1.f225ddfffffffp-1,
    -0x1.eec499fffffffp-1, -0x1.ea96d80000000p-1, -0x1.e5a15dfffffffp-1, -0x1.e00fe20000000p-1,
    -0x1.d96c340000000p-1, -0x1.d1d46e0000000p-1, -0x1.c900540000001p-1, -0x1.be6e720000001p-1,
    -0x1.b0674c0000000p-1, -0x1.9d1da20000001p-1, -0x1.8166adfffffffp-1, -0x1.5f06020000000p-1,
    -0x1.2fa5680000000p-1, -0x1.f824780000001p-2, -0x1.7dd0bdfffffffp-2, -0x1.e6704e0000001p-3,
    -0x1.6245940000000p-4, 0x1.c6b3600000001p-5, 0x1.98fd860000002p-3, 0x1.5fe3de0000000p-2,
    0x1.f58e500000001p-2, 0x1.3bf1160000000p-1, 0x1.8317660000000p-1, 0x1.b75ee80000001p-1,
    -0x1.adbc03fffffffp-1, -0x1.7ef4940000000p-1, -0x1.428fdc0000000p-1, -0x1.1160ce0000000p-1,
    -0x1.bb586c0000001p-2, -0x1.60660ffffffffp-2, -0x1.0721380000000p-2, -0x1.5da9ec0000000p-3,
    -0x1.78a905fffffffp-4, -0x1.37ef600000001p-6, 0x1.abd243fffffffp-5, 0x1.e4f033fffffffp-4,
    0x1.6d455e0000001p-3, 0x1.e5e583ffffffep-3, 0x1.3059120000000p-2, 0x1.6a41ddfffffffp-2,
    0x1.a1a15dfffffffp-2, 0x1.d1ba7c0000000p-2, 0x1.00a9420000000p-1, 0x1.17b15a0000000p-1,
    0x1.2cc92e0000000p-1, 0x1.41a6760000000p-1, 0x1.54adb00000000p-1, 0x1.6776540000000p-1,
    0x1.7936a40000000p-1, 0x1.8a84b00000000p-1, 0x1.9a8d720000000p-1, 0x1.aa8fb9fffffffp-1,
    0x1.bb1c680000000p-1, 0x1.c904380000000p-1, 0x1.d69727fffffffp-1, 0x1.e73ab80000000p-1,
    -0x1.b2ccd80000000p-1, -0x1.7a40860000000p-1, -0x1.473bca0000000p-1, -0x1.15d36c0000000p-1,
    -0x1.cfd8c60000000p-2, -0x1.723dadfffffffp-2, -0x1.1b4c320000000p-2, -0x1.8989b60000002p-3,
    -0x1.c985d40000000p-4, -0x1.c733640000000p-6, 0x1.df9df60000000p-5, 0x1.3a0d120000000p-3,
    0x1.00f64ffffffffp-2, 0x1.6c05720000000p-2, 0x1.d773300000000p-2, 0x1.1fdbec0000000p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.0b9e800000000p-1, -0x1.76369c0000000p-2, -0x1.026c2e0000000p-2, -0x1.402ef80000000p-3,
    -0x1.1efb400000000p-4, 0x1.6e5ad40000000p-8, 0x1.38ef380000000p-4, 0x1.26303e0000000p-3,
    0x1.a8dc940000000p-3, 0x1.1784760000000p-2, 0x1.592859fffffffp-2, 0x1.9db65c0000000p-2,
    0x1.e53cadfffffffp-2, 0x1.1d21580000000p-1, 0x1.49eb520000000p-1, 0x1.8344080000001p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.50d3e60000000p-1, -0x1.0e00b20000000p-1, -0x1.be732e0000000p-2, -0x1.6bde6c0000000p-2,
    -0x1.219eb1fffffffp-2, -0x1.baf243fffffffp-3, -0x1.34acd20000000p-3, -0x1.61e7f80000000p-4,
    -0x1.5943d80000000p-6, 0x1.5fa2d60000000p-5, 0x1.ba0d460000002p-4, 0x1.6591280000000p-3,
    0x1.fbc9b40000001p-3, 0x1.4daa180000000p-2, 0x1.acc45c0000001p-2, 0x1.150dec0000000p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.d5b16c0000001p-2, -0x1.4444da0000001p-2, -0x1.b7a37fffffffep-3, -0x1.0e74520000000p-3,
    -0x1.dae0a60000000p-5, 0x1.6c48ee0000003p-7, 0x1.333b200000000p-4, 0x1.161b8c0000000p-3,
    0x1.9160b7fffffffp-3, 0x1.05ae860000001p-2, 0x1.44301e0000000p-2, 0x1.86307bfffffffp-2,
    0x1.cbccd40000000p-2, 0x1.09921e0000000p-1, 0x1.32a9620000000p-1, 0x1.66a9260000000p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.f8d7ec0000000p-2, -0x1.824ee9ffffffep-2, -0x1.2f0cabfffffffp-2, -0x1.d23ea7ffffffep-3,
    -0x1.489fcc0000001p-3, -0x1.a3e7380000003p-4, -0x1.82b8480000000p-5, 0x1.b575800000000p-8,
    0x1.df0c5c0000000p-5, 0x1.c1d8000000004p-4, 0x1.4b3277ffffffep-3, 0x1.b51bd80000001p-3,
    0x1.1818480000001p-2, 0x1.5ece51fffffffp-2, 0x1.aca1100000001p-2, 0x1.0957740000000p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.ccad4bfffffffp-2, -0x1.33bec80000000p-2, -0x1.b08d040000002p-3, -0x1.228367fffffffp-3,
    -0x1.5a2dc40000000p-4, -0x1.0a5ba60000000p-5, 0x1.f2b7adffffffdp-7, 0x1.046bfe0000000p-4,
    0x1.c61c3a0000001p-4, 0x1.4636da0000000p-3, 0x1.ac942bfffffffp-3, 0x1.0c23fa0000001p-2,
    0x1.455159fffffffp-2, 0x1.8bfde80000000p-2, 0x1.e648b40000000p-2, 0x1.3a14900000000p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.5ad96e0000001p-2, -0x1.b667480000001p-3, -0x1.e1d3a7ffffffcp-4, -0x1.00391a0000000p-5,
    0x1.a3eae00000000p-5, 0x1.1bf95dfffffffp-3, 0x1.e7f4d80000001p-3, 0x1.74e2380000000p-2,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.1b22fe0000000p-2, -0x1.1da4e7fffffffp-3, -0x1.594e49fffffffp-5, 0x1.62ae2e0000000p-5,
    0x1.f84de40000005p-4, 0x1.a484380000001p-3, 0x1.3449c40000000p-2, 0x1.ba24940000000p-2,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.85109dfffffffp-2, -0x1.f3158dfffffffp-3, -0x1.1f432a0000000p-3, -0x1.b169ce0000000p-5,
    0x1.e08677fffffffp-6, 0x1.c5a8400000003p-4, 0x1.ad0d7e0000001p-3, 0x1.5c2e480000000p-2,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.71b4620000000p-2, -0x1.ceb98fffffffdp-3, -0x1.f53cf80000000p-4, -0x1.120f460000001p-5,
    0x1.9180cc0000000p-5, 0x1.0c2dda0000000p-3, 0x1.cd9895fffffffp-3, 0x1.66cdf40000000p-2,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.a2e261fffffffp-2, -0x1.210c4e0000000p-2, -0x1.76aa7ffffffffp-3, -0x1.9e0345fffffffp-4,
    -0x1.bd0fc20000000p-6, 0x1.8e7fdffffffffp-5, 0x1.17051a0000001p-3, 0x1.003039fffffffp-2,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.5f382c0000001p-2, -0x1.c33f480000001p-3, -0x1.f8f1d60000002p-4, -0x1.5f82460000001p-5,
    0x1.e576b5fffffffp-6, 0x1.9f2f040000001p-4, 0x1.776017fffffffp-3, 0x1.2f0731fffffffp-2,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
};

static const double grundig_lpc1[448] = {
    -0x1.fe33e3fffffffp-1, -0x1.fd2bce0000002p-1, -0x1.fc0b47fffffffp-1, -0x1.fad7340000001p-1,
    -0x1.f955420000000p-1, -0x1.f7602c0000001p-1, -0x1.f4f8520000000p-1, -0x1.f225ddfffffffp-1,
    -0x1.eec499fffffffp-1, -0x1.ea96d80000000p-1, -0x1.e5a15dfffffffp-1, -0x1.e00fe20000000p-1,
    -0x1.d96c340000000p-1, -0x1.d1d46e0000000p-1, -0x1.c900540000001p-1, -0x1.be6e720000001p-1,
    -0x1.b0674c0000000p-1, -0x1.9d1da20000001p-1, -0x1.8166adfffffffp-1, -0x1.5f06020000000p-1,
    -0x1.2fa5680000000p-1, -0x1.f824780000001p-2, -0x1.7dd0bdfffffffp-2, -0x1.e6704e0000001p-3,
    -0x1.6245940000000p-4, 0x1.c6b3600000001p-5, 0x1.98fd860000002p-3, 0x1.5fe3de0000000p-2,
    0x1.f58e500000001p-2, 0x1.3bf1160000000p-1, 0x1.8317660000000p-1, 0x1.b75ee80000001p-1,
    -0x1.adbc03fffffffp-1, -0x1.7ef4940000000p-1, -0x1.428fdc0000000p-1, -0x1.1160ce0000000p-1,
    -0x1.bb586c0000001p-2, -0x1.60660ffffffffp-2, -0x1.0721380000000p-2, -0x1.5da9ec0000000p-3,
    -0x1.78a905fffffffp-4, -0x1.37ef600000001p-6, 0x1.abd243fffffffp-5, 0x1.e4f033fffffffp-4,
    0x1.6d455e0000001p-3, 0x1.e5e583ffffffep-3, 0x1.3059120000000p-2, 0x1.6a41ddfffffffp-2,
    0x1.a1a15dfffffffp-2, 0x1.d1ba7c0000000p-2, 0x1.00a9420000000p-1, 0x1.17b15a0000000p-1,
    0x1.2cc92e0000000p-1, 0x1.41a6760000000p-1, 0x1.54adb00000000p-1, 0x1.6776540000000p-1,
    0x1.7936a40000000p-1, 0x1.8a84b00000000p-1, 0x1.9a8d720000000p-1, 0x1.aa8fb9fffffffp-1,
    0x1.bb1c680000000p-1, 0x1.c904380000000p-1, 0x1.d69727fffffffp-1, 0x1.e73ab80000000p-1,
    -0x1.b2ccd80000000p-1, -0x1.7a40860000000p-1, -0x1.473bca0000000p-1, -0x1.15d36c0000000p-1,
    -0x1.cfd8c60000000p-2, -0x1.723dadfffffffp-2, -0x1.1b4c320000000p-2, -0x1.8989b60000002p-3,
    -0x1.c985d40000000p-4, -0x1.c733640000000p-6, 0x1.df9df60000000p-5, 0x1.3a0d120000000p-3,
    0x1.00f64ffffffffp-2, 0x1.6c05720000000p-2, 0x1.d773300000000p-2, 0x1.1fdbec0000000p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.0b9e800000000p-1, -0x1.76369c0000000p-2, -0x1.026c2e0000000p-2, -0x1.402ef80000000p-3,
    -0x1.1efb400000000p-4, 0x1.6e5ad40000000p-8, 0x1.38ef380000000p-4, 0x1.26303e0000000p-3,
    0x1.a8dc940000000p-3, 0x1.1784760000000p-2, 0x1.592859fffffffp-2, 0x1.9db65c0000000p-2,
    0x1.e53cadfffffffp-2, 0x1.1d21580000000p-1, 0x1.49eb520000000p-1, 0x1.8344080000001p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.50d3e60000000p-1, -0x1.0e00b20000000p-1, -0x1.be732e0000000p-2, -0x1.6bde6c0000000p-2,
    -0x1.219eb1fffffffp-2, -0x1.baf243fffffffp-3, -0x1.34acd20000000p-3, -0x1.61e7f80000000p-4,
    -0x1.5943d80000000p-6, 0x1.5fa2d60000000p-5, 0x1.ba0d460000002p-4, 0x1.6591280000000p-3,
    0x1.fbc9b40000001p-3, 0x1.4daa180000000p-2, 0x1.acc45c0000001p-2, 0x1.150dec0000000p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.d5b16c0000001p-2, -0x1.4444da0000001p-2, -0x1.b7a37fffffffep-3, -0x1.0e74520000000p-3,
    -0x1.dae0a60000000p-5, 0x1.6c48ee0000003p-7, 0x1.333b200000000p-4, 0x1.161b8c0000000p-3,
    0x1.9160b7fffffffp-3, 0x1.05ae860000001p-2, 0x1.44301e0000000p-2, 0x1.86307bfffffffp-2,
    0x1.cbccd40000000p-2, 0x1.09921e0000000p-1, 0x1.32a9620000000p-1, 0x1.66a9260000000p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.f8d7ec0000000p-2, -0x1.824ee9ffffffep-2, -0x1.2f0cabfffffffp-2, -0x1.d23ea7ffffffep-3,
    -0x1.489fcc0000001p-3, -0x1.a3e7380000003p-4, -0x1.82b8480000000p-5, 0x1.b575800000000p-8,
    0x1.df0c5c0000000p-5, 0x1.c1d8000000004p-4, 0x1.4b3277ffffffep-3, 0x1.b51bd80000001p-3,
    0x1.1818480000001p-2, 0x1.5ece51fffffffp-2, 0x1.aca1100000001p-2, 0x1.0957740000000p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.ccad4bfffffffp-2, -0x1.33bec80000000p-2, -0x1.b08d040000002p-3, -0x1.228367fffffffp-3,
    -0x1.5a2dc40000000p-4, -0x1.0a5ba60000000p-5, 0x1.f2b7adffffffdp-7, 0x1.046bfe0000000p-4,
    0x1.c61c3a0000001p-4, 0x1.4636da0000000p-3, 0x1.ac942bfffffffp-3, 0x1.0c23fa0000001p-2,
    0x1.455159fffffffp-2, 0x1.8bfde80000000p-2, 0x1.e648b40000000p-2, 0x1.3a14900000000p-1,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.5ad96e0000001p-2, -0x1.b667480000001p-3, -0x1.e1d3a7ffffffcp-4, -0x1.00391a0000000p-5,
    0x1.a3eae00000000p-5, 0x1.1bf95dfffffffp-3, 0x1.e7f4d80000001p-3, 0x1.74e2380000000p-2,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.1b22fe0000000p-2, -0x1.1da4e7fffffffp-3, -0x1.594e49fffffffp-5, 0x1.62ae2e0000000p-5,
    0x1.f84de40000005p-4, 0x1.a484380000001p-3, 0x1.3449c40000000p-2, 0x1.ba24940000000p-2,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.85109dfffffffp-2, -0x1.f3158dfffffffp-3, -0x1.1f432a0000000p-3, -0x1.b169ce0000000p-5,
    0x1.e08677fffffffp-6, 0x1.c5a8400000003p-4, 0x1.ad0d7e0000001p-3, 0x1.5c2e480000000p-2,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.71b4620000000p-2, -0x1.ceb98fffffffdp-3, -0x1.f53cf80000000p-4, -0x1.120f460000001p-5,
    0x1.9180cc0000000p-5, 0x1.0c2dda0000000p-3, 0x1.cd9895fffffffp-3, 0x1.66cdf40000000p-2,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.a2e261fffffffp-2, -0x1.210c4e0000000p-2, -0x1.76aa7ffffffffp-3, -0x1.9e0345fffffffp-4,
    -0x1.bd0fc20000000p-6, 0x1.8e7fdffffffffp-5, 0x1.17051a0000001p-3, 0x1.003039fffffffp-2,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    -0x1.fe5eb80000000p-3, -0x1.2e71f00000000p-4, 0x1.f33c9e0000001p-5, 0x1.a30cf00000000p-3,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, 0x0.0p+0, 0x0.0p+0,
};

static const double grundig_pitch_gain[32] = {
    0x1.9800000000000p-5, 0x1.ce00000000000p-4, 0x1.6800000000000p-3, 0x1.e800000000000p-3,
    0x1.3480000000000p-2, 0x1.7500000000000p-2, 0x1.b580000000000p-2, 0x1.f600000000000p-2,
    0x1.1b40000000000p-1, 0x1.3b40000000000p-1, 0x1.5b80000000000p-1, 0x1.7bc0000000000p-1,
    0x1.9c00000000000p-1, 0x1.bc40000000000p-1, 0x1.dc40000000000p-1, 0x1.fc80000000000p-1,
    0x1.0e60000000000p+0, 0x1.1e80000000000p+0, 0x1.2ea0000000000p+0, 0x1.3ec0000000000p+0,
    0x1.4ec0000000000p+0, 0x1.5ee0000000000p+0, 0x1.6f00000000000p+0, 0x1.7f20000000000p+0,
    0x1.8f40000000000p+0, 0x1.9f60000000000p+0, 0x1.af60000000000p+0, 0x1.bf80000000000p+0,
    0x1.cfa0000000000p+0, 0x1.dfc0000000000p+0, 0x1.efe0000000000p+0, 0x1.0000000000000p+1,
};

static const double grundig_fcb_lead[64] = {
    0x0.0p+0, 0x1.0000000000000p+2, 0x1.0000000000000p+3, 0x1.a000000000000p+3,
    0x1.1000000000000p+4, 0x1.6000000000000p+4, 0x1.a000000000000p+4, 0x1.f000000000000p+4,
    0x1.1800000000000p+5, 0x1.4000000000000p+5, 0x1.6000000000000p+5, 0x1.8000000000000p+5,
    0x1.a800000000000p+5, 0x1.d000000000000p+5, 0x1.f800000000000p+5, 0x1.1400000000000p+6,
    0x1.3000000000000p+6, 0x1.4c00000000000p+6, 0x1.6c00000000000p+6, 0x1.8c00000000000p+6,
    0x1.b400000000000p+6, 0x1.dc00000000000p+6, 0x1.0400000000000p+7, 0x1.1c00000000000p+7,
    0x1.3600000000000p+7, 0x1.5400000000000p+7, 0x1.7200000000000p+7, 0x1.9600000000000p+7,
    0x1.bc00000000000p+7, 0x1.e400000000000p+7, 0x1.0900000000000p+8, 0x1.2200000000000p+8,
    0x1.3d00000000000p+8, 0x1.5a00000000000p+8, 0x1.7a00000000000p+8, 0x1.9e00000000000p+8,
    0x1.c400000000000p+8, 0x1.ee00000000000p+8, 0x1.0e00000000000p+9, 0x1.2780000000000p+9,
    0x1.4300000000000p+9, 0x1.6100000000000p+9, 0x1.8180000000000p+9, 0x1.a580000000000p+9,
    0x1.cd00000000000p+9, 0x1.f780000000000p+9, 0x1.1340000000000p+10, 0x1.2d00000000000p+10,
    0x1.4900000000000p+10, 0x1.6780000000000p+10, 0x1.8900000000000p+10, 0x1.adc0000000000p+10,
    0x1.d5c0000000000p+10, 0x1.00a0000000000p+11, 0x1.1880000000000p+11, 0x1.32a0000000000p+11,
    0x1.4f40000000000p+11, 0x1.6e60000000000p+11, 0x1.9080000000000p+11, 0x1.b5c0000000000p+11,
    0x1.de80000000000p+11, 0x1.0580000000000p+12, 0x1.1de0000000000p+12, 0x1.3880000000000p+12,
};

static const double grundig_pulse_amp[8] = {
    -0x1.e738000000000p-1, -0x1.5c04000000000p-1, -0x1.a1a0000000000p-2, -0x1.1670000000000p-3,
    0x1.1670000000000p-3, 0x1.a1a0000000000p-2, 0x1.5c04000000000p-1, 0x1.e738000000000p-1,
};

static const double grundig_unvoiced_gain[32] = {
    0x0.0p+0, 0x1.4000000000000p-13, 0x1.4000000000000p-12, 0x1.e000000000000p-12,
    0x1.5000000000000p-11, 0x1.c000000000000p-11, 0x1.1800000000000p-10, 0x1.5000000000000p-10,
    0x1.9000000000000p-10, 0x1.d800000000000p-10, 0x1.1400000000000p-9, 0x1.3c00000000000p-9,
    0x1.6800000000000p-9, 0x1.9800000000000p-9, 0x1.cc00000000000p-9, 0x1.0200000000000p-8,
    0x1.2000000000000p-8, 0x1.4000000000000p-8, 0x1.6400000000000p-8, 0x1.8a00000000000p-8,
    0x1.b400000000000p-8, 0x1.e000000000000p-8, 0x1.0800000000000p-7, 0x1.2200000000000p-7,
    0x1.3e00000000000p-7, 0x1.5c00000000000p-7, 0x1.7d00000000000p-7, 0x1.a100000000000p-7,
    0x1.c700000000000p-7, 0x1.f000000000000p-7, 0x1.0e80000000000p-6, 0x1.2680000000000p-6,
};

static const int32_t grundig_cum[576] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
    36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
    48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71,
    0, 0, 1, 3, 6, 10, 15, 21, 28, 36, 45, 55,
    66, 78, 91, 105, 120, 136, 153, 171, 190, 210, 231, 253,
    276, 300, 325, 351, 378, 406, 435, 465, 496, 528, 561, 595,
    630, 666, 703, 741, 780, 820, 861, 903, 946, 990, 1035, 1081,
    1128, 1176, 1225, 1275, 1326, 1378, 1431, 1485, 1540, 1596, 1653, 1711,
    1770, 1830, 1891, 1953, 2016, 2080, 2145, 2211, 2278, 2346, 2415, 2485,
    0, 0, 0, 1, 4, 10, 20, 35, 56, 84, 120, 165,
    220, 286, 364, 455, 560, 680, 816, 969, 1140, 1330, 1540, 1771,
    2024, 2300, 2600, 2925, 3276, 3654, 4060, 4495, 4960, 5456, 5984, 6545,
    7140, 7770, 8436, 9139, 9880, 10660, 11480, 12341, 13244, 14190, 15180, 16215,
    17296, 18424, 19600, 20825, 22100, 23426, 24804, 26235, 27720, 29260, 30856, 32509,
    34220, 35990, 37820, 39711, 41664, 43680, 45760, 47905, 50116, 52394, 54740, 57155,
    0, 0, 0, 0, 1, 5, 15, 35, 70, 126, 210, 330,
    495, 715, 1001, 1365, 1820, 2380, 3060, 3876, 4845, 5985, 7315, 8855,
    10626, 12650, 14950, 17550, 20475, 23751, 27405, 31465, 35960, 40920, 46376, 52360,
    58905, 66045, 73815, 82251, 91390, 101270, 111930, 123410, 135751, 148995, 163185, 178365,
    194580, 211876, 230300, 249900, 270725, 292825, 316251, 341055, 367290, 395010, 424270, 455126,
    487635, 521855, 557845, 595665, 635376, 677040, 720720, 766480, 814385, 864501, 916895, 971635,
    0, 0, 0, 0, 0, 1, 6, 21, 56, 126, 252, 462,
    792, 1287, 2002, 3003, 4368, 6188, 8568, 11628, 15504, 20349, 26334, 33649,
    42504, 53130, 65780, 80730, 98280, 118755, 142506, 169911, 201376, 237336, 278256, 324632,
    376992, 435897, 501942, 575757, 658008, 749398, 850668, 962598, 1086008, 1221759, 1370754, 1533939,
    1712304, 1906884, 2118760, 2349060, 2598960, 2869685, 3162510, 3478761, 3819816, 4187106, 4582116, 5006386,
    5461512, 5949147, 6471002, 7028847, 7624512, 8259888, 8936928, 9657648, 10424128, 11238513, 12103014, 13019909,
    0, 0, 0, 0, 0, 0, 1, 7, 28, 84, 210, 462,
    924, 1716, 3003, 5005, 8008, 12376, 18564, 27132, 38760, 54264, 74613, 100947,
    134596, 177100, 230230, 296010, 376740, 475020, 593775, 736281, 906192, 1107568, 1344904, 1623160,
    1947792, 2324784, 2760681, 3262623, 3838380, 4496388, 5245786, 6096454, 7059052, 8145060, 9366819, 10737573,
    12271512, 13983816, 15890700, 18009460, 20358520, 22957480, 25827165, 28989675, 32468436, 36288252, 40475358, 45057474,
    50063860, 55525372, 61474519, 67945521, 74974368, 82598880, 90858768, 99795696, 109453344, 119877472, 131115985, 143218999,
    0, 0, 0, 0, 0, 0, 0, 1, 8, 36, 120, 330,
    792, 1716, 3432, 6435, 11440, 19448, 31824, 50388, 77520, 116280, 170544, 245157,
    346104, 480700, 657800, 888030, 1184040, 1560780, 2035800, 2629575, 3365856, 4272048, 5379616, 6724520,
    8347680, 10295472, 12620256, 15380937, 18643560, 22481940, 26978328, 32224114, 38320568, 45379620, 53524680, 62891499,
    73629072, 85900584, 99884400, 115775100, 133784560, 154143080, 177100560, 202927725, 231917400, 264385836, 300674088, 341149446,
    386206920, 436270780, 491796152, 553270671, 621216192, 696190560, 778789440, 869648208, 969443904, 1078897248, 1198774720, 1329890705,
};

static const uint8_t grundig_widths[GRUNDIG_ORDER] = {
    5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3,
};

typedef struct GrundigSPContext {
    double hist[GRUNDIG_HIST + 1]; /* excitation history; base read index 187 */
    double syn[16];                /* lattice b-state (order 14, +slack)      */
    double deemph;                 /* de-emphasis memory                      */
    uint16_t rnd;                  /* unvoiced LCG state                      */
} GrundigSPContext;

static av_cold int grundig_sp_decode_init(AVCodecContext *avctx)
{
    GrundigSPContext *p = avctx->priv_data;

    av_channel_layout_uninit(&avctx->ch_layout);
    avctx->ch_layout      = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
    avctx->sample_fmt     = AV_SAMPLE_FMT_S16;
    if (!avctx->sample_rate)
        avctx->sample_rate = 12000;

    memset(p, 0, sizeof(*p));
    return 0;
}

/* Build a voiced subframe excitation: adaptive (pitch) part + 7 fixed pulses. */
static void grundig_build_voiced(GrundigSPContext *p, double *out, int lag,
                                 int gain_idx, uint32_t fixed31,
                                 const int *pulse)
{
    int i, j, k, sv, col, hi;
    int64_t row, val;
    int pos[7];
    double lead;
    double g = grundig_pitch_gain[gain_idx];

    /* adaptive codebook: periodic repetition of the excitation history */
    j = 0; sv = 1;
    while (j < GRUNDIG_SUBLEN) {
        hi = sv * lag;
        while (j < hi && j < GRUNDIG_SUBLEN) {
            out[j] = p->hist[GRUNDIG_HIST + (j - sv * lag)] * g;
            j++;
        }
        sv++;
    }

    /* fixed codebook: decode 7 pulse positions from the 31-bit index via the
     * cumulative binomial table (base-151/.. enumeration). */
    val = fixed31;
    if (val > 0x57cddec7)
        val = 0x57cddec7;
    col = 0x47; row = 0x1f8;
    for (k = 0; k < 7; k++) {
        while (val < (int64_t)grundig_cum[col + row])
            col--;
        pos[k] = col;
        val   -= grundig_cum[col + row];
        row   -= 0x48;
    }

    lead = grundig_fcb_lead[pulse[0]];
    for (k = 0; k < 7; k++)
        out[pos[k]] += grundig_pulse_amp[pulse[1 + k]] * lead;

    /* shift history and append this subframe's excitation */
    for (i = 0; i < 115; i++)
        p->hist[i] = p->hist[i + 72];
    for (i = 0; i < 72; i++)
        p->hist[115 + i] = out[i];
}

/* Build an unvoiced subframe excitation from the LCG noise generator.
 * Reserved for the unvoiced-frame path (not yet wired up: the decoder
 * currently treats every frame as voiced). */
static av_unused void grundig_build_unvoiced(GrundigSPContext *p, double *out,
                                             int gain_idx)
{
    int i, s;
    double sc = grundig_unvoiced_gain[gain_idx];

    for (i = 0; i < GRUNDIG_SUBLEN; i++) {
        p->rnd = p->rnd * 0x209 + 0x103;
        s = p->rnd >= 0x8000 ? (int)p->rnd - 0x10000 : (int)p->rnd;
        out[i] = s * sc;
    }
    for (i = 0; i < 115; i++)
        p->hist[i] = p->hist[i + 72];
    for (i = 0; i < 72; i++)
        p->hist[115 + i] = out[i];
}

/* Order-14 lattice synthesis filter. */
static void grundig_lattice(GrundigSPContext *p, double *out,
                            const double *exc, const double *refl)
{
    int n, m;
    double *b = p->syn;

    for (n = 0; n < GRUNDIG_SUBLEN; n++) {
        double f = exc[n] - refl[GRUNDIG_ORDER - 1] * b[GRUNDIG_ORDER - 1];
        for (m = GRUNDIG_ORDER - 2; m >= 0; m--) {
            f      = f - refl[m] * b[m];
            b[m + 1] = refl[m] * f + b[m];
        }
        b[0]   = f;
        out[n] = f;
    }
}

/* De-emphasis + rounding/clamping to int16. */
static void grundig_deemph(GrundigSPContext *p, int16_t *out, const double *synth,
                           int n)
{
    int i;
    double prev = p->deemph;

    for (i = 0; i < n; i++) {
        double y = synth[i] - prev * GRUNDIG_DEEMPH_POLE;
        prev = y;
        if (y <= 32767.5) {
            if (y >= -32767.5)
                out[i] = (int16_t)floor(y + 0.5);
            else
                out[i] = -32767;
        } else {
            out[i] = 32767;
        }
    }
    p->deemph = prev;
}

static void grundig_decode_one_frame(GrundigSPContext *p, int16_t *out,
                                     GetBitContext *gb)
{
    int i, sf;
    int lsf[GRUNDIG_ORDER];
    double refl[GRUNDIG_ORDER];
    double synth[GRUNDIG_SAMPLES];
    int gains[GRUNDIG_SUBFRAMES];
    uint32_t fixed31[GRUNDIG_SUBFRAMES];
    int pulses[GRUNDIG_SUBFRAMES][8];
    int pit[GRUNDIG_SUBFRAMES];
    int prev_lag;
    uint32_t v;

    /* 14 reflection-coefficient indices (mode 0: always voiced). */
    for (i = 0; i < GRUNDIG_ORDER; i++)
        lsf[i] = get_bits(gb, grundig_widths[i]);

    for (sf = 0; sf < GRUNDIG_SUBFRAMES; sf++) {
        gains[sf]     = get_bits(gb, 5);
        fixed31[sf]   = get_bits_long(gb, 31);
        pulses[sf][0] = get_bits(gb, 6);
        for (i = 1; i < 8; i++)
            pulses[sf][i] = get_bits(gb, 3);
    }

    /* 24-bit field -> 4 differential pitch lags (base 151 then base 48). */
    v = get_bits_long(gb, 24);
    pit[0] = v % 0x97; v /= 0x97;
    for (i = 1; i < 4; i++) {
        pit[i] = v % 0x30; v /= 0x30;
    }

    for (i = 0; i < GRUNDIG_ORDER; i++)
        refl[i] = grundig_lpc0[lsf[i] + 32 * i];

    prev_lag = 0;
    for (sf = 0; sf < GRUNDIG_SUBFRAMES; sf++) {
        double exc[GRUNDIG_SUBLEN];
        int lag;

        if (sf == 0) {
            lag = pit[0] + 0x24;
        } else if (prev_lag < 0xa3) {
            int base = prev_lag - 0x17;
            if (base < 0x24)
                base = 0x24;
            lag = pit[sf] + base;
        } else {
            lag = pit[sf] + 0x8b;
        }
        prev_lag = lag;

        grundig_build_voiced(p, exc, lag, gains[sf], fixed31[sf], pulses[sf]);
        grundig_lattice(p, synth + sf * GRUNDIG_SUBLEN, exc, refl);
    }

    grundig_deemph(p, out, synth, GRUNDIG_SAMPLES);
}

static int grundig_sp_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                                   int *got_frame_ptr, AVPacket *avpkt)
{
    GrundigSPContext *p = avctx->priv_data;
    GetBitContext gb;
    int16_t *out;
    int ret;

    if (avpkt->size < GRUNDIG_FRAME_BYTES) {
        if (avpkt->size)
            av_log(avctx, AV_LOG_WARNING,
                   "Expected %d bytes, got %d - skipping packet.\n",
                   GRUNDIG_FRAME_BYTES, avpkt->size);
        *got_frame_ptr = 0;
        return AVERROR_INVALIDDATA;
    }

    if ((ret = init_get_bits8(&gb, avpkt->data, GRUNDIG_FRAME_BYTES)) < 0)
        return ret;

    frame->nb_samples = GRUNDIG_SAMPLES;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    out = (int16_t *)frame->data[0];
    grundig_decode_one_frame(p, out, &gb);

    *got_frame_ptr = 1;
    return GRUNDIG_FRAME_BYTES;
}

const FFCodec ff_grundig_sp_decoder = {
    .p.name         = "grundig_sp",
    CODEC_LONG_NAME("Grundig DSS-SP (Digta CELP)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_GRUNDIG_SP,
    .priv_data_size = sizeof(GrundigSPContext),
    .init           = grundig_sp_decode_init,
    FF_CODEC_DECODE_CB(grundig_sp_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_CHANNEL_CONF,
};
