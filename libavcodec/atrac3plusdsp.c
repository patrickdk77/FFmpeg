/*
 * ATRAC3+ compatible decoder
 *
 * Copyright (c) 2010-2013 Maxim Poliakovski
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
 *  @file
 *  DSP functions for ATRAC3+ decoder.
 */

#include <math.h>

#include "libavutil/float_dsp.h"
#include "libavutil/libm.h"
#include "libavutil/mem_internal.h"

#include "atrac3plus.h"

/**
 *  Map quant unit number to its position in the spectrum.
 *  To get the number of spectral lines in each quant unit do the following:
 *  num_specs = qu_to_spec_pos[i+1] - qu_to_spec_pos[i]
 */
const uint16_t ff_atrac3p_qu_to_spec_pos[33] = {
      0,    16,   32,   48,   64,   80,   96,  112,
    128,   160,  192,  224,  256,  288,  320,  352,
    384,   448,  512,  576,  640,  704,  768,  896,
    1024, 1152, 1280, 1408, 1536, 1664, 1792, 1920,
    2048
};

/* Scalefactors table. */
/* Approx. Equ: pow(2.0, (i - 16.0 + 0.501783948) / 3.0) */
const float ff_atrac3p_sf_tab[64] = {
    0.027852058,  0.0350914, 0.044212341, 0.055704117,  0.0701828,
    0.088424683, 0.11140823,   0.1403656,  0.17684937, 0.22281647, 0.2807312, 0.35369873,
    0.44563293,   0.5614624,  0.70739746,  0.89126587,  1.1229248, 1.4147949,  1.7825317,
    2.2458496,    2.8295898,   3.5650635,   4.4916992,  5.6591797,  7.130127,  8.9833984,
    11.318359,    14.260254,   17.966797,   22.636719,  28.520508, 35.933594,  45.273438,
    57.041016,    71.867188,   90.546875,   114.08203,  143.73438, 181.09375,  228.16406,
    287.46875,     362.1875,   456.32812,    574.9375,    724.375, 912.65625,   1149.875,
    1448.75,      1825.3125,     2299.75,      2897.5,   3650.625,    4599.5,     5795.0,
    7301.25,         9199.0,     11590.0,     14602.5,    18398.0,   23180.0,    29205.0,
    36796.0,        46360.0,     58410.0
};

/* Mantissa table. */
/* pow(10, x * log10(2) + 0.05) / 2 / ([1,2,3,5,7,15,31] + 0.5) */
const float ff_atrac3p_mant_tab[8] = {
    0.0,
    0.74801636,
    0.44882202,
    0.32058716,
    0.20400238,
    0.1496048,
    0.07239151,
    0.035619736
};

#define ATRAC3P_MDCT_SIZE (ATRAC3P_SUBBAND_SAMPLES * 2)

#define TWOPI (2 * M_PI)

#define DEQUANT_PHASE(ph) (((ph) & 0x1F) << 6)

static DECLARE_ALIGNED(32, float, sine_table)[2048]; ///< wave table
static DECLARE_ALIGNED(32, float, hann_window)[256]; ///< Hann windowing function
static float amp_sf_tab[64];   ///< scalefactors for quantized amplitudes

/* IMDCT twiddles: [0..62] FFT stages, [63..126] pre-rotation */
static float imdct_cos[127], imdct_sin[127];
static float imdct_win[4][ATRAC3P_MDCT_SIZE]; ///< windows indexed by wind_id
/* gain interpolation steps 2^(-(d + 1) * (3 - j) / 4) in Q15, for any level step */
static float gain_interp[31][3];

/* position of each IMDCT output pair in the FFT working buffer */
static const uint8_t imdct_order[128] = {
      0,  64,  96,  32,  48, 112,  80,  16,  24,  88, 120,  56,  40, 104,  72,   8,
     12,  76, 108,  44,  60, 124,  92,  28,  20,  84, 116,  52,  36, 100,  68,   4,
      6,  70, 102,  38,  54, 118,  86,  22,  30,  94, 126,  62,  46, 110,  78,  14,
     10,  74, 106,  42,  58, 122,  90,  26,  18,  82, 114,  50,  34,  98,  66,   2,
      3,  67,  99,  35,  51, 115,  83,  19,  27,  91, 123,  59,  43, 107,  75,  11,
     15,  79, 111,  47,  63, 127,  95,  31,  23,  87, 119,  55,  39, 103,  71,   7,
      5,  69, 101,  37,  53, 117,  85,  21,  29,  93, 125,  61,  45, 109,  77,  13,
      9,  73, 105,  41,  57, 121,  89,  25,  17,  81, 113,  49,  33,  97,  65,   1,
};

static av_cold void init_imdct_tables(void)
{
    int i, m, r, d, j;

    for (m = 1; m <= 32; m <<= 1)
        for (r = 0; r < m; r++) {
            double a = M_PI * (1 + 4 * r) / (4 * m);
            imdct_cos[m - 1 + r] = cos(a);
            imdct_sin[m - 1 + r] = sin(a);
        }
    for (i = 0; i < 64; i++) {
        imdct_cos[63 + i] = cos((i + 0.25) * M_PI / 128);
        imdct_sin[63 + i] = sin((i + 0.25) * M_PI / 128);
    }

    for (i = 0; i < ATRAC3P_MDCT_SIZE; i++) {
        float w = sin((i + 0.5) * M_PI / ATRAC3P_MDCT_SIZE);
        float s = i < 32 ? 0.0f : i < 96 ? sin((i - 32 + 0.5) * M_PI / 128) : 1.0f;
        int  rev = ATRAC3P_MDCT_SIZE - 1 - i;
        float s2 = rev < 32 ? 0.0f : rev < 96 ? sin((rev - 32 + 0.5) * M_PI / 128) : 1.0f;

        imdct_win[0][i] = w;
        imdct_win[1][i] = i < 128 ? w : s2;
        imdct_win[2][i] = i < 128 ? s : w;
        imdct_win[3][i] = i < 128 ? s : s2;
    }

    for (d = 0; d < 31; d++)
        for (j = 0; j < 3; j++)
            gain_interp[d][j] = floor(exp2(-(d + 1) * (3 - j) / 4.0) * 32768 + 0.5) / 32768;
}

av_cold void ff_atrac3p_init_dsp_static(void)
{
    float amp_base[4] = { 0.5f };
    int i;

    /* The reference tables use 2 * pi rounded to float as the angle step. */
    for (i = 0; i < 2048; i++)
        sine_table[i] = sin(i * (double)(float)(TWOPI / 2048));

    for (i = 0; i < 256; i++)
        hann_window[i] = (1.0 - cos(i / 256.0 * (double)(float)TWOPI)) * 0.5;

    /* 2^((i - 3) / 4) built from Q15 quarter steps, as in the reference */
    for (i = 1; i < 4; i++)
        amp_base[i] = floor(exp2(-(4 - i) / 4.0) * 32768 + 0.5) / 32768;
    for (i = 0; i < 64; i++)
        amp_sf_tab[i] = ldexpf(amp_base[(i + 1) & 3], (i + 1) >> 2);

    init_imdct_tables();
}

/* amplitude levels for amplitude mode 0, in units of 2^-17 */
static const int32_t amp_idx_tab[16] = {
      8663,  17326,  25989,  34652,  43316,  51978,  60642,  69304,
     77968,  86632,  95292, 103956, 112620, 121284, 129944, 138608,
};

/* steep Hann fade, quantized as in the reference decoder */
static const float steep_fade[4] = { 0.0f, 0.14642334f, 0.5f, 0.85357666f };

/* sample position of the envelope start/end within the 256-sample window */
static int env_start(const Atrac3pWaveEnvelope *env)
{
    return env->has_start_point ? env->start_pos << 2 : 0;
}

static int env_stop(const Atrac3pWaveEnvelope *env)
{
    return env->has_stop_point ? FFMIN((env->stop_pos + 1) << 2, 256) : 256;
}

/**
 *  Synthesize sine waves according to given parameters.
 *
 *  @param[in]    synth_param   ptr to common synthesis parameters
 *  @param[in]    waves_info    parameters for each sine wave
 *  @param[in]    envelope      envelope data for all waves in a group
 *  @param[in]    fdsp          ptr to floating-point DSP context
 *  @param[in]    invert_phase  flag indicating 180° phase shift
 *  @param[in]    reg_offset    region offset for trimming envelope data
 *  @param[out]   out           receives synthesized data
 */
static void waves_synth(Atrac3pWaveSynthParams *synth_param,
                        Atrac3pWavesData *waves_info,
                        Atrac3pWaveEnvelope *envelope,
                        AVFloatDSPContext *fdsp,
                        int invert_phase, int reg_offset, float *out)
{
    DECLARE_ALIGNED(32, float, env)[256];
    int i, wn, inc, pos;
    float amp;
    Atrac3pWaveParam *wave_param = &synth_param->waves[waves_info->start_index];

    for (wn = 0; wn < waves_info->num_wavs; wn++, wave_param++) {
        /* amplitude dequantization */
        amp = amp_sf_tab[wave_param->amp_sf];
        if (!synth_param->amplitude_mode)
            amp = (double)amp * (amp_idx_tab[wave_param->amp_index] / 131072.0);

        inc = wave_param->freq_index;
        pos = DEQUANT_PHASE(wave_param->phase_index) - (reg_offset ^ 128) * inc & 2047;

        /* waveform generation */
        for (i = 0; i < 128; i++) {
            out[i] += sine_table[pos] * (double)amp;
            pos     = (pos + inc) & 2047;
        }
    }

    /* invert phase if requested */
    if (invert_phase)
        fdsp->vector_fmul_scalar(out, out, -1.0f, 128);

    /* envelope over the whole window, steep fades at its start and end */
    for (i = 0; i < 256; i++)
        env[i] = 1.0f;
    if (envelope->has_start_point) {
        pos = env_start(envelope);
        memset(env, 0, pos * sizeof(*env));
        memcpy(&env[pos], steep_fade, sizeof(steep_fade));
    }
    if (envelope->has_stop_point) {
        pos = env_stop(envelope);
        for (i = 0; i < 4; i++)
            env[pos - 1 - i] = steep_fade[i];
        memset(&env[pos], 0, (256 - pos) * sizeof(*env));
    }
    fdsp->vector_fmul(out, out, &env[reg_offset], 128);
}

void ff_atrac3p_generate_tones(Atrac3pChanUnitCtx *ch_unit, AVFloatDSPContext *fdsp,
                               int ch_num, int sb, float *out)
{
    DECLARE_ALIGNED(32, float, wavreg1)[128] = { 0 };
    DECLARE_ALIGNED(32, float, wavreg2)[128] = { 0 };
    int i;
    Atrac3pWavesData *tones_now  = &ch_unit->channels[ch_num].tones_info_prev[sb];
    Atrac3pWavesData *tones_next = &ch_unit->channels[ch_num].tones_info[sb];

    /* reconstruct full envelopes for both overlapping regions
     * from truncated bitstream data */
    if (tones_next->pend_env.has_start_point &&
        tones_next->pend_env.start_pos < tones_next->pend_env.stop_pos) {
        tones_next->curr_env.has_start_point = 1;
        tones_next->curr_env.start_pos       = tones_next->pend_env.start_pos + 32;
    } else if (tones_now->pend_env.has_start_point) {
        tones_next->curr_env.has_start_point = 1;
        tones_next->curr_env.start_pos       = tones_now->pend_env.start_pos;
    } else {
        tones_next->curr_env.has_start_point = 0;
        tones_next->curr_env.start_pos       = 0;
    }

    if (tones_now->pend_env.has_stop_point &&
        tones_now->pend_env.stop_pos >= tones_next->curr_env.start_pos) {
        tones_next->curr_env.has_stop_point = 1;
        tones_next->curr_env.stop_pos       = tones_now->pend_env.stop_pos;
    } else if (tones_next->pend_env.has_stop_point) {
        tones_next->curr_env.has_stop_point = 1;
        tones_next->curr_env.stop_pos       = tones_next->pend_env.stop_pos + 32;
    } else {
        tones_next->curr_env.has_stop_point = 0;
        tones_next->curr_env.stop_pos       = 64;
    }

    /* synthesize waves for both overlapping regions */
    if (tones_now->num_wavs)
        waves_synth(ch_unit->waves_info_prev, tones_now, &tones_now->curr_env,
                    fdsp, ch_unit->waves_info_prev->invert_phase[sb] & ch_num,
                    128, wavreg1);

    if (tones_next->num_wavs)
        waves_synth(ch_unit->waves_info, tones_next, &tones_next->curr_env, fdsp,
                    ch_unit->waves_info->invert_phase[sb] & ch_num, 0, wavreg2);

    /* Hann windowing for overlapping or non-faded wave signals */
    if (tones_now->num_wavs && tones_next->num_wavs &&
        env_start(&tones_next->curr_env) <= env_stop(&tones_now->curr_env) - 128) {
        fdsp->vector_fmul(wavreg1, wavreg1, &hann_window[128], 128);
        fdsp->vector_fmul(wavreg2, wavreg2,  hann_window,      128);
    } else {
        if (tones_now->num_wavs && !tones_now->curr_env.has_stop_point)
            fdsp->vector_fmul(wavreg1, wavreg1, &hann_window[128], 128);

        if (tones_next->num_wavs && !tones_next->curr_env.has_start_point)
            fdsp->vector_fmul(wavreg2, wavreg2, hann_window, 128);
    }

    /* Overlap and add to residual */
    for (i = 0; i < 128; i++)
        out[i] += wavreg1[i] + wavreg2[i];
}

static const uint8_t subband_to_powgrp[ATRAC3P_SUBBANDS] = {
    0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4
};

/* noise table for power compensation, in units of 1/32768 */
static const int16_t noise_tab[1024] = {
      -445,  -1833,    556,  -4873,  -8655,  -3242,   8429,    658, -23670, -14533,  -7532,   5516,
     15369,   1939,  -4974,  13533,   -422,   4378,  14273,  -3123, -19144,  12867,  -2734,  -8390,
      7487,   8524, -21492,   5665,  -2842,  -1705,   2392,  -9393, -11733,   2147,  -3076,   6914,
     -9289, -23725,   1655,  -3485,  -6178,   9740,   6784,  -9762, -12332,  15719,  -3269,  10696,
     -9584,  -9253,   8529, -11848,   7223,  -1056, -12212,  -1232,   3247,   7630,   6331,   4736,
      -701,  -6456,  10333,  -5486,  11479, -11684, -21669,   -230,   6848,   8794, -11045,  -1497,
      4129,   3370,   2399,  -5990,  12485,   6980,   1488,    379,  -8648,   -769,  -7250,   9752,
    -11873, -10261,  -4577,  -3695,  -6368,  11302,    768,   7197,   -910, -22177,  -2949,   4664,
     -9076,  16850,  -1654,  15836,  12179, -19925,   6137,  -4978,    209,    928,  -5118,  19872,
      7152,   2092, -10441,   2889,   5062,  -2299,  -2672, -13424, -13037,  -3837,   7226,   6051,
     -5655,   1232,  -2030,    142,   3990,  11145,   8392,  -1735,  13446,   5523, -24965,   4537,
     -6363,  -7053, -14553,  -2674,  -9239,   -614,   5087, -12268, -14700,  14122,  -8109,  16337,
      4869,  12504,   6731, -12837, -21578, -14325,   4272,  -1675,  13093,   7028,  -1593,  11028,
      3470, -28864, -10100,  12641,   9846,  12726,   2988, -11953,   -538,  -7547,   8539,   6180,
     -7166,  -5853,  -9750,   3204,   3541, -16910,     25,   4345,   3634,  -7892,   8178,    452,
     -1357,  -1618,  18807,   8982,   9040,  14750, -10680,   3657,   -318,   9533,   1100, -23241,
      1600,  14254,   2530,  -9029,   -571,  -2757,  -1320, -17318,  -2402,   1698,   7000,  -6149,
      2313,   -986,  10703,   8855,  -9176,   5757,   1094,   9910,  -1285,  10543,  -7777,   2810,
    -12540,   3013,  -2372,   5879, -16803,  -8240,   6485,  -6290,   5423,  13912,  12822,   -940,
      5892,   8160, -25095,  -6868,  19993,  12128,  12078,  -1470,   3499,  -5241,  -1746,   9327,
     -2104, -12120,  -9178,  -3855,   1413,  -2634,   1601,  -1005,    -14,  11236,   -324,  11416,
     10655,  -5332,  12540,   1062,   4069,  -4696, -11437,  -6005,   9641,  14511,  24632, -22035,
     -3990,   9147,  -9767,   6509,   9859,   7432, -12004, -10661, -18859,  -5453,  14356,   2730,
      5224,   8689,  -7943,  -3854,  -5514,  -4673,  15186,   3868, -18191,   -826,  -4689,    147,
       901, -10009,  23254,  14781,  21771, -22366,  -4095,   2997,  -6917,  -7796,  26196, -11536,
     -8180,   6134,  -6252,   2444,   2498,  -9431, -12164,  -6416,   -581,  -5478,   4273,   4808,
     13887,   1174,   3746,   1686,  14648,   2937,    448, -17865,  -4159,   6939,   1614,  10050,
     -3614,   6408,  -9090,   7551,   5097,  -6324,  -8431,   5833,  -1173,  -1660,  -2272,  -3113,
     -2563,   7828, -10453,   1007,   -136,  20092,  12182,  -4405,  -5091,  -6297, -12419,   6551,
       862,  -4120,   8496,   -802,   9709, -13347,  -5680,   7966,  -3476,   4859,   2881,  -6142,
     -1826,  -1584,   3432,  -9512, -22180,  -9231, -13660,   1774,  -7630,   3131,  -2939,  -4952,
      4174,  11792,   9486,  10057,   7850,   2285, -10223,  -6046,   7301,  -7078,   7708, -20264,
     -4264,   8655,  15506,  -8664,  12700,   5686,  -8554,  -8476,  -4022,  -9743,   2519,   6091,
     -2910,  10219,   1741,  -3567,  -4883,  -7315,  -2798,   6492,   6466,  -6202,   9522,   8374,
    -15925,   6150,    441,  11432,   7055,   6507,   6115,   -457,  -2970, -12708,   8900,  -6678,
    -10525,  -7828,   8879,   8176,   2596,   2451, -16694,   3349,  19125,  -6391,  -6082,   -382,
     15465,   7350, -12660,  -1223,  -3008,   2291,   4950,   6752,  -1956, -13704,   4008,  -1531,
     -1677,  -3776, -16990,  -1336,  14511, -21000,   5577,    733,   7487,  -7607, -10602,  -4610,
     -9729,  -6518,    909,  -6870, -17102,  -6324,  -2389,  16074,  -7308,   1849,   4300,  -6822,
    -27391,  -2681,   8781,    -30,  -3099,   5600,  -8854,   9919,   1801,   3134,  -2815,    698,
     11774,   7120,  -5045,  -1656,    891,   5342,  14160,   3352, -13376,   7093,   2764,  -3652,
    -13095,  -1841,  -3350, -19936,  11051,  -2078, -15522, -10765,   1718,   6434,  -6271, -22250,
      5302,   1682,  -7302,   3161,   7959, -14748,   2885,   4605,   7412,   -328,  11913,  -6509,
     -9950,  -7903, -18829,   8683,   1424,  -3210, -15635,  -7779,   6140,   2925,   -916,    100,
     -2851, -12685,   1056,   3585, -13732,   -531, -15421,  12447,  -7086,  15721,  -6569, -20491,
     -8817,   -943,    220,  -9526,  -8089,  -5152,  -8538,   9820,   9249,  -2946,  -5488, -15322,
     -8214,  -8252, -15051,   1910,   9505,   7812,  -5890,   8825,   3505,  -2244,  -5612,  -3448,
    -18149, -13863,  -2493,   7989,  12644,   5532,   8766,   5670,  11734,  -7240, -10061,  -9338,
     -1416,   4987,  -5594, -12889,   1127,  -3542, -10024,   8667,  -2429,   4469,   -348,   9174,
      1441,  -1821, -18656,   1102,   -311,  17250, -14526,   9276,  -1802,  -7639,  -9788,  -1679,
      8792, -11547, -10156,  11353,  -6480,  14613,   4512,  14863,   6541,  13986,   5152,     42,
      7785,  13431,   8246,   -240,  -4065, -14228,   -132,  11221,  11883,   6163,  -3048,  -3374,
      1813, -10372,   4698,  11482,  10663,   1877,  -3942,  -8501,  -6656,  -5404, -15090,  -2623,
     16637,   5298,   9319,   2656,  -6260,  12645,  -5374,  -8341,   6646,  -6793,  -2068,  10793,
      4926,  -1750,   9990,  -3817,  12455,  -6574,  14243,    377,  11716,  -4308,  22115,   1708,
      9147, -12921,  24677,  -4380,   8013,  13844, -21596,   -179,  -1144,  -4476,  -2718, -14964,
      8909,   4172,  20037,   3424,  -9449,     30,   8067,   4299,   1863,   5687,  13774,   2790,
      8040,   6210,  -6261,  -4927,    765,  18018,  10591,  -5277,  -1001,  14285,   5672,  20006,
      8104,  10923,   4994,   5108,  11936,  -9964,  -4359,    140,   1367, -10872,  11212,  -4084,
      -906,   1935, -10301,   5417,  11319,  -1079,   4105,  -4035,  -4316,   1559, -14777,  -6254,
     11581,  -7615, -11268,   3691,  -5158,   1728,  24557,   7047,   2790, -13891,  -6400,  11565,
      8292,   5155, -23044,  -7075,  11667,   4631,   3842,   5195,  -8060,   2300,    697,   4148,
      2094,  -4028,   5060, -15611,   2030,  19327,  -4965,  -1181, -14765,   -548, -15400,   8438,
      5926,   2183,   4343,  22226,   3726,   3711,  -1511,  -2553, -11975,  -2211, -10353,   5286,
     13757,   3628, -12088,  10146, -12411,  19172,    129,   4043,   1122,  -7120,  -4207, -15395,
      1923,  -8197,   8981,  21378, -11282,  -4974,   4606,  10979,   9446,   3251,  11269,   4475,
     -7736,    283,    861,  -6265,   6480,  -2364,   2559,   1132,  -1120,     11,  -7568,   6119,
      1900,   1250,  -1415,   1798,   2829,  18334,  -2080,   3386,   8292,   1918,   4013,  -8370,
    -11227, -11940,  -6981,  -1669,    910,   2497,  -6952,  -3056,   6233, -13297,   -391,   6503,
    -11579,  -2424,   6588,   2823, -14541,   3742,   8013,  -5425,  -1665,   -461,   9140,  -6125,
      2449,   1131, -13020,   7061,  -7384, -12993,  -1635, -18895,   2094,  20323,  -4414,   1424,
     -7164,   -914,  16940,  -9040,  12781,   3230,   -242,  10262,     16, -15261,   9677,  -3280,
     -9128,   1178,   8208, -24440,  -8307,  -1082,  10220,    515,  15306,   7075,   2379, -13806,
    -17662,  -4481,  -5430, -11290,   4050,  -3665,    317,  11330,  -1888,  11419,   5782, -12909,
      3388,   5969,   6782,  19369,  -3788,  -2420,   3407,  11612,  -7492,    588,   5754,  -5733,
      3979,   3581, -32768,   -452,   6959,  -8859,   8952, -12146,  13551, -13125,    226,   8625,
     12867,  21397,  13640,  -4558,  25682, -10139,   6609, -12900,  30809,  -7886,   1821,  -8039,
     -1784, -25151,  -6349,  -3618,  -5745,   5245,    717,   5723,  -1423, -18605, -13431,   8745,
      3834,   -113,  -1768,  12264, -13343,  -4926,   -544,   1375,  -1494,  -4250,   3936,   2759,
     -3670,  11526,   3367,  -8417,  15875,   1057,   9309,  -6128,  11889,  -1075,  10585,   5647,
     15817,  10195,  -1621,   4186,  17264,   3346,   1951,   1966,   6601,    244,   8951,   7988,
     13000,    558, -11637,   3724,  -4442,  16017,  -5052,  -2998,   -857,  15154, -23675,   7318,
      7815,  -8417,  -2790,   -676, -11303,   -753, -26341,  -2646,  -4158,  -2260,   7897, -11682,
     -3132,   9586,   7580,  -5481,      0,  -7942,  10125,   9668,  -6848,   5590,  -2514,  -4717,
      1826,   6751, -16892,  -1038, -16004,  -6693,   4635,   2384, -25123,  -7369,   6767,   1068,
        28,    996,  15583,  12567
};

/** Noise level table for power compensation.
 *  Equ: pow(2.0f, (double)(6 - i) / 3.0f) where i = 0...15 */
static const float pwc_levs[16] = {
    3.96875, 3.15625,     2.5,    2.0, 1.59375,   1.25,     1.0, 0.78125,
    0.625,       0.5, 0.40625, 0.3125,    0.25, 0.1875, 0.15625, 0.0
};

/** Map subband number to quant unit number. */
static const uint8_t subband_to_qu[17] = {
    0, 8, 12, 16, 18, 20, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
};

/* start offsets into noise_tab for the time-domain noise, indexed by
 * noise_table_index + channel * 16 + subband */
static const uint16_t noise_seq[48] = {
    579, 215, 556, 247, 561, 854, 237, 658,  10, 827, 660, 396,   9,  19, 146, 221,
    602, 282,  65,  77, 312, 579, 338, 826, 319, 332,  12, 545, 230, 343, 344, 847,
    555,  99, 124, 770,  55, 851, 631, 611,  94, 524, 640, 478,  84, 359, 827, 456,
};

void ff_atrac3p_add_noise(const Atrac3pChanUnitCtx *ctx, int ch, float *out)
{
    double lev = ldexp(1.0, ctx->noise_level_index - 15);
    int i, sb;

    for (sb = 0; sb < ATRAC3P_SUBBANDS; sb++, out += ATRAC3P_SUBBAND_SAMPLES) {
        const int16_t *n = &noise_tab[noise_seq[ctx->noise_table_index +
                                                ch * ATRAC3P_SUBBANDS + sb]];

        for (i = 0; i < ATRAC3P_SUBBAND_SAMPLES; i++)
            out[i] = n[i] * lev + out[i];
    }
}

void ff_atrac3p_power_compensation(Atrac3pChanUnitCtx *ctx, AVFloatDSPContext *fdsp,
                                   int ch_index, float *sp, int rng_index, int sb)
{
    AtracGainInfo *g1, *g2;
    LOCAL_ALIGNED_32(float, pwcsp, [ATRAC3P_SUBBAND_SAMPLES]);
    float *dst, grp_lev, qu_lev;
    int i, gain_lev, gcv = 0, qu, nsp;
    int swap_ch = (ctx->unit_type == CH_UNIT_STEREO && ctx->swap_channels[sb]) ? 1 : 0;

    if (ctx->channels[ch_index ^ swap_ch].power_levs[subband_to_powgrp[sb]] == ATRAC3P_POWER_COMP_OFF)
        return;

    /* generate initial noise spectrum */
    for (i = 0; i < ATRAC3P_SUBBAND_SAMPLES; i++, rng_index++)
        pwcsp[i] = noise_tab[rng_index & 0x3FF] * (1.0f / 32768);

    /* check gain control information */
    g1 = &ctx->channels[ch_index ^ swap_ch].gain_data[sb];
    g2 = &ctx->channels[ch_index ^ swap_ch].gain_data_prev[sb];

    gain_lev = (g1->num_points > 0) ? (6 - g1->lev_code[0]) : 0;

    for (i = 0; i < g2->num_points; i++)
        gcv = FFMAX(gcv, gain_lev - (g2->lev_code[i] - 6));

    for (i = 0; i < g1->num_points; i++)
        gcv = FFMAX(gcv, 6 - g1->lev_code[i]);

    grp_lev = pwc_levs[ctx->channels[ch_index ^ swap_ch].power_levs[subband_to_powgrp[sb]]] / (1 << gcv);

    /* skip the lowest two quant units (frequencies 0...351 Hz) for subband 0 */
    for (qu = subband_to_qu[sb] + (!sb ? 2 : 0); qu < subband_to_qu[sb + 1]; qu++) {
        if (ctx->channels[ch_index].qu_wordlen[qu] <= 0)
            continue;

        /* level and noise mix are rounded once, as in the reference decoder */
        qu_lev = (double)ff_atrac3p_sf_tab[ctx->channels[ch_index].qu_sf_idx[qu]] *
                 ff_atrac3p_mant_tab[ctx->channels[ch_index].qu_wordlen[qu]] *
                 grp_lev / (1 << ctx->channels[ch_index].qu_wordlen[qu]);

        dst = &sp[ff_atrac3p_qu_to_spec_pos[qu]];
        nsp = ff_atrac3p_qu_to_spec_pos[qu + 1] - ff_atrac3p_qu_to_spec_pos[qu];

        for (i = 0; i < nsp; i++)
            dst[i] = (double)qu_lev * pwcsp[i] + dst[i];
    }
}

/*
 * The rounding follows the reference decoder: products are exact, each
 * stored value is rounded once, and the butterfly differences are kept at
 * double precision until they are rotated.
 */
void ff_atrac3p_imdct(const float *in, float *out, int wind_id, int sb)
{
    const float *win = imdct_win[wind_id];
    float z[ATRAC3P_SUBBAND_SAMPLES];
    int k, m, g, r, t = 63;

    /* pre-rotation; odd subbands are spectrally reversed */
    for (k = 0; k < 64; k++) {
        double c = imdct_cos[63 + k], s = imdct_sin[63 + k];
        double a = in[2 * k], b = in[127 - 2 * k];

        if (sb & 1)
            FFSWAP(double, a, b);
        z[2 * k]     = a * c + b * s;
        z[2 * k + 1] = a * s - b * c;
    }

    /* 64-point complex FFT, radix 2, decimation in frequency */
    for (m = 32; m >= 1; m >>= 1) {
        const float *c = &imdct_cos[t - m], *s = &imdct_sin[t - m];

        for (g = 0; g < 32 / m; g++) {
            int i = g * 4 * m;

            for (r = 0; r < m; r++, i += 2) {
                int    j  = i + 2 * m;
                double d0 = (double)z[i]     - z[j];
                double d1 = (double)z[i + 1] - z[j + 1];

                z[i]     = (double)z[j]     + z[i];
                z[i + 1] = (double)z[j + 1] + z[i + 1];
                z[j]     = d0 * c[r] + d1 * s[r];
                z[j + 1] = d0 * s[r] - d1 * c[r];
            }
        }
        t -= m;
    }

    /* reorder, post-rotation sign and windowing */
    for (k = 0; k < 64; k++) {
        out[k]       =   z[imdct_order[64 + k]] * win[k];
        out[192 + k] = -(z[imdct_order[k]]      * win[192 + k]);
    }
    for (k = 0; k < 128; k++)
        out[64 + k] = -(z[imdct_order[127 - k]] * win[64 + k]);
}

static float gain_exp2(int e)
{
    return e >= 0 ? 1.0f / (1 << e) : (float)(1 << -e);
}

/**
 * Build the gain curve over one IMDCT window (64 positions of 4 samples)
 * from the gain points of the previous and the current frame.
 *
 * @return last sample index (from the end) up to which the second half
 *         has to be scaled
 */
static int gain_curve(const AtracGainInfo *gc_prev, const AtracGainInfo *gc_cur,
                      float *curve)
{
    int lev[64] = { 0 };
    int i, p, pos = 256, last = 0, idx = ATRAC3P_MDCT_SIZE - 1;
    float f;

    for (i = 0, p = 0; i < gc_cur->num_points; i++)
        for (; p <= gc_cur->loc_code[i] + 32; p++)
            lev[p] = gc_cur->lev_code[i] - 6;
    for (i = 0, p = 0; i < gc_prev->num_points; i++)
        for (; p <= gc_prev->loc_code[i]; p++)
            lev[p] += gc_prev->lev_code[i] - 6;

    f = gain_exp2(lev[63]);
    for (p = 63; p >= 0; p--, idx -= 4) {
        int e = lev[p];

        if (e != last) {
            if (pos == 256)
                pos = idx;
            if (e < last) {
                const float *ip = gain_interp[last - e - 1];

                f = gain_exp2(e);
                curve[idx]     = f * ip[0];
                curve[idx - 1] = f * ip[1];
                curve[idx - 2] = f * ip[2];
            } else {
                const float *ip = gain_interp[e - last - 1];
                float g = gain_exp2(last);

                curve[idx]     = g * ip[2];
                curve[idx - 1] = g * ip[1];
                curve[idx - 2] = g * ip[0];
                f = gain_exp2(e);
            }
            last = e;
        } else {
            curve[idx]     = f;
            curve[idx - 1] = f;
            curve[idx - 2] = f;
        }
        curve[idx - 3] = f;
    }

    return pos == 256 ? 255 : pos;
}

void ff_atrac3p_gain_compensation(float *in, float *prev,
                                  const AtracGainInfo *gc_prev,
                                  const AtracGainInfo *gc_cur, float *out)
{
    float curve[ATRAC3P_MDCT_SIZE];
    int i, end;

    if (!gc_prev->num_points && !gc_cur->num_points) {
        for (i = 0; i < ATRAC3P_SUBBAND_SAMPLES; i++)
            out[i] = prev[i] + in[i];
    } else {
        end = gain_curve(gc_prev, gc_cur, curve);
        for (i = ATRAC3P_SUBBAND_SAMPLES; i <= end; i++)
            in[i] *= curve[i];
        for (i = 0; i < ATRAC3P_SUBBAND_SAMPLES; i++)
            out[i] = prev[i] + in[i] * curve[i];
    }

    memcpy(prev, &in[ATRAC3P_SUBBAND_SAMPLES], ATRAC3P_SUBBAND_SAMPLES * sizeof(*prev));
}

/*
 * Subband synthesis in the factored form of the reference decoder: a fast
 * 16-point DCT per time slot, then a polyphase filter over a history with
 * three phases. Values stay at 16-bit sample scale until the final scaling.
 * Products and stores round to float as in the reference, and the DCT sums
 * stay at double precision between stores.
 */
static const float ipqf_pre[16] = {
    1.9975909f, 1.97835302f, 1.94006252f, 1.88308811f,
    1.80797863f, 1.7154572f, 1.60641503f, 1.48190224f,
    1.34311795f, 1.19139862f, 1.02820551f, 0.855110168f,
    0.673779726f, 0.485960364f, 0.293460965f, 0.0981353521f,
};
static const float ipqf_dct_c[12] = {
    1.99036944f, 1.91388071f, 1.76384258f, 1.54602087f,
    1.26878655f, 0.942793489f, 0.580569386f, 0.196034282f,
    1.9615705f, 1.66293919f, 1.11114049f, 0.390180647f,
};
static const float ipqf_dct_k[7] = {
    1.84775901f, 0.765366852f, 1.84775901f, 0.765366852f, 0.707106769f, 0.207106784f, 1.20710683f,
};
static const float ipqf_q0[4][4] = {
    { -1.86675861e-05f, -2.57933534e-05f, -1.34417314e-05f, -1.42081831e-06f },
    { 1.03239904e-06f, 1.12987391e-06f, 4.05132056e-07f, 0.00033653705f },
    { 1.6725824e-05f, 2.93486337e-05f, 3.48729081e-05f, 7.56790541e-06f },
    { -8.89365947e-06f, -1.68284041e-05f, -1.48158533e-05f, 0.000346911751f },
};
static const float ipqf_q1[10][8] = {
    { 1.62659013f, 0.638275385f, -1.7741518f, -1.82223821f,
      -1.58538806f, -1.38268912f, -1.21992135f, -2.20914412f },
    { -6.69625759f, -20.3413429f, -5.47101021f, -2.55776572f,
      -2.15493703f, -2.01362944f, -1.96669734f, 2.44687843f },
    { -2.83530784f, -0.96154052f, 1.4561919f, 2.13812304f,
      2.33115721f, 2.41837096f, 2.46523619f, 1.27684009f },
    { 0.840129673f, 1.06234288f, 1.44598281f, 1.36445045f,
      1.29641485f, 1.26052153f, 1.24017274f, -2.10737586f },
    { -1.02899289f, -1.0333575f, -1.43625093f, -1.93646526f,
      -2.18525839f, -2.34563589f, -2.44903207f, -1.91270649f },
    { 0.745846152f, 0.405315548f, -0.848005235f, -1.37126482f,
      -1.56435275f, -1.71086633f, -1.84778762f, -2.63118148f },
    { -4.22934198f, -8.49816418f, -7.35008192f, -3.9679594f,
      -3.15299749f, -2.69677758f, -2.37674069f, 1.18752003f },
    { -4.095644f, -1.64397204f, 0.145435691f, 0.549174666f,
      0.758587778f, 0.947208464f, 1.14199519f, 2.55337977f },
    { 2.03692651f, 2.76282835f, 4.27371407f, 4.14585066f,
      3.67397451f, 3.18434763f, 2.70790529f, -1.8614639f },
    { -0.724812627f, -0.72320044f, -0.815565884f, -1.10647333f,
      -1.36276865f, -1.63082302f, -1.92720461f, -1.07240236f },
};
static const float ipqf_q2[2][4] = {
    { -12.0513439f, -7.92691183f, -10.9967785f, -73.3543243f },
    { 71.6271439f, 40.5956917f, 47.4345169f, -1.1256882f },
};
static const uint8_t ipqf_idx[3][7] = {
    {  0,  8, 24,  7, 31, 15, 32 },
    { 16, 24, 40, 23, 47, 31,  0 },
    { 32, 40,  8, 39, 15, 47, 16 },
};

void ff_atrac3p_ipqf(Atrac3pIPQFChannelCtx *hist, const float *in, float *out)
{
    const float k0 = ipqf_dct_k[0], k1 = ipqf_dct_k[1], k2 = ipqf_dct_k[2],
                k3 = ipqf_dct_k[3], k4 = ipqf_dct_k[4], k5 = ipqf_dct_k[5],
                k6 = ipqf_dct_k[6];
    const double half = 0.5;
    float *H = hist->buf;
    int i, s, it, ph = 0;

    for (s = 0; s < ATRAC3P_SUBBAND_SAMPLES; s++, out += ATRAC3P_SUBBANDS) {
        const uint8_t *idx;
        int c80, c8c, c98, ca4, cb0, cbc, cc8;
        float x[ATRAC3P_SUBBANDS], v[ATRAC3P_SUBBANDS], r[4];
        float a0, a1, a2, a3, b0, b1, b2, b3, g0, g1, g2, g3, h0, h1, h2, h3;

        ph  = (hist->pos + s) % 3;
        idx = ipqf_idx[ph];
        c80 = idx[0]; c8c = idx[1]; c98 = idx[2]; ca4 = idx[3];
        cb0 = idx[4]; cbc = idx[5]; cc8 = idx[6];

        for (i = 0; i < ATRAC3P_SUBBANDS; i++)
            x[i] = in[i * ATRAC3P_SUBBAND_SAMPLES + s] * ipqf_pre[i];

        /* first butterfly stage, one 4-lane vector each */
        {
            float p, q, e, f, A[4], B[4], G[4], Hv[4];

            for (i = 0; i < 4; i++) {
                p     = x[i]     + x[15 - i];
                q     = x[7 - i] + x[8 + i];
                e     = ipqf_dct_c[7 - i] * (x[7 - i] - x[8 + i]);
                f     = ipqf_dct_c[i]     * (x[i]     - x[15 - i]);
                A[i]  = p + q;
                B[i]  = ipqf_dct_c[8 + i] * (p - q);
                G[i]  = f + e;
                Hv[i] = ipqf_dct_c[8 + i] * (f - e);
            }
            a0 = A[0];  a1 = A[1];  a2 = A[2];  a3 = A[3];
            b0 = B[0];  b1 = B[1];  b2 = B[2];  b3 = B[3];
            g0 = G[0];  g1 = G[1];  g2 = G[2];  g3 = G[3];
            h0 = Hv[0]; h1 = Hv[1]; h2 = Hv[2]; h3 = Hv[3];
        }

        /* remaining DCT stages; stores round to float, the rest stays double */
        {
            double sum_a = (((double)a3 + a2) + a1) + a0;
            double sum_b = (((double)b2 + b1) + b3) + b0;
            double sum_g = (((double)g3 + g2) + g1) + g0;
            double sum_h = (((double)h3 + h2) + h1) + h0;
            double da = ((double)a0 - a3) * k0, ea = ((double)a1 - a2) * k1;
            double db = ((double)b0 - b3) * k2, eb = ((double)b1 - b2) * k3;
            double dg = ((double)g0 - g3) * k0, eg = ((double)g1 - g2) * k1;
            double dh = ((double)h0 - h3) * k2, eh = ((double)h1 - h2) * k3;
            float t120, tf8, t108, t118, t128, te0, te8, t104, tdc, te4;
            double u;

            v[0]  = sum_a * half;
            t120  = k5 * da - k6 * ea;
            tf8   = sum_b * half;
            t108  = (eb + db) * half - tf8;
            t118  = ((((double)b0 - b1) - b2) + b3) * k4 - t108;
            t128  = (k5 * db - eb * k6) - t118;
            v[1]  = sum_g * half - v[0];
            v[2]  = (double)tf8 - v[1];
            te0   = ((((double)g0 - g1) - g2) + g3) * k4;
            te8   = dg * k5 - eg * k6;
            v[3]  = (sum_h * half - sum_g * half) - v[2];
            t104  = (eg + dg) * half - (sum_h * half - sum_g * half);
            v[4]  = (ea + da) * half - v[3];
            v[5]  = (double)t104 - v[4];
            v[6]  = (double)t108 - v[5];
            tdc   = ((((eh + dh) - h0) - h1) - h2 - h3) * half;
            te4   = ((((double)h0 - h1) - h2) + h3) * k4 - tdc;
            u     = (double)tdc - t104;
            v[7]  = u - v[6];
            v[8]  = ((((double)a0 - a1) - a2) + a3) * k4 - v[7];
            u     = te0 - u;
            v[9]  = u - v[8];
            v[10] = (double)t118 - v[9];
            u     = te4 - u;
            v[11] = u - v[10];
            v[12] = (double)t120 - v[11];
            u     = te8 - u;
            v[13] = u - v[12];
            v[14] = (double)t128 - v[13];
            v[15] = ((dh * k5 - eh * k6) - te4 - u) - v[14];
        }

        for (i = 0; i < ATRAC3P_SUBBANDS; i++)
            H[cbc - i] = v[i];

        for (i = 0; i < 4; i++) {
            H[c80 + 48 + i] = ipqf_q0[0][i]     * H[c8c + i]     + ipqf_q0[2][i]     * H[cc8 + 7 - i];
            H[c8c + 48 + i] = ipqf_q0[3][3 - i] * H[cbc - i]     - ipqf_q0[1][3 - i] * H[cc8 + i];
        }
        for (i = 0; i < 4; i++) {
            H[c80 + 52 + i] = ipqf_q0[1][i]     * H[c8c + 4 + i] + ipqf_q0[3][i]     * H[cc8 + 3 - i];
            H[c8c + 52 + i] = ipqf_q0[2][3 - i] * H[cbc - 4 - i] - ipqf_q0[0][3 - i] * H[cc8 + 4 + i];
        }

        for (it = 0; it < 10; it++) {
            const float *ql = ipqf_q1[it], *qh = ipqf_q1[it] + 4;
            float *pd  = &H[c80 + 144 + 48 * it];
            float *p18 = &H[c8c +  96 + 48 * it];
            float *p1c = &H[cb0 +  41 + 48 * it];
            float *p20 = &H[c98 +  48 + 48 * it];
            float *p28 = &H[ca4 +  41 + 48 * it];

            for (i = 0; i < 4; i++)
                r[i] = pd[-96 + i] + ql[i] * p1c[7 - i];
            memcpy(&pd[-48], r, sizeof(r));
            for (i = 0; i < 4; i++)
                r[i] = qh[3 - i] * p28[7 - i] - p20[i];
            memcpy(&p18[0], r, sizeof(r));
            for (i = 0; i < 4; i++)
                r[i] = pd[-92 + i] + qh[i] * p1c[3 - i];
            memcpy(&pd[-44], r, sizeof(r));
            for (i = 0; i < 4; i++)
                r[i] = ql[3 - i] * p28[3 - i] - p20[4 + i];
            memcpy(&p18[4], r, sizeof(r));
        }

        for (i = 0; i < 4; i++) {
            out[i]      = (H[c80 + 528 + i] + ipqf_q2[0][i] * H[cb0 + 528 - i]) * (1.0f / 32768);
            out[4 + i]  = (H[c80 + 532 + i] + ipqf_q2[1][i] * H[cb0 + 524 - i]) * (1.0f / 32768);
            out[8 + i]  = (ipqf_q2[1][3 - i] * H[ca4 + 528 - i] - H[c98 + 528 + i]) * (1.0f / 32768);
            out[12 + i] = (ipqf_q2[0][3 - i] * H[ca4 + 524 - i] - H[c98 + 532 + i]) * (1.0f / 32768);
        }
    }

    hist->pos = ph + 1;
}
