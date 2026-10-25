/*
 * LPEC audio decoder
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

/**
 * @file
 * Sony LPEC audio decoder
 *
 * Reverse-engineered from the Sony LPEC decoder.
 *
 * Bitstream is MSB-first (big-endian bit order).
 *
 * Frame structure (all modes output LPEC_FRAME_OUT = 512 samples):
 * 2 bits mode
 * Mode 0: two half-frames, each with own LSF + pitch + excitation
 * Mode 1: one frame, single LSF + pitch + excitation (only half decoded)
 * Mode 2: one shared LSF + two halves of pitch + excitation
 * Mode 3: one frame, all silent / comfort noise
 *
 * LSF: 3 VQ codebooks, 6 bits each
 *
 * Mode 0 pitch: both pitch windows first,
 * then per half-frame: 1 bit + optional 7-bit pitch CB,
 * then excitation (7+6+6).
 * Pitch (modes 1-3): 1 bit voiced flag; if voiced: 6-bit lag offset from base lag
 * Excitation: 7-bit gain idx + 6-bit cb1 idx + 6-bit cb2 idx
 *
 * LPC order: 10 for 8 kHz, 16 for >=16 kHz.
 * Synthesis: interpolated LPC in n+1 passes (9 at 16 kHz); the first and
 * last pass are frame/(2n) long, the middle ones frame/n.
 */

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
/* NOTE: NO BITSTREAM_READER_LE -- reference decoder uses MSB-first (standard get_bits) */
#include "get_bits.h"
#include "libavutil/channel_layout.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "libavutil/macros.h"
#include <math.h>
#include <string.h>

#include "lpecdata.h"
#include "lpec_excit.h"
#include "lpec_excit_route.h"
#include "lpec_ab80.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* - Constants ---- */
#define LPEC_LPC_ORDER_MAX  16      /* 10 @ 8kHz, 16 @ 16kHz */
#define LPEC_LSF_MAX        (LPEC_LPC_ORDER_MAX + 1) /* lsf[0]=0, lsf[1..order] */
#define LPEC_NUM_LSF_CB_MAX 4       /* 3 @ 8kHz, 4 @ 16kHz (the reference decoder +0xb0) */
#define LPEC_LSF_BITS       6
/*
 * Frame size as the reference decoder sets it up: 512 samples at 8 kHz,
 * 1024 (two 512-sample halves) at 16 kHz. The bit budgets follow the frame
 * size, not fixed 128/432.
 */
#define LPEC_FRAME_8K     512
#define LPEC_FRAME_16K    1024
#define LPEC_SUBFR_8K     256
#define LPEC_SUBFR_16K    512
#define LPEC_NUM_BANDS_MAX  10      /* 8 @ 8kHz, 10 @ 16kHz (the reference decoder +0x2c) */
#define LPEC_NUM_SEGMENTS 4         /* LPC interpolation segments per frame */
#define LPEC_MAX_FRAME    1024
#define LPEC_EXCIT_SRC_MAX (LPEC_MAX_FRAME * 2)
#define LPEC_MAX_FRAMES_PKT 32
#define LPEC_SCRATCH_SAMPLES (LPEC_MAX_FRAME * LPEC_MAX_FRAMES_PKT)
#define LPEC_WORK_SLACK   256
#define LPEC_F640_SHAPE_M3 (2 * LPEC_MAX_FRAME) /* 2*frame_size */
#define LPEC_QUIET_EXCIT_RMS 10.0     /* mode-3 half2 below this uses tail hist */

/*
 * Pitch lag (mode 1/2/3: ): lag = base + 6-bit-offset.
 * The 6-bit offset indexes into a table via lag = base_lag + offset * step.
 * base_lag is encoded in the context at offset 0xd0.
 * Mode 0: 7-bit index into lpec_8k_pitch_cb (128 entries).
*/
#define LPEC_PITCH_LAG_BASE  20     /* smallest allowed pitch lag (mode 1/2/3) */
#define LPEC_PITCH_LAG_MAX   83     /* 20 + 63 */

/* Excitation history in LPECExcitState (lpec_excit.h). */

/* Carry buffer: holds unconsumed bits from previous packet.
 * In the worst case the carry is almost a full 80-byte packet (640 bits).
 * We need: max_carry (80 bytes) + new packet (80 bytes) + alignment slack (8).*/
#define LPEC_CARRY_BUF_BYTES  4096 /* must hold partial frame + full MSV payload */

typedef struct LPECContext {
    AVCodecContext *avctx;
    int            codec_rate_in;
    int            frame_size;        /* 104 or 208 depending on sample_rate */
    int            subfr_size;        /* frame_size / 2 */
    int            lpc_order;         /* 10 or 16 */
    int            num_lsf_cb;        /* 3 or 4 */
    int            num_bands;         /* 8 or 10 */
    double prev_lsf[LPEC_LSF_MAX];
    double lsf1[LPEC_LSF_MAX];  /* LSF-A from last decoded frame */
    double lsf2[LPEC_LSF_MAX];  /* LSF-B from last decoded frame */
    double synth_buf[LPEC_LPC_ORDER_MAX + LPEC_MAX_FRAME];
    LPECExcitState excit;
    int    half_lsf_cb0[2];
    int    half_lsf_idx[3][LPEC_NUM_LSF_CB_MAX];
    int16_t half_lsf_i16[3][LPEC_LSF_MAX];
    int    noise_pos;                   /* legacy noise walk (unused by excit chain) */
    int    prev_pitch_lag;
    int    pitch_voiced_bits;
    int    frame_out_size;
    int    pitch_win_prev;            /* rolled from prior half-2 */
    int    pitch_win_half1;
    int    pitch_win_half2;
    int    pitch_lag_off_half1;       /* 6-bit lag table index when voiced */
    int    pitch_lag_off_half2;
    int    pitch_lag_off_prev;        /* rolled lag index (debug) */
    const double *pitch_tbl_prev;     /* f430 seg0, rolled from half2 */
    const double *pitch_tbl_half1;    /* f430 seg1 */
    const double *pitch_tbl_half2;    /* f430 seg2 */
    int    frame_count;
    int    lpec_frame_idx;              /* decoded LPEC frames (not AVPackets) */
    int    pending_frame_start;       /* bit pos at start of current frame decode */
    int    pending_frame_mode;        /* mode byte for padding on resync */
    int    pad_pending_start;         /* decoded frame awaiting mode padding */
    int    pad_pending_mode;
    /* MSV packet tail + chunk stream (the reference: decode then drop chunk). */
    uint8_t carry_buf[LPEC_CARRY_BUF_BYTES + 80 + 4];
    int    chunk_carry_len;
    uint8_t stream_buf[LPEC_CARRY_BUF_BYTES + 80 + 4];
    int    stream_len;
    int    stream_bit_off;            /* 0..7 bits already consumed in stream_buf[0] */
    uint16_t chunk_sz[4];             /* lead 0x00/0x40/0x80/0xC0 */
    int    frame_bit_target[4];
    int    synth_half_flag;             /* mode-0 LSF flag, persists */
    double excit_src[LPEC_EXCIT_SRC_MAX]; /* excitation work (2048 @ 16 kHz) */
    double frame_excit[LPEC_MAX_FRAME]; /* f640 output / saved excitation */
    double excit_work[LPEC_WORK_SLACK + LPEC_MAX_FRAME];
    double excit_ola_hist[LPEC_MAX_FRAME];
    double frame_out[LPEC_MAX_FRAME];   /* retain untouched half for modes 1-3 */
    double scratch[LPEC_SCRATCH_SAMPLES];
    double pitch_shape[LPEC_MAX_FRAME];
    double pitch_weight[LPEC_MAX_FRAME];
    double f640_shape_m3[LPEC_F640_SHAPE_M3];
    double pitch_cb_aux[LPEC_SUBFR_16K]; /* the reference decoder: dc80/03e30 OLA hist */
    double mode0_shaped_tail[LPEC_SUBFR_16K]; /* cross-frame carry */
    LPECQmfState qmf;
    int    prev_half2_pitch_cb;         /* 0x138: prior frame half-2 CB flag */
    int    prev_half2_pitch_cb_idx;     /* prior frame half-2 CB index */
    int    pitch_cb_flag_h1;            /* mode-0 half-1 pitch CB bit */
    int    pitch_cb_flag_h2;            /* mode-0 half-2 pitch CB bit */
    int    pitch_cb_half2_idx;          /* current frame half-2 CB idx */
    int    pitch_cb_tbl_prev_idx;       /* rolled from prior half-2 tbl */
    int    prev_lsf_cb_half2;           /* rolled LSF CB for half 1 */
    int    mode0_excit_rem;             /* rem after pitch, before pitch CB */
    int    prev_decode_mode;            /* prior frame mode for cross-mode hist */
    int    next_frame_mode_peek;        /* show_bits(gb,2) at next frame; -1 unknown */
    int    eof_flush_done;              /* one-shot trailing frame at EOF */
    int    eof_draining;                /* avpkt flush: relax padding lookahead */
    int    is_xaudio_lp;                /* codec_tag == 0x19 (the reference decoder) */
    int    qmf_seg_m2_h1;               /* mode-2 h1 ab80 len */
} LPECContext;

static const double lpec_pitch_cb_stub[8] = {
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
};

static const double *lpec_pitch_cb_ptr(LPECContext *s, int idx)
{
    if (idx < 0)
        return lpec_pitch_cb_stub;
    if (s->num_bands == 10)
        return lpec_16k_pitch_cb + (idx & 127) * 8;
    if (s->is_xaudio_lp)
        return lpec_xa_8k_pitch_cb + (idx & 127) * 8;
    return lpec_8k_pitch_cb + (idx & 127) * 8;
}

static const double lpec_16k_f430_coef[192] = {
    0.028729972606089303, 0.15328675419214269, 0.017482854866846635,
    0.10878427834873362, 0.36091455724888222, 0.15450039144327535,
    -0.037912732848629414, 0.13402055378102512, -0.05640739089096812,
    -0.095810860415539115, 0.22953521704456253, 0.13770390032460919,
    0.038338820504945768, 0.18852875052358917, 0.124109632322392,
    0.059198818145002982, 0.071288007191259481, -0.00071052372631450898,
    0.014247479769313636, 0.22323341873378946, -0.061118843132450473,
    -0.067736931284241367, 0.18913880237276362, 0.033022559269365197,
    -0.054642149210748572, 0.51341028423105839, -0.013791261255198233,
    0.014869778311855293, 0.08598482484787677, -0.16670320324117285,
    -0.042988132588338203, 0.102197686075399, 0.074773063059378725,
    0.1535806418890629, 0.27675837172135109, -0.019378533816776764,
    -0.1007054843263069, 0.21256647310614615, -0.071223448850374016,
    -0.018281319888986593, 0.322252719547516, -0.11854152510991044,
    -0.046622655289896731, 0.55533480407402713, 0.4124866162781517,
    0.48178200976672214, 0.61082553988031429, -0.13163494158785058,
    -0.047011526423114909, 0.47550923109643273, 0.17137478892132851,
    0.30232387024689783, 0.33785094587443498, 0.098251600547813442,
    0.26278961520453414, 0.45710782638910086, 0.18174499802345231,
    -0.042785671156202548, 0.20075747124376522, -0.19479352892426127,
    0.025969530357584508, 0.69070595977297244, 0.19598077718542548,
    0.16697891872993831, 0.80054487743018299, -0.027305370338162598,
    0.24709452307320348, 0.32658981078404603, 0.26873899927946371,
    0.12179172832423819, 0.18904225086113655, 0.029406148628194271,
    -0.17374437920223315, 0.11932080351460912, -0.0046716640142863792,
    0.01286185295854061, 0.27337602977216652, 0.050337397732274705,
    -0.17836897659608555, 0.73176141896340341, 0.37865688295464384,
    -0.049140363587001565, 0.85755475030562178, 0.092475304647522377,
    0.064958629094484827, 0.40050681560041562, 0.34528623944011499,
    -0.2505798469279083, 0.46026748625580427, -0.018418527913073433,
    0.16207169440656125, 0.24503840426089069, 0.134713769107867,
    0.079742102388540265, 0.40324900074357511, 0.015229826756279746,
    0.13850503966597724, 0.53227365551965244, -0.028696278418361358,
    0.34652297262377929, 0.73003080083388239, -0.16361624953102108,
    0.041279939331865877, 0.90000000000000013, 0.054603239132815291,
    -0.10681390109714731, 0.71649860701116741, -0.038564291184854695,
    -0.14238914166629621, 0.1049638204385686, -0.14371521041545468,
    0.29047782581071435, 0.50983121918110286, -0.2596310054441972,
    -0.085350422331284231, 0.34283987326740489, 0.22472607988499074,
    -0.27290364685621649, 0.38907757160737122, 0.17075550488153196,
    0.14085730914489536, 0.5391504982168448, 0.27105842841904326,
    -0.043791212853548513, 0.041548334287569079, -0.02246947520320576,
    -0.25336591989358093, 0.62453262562775991, -0.11569370186635035,
    0.24433637646807269, 0.3664768687878916, -0.10897982108340647,
    -0.26435132625820451, 0.50425838607926199, 0.31373139195434746,
    0.12138539226541471, 0.25494780934978611, -0.22763471440883193,
    -0.21547921347979329, 0.24595287195186277, -0.13802897939207842,
    -0.11489354867351798, 0.34500176546041339, 0.012920473416825888,
    -0.17034430958995331, 0.58916035660230803, -0.17557225493785608,
    0.60145395181647698, 0.18943870261314219, -0.25625488260597284,
    0.092843210480641952, 0.16527343379158657, -0.097102957226228581,
    0.1489142660219453, 0.90000000000000013, -0.048993871998479585,
    -0.17088157098464316, 0.39332482695695792, -0.23449683024567222,
    -0.058058487910880398, 0.82742687449053443, -0.10787855639491233,
    0.07152800348241789, 0.67975115156543664, -0.19253574472201593,
    0.24195135279312804, 0.62973287316169335, 0.079760298410232494,
    0.059532812377510264, 0.44995896608413433, -0.23066816022552866,
    0.36710182996727153, 0.47732365844035324, 0.0080308570084168564,
    -0.20241249488224278, 0.6359140538467325, 0.12379316474146698,
    0.29383330446101663, 0.19251414845253564, 0.11889079589527415,
    0.21478583820351094, 0.18044218654621838, 0.25819246983736727,
    -0.21941493355392386, 0.23159496269108953, 0.03725220835278905,
    0.056998823685529158, 0.26997752802043051, 0.224761970892077,
    0.11156547038317294, 0.11686665850784031, 0.10520788955080092,
};
/* 8 kHz Xaudio OLA pitch taps (64 triples); the Q15 copy is lpec_xa_8k_pitch_i16. */
static const double lpec_xa_8k_f430_coef[192] = {
    0.012849134604929847, 0.2936397061490345, 0.05226251984983754,
    0.021634311713913713, 0.5125102893474466, 0.1753466613476382,
    0.10868714167009581, 0.3734859472594949, 0.1300017608103456,
    0.17864899261710207, 0.4422362343852147, -0.043843130365079065,
    0.1542959492696115, 0.23945285061421362, 0.07268538016146706,
    -0.02645029651552375, 0.25290458075956374, -0.06808385278115968,
    -0.05631650118361484, 0.15037086220433588, -0.013559613363369138,
    0.05319960541800167, 0.1941579154564574, 0.0012934081188733422,
    -0.15951485049689892, 0.733946755327893, -0.043819288709140015,
    0.1026648661827326, 0.2055227316919547, -0.14039252021880924,
    -0.04672604660322771, 0.19949441547662217, 0.10349866685279485,
    0.199383638944563, 0.26014022111460194, 0.24515929093603833,
    -0.13333207842600747, 0.2810982118767152, 0.006862525478344219,
    -0.025645057443645966, 0.052531438465579866, 0.026466244837743375,
    -0.2421710666602211, 0.30483309559427574, 0.1587550120710201,
    -0.19986497972373893, 0.08221332007762415, -0.06139467238400686,
    0.1327195131846583, 0.31141237787884957, -0.07300911108795653,
    0.023710465225727707, 0.8668220145702392, 0.05088385849270226,
    0.3007417610515482, 0.7112752630353707, -0.08108720612005287,
    0.021417579867405574, 0.4128479636186849, 0.36139178522235127,
    0.48573029089215064, 0.6225900704773198, -0.1314577324007631,
    -0.12085265842915606, 0.867061816735899, 0.186476375108422,
    -0.0452360538837309, 0.5669651063130805, 0.3844399267333616,
    -0.03308857950728417, 0.06885143270276857, -0.13503935472498227,
    -0.09856286540006168, 0.3932807270060328, 0.1043244774310357,
    0.0038208118462102494, 0.5414900404163976, -0.05617554208119145,
    -0.01169093568272416, 0.28891495345553037, 0.2203276364845544,
    0.1442308501552162, 0.6866140054395565, 0.05724725434799798,
    0.08780632045355012, 0.09742640673356191, -0.03121088992202535,
    -0.03870122595854788, 0.39110521487260175, -0.07177760098898939,
    0.1386384520225713, 0.6068862765947846, 0.24377029741946243,
    0.09885429843796603, 0.44642928699133844, -0.24753798388403278,
    -0.15936907152610857, 0.19011437281925717, -0.0787132433046521,
    0.2203021563953184, 0.2966176105470478, -0.2744811649876181,
    -0.27258949945073485, 0.633933126837319, 0.19672622190641856,
    -0.158232408030972, 0.7700796928186721, 0.3750956089744355,
    -0.10487487192556295, 0.49826887342185483, -0.2901398857987164,
    -0.03665277950987278, 0.175377218974723, -0.13899486197785352,
    0.23268113079524919, 0.4092590237027595, 0.2846120996362095,
    -0.19444791472823747, 0.1693272051417208, 0.07011598146008566,
    -0.2659437165434435, 0.5739187357014366, -0.14152102541685954,
    -0.14552404023964052, 0.26192196829875636, -0.15977296838876948,
    -0.021902854190489065, 0.7040280921285846, 0.21530318699083273,
    -0.15056469471222178, 0.5330497978689965, 0.05236105751216449,
    -0.226055519509001, 0.44731458242393946, 0.300092822025013,
    0.33145118341790053, 0.5768698805809149, 0.000897460257969684,
    0.35180331026769357, 0.17765569786485919, 0.08728523569646277,
    0.24996983420890354, 0.5024843402225971, 0.1444138046820252,
    0.29499948699759265, 0.34354328782498633, 0.08184329190069496,
    -0.1887387914187385, 0.6208096220169363, 0.5205665417449463,
    0.4211850692021124, 0.4320581055217141, -0.028914508944127934,
    0.09334915880974594, 0.14222481660032615, 0.1267077034540649,
    0.1841485862610346, 0.8501518573465071, -0.0738651169932576,
    -0.23580961078655666, 0.3997611178621487, -0.09522902558626624,
    -0.055160296362212966, 0.8769981727225309, -0.06513241479788316,
    0.34066897978955607, 0.809169803474421, -0.18276038123521743,
    0.053223782596285726, 0.6629139311009441, -0.21727282909613413,
    -0.34388875910329647, 0.2476391841027002, -0.05743813029901223,
    -0.35931188485547244, 0.4570159905424926, 0.0742004895867988,
    -0.17599095222822558, 0.12674240933628428, -0.22662697572143808,
    0.32801342916777687, 0.5101205594713221, -0.2739073765422465,
    -0.24848664737303336, 0.30472486801756216, -0.33869819569007953,
    0.010914914161181824, 0.16015352361693252, -0.3009847500874043,
    -0.020601825539139074, 0.3128706073779449, -0.22705755925843252,
};


/*
 * The reference raised-cosine: 2*pi is the 12-digit constant 6.28318530718,
 * the cosine is taken in extended precision and 1 - cos is rounded to double.
 */
static double lpec_raised_cos(int i, int n)
{
    double phase = (((double) i + 0.5) * 6.28318530718) * (1.0 / (double) n);

    return (double) (1.0L - cosl(phase)) * 0.5;
}

static void lpec_init_pitch_windows(LPECContext *s)
{
    int n = s->frame_size;
    int i;

    for (i = 0; i < n; i++) {
        double w = lpec_raised_cos(i, n);

        s->pitch_weight[i] = w;
        s->pitch_shape[i]  = sqrt(w);
    }
}

/* sqrt raised-cosine for modes 1/3. */
static void lpec_init_f640_shape_m3(LPECContext *s)
{
    /* the reference decoder: sqrt raised-cosine over overlap=frame_size needs phase 0..pi,
 * so table period n = 2*frame_size (2048 @ 16 kHz, 1024 @ 8 kHz).*/
    int n = 2 * s->frame_size;

    for (int i = 0; i < n; i++)
        s->f640_shape_m3[i] = sqrt(lpec_raised_cos(i, n));
}

/* mode 2 uses the b400 shape; modes 1/3 use b380. */
static const double *lpec_f640_window(const LPECContext *s, int mode)
{
    if (mode == 1 || mode == 3)
        return s->f640_shape_m3;
    return s->pitch_shape;
}

static int lpec_frame_bit_base(const LPECContext *s)
{
    int l1c = s->frame_size;
    int rate_out = s->avctx->sample_rate;
    int rate_in  = s->codec_rate_in;

    /* Dual-rate LP (InitCodec 8000/6000): scales with rate_in/rate_out. */
    if (rate_in > 0 && rate_in < 0x1771 && rate_out > 0 && rate_in != rate_out)
        l1c = (l1c * rate_in + rate_out / 2) / rate_out;
    return l1c;
}

/* Per-mode bit budgets. */
static void lpec_set_frame_bit_targets(LPECContext *s)
{
    int l1c = lpec_frame_bit_base(s);
    int l30 = l1c * 3;

    s->frame_bit_target[0] = l1c;
    s->frame_bit_target[3] = l1c;
    s->frame_bit_target[2] = (l1c * 5 + (unsigned) (l1c * 5 >> 1 >> 30)) >> 2;
    s->frame_bit_target[1] = (l30 + (unsigned) (l30 >> 1 >> 30)) >> 2;
}

static void lpec_skip_to_frame_boundary(GetBitContext *gb, int mode,
                                        const int *frame_bit_target,
                                        int frame_start)
{
    int limit = frame_start + frame_bit_target[mode & 3];
    int pos   = get_bits_count(gb);

    if (limit > pos) {
        int skip = limit - pos;

        if (skip > get_bits_left(gb))
            skip = get_bits_left(gb);
        if (skip > 0)
            skip_bits_long(gb, skip);
    }

}

/* True when the full mode bit budget is present (padding). */
static int lpec_frame_bits_complete(GetBitContext *gb, int mode,
                                    const int *frame_bit_target,
                                    int frame_start)
{
    int limit = frame_start + frame_bit_target[mode & 3];

    return get_bits_count(gb) >= limit ||
           get_bits_left(gb) >= limit - get_bits_count(gb);
}

static int lpec_complete_frame_bits(LPECContext *s, GetBitContext *gb, int mode,
                                    int frame_start, int frame_size)
{
    if (lpec_frame_bits_complete(gb, mode, s->frame_bit_target, frame_start)) {
        lpec_skip_to_frame_boundary(gb, mode, s->frame_bit_target, frame_start);
        s->pad_pending_start = -1;
    } else {
        s->pad_pending_start = frame_start;
        s->pad_pending_mode  = mode;
    }
    s->pending_frame_start = -1;
    return frame_size;
}

static int lpec_finish_frame(LPECContext *s, GetBitContext *gb, int mode,
                             int frame_start, int frame_size)
{
    s->prev_decode_mode = mode;
    return lpec_complete_frame_bits(s, gb, mode, frame_start, frame_size);
}

/* --------------------------------------------------------------- */
/* LSF decoding */
/* --------------------------------------------------------------- */

/* the reference decoder: sort + min spacing only (no endpoint clamp loops). */
static void lpec_xa_stabilize_lsf_once(double *lsf, int order)
{
    lsf[0] = 0.0;
    for (int i = 1; i <= order; i++) {
        if (lsf[i] < lsf[i - 1]) {
            double t = lsf[i];
            lsf[i]     = lsf[i - 1];
            lsf[i - 1] = t;
            if (i >= 2 && lsf[i - 1] < lsf[i - 2]) {
                t = lsf[i - 1];
                lsf[i - 1] = lsf[i - 2];
                lsf[i - 2] = t;
            }
        }
    }
    lsf[0] = 0.0;
    for (int i = 1; i <= order; i++) {
        if (lsf[i] - lsf[i - 1] < 0.01) {
            if (i < 2) {
                lsf[i] = 0.01;
            } else {
                const double curr = lsf[i];
                const double prev = lsf[i - 1];

                lsf[i]     = (0.01 + prev + curr) * 0.5;
                lsf[i - 1] = ((prev + curr) - 0.01) * 0.5;
            }
        }
    }
    lsf[0] = 0.0;
}

/* the reference decoder: single sort + min-spacing pass (no multi-pass loop). */
static void lpec_xa_stabilize_lsf(double *lsf, int order)
{
    lpec_xa_stabilize_lsf_once(lsf, order);
}

/* lsf[0] sentinel (0), active lines at lsf[1..order]. */
static void lpec_stabilize_lsf(double *lsf, int order)
{
    int half = order / 2;
    double d2;

    if (half < order) {
        d2 = 0.49;
        for (int i = order; i > half; i--) {
            if (lsf[i] >= 0.5)
                lsf[i] = d2;
            d2 = lsf[i] - 0.01;
        }
    }
    if (half > 0) {
        d2 = 0.01;
        for (int i = 1; i <= half; i++) {
            if (lsf[i] < 0.0)
                lsf[i] = d2;
            d2 = lsf[i] + 0.01;
        }
    }
    for (int i = 1; i <= order; i++) {
        if (lsf[i] < lsf[i - 1]) {
            double t = lsf[i];
            lsf[i]     = lsf[i - 1];
            lsf[i - 1] = t;
            if (i >= 2 && lsf[i - 1] < lsf[i - 2]) {
                t = lsf[i - 1];
                lsf[i - 1] = lsf[i - 2];
                lsf[i - 2] = t;
            }
        }
    }
    for (int i = 1; i <= order; i++) {
        if (lsf[i] - lsf[i - 1] < 0.01) {
            if (i < 2) {
                lsf[i] = 0.01;
            } else {
                /* uses pre-update lsf[i] for both lines. */
                const double curr = lsf[i];
                const double prev = lsf[i - 1];

                lsf[i]     = (0.01 + prev + curr) * 0.5;
                lsf[i - 1] = ((prev + curr) - 0.01) * 0.5;
            }
        }
    }
    lsf[0] = 0.0;
}

/* refresh (Sony param 1 -> slot1, 2 -> slot2). */
static void lpec_refresh_half_lsf_i16(LPECContext *s, int sony_half)
{
    if ((unsigned) sony_half < 1 || sony_half > 2)
        return;
    if (s->is_xaudio_lp)
        ff_lpec_xa_lsf_indices_to_i16(s->lpc_order, s->num_lsf_cb,
                                   s->half_lsf_idx[sony_half], s->half_lsf_i16[sony_half]);
    else
        ff_lpec_lsf_indices_to_i16(s->num_bands, s->lpc_order, s->num_lsf_cb,
                                s->half_lsf_idx[sony_half], s->half_lsf_i16[sony_half]);
}

/* flag==0: average slot0 (0x2e8) and slot2 (0x32c) into slot1 (0x30a). */
static void lpec_avg_half_lsf_i16(LPECContext *s)
{
    int i, order = s->lpc_order;

    for (i = 0; i <= order; i++) {
        uint32_t u = (uint32_t) (int) s->half_lsf_i16[0][i] +
                     (uint32_t) (int) s->half_lsf_i16[2][i];

        s->half_lsf_i16[1][i] = (int16_t) ((int) (u + (u > 0x7fffffffU)) >> 1);
    }
}

/* --------------------------------------------------------------- */
/* LSF decode */
/* --------------------------------------------------------------- */

static void decode_lsf(LPECContext *s, const int *idx, double *lsf)
{
    const int order = s->lpc_order;
    const int cb    = s->num_lsf_cb;

    lsf[0] = 0.0; /* /: line 0 unused */
    for (int i = 0; i < order; i++) {
        lsf[i + 1] = 0;
        if (order == 16) {
            if (cb > 0)
                lsf[i + 1] += lpec_16k_lsf_quant_1[idx[0] * 16 + i];
            if (cb > 1)
                lsf[i + 1] += lpec_16k_lsf_quant_2[idx[1] * 16 + i];
            if (cb > 2)
                lsf[i + 1] += lpec_16k_lsf_quant_3[idx[2] * 16 + i];
            if (cb > 3)
                lsf[i + 1] += lpec_16k_lsf_quant_4[idx[3] * 16 + i];
        } else if (order == 10) {
            if (s->is_xaudio_lp) {
                if (cb > 0)
                    lsf[i + 1] += lpec_xa_8k_lsf_quant_1[idx[0] * 10 + i];
                if (cb > 1)
                    lsf[i + 1] += lpec_xa_8k_lsf_quant_2[idx[1] * 10 + i];
                if (cb > 2)
                    lsf[i + 1] += lpec_xa_8k_lsf_quant_3[idx[2] * 10 + i];
            } else {
                if (cb > 0)
                    lsf[i + 1] += lpec_8k_lsf_quant_1[idx[0] * 10 + i];
                if (cb > 1)
                    lsf[i + 1] += lpec_8k_lsf_quant_2[idx[1] * 10 + i];
                if (cb > 2)
                    lsf[i + 1] += lpec_8k_lsf_quant_3[idx[2] * 10 + i];
            }
        }
    }


    if (s->is_xaudio_lp)
        lpec_xa_stabilize_lsf(lsf, order);
    else
        lpec_stabilize_lsf(lsf, order);
}

/* --------------------------------------------------------------- */
/* LSF -> LPC conversion */
/* --------------------------------------------------------------- */

static void lsf_to_lpc(const double *lsf, double *lpc, int order)
{
    double P[LPEC_LPC_ORDER_MAX + 2];
    double Q[LPEC_LPC_ORDER_MAX + 2];
    double cosv[LPEC_LPC_ORDER_MAX + 2];
    double A144[LPEC_LPC_ORDER_MAX + 2];
    double Ab4[LPEC_LPC_ORDER_MAX + 2];
    int edi, ecx, k, i;

    for (k = 0; k <= order; k++) {
        P[k] = 0.0;
        Q[k] = 0.0;
    }
    /* cos on lsf[1..order]; the reference uses pi = 3.14159265359 and an
     * extended-precision cosine rounded to double. */
    for (k = 1; k <= order; k++)
        cosv[k + 1] = (double) cosl(lsf[k] * 3.14159265359 * 2.0);

    Q[0] = -1.0;
    P[0] = -1.0;
    P[1] =  1.0;
    Q[1] = -1.0;

    if (order <= 1)
        goto finish;

    ecx = 3;
    for (edi = 2; edi <= order; edi += 2, ecx += 2) {
        A144[1] = cosv[edi]     * 2.0 * Q[0];
        Ab4[1]  = cosv[edi + 1] * 2.0 * P[0];

        for (k = 2; k <= order; k++) {
            Ab4[k] = P[k - 1] * cosv[edi + 1] * 2.0 - P[k - 2];
            A144[k] = Q[k - 1] * cosv[edi]     * 2.0 - Q[k - 2];
        }

        Ab4[order + 1] = cosv[edi + 1] * P[order] * 2.0;
        A144[order + 1] = cosv[edi]     * Q[order] * 2.0;

        for (i = 1; i <= edi / 2; i++) {
            double old_p = P[i];
            double old_q = Q[i];
            double new_p = old_p - Ab4[i];
            double new_q = old_q - A144[i];

            P[i] = new_p;
            Q[i] = new_q;
            P[ecx - i] = -new_p;
            Q[ecx - i] =  new_q;
        }
    }

finish:
    for (k = 0; k < order; k += 2) {
        lpc[k]     = (P[k]     + Q[k])     * -0.5;
        lpc[k + 1] = (P[k + 1] + Q[k + 1]) * -0.5;
    }
    if (k < order)
        lpc[k] = (P[k] + Q[k]) * -0.5;
    if (!(order & 1))
        lpc[order] = (P[order] + Q[order]) * -0.5;
}

/* --------------------------------------------------------------- */
/* Interpolated LPC synthesis */
/* --------------------------------------------------------------- */


/**
 * Interpolated LPC synthesis.
 *
 * out_base = synth_buf + order; the history is in out_base[-k].
 * lpec_roll_lpc_hist rolls the tail after each frame.
 */
static void interpolated_lpc_synthesis(const double *prev_lsf,
                                       const double *curr_lsf,
                                       const double *excitation,
                                       double *out_base,
                                       int out_off,
                                       int order,
                                       int total_len,
                                       int num_seg)
{
    double interp_lsf[LPEC_LSF_MAX];
    double lpc[LPEC_LPC_ORDER_MAX + 2];
    const double step = num_seg > 0 ? 1.0 / (double)num_seg : 1.0;
    int out_pos = 0;

    for (int seg = 0; seg <= num_seg; seg++) {
        double w_curr = (double)seg * step;
        double w_prev = 1.0 - w_curr;
        int div = (seg == 0 || seg == num_seg) ? num_seg * 2 : num_seg;
        int seg_len = total_len / div;

        for (int i = 0; i <= order; i++)
            interp_lsf[i] = prev_lsf[i] * w_prev + curr_lsf[i] * w_curr;

        lsf_to_lpc(interp_lsf, lpc, order);

        for (int n = 0; n < seg_len; n++) {
            int pos = out_off + out_pos + n;
            double s = excitation[out_pos + n];

            /* LPC taps via out[pos-k]; hist in out_base[-k]. */
            for (int k = 1; k <= order; k++)
                s -= lpc[k] * out_base[pos - k];

            if (!isfinite(s))
                s = 0.0;

            out_base[pos] = s;
        }
        out_pos += seg_len;
    }
}

/* tail: last order PCM samples -> ctx hist before next frame. */
static void lpec_roll_lpc_hist(LPECContext *s)
{
    int order = s->lpc_order;
    int fs    = s->frame_size;
    double *pcm = s->synth_buf + order;

    if (order > 0 && fs > 0)
        memcpy(s->synth_buf, pcm + fs - order, order * sizeof(double));
}

/* --------------------------------------------------------------- */


/* always 8 per half, 16 full frame. */
static int lpec_lpc_segments_half(const LPECContext *s)
{
    (void) s;
    return 8;
}

static int lpec_lpc_segments_full(const LPECContext *s)
{
    (void) s;
    return 16;
}

static void lpec_roll_excit_work(LPECContext *s);

/* the reference decoder (not LPEC) holds excit OLA / 03e30 aux. */
static double *lpec_ola_hist(LPECContext *s)
{
    return s->is_xaudio_lp ? s->pitch_cb_aux : s->excit_ola_hist;
}

static void lpec_roll_pitch_win(LPECContext *s)
{
    s->pitch_win_prev     = s->pitch_win_half2;
    s->pitch_lag_off_prev = s->pitch_lag_off_half2;
    s->pitch_tbl_prev     = s->pitch_tbl_half2;
}

/* roll half-LSF CB indices + pitch (LSF doubles handled per mode). */
static void lpec_roll_half_lsf(LPECContext *s)
{
    int ncb = s->num_lsf_cb;

    memcpy(s->half_lsf_idx[0], s->half_lsf_idx[2],
           ncb * sizeof(s->half_lsf_idx[0][0]));
    memcpy(s->half_lsf_i16[0], s->half_lsf_i16[2],
           sizeof(s->half_lsf_i16[0]));
    lpec_roll_pitch_win(s);
    s->pitch_cb_tbl_prev_idx   = s->pitch_cb_half2_idx;
    s->prev_half2_pitch_cb     = s->pitch_cb_flag_h2;
    s->prev_half2_pitch_cb_idx = s->pitch_cb_half2_idx;
}

/* roll LSF / pitch state at end of each decoded frame. */
static void lpec_end_frame(LPECContext *s)
{
    int order = s->lpc_order;

    /* then */
    lpec_roll_excit_work(s);
    lpec_roll_lpc_hist(s);
    memcpy(s->lsf1, s->lsf2, (order + 1) * sizeof(double));
    lpec_roll_half_lsf(s);
}


static const LPECQmfDesc *lpec_qmf_mode2_desc(const LPECContext *s, int half_idx)
{
    if (s->frame_size >= LPEC_FRAME_16K) {
        if (half_idx == 2)
            return &s->qmf.desc1536;
        return &s->qmf.desc1024;
    }
    if (half_idx == 2)
        return &s->qmf.desc768;
    return &s->qmf.desc512;
}

static void lpec_qmf_mode2_process(LPECContext *s, double *buf, int off,
                                   int seg_len, int half_idx)
{
    const LPECQmfDesc *desc = lpec_qmf_mode2_desc(s, half_idx);

    /* the reference zeros excit_src[seg:flen) before mode-2 h2 ab80 only. */
    if (half_idx == 2 && seg_len < desc->len &&
        off + desc->len <= LPEC_EXCIT_SRC_MAX)
        memset(buf + off + seg_len, 0, (desc->len - seg_len) * sizeof(double));

    /* In-place when buf spans desc->len (the reference path). Scratch tmp is only
 * LPEC_QMF_MAX; desc1536+seg1024 needs 1536 samples -- must not scratch.*/
    if (off + desc->len <= LPEC_EXCIT_SRC_MAX)
        ff_lpec_qmf_process_seg(&s->qmf, buf + off, seg_len, desc);
    else if (desc->len > seg_len)
        ff_lpec_qmf_process_scratch(&s->qmf, buf + off, seg_len, desc);
    else
        ff_lpec_qmf_process_seg(&s->qmf, buf + off, seg_len, desc);
}

/* Mode-1/3 ab80 @ 16 kHz: mode 1 desc1536; mode 3 desc2048. */
static const LPECQmfDesc *lpec_qmf_mode13_desc(const LPECContext *s, int mode)
{
    if (s->frame_size >= LPEC_FRAME_16K)
        return mode == 3 ? &s->qmf.desc2048 : &s->qmf.desc1536;
    return mode == 1 ? &s->qmf.desc768 : &s->qmf.desc1024;
}

static void lpec_qmf_mode13_process(LPECContext *s, int mode, double *buf, int off,
                                    int seg_len)
{
    const LPECQmfDesc *desc = lpec_qmf_mode13_desc(s, mode);

    /* the reference zeros excit_src[seg:flen) before ab80 when desc spans more than seg. */
    if (desc->len > seg_len && off + desc->len <= LPEC_EXCIT_SRC_MAX)
        memset(buf + off + seg_len, 0, (desc->len - seg_len) * sizeof(double));

    /* In-place when excit_src spans desc->len (the reference path); scratch tmp is only
 * LPEC_QMF_MAX wide, so prefer in-place for the large 2048-tap descriptor.*/
    if (off + desc->len <= LPEC_EXCIT_SRC_MAX)
        ff_lpec_qmf_process_seg(&s->qmf, buf + off, seg_len, desc);
    else if (desc->len > seg_len)
        ff_lpec_qmf_process_scratch(&s->qmf, buf + off, seg_len, desc);
    else
        ff_lpec_qmf_process_seg(&s->qmf, buf + off, seg_len, desc);
}

/* overlap-add excitation segment into frame buffer. */
static void lpec_excit_window_f640(LPECContext *s, int half_idx, double *dst,
                                   const double *src, double *hist,
                                   const double *window, int overlap, int copy_len)
{
    for (int i = 0; i < overlap; i++)
        dst[i] = src[i] * window[i] + hist[i] * window[overlap - 1 - i];
    memcpy(hist, src + overlap, copy_len * sizeof(double));
}

static const double *lpec_f430_pitch_table(const LPECContext *s)
{
    if (s->num_bands == 10)
        return lpec_16k_f430_coef;
    /* at 8 kHz the reference decoder uses pitch_cb here, not the LPEC 8 kHz pitch_gain */
    if (s->is_xaudio_lp)
        return lpec_xa_8k_pitch_cb;
    return lpec_8k_pitch_gain;
}

/* unvoiced: *(ctx+half*4+0x368) = & (identity triple). */
static const double lpec_f430_unvoiced_tbl[3] = { 1.0, 1.0, 1.0 };

static const double *lpec_pitch_f430_tbl(LPECContext *s, int lag_off, int voiced)
{
    if (!voiced)
        return lpec_f430_unvoiced_tbl;
    if (lag_off < 0)
        lag_off = 0;
    if (lag_off > 63)
        lag_off = 63;
    if (s->is_xaudio_lp)
        return (const double *)(lpec_xa_8k_pitch_i16 + lag_off * 3);
    return lpec_f430_pitch_table(s) + lag_off * 3;
}

/* the reference decoder's f8e0 is floor() */
static double lpec_f8e0_floor(double x)
{
    if (!isfinite(x))
        return x;
    return floor(x);
}

static double lpec_e330_quant(double x)
{
    return lpec_f8e0_floor(x * 4096.0) / 32768.0;
}

/* Levinson-style coefficient refinement. */
static int lpec_e330(int n, const double *diag, double *mat, double *tmp, double *upd)
{
    int m, k;

    if (n < 0)
        return 0;

    for (m = 0; m <= n; m++)
        tmp[m] = lpec_e330_quant(diag[m]);

    while (n > 0) {
        double mn;

        if (fabs(tmp[n]) > 0.1225)
            return -1;

        mn = tmp[n] * 8.0;
        mat[n] = mn;

        for (k = 0; k < n; k++)
            upd[k] = lpec_f8e0_floor(((tmp[k] - tmp[n] * tmp[n - k] * 8.0) /
                                      (1.0 - mn * mn)) * 32768.0) / 32768.0;

        for (k = 0; k < n; k++)
            tmp[k] = upd[k];

        n--;
    }
    return 0;
}

/* pitch triple for voiced OLA. */
static void lpec_e4a0_coefs(const double *tbl, int pitch_win, double out[3])
{
    int n, pw = pitch_win;
    double *diag, *mat, *tmp, *upd;
    int sz;

    if (pw <= 0) {
        out[0] = tbl[0];
        out[1] = tbl[1];
        out[2] = tbl[2];
        return;
    }

    if (pw > 1020)
        pw = 1020;

    n  = pw + 1;
    sz = n + 3;
    diag = av_malloc_array(sz, 4 * sizeof(double));
    if (!diag) {
        out[0] = tbl[0];
        out[1] = tbl[1];
        out[2] = tbl[2];
        return;
    }
    mat = diag + sz;
    tmp = mat + sz;
    upd = tmp + sz;

    /* 1 - c0 z^-(pw-1) - c1 z^-pw - c2 z^-(pw+1), the taps the OLA applies */
    memset(diag, 0, sz * sizeof(double));
    diag[0]      = 1.0;
    diag[pw - 1] = -tbl[0];
    diag[pw]     = -tbl[1];
    diag[pw + 1] = -tbl[2];

    if (lpec_e330(n, diag, mat, tmp, upd) < 0) {
        out[0] = 0.0;
        out[1] = tbl[1];
        out[2] = 0.0;
    } else {
        out[0] = tbl[0];
        out[1] = tbl[1];
        out[2] = tbl[2];
    }
    av_free(diag);
}

/*
 * Copy or voiced OLA-filter into the synthesis work buffer. pitch_win is the
 * lookback distance into work; when it exceeds len, early samples tap only
 * from work_base (the slack area).
 */
static void lpec_excit_ola_f430(LPECContext *s, int seg, double *work, const double *src,
                                int len, int pitch_win, const double *lag_tbl)
{
    const double *work_base = s->excit_work;
    double c[3];
    double xa_tbl[3];
    int w;


    /* the reference decoder voiced 3-tap OLA: identical recurrence to the LPEC path
 * below; the lag coefs come from xa_pitch_f430 (i16/32768). Off-switch for A/B.*/



    if (pitch_win == 0) {
        memcpy(work, src, len * sizeof(double));
        return;
    }

    if (!lag_tbl)
        lag_tbl = lpec_f430_unvoiced_tbl;

    /* The stored Xaudio pointer is the Q15 triple used by the excitation
     * generator; the OLA taps come from the matching double table. */
    if (s->is_xaudio_lp && lag_tbl != lpec_f430_unvoiced_tbl) {
        int idx = (const int16_t *)lag_tbl - lpec_xa_8k_pitch_i16;

        memcpy(xa_tbl, lpec_xa_8k_f430_coef + idx, sizeof(xa_tbl));
        lag_tbl = xa_tbl;
    }

    lpec_e4a0_coefs(lag_tbl, pitch_win, c);

    w = pitch_win;
    if (w < 1)
        w = 1;

    for (int i = 0; i < len; i++) {
        double v = src[i];

        if (work + i - (w + 1) >= work_base)
            v = work[i - (w + 1)] * c[2] + v;
        if (work + i - w >= work_base)
            v = v + c[1] * work[i - w];
        if (work + i - (w - 1) >= work_base)
            v = v + work[i - (w - 1)] * c[0];

        work[i] = v;
    }

}

static void lpec_maybe_inject_frame_excit(LPECContext *s)
{

}

/* mode 1: the segment is subfr_size (256 @ 8 kHz). */
static int lpec_mode1_ab80_len(const LPECContext *s)
{
    if (s->is_xaudio_lp && s->frame_size < LPEC_FRAME_16K)
        return s->subfr_size;
    return s->qmf_seg_m2_h1;
}

/* pitch_cb_aux is the dc80 OLA history only (lpec_ola_hist for xaudio). */
static double *lpec_shaped_excit_dst(LPECContext *s, int mode, double *dst)
{
    if (s->is_xaudio_lp && s->frame_size < LPEC_FRAME_16K && (mode == 1 || mode == 3))
        return s->excit_work + LPEC_WORK_SLACK;
    return dst;
}

static const double *lpec_prepare_synth_excit(LPECContext *s, int mode,
                                              const double *src, double *dst)
{
    const int fo = s->frame_out_size;
    const int sf = s->subfr_size;
    const int fs = s->frame_size;
    double *work = s->excit_work + LPEC_WORK_SLACK;
    double *shaped = lpec_shaped_excit_dst(s, mode, dst);


    if (mode == 1) {
        /* mode 1: f640 uses excit_ola_hist unchanged (no excit_src->ola copy). */
        lpec_excit_window_f640(s, 1, shaped, src, lpec_ola_hist(s),
                               lpec_f640_window(s, mode), fs,
                               lpec_mode1_ab80_len(s));
        /* mode 1: a single f640, then f430; no excit_src->ola refresh */
    } else if (mode == 3) {
        lpec_excit_window_f640(s, 2, shaped, src, lpec_ola_hist(s),
                               lpec_f640_window(s, mode), fs, fs);
    }
    if (mode == 3 || mode == 1) {
        int seg = s->qmf_seg_m2_h1;

        lpec_excit_ola_f430(s, 0, work, shaped, seg, s->pitch_win_prev, s->pitch_tbl_prev);
        lpec_excit_ola_f430(s, 1, work + seg, shaped + seg, seg,
                            s->pitch_win_half2, s->pitch_tbl_half2);
        return work;
    }
    if (mode == 2) {
        /*
         * Modes 0 and 2 share the 128/256/128 segment lengths (8 kHz); modes 1 and 3
         * use the subframe length twice.
         */
        if (s->is_xaudio_lp) {
            lpec_excit_ola_f430(s, 0, work, src, fo, s->pitch_win_prev,
                                s->pitch_tbl_prev);
            lpec_excit_ola_f430(s, 1, work + fo, src + fo, sf,
                                s->pitch_win_half1, s->pitch_tbl_half1);
            lpec_excit_ola_f430(s, 2, work + fo + sf, src + fo + sf, fo,
                                s->pitch_win_half2, s->pitch_tbl_half2);
        } else {
            const int qmf_seg = s->qmf_seg_m2_h1;

            lpec_excit_ola_f430(s, 0, work, src, fo, s->pitch_win_prev,
                                s->pitch_tbl_prev);
            lpec_excit_ola_f430(s, 1, work + fo, src + fo, qmf_seg,
                                s->pitch_win_half1, s->pitch_tbl_half1);
            lpec_excit_ola_f430(s, 2, work + fo + qmf_seg, src + fo + qmf_seg, fo,
                                s->pitch_win_half2, s->pitch_tbl_half2);
        }
        return work;
    }

    /* Mode 0: pitch-CB output in src; three f430 segments (no f640 in reference decoder).
 * fo/sf/fo layout (128/256/128 @ 8 kHz) for the reference decoder and the reference decoder.*/
    lpec_excit_ola_f430(s, 0, work, src, fo, s->pitch_win_prev, s->pitch_tbl_prev);
    lpec_excit_ola_f430(s, 1, work + fo, src + fo, sf, s->pitch_win_half1,
                        s->pitch_tbl_half1);
    lpec_excit_ola_f430(s, 2, work + fo + sf, src + fo + sf, fo,
                        s->pitch_win_half2, s->pitch_tbl_half2);
    return work;
}

static void lpec_roll_excit_work(LPECContext *s)
{
    const int roll_bytes = 0x800;
    double *base = s->excit_work;
    int fs = s->frame_size;

    if (fs > 0 && roll_bytes > 0 &&
        fs + roll_bytes / (int)sizeof(double) <= LPEC_WORK_SLACK + LPEC_MAX_FRAME)
        memmove(base, base + fs, roll_bytes);
}

static void lpec_save_frame(LPECContext *s, const double *out,
                            const double *excit, int frame_size, int mode)
{
    int sf = s->subfr_size;


    memcpy(s->mode0_shaped_tail, s->frame_excit + sf, sf * sizeof(double));
    memcpy(s->frame_out,   out,   frame_size * sizeof(double));
    memcpy(s->frame_excit, excit, frame_size * sizeof(double));
    /* excit_src and frame_excit stay separate; ab80 (desc768/1024) updates
     * excit_src[seg:flen) in place, so there is no memmove at the end of a frame. */

}

/**
 * Mode-0 pitch codebook shaping. The aux copy is always the upper half of
 * excit_src (excit_src[512:1024] at 16 kHz); half 2 overwrites the lower
 * slot, and the QMF filters [0:subfr).
 */
static void lpec_apply_pitch_cb_mode0(LPECContext *s, double *dst_half,
                                      const double *src_lo,
                                      const double *aux_copy_src,
                                      int subfr_len,
                                      const double *aux_in,
                                      int cb_a, int cb_b, int idx_a, int idx_b,
                                      int update_aux, int half_idx)
{
    const double *shape  = s->pitch_shape;
    const double *weight = s->pitch_weight;
    double *aux_store = lpec_ola_hist(s);
    const double *aux = aux_in ? aux_in : aux_store;
    int half = subfr_len;
    int frame_len = s->frame_size;

    if (!cb_a && !cb_b) {
        for (int i = 0; i < half; i++)
            dst_half[i] = shape[i] * src_lo[i] + aux[i] * shape[i + half];
    } else {
        const double *tbl0 = lpec_pitch_cb_ptr(s, cb_a ? idx_a : -1);
        const double *tbl1 = lpec_pitch_cb_ptr(s, cb_b ? idx_b : -1);


        for (int seg = 0; seg < 4; seg++) {
            int start = (seg * frame_len) >> 3;
            int end   = ((seg + 1) * frame_len) >> 3;
            double d6 = tbl0[4 + seg];
            double d7 = tbl1[seg];
            double d8 = tbl0[7 - seg];
            double d9 = tbl1[3 - seg];

            if (end > half)
                end = half;
            for (int i = start; i < end; i++) {
                int j = half + i;
                double den = weight[i] * (d9 / d7) + weight[j] * (d8 / d6);

                if (fabs(den) < 1e-30)
                    den = 1e-30;
                dst_half[i] = (d9 * shape[i] * src_lo[i]
                               + d8 * shape[j] * aux[i]) / den;
            }
        }
    }


    if (update_aux)
        memcpy(aux_store, aux_copy_src + half, half * sizeof(double));

}

/* --------------------------------------------------------------- */
/* Excitation generation */
/* --------------------------------------------------------------- */

/**
 * Per-half pitch codebook select (loop at 0x13c):
 * 1 bit; if set, 7-bit index scaled to lpec_*_pitch_cb (idx * 0x40 + base).
 * Returns CB index 0..127, or -1 if unvoiced.
*/
/* pitch table idx = *(ctx + half*4 + 0x140); if <0 use +0x144. */
static int lpec_pitch_route_arg(LPECContext *s, int mode, int half)
{
    int route = ff_lpec_excit_route_idx(mode, half);
    int idx = route - 1;
    int v;

    if (idx < 0)
        return 0;
    if (idx >= FF_ARRAY_ELEMS(s->half_lsf_cb0))
        idx = FF_ARRAY_ELEMS(s->half_lsf_cb0) - 1;

    v = s->half_lsf_cb0[idx];
    if (v < 0 && idx + 1 < FF_ARRAY_ELEMS(s->half_lsf_cb0))
        v = s->half_lsf_cb0[idx + 1];
    return v >= 0 ? v : 0;
}

static int read_pitch_cb_half(GetBitContext *gb)
{
    if (get_bits_left(gb) < 1)
        return AVERROR(EAGAIN);
    if (!get_bits(gb, 1))
        return -1;
    if (get_bits_left(gb) < 7)
        return AVERROR(EAGAIN);
    return get_bits(gb, 7);
}

/* store the lag table pointer per half. */
static void lpec_bind_pitch_tbl(LPECContext *s, int *lag_off_out, int voiced, int lag_off)
{
    const double *tbl = lpec_pitch_f430_tbl(s, lag_off, voiced);

    if (lag_off_out == &s->pitch_lag_off_half1)
        s->pitch_tbl_half1 = tbl;
    else if (lag_off_out == &s->pitch_lag_off_half2)
        s->pitch_tbl_half2 = tbl;
}

/**
 * Read pitch parameters:
 * pitch_voiced_bits-bit field; 0 = unvoiced, else 6-bit lag offset.
 * Returns lag, or 0 if unvoiced.
*/
static int read_pitch(LPECContext *s, GetBitContext *gb, int *voiced_out,
                      int *win_idx_out, int *lag_off_out)
{
    int n = s->pitch_voiced_bits;

    if (get_bits_left(gb) < n)
        return AVERROR(EAGAIN);
    int v = get_bits(gb, n);
    if (win_idx_out)
        *win_idx_out = v;
    *voiced_out = v != 0;
    if (!v) {
        if (lag_off_out)
            *lag_off_out = 0;
        lpec_bind_pitch_tbl(s, lag_off_out, 0, 0);
        return 0;
    }
    if (get_bits_left(gb) < 6)
        return AVERROR(EAGAIN);
    int off = get_bits(gb, 6);
    if (lag_off_out)
        *lag_off_out = off;
    lpec_bind_pitch_tbl(s, lag_off_out, 1, off);
    return LPEC_PITCH_LAG_BASE + off;
}

static int lpec_excit_bits_limit(const LPECContext *s, int mode, int half,
                                 int frame_start)
{
    if (mode == 2 && half == 1)
        return frame_start + s->frame_bit_target[0];
    return frame_start + s->frame_bit_target[mode & 3];
}

#define LPEC_GEN_EXCIT_HALF(s, gb, dst, half, lag, lag_off, win, voiced,      \
                            pitch_tbl, frame_start, mode, frame_size, order,  \
                            pitch_cb, lsf_i16_override)                       \
    ((s)->excit.dump_frame = (s)->lpec_frame_idx,                             \
     ff_lpec_generate_excitation(&(s)->excit, gb, dst, (s)->subfr_size, half, lag,  \
                             lag_off, win, voiced, pitch_tbl,                   \
                             lpec_excit_bit_budget(s, gb, frame_start, mode,    \
                                                  pitch_cb,                     \
                                                  mode == 0 || mode == 2, half),\
                             lpec_excit_bits_limit(s, mode, half, frame_start), \
                             mode, lpec_pitch_route_arg(s, mode, half),         \
                             (s)->half_lsf_idx[half],                            \
                             lsf_i16_override, frame_size, order,               \
                             (s)->num_lsf_cb))
static int lpec_excit_bit_budget(LPECContext *s, GetBitContext *gb,
                                 int frame_start, int mode, int cb_overhead,
                                 int per_half, int half)
{
    int used = get_bits_count(gb) - frame_start;
    int rem, budget;
    int avail = get_bits_left(gb);

    if (mode == 2 && half == 1) {
        /* mode 2 half1: half of the remaining bits, no pitch CB. */
        rem = s->frame_bit_target[0] - used;
        budget = (rem >> 1) - cb_overhead;
    } else if (mode == 2 && half == 2) {
        rem = s->frame_bit_target[2] - used;
        budget = rem - cb_overhead;
    } else if (per_half && mode == 0) {
        /* mode 0: after pitch, before pitch CB; same for both halves. */
        budget = (s->mode0_excit_rem >> 1) - cb_overhead;
    } else if (per_half) {
        rem = s->frame_bit_target[mode & 3] - used;
        budget = (rem >> 1) - cb_overhead;
    } else {
        rem = s->frame_bit_target[mode & 3] - used;
        budget = rem - cb_overhead;
    }

    if (budget < 0)
        budget = 0;
    if (budget > avail)
        budget = avail;
    return budget;
}

/* QMF post-process on excitation before f640 / pitch CB. */
static void lpec_postprocess_excit(LPECContext *s, int mode, double *buf, int off,
                                   int len, int half_idx)
{
    /* QMF on excit_src; seg len/desc vary by mode (the reference decoder). */
    if (mode == 0 || mode == 2) {
        int qmf_len;
        int desc_idx = 0;

        if (mode == 2 && half_idx == 2) {
            qmf_len  = s->frame_size;
            desc_idx = 1;
        } else if (mode == 2 && half_idx == 1) {
            qmf_len  = s->qmf_seg_m2_h1;
            desc_idx = 0;
        } else if (mode == 0) {
            qmf_len = s->subfr_size;
        } else {
            qmf_len = s->qmf_seg_m2_h1;
        }
        {
            /* Mode 2: ab80 in-place on excit_src (same ptr, seg=len). */
            if (mode == 2 && half_idx == 2) {
                const LPECQmfDesc *desc = lpec_qmf_mode2_desc(s, 2);
                int ext = desc->len - qmf_len;

                /* the reference leaves excit_src[len:flen) zero before h2 ab80. */
                if (ext > 0 && off + qmf_len + ext <= LPEC_EXCIT_SRC_MAX)
                    memset(buf + off + qmf_len, 0, ext * sizeof(double));
            }
            if (mode == 2 && (half_idx == 1 || half_idx == 2)) {
                lpec_qmf_mode2_process(s, buf, off, qmf_len, half_idx);
            } else if (mode == 0 && s->frame_size >= LPEC_FRAME_16K) {
                const LPECQmfDesc *desc = s->qmf.active ? s->qmf.active : &s->qmf.desc1024;

                /* 16 kHz mode-0 follows the descriptor selection (1024 taps). */
                if (desc->len > qmf_len)
                    ff_lpec_qmf_process_scratch(&s->qmf, buf + off, qmf_len, desc);
                else
                    ff_lpec_qmf_process_seg(&s->qmf, buf + off, qmf_len, desc);
            } else
                ff_lpec_qmf_process_slot(&s->qmf, buf + off, qmf_len, desc_idx);
        }
    } else if (mode == 3) {
        if (s->frame_size >= LPEC_FRAME_16K) {
            /* 16 kHz: mode 3 uses the 1024-tap descriptor family. */
            lpec_qmf_mode13_process(s, mode, buf, off, len);
        } else {
            /* 8 kHz keeps slot path with persistent tail state. */
            ff_lpec_qmf_process_slot(&s->qmf, buf + off, len, 2);
        }
    } else if (mode == 1) {
        int qmf_len = lpec_mode1_ab80_len(s);
        const LPECQmfDesc *desc = s->frame_size >= LPEC_FRAME_16K
            ? lpec_qmf_mode13_desc(s, mode)
            : &s->qmf.desc768;

        /* Mode-1 transform in place on excit_src; excit_src[seg:flen) is not
         * zeroed first. */
        ff_lpec_qmf_process_seg(&s->qmf, buf + off, qmf_len, desc);
    }
}

/* --------------------------------------------------------------- */
/* Initialiser / close */
/* --------------------------------------------------------------- */

/* the reference decoder profile rates (d430 in, d598 out). */
static int lpec_rates_from_quality(int quality, int *rate_in, int *rate_out)
{
    switch (quality) {
    case 0x15:
    case 0x2a:
    case 0x4a:
        *rate_in = *rate_out = 16000;
        return 0;
    case 0x19:
    case 0x2c:
    case 0x4c:
        *rate_in  = 6000;
        *rate_out = 8000;
        return 0;
    default:
        return AVERROR(EINVAL);
    }
}

static int lpec_chunk_bytes(const LPECContext *s, uint8_t lead)
{
    switch (lead & 0xC0) {
    case 0x00: return s->chunk_sz[0];
    case 0x40: return s->chunk_sz[1];
    case 0x80: return s->chunk_sz[2];
    default:   return s->chunk_sz[3];
    }
}

static void lpec_consume_chunk(LPECContext *s, int nbytes)
{
    if (nbytes <= 0 || nbytes > s->chunk_carry_len)
        return;
    memmove(s->carry_buf, s->carry_buf + nbytes, s->chunk_carry_len - nbytes);
    s->chunk_carry_len -= nbytes;
}

static int decode_one_lpec_frame(LPECContext *s, GetBitContext *gb,
                                 double *output, int frame_size, int subfr_size);

/* Drop consumed frame bits from the front of stream_buf (bit-accurate). */
static void lpec_stream_consume_bits(LPECContext *s, int bits)
{
    int total = s->stream_bit_off + bits;
    int nbytes = total >> 3;
    int new_off = total & 7;

    if (nbytes > s->stream_len)
        nbytes = s->stream_len;
    if (nbytes > 0) {
        memmove(s->stream_buf, s->stream_buf + nbytes, s->stream_len - nbytes);
        s->stream_len -= nbytes;
    }
    s->stream_bit_off = s->stream_len ? new_off : 0;
}

/* Feed 48-byte LPEC chunks from carry_buf (the reference Decode chunk size). */
static void lpec_feed_chunks(LPECContext *s)
{
    while (s->chunk_carry_len > 0) {
        int need = lpec_chunk_bytes(s, s->carry_buf[0]);

        if (need <= 0 || s->chunk_carry_len < need)
            break;
        if (s->stream_len + need > (int)sizeof(s->stream_buf))
            break;
        memcpy(s->stream_buf + s->stream_len, s->carry_buf, need);
        s->stream_len += need;
        lpec_consume_chunk(s, need);
    }
}

/* Decode as many frames as stream_buf allows (called before new packet bytes). */
static int lpec_decode_from_stream(LPECContext *s, double *out, int out_cap,
                                   int frame_size, int subfr_size,
                                   int max_frames, int *frames_out)
{
    GetBitContext gb;
    int total_samples = 0;
    int num_frames = 0;
    int ret;

    while (s->stream_len > 0 &&
           num_frames < max_frames &&
           total_samples + frame_size <= out_cap) {
        int n, bits_pos;

        if ((ret = init_get_bits(&gb, s->stream_buf, s->stream_len * 8)) < 0)
            return ret;

        if (s->stream_bit_off > 0)
            skip_bits(&gb, s->stream_bit_off);

        if (get_bits_left(&gb) < 2)
            break;

        bits_pos = get_bits_count(&gb);
        n = decode_one_lpec_frame(s, &gb, out + total_samples,
                                  frame_size, subfr_size);

        if (n == AVERROR(EAGAIN))
            break;

        if (n < 0) {
            if (s->pending_frame_start >= 0) {
                lpec_skip_to_frame_boundary(&gb, s->pending_frame_mode,
                                            s->frame_bit_target,
                                            s->pending_frame_start);
                s->pending_frame_start = -1;
                lpec_stream_consume_bits(s, get_bits_count(&gb));
            } else {
                lpec_stream_consume_bits(s, 8);
            }
            continue;
        }

        lpec_stream_consume_bits(s, get_bits_count(&gb) - bits_pos);
        total_samples += n;
        num_frames++;
        s->lpec_frame_idx++;
    }

    *frames_out = num_frames;
    return total_samples;
}

static av_cold int lpec_decode_init(AVCodecContext *avctx)
{
    LPECContext *s = avctx->priv_data;

    if (avctx->ch_layout.nb_channels != 1) {
        av_log(avctx, AV_LOG_ERROR, "LPEC: only mono supported\n");
        return AVERROR_PATCHWELCOME;
    }
    s->avctx = avctx;
    avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    s->codec_rate_in = 0;
    s->is_xaudio_lp = avctx->codec_tag == 0x19;

    {
        int rate_in = 0, rate_out = 0;

        if (lpec_rates_from_quality(avctx->codec_tag, &rate_in, &rate_out) >= 0) {
            avctx->sample_rate = rate_out;
            s->codec_rate_in   = rate_in;
        } else if (avctx->sample_rate == 6000) {
            /* LP LPEC header rate without a known quality tag. */
            avctx->sample_rate = 8000;
            s->codec_rate_in   = 6000;
        }
    }

    /* Parameters from the reference decoder init (the reference decoder ) */
    if (avctx->sample_rate >= 8001) {
        int cap = avctx->sample_rate >= 22051 ? 4096 : 2048;

        s->frame_size = LPEC_FRAME_16K;
        s->subfr_size = LPEC_SUBFR_16K;
        s->frame_out_size = LPEC_FRAME_16K / 4;
        s->qmf_seg_m2_h1 = cap >> 2;
        s->lpc_order  = 16;
        s->num_lsf_cb = 4;
        s->num_bands  = 10;
    } else {
        int cap = 1024;

        s->frame_size = LPEC_FRAME_8K;
        s->subfr_size = LPEC_SUBFR_8K;
        s->frame_out_size = LPEC_FRAME_8K / 4;
        s->qmf_seg_m2_h1 = cap >> 2;
        s->lpc_order  = 10;
        s->num_lsf_cb = 3;
        s->num_bands  = 8;
    }

    if (s->frame_size >= LPEC_FRAME_16K) {
        s->chunk_sz[0] = s->chunk_sz[3] = 128;
        s->chunk_sz[1] = 96;
        s->chunk_sz[2] = 160;
    } else {
        s->chunk_sz[0] = s->chunk_sz[3] = 48;
        s->chunk_sz[1] = 36;
        s->chunk_sz[2] = 60;
    }

    s->pitch_voiced_bits = 7;
    {
        int t = (avctx->sample_rate / 8000) * 0x78;

        if (t > 0x80)
            s->pitch_voiced_bits++;
        if (t > 0x100)
            s->pitch_voiced_bits++;
    }

    /* Default LSFs i * 0.48 / (order + 1); the Q16 copy seeds slot 0. */
    {
        double step = 0.48 / (double)(s->lpc_order + 1);

        for (int i = 0; i <= s->lpc_order; i++) {
            double v = (double) i * step;

            s->prev_lsf[i] = s->lsf1[i] = s->lsf2[i] = v;
        }
    }

    memset(s->synth_buf, 0, sizeof(s->synth_buf));
    memset(&s->excit, 0, sizeof(s->excit));
    ff_lpec_excit_init(&s->excit, avctx->sample_rate, s->codec_rate_in,
                    s->num_bands, s->frame_size, s->subfr_size,
                    s->is_xaudio_lp);
    memset(s->frame_excit, 0, sizeof(s->frame_excit));
    memset(s->excit_src, 0, sizeof(s->excit_src));
    memset(s->excit_work, 0, sizeof(s->excit_work));
    memset(s->excit_ola_hist, 0, sizeof(s->excit_ola_hist));
    memset(s->frame_out,   0, sizeof(s->frame_out));
    s->noise_pos      = 0;
    s->half_lsf_cb0[0] = s->half_lsf_cb0[1] = 0;
    memset(s->half_lsf_idx, 0, sizeof(s->half_lsf_idx));
    memset(s->half_lsf_i16, 0, sizeof(s->half_lsf_i16));
    for (int i = 0; i <= s->lpc_order; i++)
        s->half_lsf_i16[0][i] = (int16_t) floor(s->prev_lsf[i] * 65536.0);
    s->prev_pitch_lag = LPEC_PITCH_LAG_BASE;
    s->frame_count    = 0;
    s->lpec_frame_idx = 0;
    s->pending_frame_start = -1;
    s->pending_frame_mode  = 0;
    s->pad_pending_start   = -1;
    s->pad_pending_mode    = 0;
    s->synth_half_flag     = 0;
    s->chunk_carry_len     = 0;
    s->stream_len          = 0;
    memset(s->carry_buf, 0, sizeof(s->carry_buf));
    memset(s->stream_buf, 0, sizeof(s->stream_buf));
    s->stream_bit_off      = 0;
    lpec_set_frame_bit_targets(s);
    lpec_init_pitch_windows(s);
    lpec_init_f640_shape_m3(s);
    memset(s->pitch_cb_aux, 0, sizeof(s->pitch_cb_aux));
    memset(s->mode0_shaped_tail, 0, sizeof(s->mode0_shaped_tail));
    s->prev_half2_pitch_cb     = 0;
    s->prev_half2_pitch_cb_idx = -1;
    s->pitch_cb_flag_h1        = 0;
    s->pitch_cb_flag_h2        = 0;
    s->pitch_cb_half2_idx      = -1;
    s->pitch_cb_tbl_prev_idx   = -1;
    s->prev_lsf_cb_half2       = 0;
    s->prev_decode_mode        = -1;
    s->next_frame_mode_peek    = -1;
    s->eof_flush_done          = 0;
    s->eof_draining            = 0;
    s->pitch_win_prev          = 0;
    s->pitch_win_half1         = 0;
    s->pitch_win_half2         = 0;
    s->pitch_lag_off_half1     = 0;
    s->pitch_lag_off_half2     = 0;
    s->pitch_lag_off_prev      = 0;
    s->pitch_tbl_prev          = lpec_f430_unvoiced_tbl;
    s->pitch_tbl_half1         = lpec_f430_unvoiced_tbl;
    s->pitch_tbl_half2         = lpec_f430_unvoiced_tbl;
    ff_lpec_qmf_init(&s->qmf);
    ff_lpec_qmf_configure(&s->qmf, avctx->sample_rate, s->codec_rate_in);
    return 0;
}

static av_cold int lpec_decode_close(AVCodecContext *avctx)
{

    return 0;
}

/* --------------------------------------------------------------- */
/* Main decode function */
/* --------------------------------------------------------------- */

static double lpec_xa_output_gain(const LPECContext *s)
{
    int rate_in  = s->codec_rate_in;
    int rate_out = s->avctx->sample_rate;

    if (!s->is_xaudio_lp || rate_in != 6000 || rate_in >= rate_out)
        return 1.0;
    /* the reference output gain when the codec runs at 6000 Hz */
    return 1.487732;
}

static void clamp_and_copy(int16_t *dst, const double *src, int len, double scale)
{
    /* floor(sample + 0.5), then int16 limiter */
    for (int i = 0; i < len; i++) {
        double v = src[i];

        if (!isfinite(v))
            v = 0.0;
        v = floor(v * scale + 0.5);
        if (v > 32767.0)
            dst[i] = 32767;
        else if (v < -32768.0)
            dst[i] = -32768;
        else
            dst[i] = (int16_t)v;
    }
}

/* --------------------------------------------------------------- */
/* Per-frame decode helper */
/* --------------------------------------------------------------- */
/**
 * Decode one LPEC frame from the bit reader.
 * Returns number of samples produced on success, AVERROR(EAGAIN) if
 * insufficient bits remain.
*/
static int decode_one_lpec_frame(LPECContext *s, GetBitContext *gb,
                                  double *output, int frame_size, int subfr_size)
{
    double cur_lsf1[LPEC_LSF_MAX];
    double cur_lsf2[LPEC_LSF_MAX];
    int lsf_idx[LPEC_NUM_LSF_CB_MAX];
    const int order = s->lpc_order;
    double *pcm = s->synth_buf + order;

    cur_lsf1[0] = cur_lsf2[0] = 0.0;
    if (get_bits_left(gb) < 2)
        return AVERROR(EAGAIN);

    if (s->pad_pending_start >= 0) {
        if (!lpec_frame_bits_complete(gb, s->pad_pending_mode, s->frame_bit_target,
                                      s->pad_pending_start) && !s->eof_draining)
            return AVERROR(EAGAIN);
        lpec_skip_to_frame_boundary(gb, s->pad_pending_mode, s->frame_bit_target,
                                    s->pad_pending_start);
        s->pad_pending_start = -1;
    }

    int frame_start = get_bits_count(gb);

    s->pending_frame_start = frame_start;

    int mode = get_bits(gb, 2);
    s->pending_frame_mode = mode;
    s->synth_half_flag = 0; /* clears each frame */
    {
        int bits_in_frame = s->frame_bit_target[mode];
        GetBitContext pk;

        s->next_frame_mode_peek = -1;
        if (bits_in_frame > 2 && get_bits_left(gb) >= bits_in_frame) {
            pk = *gb;
            skip_bits(&pk, bits_in_frame - 2);
            s->next_frame_mode_peek = show_bits(&pk, 2);
        }
    }



    memset(pcm, 0, frame_size * sizeof(double));
    memset(output, 0, frame_size * sizeof(double));

    if (mode == 0) {
        if (get_bits_left(gb) < 1)
            return AVERROR(EAGAIN);
        int flag = get_bits(gb, 1);
        s->synth_half_flag = flag;

        int lsf_bits = s->num_lsf_cb * LPEC_LSF_BITS;

        if (flag == 0) {
            if (get_bits_left(gb) < lsf_bits)
                return AVERROR(EAGAIN);
            for (int i = 0; i < s->num_lsf_cb; i++)
                lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
            decode_lsf(s, lsf_idx, cur_lsf2);
            s->half_lsf_cb0[0] = -1;
            s->half_lsf_cb0[1] = lsf_idx[0];
            memcpy(s->half_lsf_idx[2], lsf_idx, s->num_lsf_cb * sizeof(int));
            lpec_refresh_half_lsf_i16(s, 2);
            for (int i = 0; i <= order; i++)
                cur_lsf1[i] = (s->lsf1[i] + cur_lsf2[i]) * 0.5;
            lpec_avg_half_lsf_i16(s);
        } else {
            if (get_bits_left(gb) < 2 * lsf_bits)
                return AVERROR(EAGAIN);
            for (int i = 0; i < s->num_lsf_cb; i++)
                lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
            decode_lsf(s, lsf_idx, cur_lsf1);
            s->half_lsf_cb0[0] = lsf_idx[0];
            memcpy(s->half_lsf_idx[1], lsf_idx, s->num_lsf_cb * sizeof(int));
            lpec_refresh_half_lsf_i16(s, 1);

            for (int i = 0; i < s->num_lsf_cb; i++)
                lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
            decode_lsf(s, lsf_idx, cur_lsf2);
            s->half_lsf_cb0[1] = lsf_idx[0];
            memcpy(s->half_lsf_idx[2], lsf_idx, s->num_lsf_cb * sizeof(int));
            lpec_refresh_half_lsf_i16(s, 2);
        }

        int voiced1, pitch1_lag = read_pitch(s, gb, &voiced1, &s->pitch_win_half1,
                                             &s->pitch_lag_off_half1);
        if (pitch1_lag < 0)
            return pitch1_lag;
        int voiced2, pitch2_lag = read_pitch(s, gb, &voiced2, &s->pitch_win_half2,
                                             &s->pitch_lag_off_half2);
        if (pitch2_lag < 0)
            return pitch2_lag;
        if (!voiced1)
            pitch1_lag = s->prev_pitch_lag;
        if (!voiced2)
            pitch2_lag = s->prev_pitch_lag;

        s->mode0_excit_rem = s->frame_bit_target[0] - (get_bits_count(gb) - frame_start);

        int pitch1_cb = read_pitch_cb_half(gb);
        int pitch2_cb;
        if (pitch1_cb < 0 && pitch1_cb != -1)
            return pitch1_cb;
        pitch2_cb = read_pitch_cb_half(gb);
        if (pitch2_cb < 0 && pitch2_cb != -1)
            return pitch2_cb;
        s->pitch_cb_half2_idx = pitch2_cb;
        s->pitch_cb_flag_h1   = pitch1_cb >= 0;
        s->pitch_cb_flag_h2   = pitch2_cb >= 0;


        {
            if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 1, pitch1_lag,
                                    s->pitch_lag_off_half1, s->pitch_win_half1,
                                    voiced1, s->pitch_tbl_half1, frame_start, mode,
                                    frame_size, order, pitch1_cb >= 0 ? 8 : 1,
                                    s->half_lsf_i16[1]) < 0)
                return AVERROR(EAGAIN);
            lpec_postprocess_excit(s, mode, s->excit_src, 0, subfr_size, 1);
            {

            }
            /* half 0: flags /0x13c, tbl /0x360 */
            lpec_apply_pitch_cb_mode0(s, s->frame_excit, s->excit_src, s->excit_src,
                                      subfr_size, NULL, s->prev_half2_pitch_cb,
                                      s->pitch_cb_flag_h1, s->pitch_cb_tbl_prev_idx,
                                      pitch1_cb, 1, 1);
            if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 2, pitch2_lag,
                                    s->pitch_lag_off_half2, s->pitch_win_half2,
                                    voiced2, s->pitch_tbl_half2, frame_start, mode,
                                    frame_size, order, pitch2_cb >= 0 ? 8 : 1,
                                    s->half_lsf_i16[2]) < 0)
                return AVERROR(EAGAIN);
            lpec_postprocess_excit(s, mode, s->excit_src, 0, subfr_size, 2);
            lpec_apply_pitch_cb_mode0(s, s->frame_excit + subfr_size,
                                      s->excit_src, s->excit_src,
                                      subfr_size, NULL, s->pitch_cb_flag_h1,
                                      s->pitch_cb_flag_h2, pitch1_cb, pitch2_cb,
                                      1, 2);
        }

        s->prev_pitch_lag = pitch2_lag;
        s->prev_lsf_cb_half2       = s->half_lsf_cb0[1];

        const double *synth_excit = lpec_prepare_synth_excit(s, mode, s->frame_excit,
                                                             s->frame_excit);




        if (s->synth_half_flag == 0) {
            interpolated_lpc_synthesis(s->lsf1, cur_lsf2,
                                       synth_excit, pcm, 0, s->lpc_order,
                                       frame_size, lpec_lpc_segments_full(s));
        } else {
            int nseg = lpec_lpc_segments_half(s);

            /* flag==1: f1c0 lsf1->half1, half1->lsf2. */
            interpolated_lpc_synthesis(s->lsf1, cur_lsf1,
                                       synth_excit, pcm, 0, s->lpc_order,
                                       subfr_size, nseg);
            interpolated_lpc_synthesis(cur_lsf1, cur_lsf2,
                                       synth_excit + subfr_size, pcm,
                                       subfr_size, s->lpc_order, subfr_size,
                                       nseg);
        }

        memcpy(output, pcm, frame_size * sizeof(double));

        memcpy(s->prev_lsf, cur_lsf2, (order + 1) * sizeof(double));
        memcpy(s->lsf2, cur_lsf2, (order + 1) * sizeof(double));
        lpec_end_frame(s);
        lpec_save_frame(s, output, s->frame_excit, frame_size, mode);
        return lpec_finish_frame(s, gb, mode, frame_start, frame_size);

    } else if (mode == 2) {
        /*
         * Mode 2: both pitch windows, two pitch halves, averaged LSF, then
         * 05d50, ab80 and f640 twice. No flag bit and no pitch CB; f640 uses
         * the mode-2 shape. The pitch CB flags are cleared before ab80.
         */
        int lsf_bits = s->num_lsf_cb * LPEC_LSF_BITS;

        s->pitch_cb_flag_h1 = 0;
        s->pitch_cb_flag_h2 = 0;

        if (get_bits_left(gb) < lsf_bits)
            return AVERROR(EAGAIN);
        for (int i = 0; i < s->num_lsf_cb; i++)
            lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
        decode_lsf(s, lsf_idx, cur_lsf2);
        s->half_lsf_cb0[1] = lsf_idx[0];
        memcpy(s->half_lsf_idx[2], lsf_idx, s->num_lsf_cb * sizeof(int));
        lpec_refresh_half_lsf_i16(s, 2);
        for (int i = 0; i <= order; i++)
            cur_lsf1[i] = (s->lsf1[i] + cur_lsf2[i]) * 0.5;
        lpec_avg_half_lsf_i16(s);
        s->half_lsf_cb0[0] = -1;

        int voiced1, pitch1_lag = read_pitch(s, gb, &voiced1, &s->pitch_win_half1,
                                             &s->pitch_lag_off_half1);
        if (pitch1_lag < 0)
            return pitch1_lag;
        int voiced2, pitch2_lag = read_pitch(s, gb, &voiced2, &s->pitch_win_half2,
                                             &s->pitch_lag_off_half2);
        if (pitch2_lag < 0)
            return pitch2_lag;
        if (!voiced1)
            pitch1_lag = s->prev_pitch_lag;
        if (!voiced2)
            pitch2_lag = s->prev_pitch_lag;

        if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 1, pitch1_lag,
                                s->pitch_lag_off_half1, s->pitch_win_half1,
                                voiced1, s->pitch_tbl_half1, frame_start, mode,
                                frame_size, order, 0, s->half_lsf_i16[1]) < 0)
            return AVERROR(EAGAIN);
        lpec_postprocess_excit(s, mode, s->excit_src, 0, s->qmf_seg_m2_h1, 1);
        /* mode-2 h1: f640 overlap and copy are qmf_seg_m2_h1 (384 @ 8 kHz). */
        lpec_excit_window_f640(s, 1, s->frame_excit, s->excit_src, lpec_ola_hist(s),
                               lpec_f640_window(s, mode), s->qmf_seg_m2_h1,
                               s->qmf_seg_m2_h1);
        if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 2, pitch2_lag,
                                s->pitch_lag_off_half2, s->pitch_win_half2,
                                voiced2, s->pitch_tbl_half2, frame_start, mode,
                                frame_size, order, 0, s->half_lsf_i16[2]) < 0)
            return AVERROR(EAGAIN);
        lpec_postprocess_excit(s, mode, s->excit_src, 0, frame_size, 2);
        lpec_excit_window_f640(s, 2, s->frame_excit + s->qmf_seg_m2_h1, s->excit_src,
                               lpec_ola_hist(s), lpec_f640_window(s, mode),
                               s->qmf_seg_m2_h1, frame_size);
        s->prev_pitch_lag = pitch2_lag;

        lpec_maybe_inject_frame_excit(s);
        memcpy(output, s->frame_out, frame_size * sizeof(double));
        {
            const double *synth_excit = lpec_prepare_synth_excit(s, mode, s->frame_excit,
                                                                 s->frame_excit);


            /* mode 2: interpolate from lsf1 to lsf2, not from their average. */
            interpolated_lpc_synthesis(s->lsf1, cur_lsf2,
                                       synth_excit, pcm, 0, s->lpc_order,
                                       frame_size, lpec_lpc_segments_full(s));
        }
        memcpy(output, pcm, frame_size * sizeof(double));
        memcpy(s->prev_lsf, cur_lsf2, (order + 1) * sizeof(double));
        memcpy(s->lsf2, cur_lsf2, (order + 1) * sizeof(double));
        lpec_end_frame(s);
        lpec_save_frame(s, output, s->frame_excit, frame_size, mode);
        return lpec_finish_frame(s, gb, mode, frame_start, frame_size);

    } else if (mode == 1) {
        /*
 * Mode 1: second subframe only.
 * Reads new LSF-B; LSF-A is ctx lsf1 (rolled from prior frame lsf2 @ ).
*/
        if (get_bits_left(gb) < s->num_lsf_cb * LPEC_LSF_BITS)
            return AVERROR(EAGAIN);
        for (int i = 0; i < s->num_lsf_cb; i++)
            lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
        decode_lsf(s, lsf_idx, cur_lsf2);
        s->half_lsf_cb0[1] = lsf_idx[0];
        memcpy(s->half_lsf_idx[2], lsf_idx, s->num_lsf_cb * sizeof(int));
        lpec_refresh_half_lsf_i16(s, 2);
        memcpy(cur_lsf1, s->lsf1, (order + 1) * sizeof(double));


        int voiced, pitch_lag = read_pitch(s, gb, &voiced, &s->pitch_win_half2,
                                           &s->pitch_lag_off_half2);
        if (pitch_lag < 0)
            return pitch_lag;
        if (!voiced)
            pitch_lag = s->prev_pitch_lag;
        if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 2, pitch_lag,
                                s->pitch_lag_off_half2, s->pitch_win_half2,
                                voiced, s->pitch_tbl_half2, frame_start, mode,
                                frame_size, order, 0, s->half_lsf_i16[2]) < 0)
            return AVERROR(EAGAIN);
        s->prev_pitch_lag = pitch_lag;
        {

        }
        lpec_postprocess_excit(s, mode, s->excit_src, 0, lpec_mode1_ab80_len(s), 2);

        {

        }

        {
            const double *synth_excit = lpec_prepare_synth_excit(s, mode, s->excit_src,
                                                                 s->frame_excit);

            /* full-frame f1c0 for modes 1/3 */
            interpolated_lpc_synthesis(cur_lsf1, cur_lsf2, synth_excit, pcm,
                                       0, s->lpc_order, frame_size,
                                       lpec_lpc_segments_full(s));
            memcpy(output, pcm, frame_size * sizeof(double));
        }
        memcpy(s->prev_lsf, cur_lsf2, (order + 1) * sizeof(double));
        memcpy(s->lsf2, cur_lsf2, (order + 1) * sizeof(double));
        lpec_end_frame(s);
        lpec_save_frame(s, output,
                        s->is_xaudio_lp ? s->excit_work + LPEC_WORK_SLACK
                                        : s->frame_excit,
                        frame_size, mode);
        return lpec_finish_frame(s, gb, mode, frame_start, frame_size);

    } else if (mode == 3) {
        /*
 * Mode 3: second subframe only (route 2/3).
 * reference decoder: (2) before.
*/
        if (get_bits_left(gb) < s->num_lsf_cb * LPEC_LSF_BITS)
            return AVERROR(EAGAIN);
        for (int i = 0; i < s->num_lsf_cb; i++)
            lsf_idx[i] = get_bits(gb, LPEC_LSF_BITS);
        decode_lsf(s, lsf_idx, cur_lsf2);
        s->half_lsf_cb0[1] = lsf_idx[0];
        memcpy(s->half_lsf_idx[2], lsf_idx, s->num_lsf_cb * sizeof(int));
        lpec_refresh_half_lsf_i16(s, 2);
        memcpy(cur_lsf1, s->lsf1, (order + 1) * sizeof(double));

        int voiced, pitch_lag = read_pitch(s, gb, &voiced, &s->pitch_win_half2,
                                           &s->pitch_lag_off_half2);
        if (pitch_lag < 0)
            return pitch_lag;
        if (!voiced)
            pitch_lag = s->prev_pitch_lag;
        if (LPEC_GEN_EXCIT_HALF(s, gb, s->excit_src, 2, pitch_lag,
                                s->pitch_lag_off_half2, s->pitch_win_half2,
                                voiced, s->pitch_tbl_half2, frame_start, mode,
                                frame_size, order, 0, s->half_lsf_i16[2]) < 0)
            return AVERROR(EAGAIN);
        s->prev_pitch_lag = pitch_lag;
        lpec_postprocess_excit(s, mode, s->excit_src, 0, frame_size, 2);

        {


            const double *synth_excit = lpec_prepare_synth_excit(s, mode, s->excit_src,
                                                                 s->frame_excit);


            /* full-frame f1c0 for modes 1/3 */
            interpolated_lpc_synthesis(cur_lsf1, cur_lsf2, synth_excit, pcm, 0,
                                       s->lpc_order, frame_size,
                                       lpec_lpc_segments_full(s));
            memcpy(output, pcm, frame_size * sizeof(double));
        }
        memcpy(s->prev_lsf, cur_lsf2, (order + 1) * sizeof(double));
        memcpy(s->lsf2, cur_lsf2, (order + 1) * sizeof(double));
        lpec_end_frame(s);
        lpec_save_frame(s, output,
                        s->is_xaudio_lp ? s->excit_work + LPEC_WORK_SLACK
                                        : s->frame_excit,
                        frame_size, mode);
        return lpec_finish_frame(s, gb, mode, frame_start, frame_size);

    } else {
        /* Unknown mode - stop this packet */
        return AVERROR(EAGAIN);
    }
}


/* --------------------------------------------------------------- */
/* Main decode function */
/* --------------------------------------------------------------- */

static int lpec_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                             int *got_frame_ptr, AVPacket *avpkt)
{
    LPECContext *s   = avctx->priv_data;
    int buf_size     = avpkt->size;
    int ret;
    int draining = 0;

    int frame_size = s->frame_size;
    int subfr_size = s->subfr_size;

    if (buf_size == 0) {
        draining = 1;
        s->eof_draining = 1;
        if (s->eof_flush_done) {
            *got_frame_ptr = 0;
            return 0;
        }
        lpec_feed_chunks(s);
    } else {
        s->eof_draining = 0;
    }

    const uint8_t *src = avpkt->data;
    int payload_size = buf_size;
    int total_samples = 0;
    int num_frames = 0;


    if (!draining && payload_size > (int)sizeof(s->carry_buf) - s->chunk_carry_len)
        return AVERROR(EINVAL);

    total_samples = lpec_decode_from_stream(s, s->scratch, LPEC_SCRATCH_SAMPLES,
                                            frame_size, subfr_size,
                                            LPEC_MAX_FRAMES_PKT, &num_frames);

    if (!draining) {
        memcpy(s->carry_buf + s->chunk_carry_len, src, payload_size);
        s->chunk_carry_len += payload_size;
        lpec_feed_chunks(s);
    }

    {
        int nf2 = 0;
        int n2 = lpec_decode_from_stream(s, s->scratch + total_samples,
                                         LPEC_SCRATCH_SAMPLES - total_samples,
                                         frame_size, subfr_size,
                                         LPEC_MAX_FRAMES_PKT - num_frames, &nf2);

        total_samples += n2;
        num_frames    += nf2;
    }

    if (total_samples <= 0) {
        if (draining) {
            s->stream_len      = 0;
            s->stream_bit_off  = 0;
            s->chunk_carry_len = 0;
            s->eof_flush_done  = 1;
            s->eof_draining    = 0;
        }
        *got_frame_ptr = 0;
        return draining ? 0 : buf_size;
    }

    frame->nb_samples = total_samples;
    int16_t *dst;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    dst = (int16_t *)frame->data[0];

    clamp_and_copy(dst, s->scratch, total_samples, lpec_xa_output_gain(s));

    s->frame_count++;
    if (draining && s->stream_len == 0 && s->chunk_carry_len == 0)
        s->eof_flush_done = 1;
    *got_frame_ptr = 1;
    return draining ? 0 : buf_size;
}

/* --------------------------------------------------------------- */
/* Codec registration */
/* --------------------------------------------------------------- */

const FFCodec ff_lpec_decoder = {
    .p.name      = "lpec",
    CODEC_LONG_NAME("Sony LPEC"),
    .p.type      = AVMEDIA_TYPE_AUDIO,
    .p.id        = AV_CODEC_ID_LPEC,
    .priv_data_size = sizeof(LPECContext),
    .init        = lpec_decode_init,
    .close       = lpec_decode_close,
    FF_CODEC_DECODE_CB(lpec_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_DELAY,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16),
};


