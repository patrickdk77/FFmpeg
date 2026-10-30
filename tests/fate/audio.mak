FATE_SAMPLES_AUDIO-$(call TRANSCODE, APTX, APTX, WAV_DEMUXER PCM_S16LE_DECODER ARESAMPLE_FILTER) += fate-aptx
fate-aptx: CMD = transcode wav $(TARGET_SAMPLES)/audio-reference/luckynight_2ch_44kHz_s16.wav aptx "-af aresample -c aptx" "-af aresample -c:a pcm_s16le -t 0.25" "" "" "-f aptx -sample_rate 44100"

FATE_SAMPLES_AUDIO-$(call TRANSCODE, APTX_HD, APTX_HD, WAV_DEMUXER PCM_S16LE_DECODER \
                          ARESAMPLE_FILTER PCM_S32LE_ENCODER) += fate-aptx-hd
fate-aptx-hd: CMD = transcode wav $(TARGET_SAMPLES)/audio-reference/luckynight_2ch_44kHz_s16.wav aptx_hd "-af aresample -c aptx_hd" "-af aresample -c:a pcm_s32le -t 0.25" "" "" "-f aptx_hd -sample_rate 44100"

FATE_BINKAUDIO-$(call PCM, BINK, BINKAUDIO_DCT, ARESAMPLE_FILTER) += fate-binkaudio-dct
fate-binkaudio-dct: CMD = pcm -i $(TARGET_SAMPLES)/bink/binkaudio_dct.bik
fate-binkaudio-dct: REF = $(SAMPLES)/bink/binkaudio_dct.pcm
fate-binkaudio-dct: FUZZ = 2

FATE_BINKAUDIO-$(call PCM, BINK, BINKAUDIO_RDFT, ARESAMPLE_FILTER) += fate-binkaudio-rdft
fate-binkaudio-rdft: CMD = pcm -i $(TARGET_SAMPLES)/bink/binkaudio_rdft.bik
fate-binkaudio-rdft: REF = $(SAMPLES)/bink/binkaudio_rdft.pcm
fate-binkaudio-rdft: FUZZ = 2

$(FATE_BINKAUDIO-yes): CMP = oneoff

FATE_SAMPLES_AUDIO += $(FATE_BINKAUDIO-yes)
fate-binkaudio: $(FATE_BINKAUDIO-yes)

FATE_SAMPLES_AUDIO-$(call FRAMECRC, BMV, BMV_AUDIO) += fate-bmv-audio
fate-bmv-audio: CMD = framecrc -i $(TARGET_SAMPLES)/bmv/SURFING-partial.BMV -vn

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSICIN, DSICINAUDIO) += fate-delphine-cin-audio
fate-delphine-cin-audio: CMD = framecrc -i $(TARGET_SAMPLES)/delphine-cin/LOGO-partial.CIN -vn

FATE_SAMPLES_AUDIO-$(call PCM, S337M, DOLBY_E, ARESAMPLE_FILTER) += fate-dolby-e
fate-dolby-e: CMD = pcm -i $(TARGET_SAMPLES)/dolby_e/16-11
fate-dolby-e: CMP = oneoff
fate-dolby-e: REF = $(SAMPLES)/dolby_e/16-11.pcm

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-format1
fate-dss-format1: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-format1.dss

FATE_SAMPLES_AUDIO_FFPROBE-$(CONFIG_DSS_DEMUXER) += fate-dss-format1-duration
fate-dss-format1-duration: CMD = probe -show_entries stream=duration,nb_frames $(TARGET_SAMPLES)/dss/olympus-format1.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-format1-format-sp
fate-dss-format1-format-sp: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-format1-format-sp.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-format1-hdr6
fate-dss-format1-hdr6: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-format1-hdr6.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-format1-header-copy
fate-dss-format1-header-copy: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-format1-header-copy.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-format1-noswap
fate-dss-format1-noswap: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-format1-noswap.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-format1-noswap-prev
fate-dss-format1-noswap-prev: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-format1-noswap-prev.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, G723_1, ARESAMPLE_FILTER) += fate-dss-lp
fate-dss-lp: CMD = framecrc -i $(TARGET_SAMPLES)/dss/lp.dss -frames 30 -af aresample

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, G723_1, ARESAMPLE_FILTER) += fate-dss-lp-format-sp
fate-dss-lp-format-sp: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-lp-format-sp.dss -af aresample

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, G723_1, ARESAMPLE_FILTER) += fate-dss-lp-format4
fate-dss-lp-format4: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-lp-format4.dss -af aresample

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, ARESAMPLE_FILTER PIPE_PROTOCOL) += fate-dss-lp-format4-pipe
fate-dss-lp-format4-pipe: CMD = cat $(SAMPLES)/dss/olympus-lp-format4.dss | framecrc -i pipe:0 -af aresample
fate-dss-lp-format4-pipe: REF = $(SRC_PATH)/tests/ref/fate/dss-lp-format4

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-no-header
fate-dss-no-header: CMD = framecrc -f dss -i $(TARGET_SAMPLES)/dss/olympus-format1-no-header.dss ; test $$? -ne 0
fate-dss-no-header: CMP = grep
fate-dss-no-header: REF = Invalid data found when processing input

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp
fate-dss-sp: CMD = framecrc -i $(TARGET_SAMPLES)/dss/sp.dss -frames 30

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-empty56
fate-dss-sp-empty56: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-sp-empty56.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, PIPE_PROTOCOL) += fate-dss-sp-empty56-pipe
fate-dss-sp-empty56-pipe: CMD = cat $(SAMPLES)/dss/olympus-sp-empty56.dss | framecrc -blocksize 7 -i pipe:0
fate-dss-sp-empty56-pipe: REF = $(SRC_PATH)/tests/ref/fate/dss-sp-empty56

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-format1-noswap-prev-empty60
fate-dss-format1-noswap-prev-empty60: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-format1-noswap-prev-empty60.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, PIPE_PROTOCOL) += fate-dss-format1-noswap-prev-empty60-pipe
fate-dss-format1-noswap-prev-empty60-pipe: CMD = cat $(SAMPLES)/dss/olympus-format1-noswap-prev-empty60.dss | framecrc -blocksize 7 -i pipe:0
fate-dss-format1-noswap-prev-empty60-pipe: REF = $(SRC_PATH)/tests/ref/fate/dss-format1-noswap-prev-empty60

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-format1-hdr6-empty52
fate-dss-format1-hdr6-empty52: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-format1-hdr6-empty52.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, PIPE_PROTOCOL) += fate-dss-format1-hdr6-empty52-pipe
fate-dss-format1-hdr6-empty52-pipe: CMD = cat $(SAMPLES)/dss/olympus-format1-hdr6-empty52.dss | framecrc -blocksize 7 -i pipe:0
fate-dss-format1-hdr6-empty52-pipe: REF = $(SRC_PATH)/tests/ref/fate/dss-format1-hdr6-empty52

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-format1-noswap-prev4-empty62
fate-dss-format1-noswap-prev4-empty62: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-format1-noswap-prev4-empty62.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, PIPE_PROTOCOL) += fate-dss-format1-noswap-prev4-empty62-pipe
fate-dss-format1-noswap-prev4-empty62-pipe: CMD = cat $(SAMPLES)/dss/olympus-format1-noswap-prev4-empty62.dss | framecrc -blocksize 7 -i pipe:0
fate-dss-format1-noswap-prev4-empty62-pipe: REF = $(SRC_PATH)/tests/ref/fate/dss-format1-noswap-prev4-empty62

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-grundig-sp
fate-dss-grundig-sp: CMD = framecrc -i $(TARGET_SAMPLES)/dss/grundig-sp.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-empty-lp
fate-dss-sp-empty-lp: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-sp-empty-lp.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-format-ff
fate-dss-sp-format-ff: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-sp-format-ff.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-format3-first
fate-dss-sp-format3-first: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-sp-format3-first.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-byte3-none
fate-dss-sp-byte3-none: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-sp-byte3-none.dss ; test $$? -ne 0
fate-dss-sp-byte3-none: CMP = grep
fate-dss-sp-byte3-none: REF = Invalid data found when processing input

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-byte3-mid
fate-dss-sp-byte3-mid: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-sp-byte3-mid.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-hdr0
fate-dss-sp-hdr0: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-sp-hdr0.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-hdr1
fate-dss-sp-hdr1: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-sp-hdr1.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-no-date
fate-dss-sp-no-date: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-sp-no-date.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-pause-block
fate-dss-sp-pause-block: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-sp-pause-block.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-paused
fate-dss-sp-paused: CMD = framecrc -i $(TARGET_SAMPLES)/dss/sp-paused.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-dss-sp-swap-start
fate-dss-sp-swap-start: CMD = framecrc -i $(TARGET_SAMPLES)/dss/olympus-sp-swap-start.dss

FATE_SAMPLES_AUDIO_FFPROBE-$(CONFIG_DSS_DEMUXER) += fate-dss-sp-tags
fate-dss-sp-tags: CMD = probetags $(TARGET_SAMPLES)/dss/sp.dss

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-format1
fate-ds2-format1: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-format1.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-format1-annot-annotations
fate-ds2-format1-annot-annotations: CMD = framecrc -extract annotations -i $(TARGET_SAMPLES)/ds2/olympus-format1-annot.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, PIPE_PROTOCOL) += fate-ds2-format1-annot-annotations-pipe
fate-ds2-format1-annot-annotations-pipe: CMD = cat $(SAMPLES)/ds2/olympus-format1-annot.ds2 | framecrc -extract annotations -i pipe:0
fate-ds2-format1-annot-annotations-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-format1-annot-annotations

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-format1-annot-main
fate-ds2-format1-annot-main: CMD = framecrc -extract main -i $(TARGET_SAMPLES)/ds2/olympus-format1-annot.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, PIPE_PROTOCOL) += fate-ds2-format1-annot-main-pipe
fate-ds2-format1-annot-main-pipe: CMD = cat $(SAMPLES)/ds2/olympus-format1-annot.ds2 | framecrc -extract main -i pipe:0
fate-ds2-format1-annot-main-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-format1-annot-main

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-format1-header-copy
fate-ds2-format1-header-copy: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-format1-header-copy.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, G723_1, ARESAMPLE_FILTER) += fate-ds2-lp
fate-ds2-lp: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-lp.ds2 -af aresample

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, G723_1, ARESAMPLE_FILTER) += fate-ds2-lp-annot-annotations
fate-ds2-lp-annot-annotations: CMD = framecrc -extract annotations -i $(TARGET_SAMPLES)/ds2/olympus-lp-annot.ds2 -af aresample

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, ARESAMPLE_FILTER PIPE_PROTOCOL) += fate-ds2-lp-annot-annotations-pipe
fate-ds2-lp-annot-annotations-pipe: CMD = cat $(SAMPLES)/ds2/olympus-lp-annot.ds2 | framecrc -extract annotations -i pipe:0 -af aresample
fate-ds2-lp-annot-annotations-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-lp-annot-annotations

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, G723_1, ARESAMPLE_FILTER) += fate-ds2-lp-annot-main
fate-ds2-lp-annot-main: CMD = framecrc -extract main -i $(TARGET_SAMPLES)/ds2/olympus-lp-annot.ds2 -af aresample

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, ARESAMPLE_FILTER PIPE_PROTOCOL) += fate-ds2-lp-annot-main-pipe
fate-ds2-lp-annot-main-pipe: CMD = cat $(SAMPLES)/ds2/olympus-lp-annot.ds2 | framecrc -extract main -i pipe:0 -af aresample
fate-ds2-lp-annot-main-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-lp-annot-main

FATE_SAMPLES_AUDIO_FFPROBE-$(CONFIG_DSS_DEMUXER) += fate-ds2-lp-duration
fate-ds2-lp-duration: CMD = probe -show_entries stream=duration,nb_frames $(TARGET_SAMPLES)/ds2/olympus-lp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, G723_1, ARESAMPLE_FILTER) += fate-ds2-lp-extract-main
fate-ds2-lp-extract-main: CMD = framecrc -extract main -i $(TARGET_SAMPLES)/ds2/olympus-lp.ds2 -af aresample
fate-ds2-lp-extract-main: REF = $(SRC_PATH)/tests/ref/fate/ds2-lp

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, G723_1, ARESAMPLE_FILTER) += fate-ds2-lp-format4
fate-ds2-lp-format4: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-lp-format4.ds2 -af aresample

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp
fate-ds2-qp: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/sample-qp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-annot-annotations
fate-ds2-qp-annot-annotations: CMD = framecrc -extract annotations -i $(TARGET_SAMPLES)/ds2/olympus-qp-annot.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2, PIPE_PROTOCOL) += fate-ds2-qp-annot-annotations-pipe
fate-ds2-qp-annot-annotations-pipe: CMD = cat $(SAMPLES)/ds2/olympus-qp-annot.ds2 | framecrc -extract annotations -i pipe:0
fate-ds2-qp-annot-annotations-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-qp-annot-annotations

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-annot-main
fate-ds2-qp-annot-main: CMD = framecrc -extract main -i $(TARGET_SAMPLES)/ds2/olympus-qp-annot.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2, PIPE_PROTOCOL) += fate-ds2-qp-annot-main-pipe
fate-ds2-qp-annot-main-pipe: CMD = cat $(SAMPLES)/ds2/olympus-qp-annot.ds2 | framecrc -extract main -i pipe:0
fate-ds2-qp-annot-main-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-qp-annot-main

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-aes-mode0
fate-ds2-qp-aes-mode0: CMD = framecrc -password test1234 -i $(TARGET_SAMPLES)/ds2/olympus-qp-aes-mode0.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-aes128
fate-ds2-qp-aes128: CMD = framecrc -password 1234 -i $(TARGET_SAMPLES)/ds2/aes128-qp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2, PIPE_PROTOCOL) += fate-ds2-qp-aes128-pipe
fate-ds2-qp-aes128-pipe: CMD = cat $(SAMPLES)/ds2/aes128-qp.ds2 | framecrc -password 1234 -blocksize 628 -i pipe:0
fate-ds2-qp-aes128-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-qp-aes128

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-aes128-long-password
fate-ds2-qp-aes128-long-password: CMD = framecrc -password 12345678901234567 -i $(TARGET_SAMPLES)/ds2/aes128-qp.ds2 ; test $$? -ne 0
fate-ds2-qp-aes128-long-password: CMP = grep
fate-ds2-qp-aes128-long-password: REF = DS2 password longer than 16 bytes is not supported

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-aes128-no-password
fate-ds2-qp-aes128-no-password: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/aes128-qp.ds2 ; test $$? -ne 0
fate-ds2-qp-aes128-no-password: CMP = grep
fate-ds2-qp-aes128-no-password: REF = Encrypted DS2 file requires the -password option

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-aes128-wrong-password
fate-ds2-qp-aes128-wrong-password: CMD = framecrc -password 1235 -i $(TARGET_SAMPLES)/ds2/aes128-qp.ds2 ; test $$? -ne 0
fate-ds2-qp-aes128-wrong-password: CMP = grep
fate-ds2-qp-aes128-wrong-password: REF = DS2 password rejected

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-aes256
fate-ds2-qp-aes256: CMD = framecrc -password 1234 -i $(TARGET_SAMPLES)/ds2/aes256-qp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-aes256-mode0
fate-ds2-qp-aes256-mode0: CMD = framecrc -password test1234 -i $(TARGET_SAMPLES)/ds2/olympus-qp-aes256-mode0.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-aes256-wrong-password
fate-ds2-qp-aes256-wrong-password: CMD = framecrc -password 1235 -i $(TARGET_SAMPLES)/ds2/aes256-qp.ds2 ; test $$? -ne 0
fate-ds2-qp-aes256-wrong-password: CMP = grep
fate-ds2-qp-aes256-wrong-password: REF = DS2 password rejected

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-cut
fate-ds2-qp-cut: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-qp-cut.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-extract-annotations
fate-ds2-qp-extract-annotations: CMD = framecrc -extract annotations -i $(TARGET_SAMPLES)/ds2/sample-qp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-extract-annotations-none
fate-ds2-qp-extract-annotations-none: CMD = framecrc -extract annotations -i $(TARGET_SAMPLES)/ds2/sample-qp.ds2
fate-ds2-qp-extract-annotations-none: CMP = grep
fate-ds2-qp-extract-annotations-none: REF = No annotations to extract

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-extract-invalid
fate-ds2-qp-extract-invalid: CMD = framecrc -extract foo -i $(TARGET_SAMPLES)/ds2/sample-qp.ds2 ; test $$? -ne 0
fate-ds2-qp-extract-invalid: CMP = grep
fate-ds2-qp-extract-invalid: REF = Error setting option extract to value foo

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-extract-main
fate-ds2-qp-extract-main: CMD = framecrc -extract main -i $(TARGET_SAMPLES)/ds2/sample-qp.ds2
fate-ds2-qp-extract-main: REF = $(SRC_PATH)/tests/ref/fate/ds2-qp

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-hdr1
fate-ds2-qp-hdr1: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-qp-hdr1.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-hdr17
fate-ds2-qp-hdr17: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-qp-hdr17.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-no-header
fate-ds2-qp-no-header: CMD = framecrc -f ds2 -i $(TARGET_SAMPLES)/ds2/olympus-qp-no-header.ds2 ; test $$? -ne 0
fate-ds2-qp-no-header: CMP = grep
fate-ds2-qp-no-header: REF = Invalid data found when processing input

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-pause-block
fate-ds2-qp-pause-block: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-qp-pause-block.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-header-copy
fate-ds2-qp-header-copy: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-qp-header-copy.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-paused
fate-ds2-qp-paused: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/paused-qp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, G723_1, ARESAMPLE_FILTER) += fate-ds2-lp-empty64
fate-ds2-lp-empty64: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-lp-empty64.ds2 -af aresample

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, ARESAMPLE_FILTER PIPE_PROTOCOL) += fate-ds2-lp-empty64-pipe
fate-ds2-lp-empty64-pipe: CMD = cat $(SAMPLES)/ds2/olympus-lp-empty64.ds2 | framecrc -i pipe:0 -af aresample
fate-ds2-lp-empty64-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-lp-empty64

FATE_SAMPLES_AUDIO_FFPROBE-$(CONFIG_DS2_DEMUXER) += fate-ds2-qp-resync-pos
fate-ds2-qp-resync-pos: CMD = probe -show_entries packet=pts,pos -of compact $(TARGET_SAMPLES)/ds2/olympus-qp-resync.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp7-empty64
fate-ds2-qp7-empty64: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-qp7-empty64.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2, PIPE_PROTOCOL) += fate-ds2-qp7-empty64-pipe
fate-ds2-qp7-empty64-pipe: CMD = cat $(SAMPLES)/ds2/olympus-qp7-empty64.ds2 | framecrc -i pipe:0
fate-ds2-qp7-empty64-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-qp7-empty64

FATE_SAMPLES_AUDIO_FFPROBE-$(CONFIG_DS2_DEMUXER) += fate-ds2-qp-paused-duration
fate-ds2-qp-paused-duration: CMD = probe -show_entries stream=duration,nb_frames $(TARGET_SAMPLES)/ds2/paused-qp.ds2

FATE_SAMPLES_AUDIO_FFPROBE-$(CONFIG_DS2_DEMUXER) += fate-ds2-qp-paused-tags
fate-ds2-qp-paused-tags: CMD = probetags $(TARGET_SAMPLES)/ds2/paused-qp.ds2

FATE_SAMPLES_AUDIO_FFPROBE-$(CONFIG_DS2_DEMUXER) += fate-ds2-qp-tags
fate-ds2-qp-tags: CMD = probetags $(TARGET_SAMPLES)/ds2/sample-qp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-tail-start
fate-ds2-qp-tail-start: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-qp-tail-start.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp7
fate-ds2-qp7: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-qp7.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp7-empty-lp
fate-ds2-qp7-empty-lp: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-qp7-empty-lp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp7-extract-main
fate-ds2-qp7-extract-main: CMD = framecrc -extract main -i $(TARGET_SAMPLES)/ds2/olympus-qp7.ds2
fate-ds2-qp7-extract-main: REF = $(SRC_PATH)/tests/ref/fate/ds2-qp7

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-sp
fate-ds2-sp: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-sp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-sp-annot-annotations
fate-ds2-sp-annot-annotations: CMD = framecrc -extract annotations -i $(TARGET_SAMPLES)/ds2/olympus-sp-annot.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, PIPE_PROTOCOL) += fate-ds2-sp-annot-annotations-pipe
fate-ds2-sp-annot-annotations-pipe: CMD = cat $(SAMPLES)/ds2/olympus-sp-annot.ds2 | framecrc -extract annotations -i pipe:0
fate-ds2-sp-annot-annotations-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-sp-annot-annotations

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-sp-annot-main
fate-ds2-sp-annot-main: CMD = framecrc -extract main -i $(TARGET_SAMPLES)/ds2/olympus-sp-annot.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, PIPE_PROTOCOL) += fate-ds2-sp-annot-main-pipe
fate-ds2-sp-annot-main-pipe: CMD = cat $(SAMPLES)/ds2/olympus-sp-annot.ds2 | framecrc -extract main -i pipe:0
fate-ds2-sp-annot-main-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-sp-annot-main

FATE_SAMPLES_AUDIO_FFPROBE-$(CONFIG_DSS_DEMUXER) += fate-ds2-sp-duration
fate-ds2-sp-duration: CMD = probe -show_entries stream=duration,nb_frames $(TARGET_SAMPLES)/ds2/olympus-sp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-sp-empty-lp
fate-ds2-sp-empty-lp: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-sp-empty-lp.ds2

FATE_SAMPLES_AUDIO_FFPROBE-$(CONFIG_DSS_DEMUXER) += fate-ds2-sp-empty-lp-probe
fate-ds2-sp-empty-lp-probe: CMD = probe -show_entries stream=codec_name,sample_rate $(TARGET_SAMPLES)/ds2/olympus-sp-empty-lp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-sp-empty-qp
fate-ds2-sp-empty-qp: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-sp-empty-qp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-sp-extract-annotations-none
fate-ds2-sp-extract-annotations-none: CMD = framecrc -extract annotations -i $(TARGET_SAMPLES)/ds2/olympus-sp.ds2
fate-ds2-sp-extract-annotations-none: CMP = grep
fate-ds2-sp-extract-annotations-none: REF = No annotations to extract

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-sp-extract-main
fate-ds2-sp-extract-main: CMD = framecrc -extract main -i $(TARGET_SAMPLES)/ds2/olympus-sp.ds2
fate-ds2-sp-extract-main: REF = $(SRC_PATH)/tests/ref/fate/ds2-sp

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-sp-hdr1
fate-ds2-sp-hdr1: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-sp-hdr1.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, GRUNDIG_SP) += fate-ds2-sp-hdr17
fate-ds2-sp-hdr17: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-sp-hdr17.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, ARESAMPLE_FILTER) += fate-ds2-sp-qp
fate-ds2-sp-qp: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-sp-qp.ds2 -af aresample=16000:tsf=s16p

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, ARESAMPLE_FILTER) += fate-ds2-sp-qp-cut
fate-ds2-sp-qp-cut: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-sp-qp-cut.ds2 -af aresample=16000:tsf=s16p

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, ARESAMPLE_FILTER) += fate-ds2-sp-qp-format3
fate-ds2-sp-qp-format3: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/olympus-sp-qp-format3.ds2 -af aresample=16000:tsf=s16p

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED) += fate-ds2-sp-qp-seek
fate-ds2-sp-qp-seek: CMD = framecrc -ss 3 -i $(TARGET_SAMPLES)/ds2/olympus-sp-qp.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, ARESAMPLE_FILTER PIPE_PROTOCOL) += fate-ds2-sp-qp-pipe
fate-ds2-sp-qp-pipe: CMD = cat $(SAMPLES)/ds2/olympus-sp-qp.ds2 | framecrc -i pipe:0 -af aresample=16000:tsf=s16p

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DS2, DS2) += fate-ds2-qp-resync-past-end
fate-ds2-qp-resync-past-end: CMD = framecrc -i $(TARGET_SAMPLES)/ds2/qp-resync-past-end.ds2

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, ARESAMPLE_FILTER PIPE_PROTOCOL) += fate-ds2-qp-resync-past-end-pipe
fate-ds2-qp-resync-past-end-pipe: CMD = cat $(SAMPLES)/ds2/qp-resync-past-end.ds2 | framecrc -f dss -i pipe:0 -af aresample
fate-ds2-qp-resync-past-end-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-qp-resync-past-end

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, ARESAMPLE_FILTER) += fate-ds2-qp7-lp-empty109
fate-ds2-qp7-lp-empty109: CMD = framecrc -f dss -i $(TARGET_SAMPLES)/ds2/olympus-qp7-lp-empty109.ds2 -af aresample=16000:tsf=s16p

FATE_SAMPLES_AUDIO-$(call FRAMECRC, DSS, DSS_MIXED, ARESAMPLE_FILTER PIPE_PROTOCOL) += fate-ds2-qp7-lp-empty109-pipe
fate-ds2-qp7-lp-empty109-pipe: CMD = cat $(SAMPLES)/ds2/olympus-qp7-lp-empty109.ds2 | framecrc -f dss -blocksize 7 -i pipe:0 -af aresample=16000:tsf=s16p
fate-ds2-qp7-lp-empty109-pipe: REF = $(SRC_PATH)/tests/ref/fate/ds2-qp7-lp-empty109

FATE_SAMPLES_AUDIO_FFPROBE-$(CONFIG_DSS_DEMUXER) += fate-ds2-sp-tags
fate-ds2-sp-tags: CMD = probetags $(TARGET_SAMPLES)/ds2/olympus-sp.ds2

FATE_SAMPLES_AUDIO-$(call PCM, DSF, DST, ARESAMPLE_FILTER) += fate-dsf-dst
fate-dsf-dst: CMD = pcm -i $(TARGET_SAMPLES)/dst/dst-64fs44-2ch.dff
fate-dsf-dst: CMP = oneoff
fate-dsf-dst: REF = $(SAMPLES)/dst/dst-64fs44-2ch.pcm

FATE_SAMPLES_AUDIO-$(call PCM, G728, G728, ARESAMPLE_FILTER) += fate-g728
fate-g728: CMD = pcm -i $(TARGET_SAMPLES)/g728/CW3.g728
fate-g728: REF = $(SAMPLES)/g728/OUTA3.BIN
fate-g728: CMP = oneoff
fate-g728: FUZZ = 5

FATE_SAMPLES_AUDIO-$(call PCM, AVI, IMC, ARESAMPLE_FILTER) += fate-imc
fate-imc: CMD = pcm -i $(TARGET_SAMPLES)/imc/imc.avi
fate-imc: CMP = oneoff
fate-imc: CMP_TARGET = 59416
fate-imc: REF = $(SAMPLES)/imc/imc-201706.pcm

FATE_SAMPLES_AUDIO-$(call PCM, WAV, MSNSIREN, ARESAMPLE_FILTER) += fate-msnsiren
fate-msnsiren: CMD = pcm -i $(TARGET_SAMPLES)/msnsiren/msnsiren2.wav
fate-msnsiren: CMP = oneoff
fate-msnsiren: REF = $(SAMPLES)/msnsiren/msnsiren2.pcm

FATE_SAMPLES_AUDIO-$(call PCM, FLV, NELLYMOSER, ARESAMPLE_FILTER) += fate-nellymoser
fate-nellymoser: CMD = pcm -i $(TARGET_SAMPLES)/nellymoser/nellymoser.flv
fate-nellymoser: CMP = oneoff
fate-nellymoser: REF = $(SAMPLES)/nellymoser/nellymoser.pcm

FATE_SAMPLES_AUDIO-$(call ENCMUX, NELLYMOSER, FLV, ARESAMPLE_FILTER PIPE_PROTOCOL) += fate-nellymoser-aref-encode
fate-nellymoser-aref-encode: $(AREF) ./tests/data/asynth-16000-1.wav
fate-nellymoser-aref-encode: CMD = enc_dec_pcm flv wav s16le $(REF) -c:a nellymoser
fate-nellymoser-aref-encode: CMP = stddev
fate-nellymoser-aref-encode: REF = ./tests/data/asynth-16000-1.wav
fate-nellymoser-aref-encode: CMP_SHIFT = -256
fate-nellymoser-aref-encode: CMP_TARGET = 3863
fate-nellymoser-aref-encode: SIZE_TOLERANCE = 268

FATE_SAMPLES_AUDIO-$(call FRAMECRC, AVI, ON2AVC, ARESAMPLE_FILTER) += fate-on2avc
fate-on2avc: CMD = framecrc -i $(TARGET_SAMPLES)/vp7/potter-40.vp7 -frames 30 -vn -af aresample

FATE_SAMPLES_AUDIO-$(call FRAMECRC, PAF, PAF_AUDIO) += fate-paf-audio
fate-paf-audio: CMD = framecrc -i $(TARGET_SAMPLES)/paf/hod1-partial.paf -vn

FATE_SAMPLES_AUDIO-$(call FRAMECRC, VMD, VMDAUDIO, ARESAMPLE_FILTER) += fate-sierra-vmd-audio
fate-sierra-vmd-audio: CMD = framecrc -i $(TARGET_SAMPLES)/vmd/12.vmd -vn -af aresample

FATE_SAMPLES_AUDIO-$(call FRAMECRC, SMACKER, SMACKAUD, ARESAMPLE_FILTER) += fate-smacker-audio
fate-smacker-audio: CMD = framecrc -i $(TARGET_SAMPLES)/smacker/wetlogo.smk -vn -af aresample

FATE_SAMPLES_AUDIO-$(call DEMDEC, WSVQA, WS_SND1, ARESAMPLE_FILTER) += fate-ws_snd
fate-ws_snd: CMD = md5 -i $(TARGET_SAMPLES)/vqa/ws_snd.vqa -f s16le -af aresample

FATE_SAMPLES_AUDIO_FFPROBE-$(call DEMDEC, WAV, WMAV2, FILE_PROTOCOL) += fate-flcl1905
fate-flcl1905: CMD = run ffprobe$(PROGSSUF)$(EXESUF) -show_frames -show_packets -print_format compact $(TARGET_SAMPLES)/wav/FLCL_Ending_My-short.wav

FATE_SAMPLES_AUDIO += $(FATE_SAMPLES_AUDIO-yes)
FATE_SAMPLES_AUDIO_FFPROBE += $(FATE_SAMPLES_AUDIO_FFPROBE-yes)

FATE_SAMPLES_FFMPEG += $(FATE_SAMPLES_AUDIO)
FATE_SAMPLES_FFPROBE += $(FATE_SAMPLES_AUDIO_FFPROBE)
fate-audio: $(FATE_SAMPLES_AUDIO) $(FATE_SAMPLES_AUDIO_FFPROBE)
