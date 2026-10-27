# Sony IC recorder MSV/DVF files. The LCST (AT-X) and LPEC references are
# the output of Sony's own decoders for the same files.

FATE_MSV_LCST += fate-msv-lcst-sthq
fate-msv-lcst-sthq: CMD = pcm -i $(TARGET_SAMPLES)/msv/lcst-sthq.msv
fate-msv-lcst-sthq: REF = $(SAMPLES)/msv/lcst-sthq.pcm

FATE_MSV_LCST += fate-msv-lcst-sthq-nohfc
fate-msv-lcst-sthq-nohfc: CMD = pcm -hfc 0 -i $(TARGET_SAMPLES)/msv/lcst-sthq.msv
fate-msv-lcst-sthq-nohfc: REF = $(SAMPLES)/msv/lcst-sthq-nohfc.pcm

FATE_MSV_LCST += fate-msv-lcst-96k
fate-msv-lcst-96k: CMD = pcm -i $(TARGET_SAMPLES)/msv/lcst-96k.msv
fate-msv-lcst-96k: REF = $(SAMPLES)/msv/lcst-96k.pcm

FATE_MSV_LCST += fate-msv-lcst-64k
fate-msv-lcst-64k: CMD = pcm -i $(TARGET_SAMPLES)/msv/lcst-64k.msv
fate-msv-lcst-64k: REF = $(SAMPLES)/msv/lcst-64k.pcm

FATE_MSV_LCST += fate-msv-lcst-st
fate-msv-lcst-st: CMD = pcm -i $(TARGET_SAMPLES)/msv/lcst-st.msv
fate-msv-lcst-st: REF = $(SAMPLES)/msv/lcst-st.pcm

FATE_MSV_LCST += fate-msv-lcst-st-s1
fate-msv-lcst-st-s1: CMD = pcm -i $(TARGET_SAMPLES)/msv/lcst-st-s1.msv
fate-msv-lcst-st-s1: REF = $(SAMPLES)/msv/lcst-st-s1.pcm

FATE_MSV_LCST += fate-msv-lcst-st-exfo
fate-msv-lcst-st-exfo: CMD = pcm -i $(TARGET_SAMPLES)/msv/lcst-st-exfo.msv
fate-msv-lcst-st-exfo: REF = $(SAMPLES)/msv/lcst-st-exfo.pcm

FATE_MSV_LCST += fate-msv-lcst-stlp
fate-msv-lcst-stlp: CMD = pcm -i $(TARGET_SAMPLES)/msv/lcst-stlp.msv
fate-msv-lcst-stlp: REF = $(SAMPLES)/msv/lcst-stlp.pcm

FATE_MSV_LCST += fate-msv-lcst-voiscape
fate-msv-lcst-voiscape: CMD = pcm -i $(TARGET_SAMPLES)/msv/lcst-voiscape.msv
fate-msv-lcst-voiscape: REF = $(SAMPLES)/msv/lcst-voiscape.pcm

FATE_MSV_LCST-$(call PCM, MSV, ATRAC3P, ARESAMPLE_FILTER) += $(FATE_MSV_LCST)

FATE_MSV_LPEC += fate-msv-lpec-hac-sp
fate-msv-lpec-hac-sp: CMD = pcm -i $(TARGET_SAMPLES)/msv/lpec-hac-sp.msv
fate-msv-lpec-hac-sp: REF = $(SAMPLES)/msv/lpec-hac-sp.pcm

FATE_MSV_LPEC += fate-msv-lpec-hac-lp
fate-msv-lpec-hac-lp: CMD = pcm -i $(TARGET_SAMPLES)/msv/lpec-hac-lp.msv
fate-msv-lpec-hac-lp: REF = $(SAMPLES)/msv/lpec-hac-lp.pcm

FATE_MSV_LPEC += fate-msv-lpec-sp
fate-msv-lpec-sp: CMD = pcm -i $(TARGET_SAMPLES)/msv/lpec-sp.msv
fate-msv-lpec-sp: REF = $(SAMPLES)/msv/lpec-sp.pcm

FATE_MSV_LPEC += fate-msv-lpec-lp
fate-msv-lpec-lp: CMD = pcm -i $(TARGET_SAMPLES)/msv/lpec-lp.msv
fate-msv-lpec-lp: REF = $(SAMPLES)/msv/lpec-lp.pcm

FATE_MSV_LPEC += fate-msv-lpec-ms-sp
fate-msv-lpec-ms-sp: CMD = pcm -i $(TARGET_SAMPLES)/msv/lpec-ms-sp.msv
fate-msv-lpec-ms-sp: REF = $(SAMPLES)/msv/lpec-ms-sp.pcm

FATE_MSV_LPEC += fate-msv-lpec-ms-lp
fate-msv-lpec-ms-lp: CMD = pcm -i $(TARGET_SAMPLES)/msv/lpec-ms-lp.msv
fate-msv-lpec-ms-lp: REF = $(SAMPLES)/msv/lpec-ms-lp.pcm

FATE_MSV_LPEC += fate-msv-lpec-ms-sp-ex
fate-msv-lpec-ms-sp-ex: CMD = pcm -i $(TARGET_SAMPLES)/msv/lpec-ms-sp-ex.msv
fate-msv-lpec-ms-sp-ex: REF = $(SAMPLES)/msv/lpec-ms-sp-ex.pcm

FATE_MSV_LPEC += fate-msv-lpec-voiscape
fate-msv-lpec-voiscape: CMD = pcm -i $(TARGET_SAMPLES)/msv/lpec-voiscape.msv
fate-msv-lpec-voiscape: REF = $(SAMPLES)/msv/lpec-voiscape.pcm

FATE_MSV_LPEC += fate-msv-lpec-icd-bm1
fate-msv-lpec-icd-bm1: CMD = pcm -i $(TARGET_SAMPLES)/msv/icd-bm1-lpec-sp.msv
fate-msv-lpec-icd-bm1: REF = $(SAMPLES)/msv/icd-bm1-lpec-sp.pcm

FATE_MSV_LPEC-$(call PCM, MSV, LPEC) += $(FATE_MSV_LPEC)

$(FATE_MSV_LCST-yes) $(FATE_MSV_LPEC-yes): CMP = oneoff

FATE_MSV_TRC += fate-msv-trc-hq
fate-msv-trc-hq: CMD = framecrc -i $(TARGET_SAMPLES)/msv/trc-hq.msv

FATE_MSV_TRC += fate-msv-trc-sp
fate-msv-trc-sp: CMD = framecrc -i $(TARGET_SAMPLES)/msv/trc-sp.msv

FATE_MSV_TRC += fate-msv-trc-lp
fate-msv-trc-lp: CMD = framecrc -i $(TARGET_SAMPLES)/msv/trc-lp.msv

FATE_MSV_TRC += fate-msv-trc-sp-s1
fate-msv-trc-sp-s1: CMD = framecrc -i $(TARGET_SAMPLES)/msv/trc-sp-s1.msv

FATE_MSV_TRC += fate-msv-trc-page-hq
fate-msv-trc-page-hq: CMD = framecrc -i $(TARGET_SAMPLES)/msv/trc-page-hq.msv

FATE_MSV_TRC += fate-msv-trc-page-lp
fate-msv-trc-page-lp: CMD = framecrc -i $(TARGET_SAMPLES)/msv/trc-page-lp.msv

FATE_MSV_TRC += fate-msv-trc-pvoice
fate-msv-trc-pvoice: CMD = framecrc -i $(TARGET_SAMPLES)/msv/pvoice-trc-hq.dvf

FATE_MSV_TRC-$(call FRAMECRC, MSV, TRC) += $(FATE_MSV_TRC)

FATE_MSV_TRC-$(call CRC, MSV, TRC) += fate-msv-trc-taxis
fate-msv-trc-taxis: CMD = crc -i $(TARGET_SAMPLES)/msv/taxis-trc-hq.dvf

FATE_MSV_ADPCM += fate-msv-adpcm-sp
fate-msv-adpcm-sp: CMD = framecrc -i $(TARGET_SAMPLES)/msv/adpcm-sp.msv

FATE_MSV_ADPCM += fate-msv-adpcm-sp-plugin
fate-msv-adpcm-sp-plugin: CMD = framecrc -variant plugin -i $(TARGET_SAMPLES)/msv/adpcm-sp.msv

FATE_MSV_ADPCM += fate-msv-adpcm-lp
fate-msv-adpcm-lp: CMD = framecrc -i $(TARGET_SAMPLES)/msv/adpcm-lp.msv

FATE_MSV_ADPCM-$(call FRAMECRC, MSV, MSV_ADPCM) += $(FATE_MSV_ADPCM)

FATE_MSV_FFPROBE-$(call DEMDEC, MSV, ATRAC3P LPEC TRC MSV_ADPCM) += fate-msv-probe
fate-msv-probe: CMD = \
    probe -show_entries format=duration:stream=codec_name,sample_rate,channels,duration,bit_rate,nb_frames -of compact $(TARGET_SAMPLES)/msv/lcst-sthq.msv; \
    probe -show_entries format=duration:stream=codec_name,sample_rate,channels,duration,bit_rate,nb_frames -of compact $(TARGET_SAMPLES)/msv/lpec-hac-lp.msv; \
    probe -show_entries format=duration:stream=codec_name,sample_rate,channels,duration,bit_rate,nb_frames -of compact $(TARGET_SAMPLES)/msv/trc-page-lp.msv; \
    probe -show_entries format=duration:stream=codec_name,sample_rate,channels,duration,bit_rate,nb_frames -of compact $(TARGET_SAMPLES)/msv/adpcm-lp.msv

FATE_MSV_FFPROBE-$(call DEMDEC, MSV, LPEC TRC) += fate-msv-metadata
fate-msv-metadata: CMD = \
    probe -show_entries format_tags:chapter -of compact $(TARGET_SAMPLES)/msv/icd-px7-trc-sp.dvf; \
    probe -show_entries format_tags:chapter -of compact $(TARGET_SAMPLES)/msv/pvoice-trc-hq.dvf; \
    probe -show_entries format_tags:chapter -of compact $(TARGET_SAMPLES)/msv/taxis-trc-hq.dvf; \
    probe -show_entries format_tags:chapter -of compact $(TARGET_SAMPLES)/msv/icd-bm1-lpec-sp.msv

FATE_MSV_FFMPEG = $(FATE_MSV_LCST-yes) $(FATE_MSV_LPEC-yes) $(FATE_MSV_TRC-yes) $(FATE_MSV_ADPCM-yes)
FATE_SAMPLES_FFMPEG += $(FATE_MSV_FFMPEG)
FATE_SAMPLES_FFPROBE += $(FATE_MSV_FFPROBE-yes)
fate-msv: $(FATE_MSV_FFMPEG) $(FATE_MSV_FFPROBE-yes)
