/*
 * tdm_audio_fast — TDM/I2S multi-channel audio decoder (batch + SIMD).
 *
 * PXView 1.5.5 / C decoder API v4 high-speed sibling of tdm_audio_c.
 * It does NOT fall back to per-sample c_wait()/c_pin().  Instead it borrows
 * the v4 bit-packed input chunk directly (zero-copy) and scans 64 samples per
 * machine word.  Only actual BCK edges enter the serial frame state machine.
 *
 * Input format is one bit/sample per decoder channel, LSB-first.  This avoids
 * the old 1-byte/sample expansion buffers and cuts input memory traffic by 8x.
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include "libsigrokdecode.h"
#if defined(_MSC_VER) && defined(_M_X64)
#include <intrin.h>
#endif


#define TDM_AUDIO_MAX_CHANNELS 8
#define FETCH_BLOCK_MIN        16384
#define FETCH_BLOCK_MEDIUM     32768
#define FETCH_BLOCK_MAX        65536
#define ANALOG_BATCH_MIN       256
#define ANALOG_BATCH_MEDIUM    512
#define ANALOG_BATCH_LARGE     2048
#define ANALOG_BATCH_MAX       4096

enum tdm_audio_ann {
    ANN_CH0 = 0, ANN_CH1, ANN_CH2, ANN_CH3,
    ANN_CH4, ANN_CH5, ANN_CH6, ANN_CH7,
    NUM_ANN = TDM_AUDIO_MAX_CHANNELS,
};

typedef struct {
    uint64_t samplerate;
    uint64_t capture_samples;
    uint64_t processed_samples;
    uint64_t fetch_block;
    uint16_t analog_batch;
    int channels;
    int bitdepth;
    int edge;           /* 0=rising, 1=falling */
    int frame_edge;     /* 0=active high, 1=active low */
    int align;          /* 0=left-justified, 1=I2S */
    int is_signed;

    int out_ann;
    int out_analog;
    int emit_annotations;
    int emit_analog;

    /* Frame state (mirrors tdm_audio_c) */
    int fa, fi;         /* frame active/inactive levels */
    int i2s_bck_cnt;
    int lastframe;
    int channel;
    int bitcount;
    uint64_t data;
    uint64_t ss_block;
    int have_ss_block;
    int samplecount;
    int edge_prev_clk;  /* previous CLK level for edge detection */
    int have_prev_clk;  /* keep CLK history across live input chunks */

    /* Batch analog callbacks to avoid one DLL call and two mutex locks per
     * decoded audio sample. */
    float analog_buf[TDM_AUDIO_MAX_CHANNELS][ANALOG_BATCH_MAX];
    uint64_t analog_start_buf[TDM_AUDIO_MAX_CHANNELS][ANALOG_BATCH_MAX];
    uint64_t analog_end_buf[TDM_AUDIO_MAX_CHANNELS][ANALOG_BATCH_MAX];
    uint16_t analog_count[TDM_AUDIO_MAX_CHANNELS];

    /* Timing continuity diagnostics.  Audio values are buffered exactly as
     * before, but each sample now keeps its real capture span.  This makes it
     * possible to distinguish a true source/dropout problem from a display
     * artefact caused by batch-wide timestamp interpolation. */
    uint64_t analog_prev_start[TDM_AUDIO_MAX_CHANNELS];
    uint64_t analog_nominal_delta[TDM_AUDIO_MAX_CHANNELS];
    uint8_t analog_have_prev[TDM_AUDIO_MAX_CHANNELS];
    uint8_t analog_gap_logs[TDM_AUDIO_MAX_CHANNELS];
} tdm_audio_fast_state;

/* Select larger batches only when they can amortize the extra latency and
 * working set.  capture_samples handles completed/file captures immediately;
 * processed_samples lets live captures grow into the faster tiers. */
static void update_adaptive_batches(tdm_audio_fast_state *s)
{
    uint64_t samples = s->capture_samples;
    if (s->processed_samples > samples)
        samples = s->processed_samples;

    uint64_t seconds = s->samplerate ? samples / s->samplerate : 0;
    uint64_t fetch_block = FETCH_BLOCK_MIN;
    uint16_t analog_batch = ANALOG_BATCH_MIN;

    /* Raw input work is determined by both duration and samplerate.  Include
     * absolute sample thresholds so a short 100 MHz capture is not forced
     * through tiny blocks merely because its wall-clock duration is small. */
    if (samples >= 10000000 || seconds >= 10)
        fetch_block = FETCH_BLOCK_MAX;
    else if (samples >= 1000000 || seconds >= 1)
        fetch_block = FETCH_BLOCK_MEDIUM;

    /* Analog callback volume follows decoded audio duration rather than raw
     * logic sample count. */
    if (seconds >= 60) {
        analog_batch = ANALOG_BATCH_MAX;
    } else if (seconds >= 10) {
        analog_batch = ANALOG_BATCH_LARGE;
    } else if (seconds >= 1) {
        analog_batch = ANALOG_BATCH_MEDIUM;
    }

    /* Never shrink a live decoder: a channel may already contain more samples
     * than a smaller target, and growing preserves deterministic flush points. */
    if (fetch_block > s->fetch_block)
        s->fetch_block = fetch_block;
    if (analog_batch > s->analog_batch)
        s->analog_batch = analog_batch;
}

static struct srd_channel tdm_audio_fast_channels[] = {
    {"clock", "Bitclk", "Data bit clock", 0, SRD_CHANNEL_SCLK, NULL},
    {"frame", "Framesync", "Frame sync", 1, SRD_CHANNEL_COMMON, NULL},
    {"data", "Data", "Serial data", 2, SRD_CHANNEL_SDATA, NULL},
};

static struct srd_decoder_option tdm_audio_fast_options[] = {
    {"bps",          NULL, "Bits per sample", NULL, NULL},
    {"channels",     NULL, "Channels per frame", NULL, NULL},
    {"edge",         NULL, "BCK sample edge (rising/falling)", NULL, NULL},
    {"frame_edge",   NULL, "FSYNC/LRCK polarity (high/low)", NULL, NULL},
    {"align",        NULL, "Data alignment (left-justified/I2S)", NULL, NULL},
    {"data_format",  NULL, "Data format (signed/unsigned)", NULL, NULL},
    {"output",       NULL, "Output mode (waveform/both/annotations)", NULL, NULL},
    {"ch0_vzoom",    NULL, "V-ZOOM Ch0", NULL, NULL},
    {"ch1_vzoom",    NULL, "V-ZOOM Ch1", NULL, NULL},
    {"ch2_vzoom",    NULL, "V-ZOOM Ch2", NULL, NULL},
    {"ch3_vzoom",    NULL, "V-ZOOM Ch3", NULL, NULL},
    {"ch4_vzoom",    NULL, "V-ZOOM Ch4", NULL, NULL},
    {"ch5_vzoom",    NULL, "V-ZOOM Ch5", NULL, NULL},
    {"ch6_vzoom",    NULL, "V-ZOOM Ch6", NULL, NULL},
    {"ch7_vzoom",    NULL, "V-ZOOM Ch7", NULL, NULL},
    {"ch0_vpos",     NULL, "V-POS Ch0", NULL, NULL},
    {"ch1_vpos",     NULL, "V-POS Ch1", NULL, NULL},
    {"ch2_vpos",     NULL, "V-POS Ch2", NULL, NULL},
    {"ch3_vpos",     NULL, "V-POS Ch3", NULL, NULL},
    {"ch4_vpos",     NULL, "V-POS Ch4", NULL, NULL},
    {"ch5_vpos",     NULL, "V-POS Ch5", NULL, NULL},
    {"ch6_vpos",     NULL, "V-POS Ch6", NULL, NULL},
    {"ch7_vpos",     NULL, "V-POS Ch7", NULL, NULL},
    {"ch0_enable",   NULL, "SW Ch0", NULL, NULL},
    {"ch1_enable",   NULL, "SW Ch1", NULL, NULL},
    {"ch2_enable",   NULL, "SW Ch2", NULL, NULL},
    {"ch3_enable",   NULL, "SW Ch3", NULL, NULL},
    {"ch4_enable",   NULL, "SW Ch4", NULL, NULL},
    {"ch5_enable",   NULL, "SW Ch5", NULL, NULL},
    {"ch6_enable",   NULL, "SW Ch6", NULL, NULL},
    {"ch7_enable",   NULL, "SW Ch7", NULL, NULL},
    {"ch0_range_mode", NULL, "Range mode Ch0", NULL, NULL},
    {"ch1_range_mode", NULL, "Range mode Ch1", NULL, NULL},
    {"ch2_range_mode", NULL, "Range mode Ch2", NULL, NULL},
    {"ch3_range_mode", NULL, "Range mode Ch3", NULL, NULL},
    {"ch4_range_mode", NULL, "Range mode Ch4", NULL, NULL},
    {"ch5_range_mode", NULL, "Range mode Ch5", NULL, NULL},
    {"ch6_range_mode", NULL, "Range mode Ch6", NULL, NULL},
    {"ch7_range_mode", NULL, "Range mode Ch7", NULL, NULL},
    {"ch0_eng_min",  NULL, "Engineering Min Ch0", NULL, NULL},
    {"ch1_eng_min",  NULL, "Engineering Min Ch1", NULL, NULL},
    {"ch2_eng_min",  NULL, "Engineering Min Ch2", NULL, NULL},
    {"ch3_eng_min",  NULL, "Engineering Min Ch3", NULL, NULL},
    {"ch4_eng_min",  NULL, "Engineering Min Ch4", NULL, NULL},
    {"ch5_eng_min",  NULL, "Engineering Min Ch5", NULL, NULL},
    {"ch6_eng_min",  NULL, "Engineering Min Ch6", NULL, NULL},
    {"ch7_eng_min",  NULL, "Engineering Min Ch7", NULL, NULL},
    {"ch0_eng_max",  NULL, "Engineering Max Ch0", NULL, NULL},
    {"ch1_eng_max",  NULL, "Engineering Max Ch1", NULL, NULL},
    {"ch2_eng_max",  NULL, "Engineering Max Ch2", NULL, NULL},
    {"ch3_eng_max",  NULL, "Engineering Max Ch3", NULL, NULL},
    {"ch4_eng_max",  NULL, "Engineering Max Ch4", NULL, NULL},
    {"ch5_eng_max",  NULL, "Engineering Max Ch5", NULL, NULL},
    {"ch6_eng_max",  NULL, "Engineering Max Ch6", NULL, NULL},
    {"ch7_eng_max",  NULL, "Engineering Max Ch7", NULL, NULL},
    {"ch0_unit",     NULL, "Engineering unit Ch0", NULL, NULL},
    {"ch1_unit",     NULL, "Engineering unit Ch1", NULL, NULL},
    {"ch2_unit",     NULL, "Engineering unit Ch2", NULL, NULL},
    {"ch3_unit",     NULL, "Engineering unit Ch3", NULL, NULL},
    {"ch4_unit",     NULL, "Engineering unit Ch4", NULL, NULL},
    {"ch5_unit",     NULL, "Engineering unit Ch5", NULL, NULL},
    {"ch6_unit",     NULL, "Engineering unit Ch6", NULL, NULL},
    {"ch7_unit",     NULL, "Engineering unit Ch7", NULL, NULL},
    {"realtime_decode", NULL, "Decode and display while capturing", NULL, NULL},
    {"display_trigger_enable", NULL, "Repeat capture: analog display trigger", NULL, NULL},
    {"display_trigger_mode", NULL, "Display trigger mode", NULL, NULL},
    {"display_trigger_channel", NULL, "Display trigger analog channel", NULL, NULL},
    {"display_trigger_edge", NULL, "Display trigger edge", NULL, NULL},
    {"display_trigger_level", NULL, "Display trigger level (engineering unit)", NULL, NULL},
    {"display_trigger_position", NULL, "Display trigger horizontal position", NULL, NULL},
};

static const char *tdm_audio_fast_ann_labels[][3] = {
    {"", "ch0", "Ch0"}, {"", "ch1", "Ch1"},
    {"", "ch2", "Ch2"}, {"", "ch3", "Ch3"},
    {"", "ch4", "Ch4"}, {"", "ch5", "Ch5"},
    {"", "ch6", "Ch6"}, {"", "ch7", "Ch7"},
};

static const int row_ch0_classes[] = {ANN_CH0, -1};
static const int row_ch1_classes[] = {ANN_CH1, -1};
static const int row_ch2_classes[] = {ANN_CH2, -1};
static const int row_ch3_classes[] = {ANN_CH3, -1};
static const int row_ch4_classes[] = {ANN_CH4, -1};
static const int row_ch5_classes[] = {ANN_CH5, -1};
static const int row_ch6_classes[] = {ANN_CH6, -1};
static const int row_ch7_classes[] = {ANN_CH7, -1};

static const struct srd_c_ann_row tdm_audio_fast_ann_rows[] = {
    {"ch0-vals", "Ch0", row_ch0_classes, 1},
    {"ch1-vals", "Ch1", row_ch1_classes, 1},
    {"ch2-vals", "Ch2", row_ch2_classes, 1},
    {"ch3-vals", "Ch3", row_ch3_classes, 1},
    {"ch4-vals", "Ch4", row_ch4_classes, 1},
    {"ch5-vals", "Ch5", row_ch5_classes, 1},
    {"ch6-vals", "Ch6", row_ch6_classes, 1},
    {"ch7-vals", "Ch7", row_ch7_classes, 1},
};

static const char *tdm_audio_fast_inputs[] = {"logic"};
static const char *tdm_audio_fast_tags[] = {"Audio"};

/* ------------------------------------------------------------------ */
/* Reset / Start / Destroy                                             */
/* ------------------------------------------------------------------ */

static void tdm_audio_fast_reset(struct srd_decoder_inst *di)
{
    if (!c_decoder_get_private(di))
        c_decoder_set_private(di, g_malloc0(sizeof(tdm_audio_fast_state)));
    tdm_audio_fast_state *s = (tdm_audio_fast_state *)c_decoder_get_private(di);
    memset(s, 0, sizeof(tdm_audio_fast_state));
    s->out_ann = -1;
    s->channels = 8;
    s->bitdepth = 32;
    s->edge = 0;
    s->is_signed = 1;
    s->i2s_bck_cnt = 0;
}

static void tdm_audio_fast_start(struct srd_decoder_inst *di)
{
    tdm_audio_fast_state *s = (tdm_audio_fast_state *)c_decoder_get_private(di);
    s->fetch_block = FETCH_BLOCK_MIN;
    s->analog_batch = ANALOG_BATCH_MIN;
    update_adaptive_batches(s);
    s->out_ann = c_reg_out(di, SRD_OUTPUT_ANN, "tdm_audio_fast");
    s->out_analog = c_reg_out(di, SRD_OUTPUT_ANALOG, "tdm_audio_fast_analog");

    s->bitdepth = (int)c_opt_int(di, "bps", 16);
    if (s->bitdepth < 1)  s->bitdepth = 16;
    if (s->bitdepth > 32) s->bitdepth = 32;

    s->channels = (int)c_opt_int(di, "channels", 8);
    if (s->channels < 1) s->channels = 1;
    if (s->channels > TDM_AUDIO_MAX_CHANNELS) s->channels = TDM_AUDIO_MAX_CHANNELS;

    const char *edge_str = c_opt_str(di, "edge", "rising");
    s->edge = (strcmp(edge_str, "falling") == 0) ? 1 : 0;

    const char *fe_str = c_opt_str(di, "frame_edge", "high");
    s->frame_edge = (strcmp(fe_str, "low") == 0) ? 1 : 0;
    s->fa = s->frame_edge ? 0 : 1;
    s->fi = s->frame_edge ? 1 : 0;

    const char *align_str = c_opt_str(di, "align", "left-justified");
    s->align = (strcmp(align_str, "I2S") == 0) ? 1 : 0;

    const char *df_str = c_opt_str(di, "data_format", "signed");
    s->is_signed = (strcmp(df_str, "unsigned") == 0) ? 0 : 1;

    /* A TDM stream can contain millions of audio samples. Creating a heap
     * allocated annotation object for every sample is both redundant when a
     * waveform is shown and can exhaust the UI process. Fast mode therefore
     * defaults to waveform-only; annotations remain available explicitly. */
    const char *output_str = c_opt_str(di, "output", "waveform");
    s->emit_annotations = strcmp(output_str, "waveform") != 0;
    s->emit_analog = strcmp(output_str, "annotations") != 0;
}

static void tdm_audio_fast_destroy(struct srd_decoder_inst *di)
{
    void *priv = c_decoder_get_private(di);
    if (priv) { g_free(priv); c_decoder_set_private(di, NULL); }
}

/* ------------------------------------------------------------------ */
/* Metadata                                                             */
/* ------------------------------------------------------------------ */

static void tdm_audio_fast_metadata(struct srd_decoder_inst *di, int key,
                                    uint64_t value)
{
    tdm_audio_fast_state *s = (tdm_audio_fast_state *)c_decoder_get_private(di);
    if (key == SRD_CONF_SAMPLERATE)
        s->samplerate = value;
    else if (key == SRD_CONF_CAPTURE_SAMPLES)
        s->capture_samples = value;
}

/* ------------------------------------------------------------------ */
/* Output one decoded sample (same as tdm_audio_c)                     */
/* ------------------------------------------------------------------ */

static void flush_analog_channel(struct srd_decoder_inst *di,
                                 tdm_audio_fast_state *s, int ch)
{
    const uint16_t count = s->analog_count[ch];
    if (count == 0)
        return;

    /* Keep the DLL/callback batching, but do NOT collapse the whole batch to
     * one start/end span.  The old c_put_analog() path made DecoderAnalogData
     * redistribute the batch uniformly.  Because a PCM word has finite width,
     * that introduced a small X-axis timing error which accumulated inside a
     * batch and snapped back at every flush boundary.  The PCM values (and WAV
     * export) stayed correct while the on-screen sine acquired visible kinks. */
    c_put_analog_timed(di, s->out_analog, ch, s->channels,
                       s->analog_buf[ch],
                       s->analog_start_buf[ch], s->analog_end_buf[ch],
                       count, 1.0);
    s->analog_count[ch] = 0;
}

static void flush_analog_all(struct srd_decoder_inst *di,
                             tdm_audio_fast_state *s)
{
    for (int ch = 0; ch < s->channels; ++ch)
        flush_analog_channel(di, s, ch);
}

static void output_sample(struct srd_decoder_inst *di, tdm_audio_fast_state *s,
                          int ch, uint64_t end_abs)
{
    if (s->emit_annotations) {
        char v[32];
        if (s->bitdepth <= 8)
            snprintf(v, sizeof(v), "%02llX", (unsigned long long)s->data);
        else if (s->bitdepth <= 16)
            snprintf(v, sizeof(v), "%04llX", (unsigned long long)s->data);
        else
            snprintf(v, sizeof(v), "%08llX", (unsigned long long)s->data);

        char ann_long[64], ann_mid[48], ann_short[48];
        snprintf(ann_long,  sizeof(ann_long),  "Ch%d: %s", ch, v);
        snprintf(ann_mid,   sizeof(ann_mid),   "C%d: %s",  ch, v);
        snprintf(ann_short, sizeof(ann_short), "%d:%s",    ch, v);
        /* In batch mode di->abs_cur_samplenum points at the END of the
         * fetched block, so pass the absolute sample of this bit. */
        c_put(di, s->ss_block, end_abs, s->out_ann, ch,
              ann_long, ann_mid, ann_short);
    }

    if (!s->emit_analog)
        return;

    /* Analog: convert to float -1..+1 (identical math to tdm_audio_c) */
    float normalized;
    if (s->is_signed) {
        int32_t val;
        if (s->bitdepth <= 16) {
            val = (int32_t)(s->data & 0xFFFF);
            if (val & (1 << (s->bitdepth - 1)))
                val |= ~((1 << s->bitdepth) - 1);
        } else {
            uint32_t mask = (s->bitdepth >= 32) ? 0xFFFFFFFFu
                              : ((1u << s->bitdepth) - 1);
            val = (int32_t)(s->data & mask);
            if (val & (1u << (s->bitdepth - 1)))
                val |= ~mask;
        }
        normalized = (float)val / (float)(1u << (s->bitdepth - 1));
    } else {
        uint32_t mask = (s->bitdepth >= 32) ? 0xFFFFFFFFu
                          : ((1u << s->bitdepth) - 1);
        uint32_t uval = (uint32_t)(s->data & mask);
        float mid = (float)(1u << (s->bitdepth - 1));
        normalized = ((float)uval - mid) / mid;
    }

    uint16_t count = s->analog_count[ch];
    const uint64_t sample_start = s->ss_block;

    /* Same-channel PCM words should be nearly periodic in capture-sample
     * coordinates.  Log only a few strong discontinuities so a future real
     * source/dropout problem can be separated from a renderer problem without
     * flooding PXView.log. */
    if (s->analog_have_prev[ch]) {
        const uint64_t delta = sample_start - s->analog_prev_start[ch];
        uint64_t nominal = s->analog_nominal_delta[ch];
        if (nominal == 0 && delta != 0) {
            nominal = delta;
            s->analog_nominal_delta[ch] = delta;
        } else if (nominal != 0 && delta != 0) {
            const int strong_gap =
                (delta > nominal + nominal / 2) ||
                (delta + delta / 2 < nominal);
            if (strong_gap && s->analog_gap_logs[ch] < 4) {
                fprintf(stderr,
                        "PXView: TDM Fast analog timing discontinuity: "
                        "ch=%d prev=%llu cur=%llu delta=%llu nominal=%llu\n",
                        ch,
                        (unsigned long long)s->analog_prev_start[ch],
                        (unsigned long long)sample_start,
                        (unsigned long long)delta,
                        (unsigned long long)nominal);
                ++s->analog_gap_logs[ch];
            } else if (!strong_gap) {
                /* Slow integer IIR keeps nominal tolerant of harmless one-
                 * capture-sample quantisation jitter. */
                s->analog_nominal_delta[ch] = (nominal * 7 + delta + 4) / 8;
            }
        }
    } else {
        s->analog_have_prev[ch] = 1;
    }
    s->analog_prev_start[ch] = sample_start;

    s->analog_buf[ch][count] = normalized;
    s->analog_start_buf[ch][count] = sample_start;
    s->analog_end_buf[ch][count] = end_abs;
    ++count;
    s->analog_count[ch] = count;
    if (count >= s->analog_batch)
        flush_analog_channel(di, s, ch);
}

/* ------------------------------------------------------------------ */
/* Core per-bit state machine (identical semantics to tdm_audio_c)     */
/* ------------------------------------------------------------------ */

static inline void reset_ch0(struct srd_decoder_inst *di,
                             tdm_audio_fast_state *s, uint64_t abs_sample)
{
    s->channel   = 0;
    s->bitcount  = 0;
    s->data      = 0;
    s->ss_block  = abs_sample;
    s->have_ss_block = 1;
    (void)di;
}

static inline void handle_bit(struct srd_decoder_inst *di,
                              tdm_audio_fast_state *s,
                              uint64_t abs_sample,
                              int frame, int data)
{
    /* LRCK active edge: reset CH0 */
    if (s->lastframe == s->fi && frame == s->fa)
        s->i2s_bck_cnt = 0;
    else
        s->i2s_bck_cnt++;
    s->lastframe = frame;

    if (s->align) {
        /* I2S: skip 1 BCK */
        if (s->i2s_bck_cnt == 1)
            reset_ch0(di, s, abs_sample);
        s->data = (s->data << 1) | data;
    } else {
        /* Left-justified */
        if (s->i2s_bck_cnt == 0)
            reset_ch0(di, s, abs_sample);
        s->data = (s->data << 1) | data;
    }
    s->bitcount++;

    if (s->have_ss_block && s->bitcount >= s->bitdepth) {
        s->bitcount = 0;
        output_sample(di, s, s->channel % s->channels, abs_sample);
        s->data = 0;
        s->ss_block = abs_sample;
        s->samplecount++;
        s->channel++;
    }
}

/* ------------------------------------------------------------------ */
/* v4 packed zero-copy edge scanning                                   */
/* ------------------------------------------------------------------ */

static inline unsigned tdm_ctz64(uint64_t value)
{
#if defined(_MSC_VER) && defined(_M_X64)
    unsigned long index = 0;
    _BitScanForward64(&index, value);
    return (unsigned)index;
#else
    return (unsigned)__builtin_ctzll(value);
#endif
}

static inline uint64_t tdm_low_mask64(unsigned bits)
{
    return bits >= 64 ? UINT64_MAX
                      : (bits == 0 ? 0ULL : ((1ULL << bits) - 1ULL));
}

static inline int tdm_packed_bit(const uint8_t *buffer, uint8_t const_value,
                                 uint64_t bit_index)
{
    if (!buffer)
        return const_value ? 1 : 0;
    return (buffer[bit_index >> 3] >> (bit_index & 7u)) & 1u;
}

static inline uint64_t tdm_load_packed_word(const uint8_t *buffer,
                                            uint8_t const_value,
                                            uint64_t bit_index,
                                            unsigned width)
{
    const uint64_t valid = tdm_low_mask64(width);
    if (!buffer)
        return const_value ? valid : 0ULL;

    /* Caller aligns the bulk portion to a byte boundary.  memcpy keeps this
     * safe on architectures that dislike unaligned uint64_t loads and is
     * optimized to a native load by modern compilers. */
    const uint8_t *src = buffer + (bit_index >> 3);
    uint64_t word = 0;
    const size_t bytes = (size_t)((width + 7u) >> 3);
    memcpy(&word, src, bytes);
    return word & valid;
}

static inline void tdm_process_word(struct srd_decoder_inst *di,
                                    tdm_audio_fast_state *s,
                                    uint64_t clk, uint64_t frame,
                                    uint64_t data, unsigned width,
                                    uint64_t base_abs, int want_edge,
                                    int *prev_clk)
{
    const uint64_t valid = tdm_low_mask64(width);
    clk &= valid;

    uint64_t edges;
    if (want_edge == 0) {
        /* bit k: clk[k] == 1 && clk[k-1] == 0 */
        edges = clk & ~(clk << 1);
        edges &= ~1ULL;
        if ((clk & 1ULL) && !*prev_clk)
            edges |= 1ULL;
    } else {
        /* bit k: clk[k] == 0 && clk[k-1] == 1 */
        edges = (~clk) & (clk << 1);
        edges &= ~1ULL;
        if (!(clk & 1ULL) && *prev_clk)
            edges |= 1ULL;
    }
    edges &= valid;

    while (edges) {
        const unsigned bit = tdm_ctz64(edges);
        handle_bit(di, s, base_abs + bit,
                   (int)((frame >> bit) & 1ULL),
                   (int)((data >> bit) & 1ULL));
        edges &= edges - 1ULL;
    }

    *prev_clk = (int)((clk >> (width - 1u)) & 1ULL);
}

static void process_packed(struct srd_decoder_inst *di,
                           tdm_audio_fast_state *s,
                           const uint8_t *clk_buf, uint8_t clk_const,
                           const uint8_t *frame_buf, uint8_t frame_const,
                           const uint8_t *data_buf, uint8_t data_const,
                           uint64_t count, uint64_t base_abs,
                           uint8_t bit_offset, int want_edge)
{
    uint64_t pos = 0;
    int prev_clk = s->edge_prev_clk;

    /* The first borrowed byte may begin before base_abs because v4 aligns the
     * chunk base down to 8 samples.  Handle at most seven head samples, then
     * the rest is byte/word aligned. */
    if (bit_offset && count) {
        uint64_t head = 8u - bit_offset;
        if (head > count)
            head = count;
        for (uint64_t i = 0; i < head; ++i) {
            const uint64_t bi = (uint64_t)bit_offset + i;
            const int clk = tdm_packed_bit(clk_buf, clk_const, bi);
            const int edge = (want_edge == 0)
                                 ? (clk && !prev_clk)
                                 : (!clk && prev_clk);
            if (edge) {
                handle_bit(di, s, base_abs + i,
                           tdm_packed_bit(frame_buf, frame_const, bi),
                           tdm_packed_bit(data_buf, data_const, bi));
            }
            prev_clk = clk;
        }
        pos += head;
    }

    while (count - pos >= 64) {
        const uint64_t packed_bit = (uint64_t)bit_offset + pos;
        const uint64_t clk = tdm_load_packed_word(
            clk_buf, clk_const, packed_bit, 64);
        const uint64_t frame = tdm_load_packed_word(
            frame_buf, frame_const, packed_bit, 64);
        const uint64_t data = tdm_load_packed_word(
            data_buf, data_const, packed_bit, 64);
        tdm_process_word(di, s, clk, frame, data, 64,
                         base_abs + pos, want_edge, &prev_clk);
        pos += 64;
    }

    if (pos < count) {
        const unsigned width = (unsigned)(count - pos);
        const uint64_t packed_bit = (uint64_t)bit_offset + pos;
        const uint64_t clk = tdm_load_packed_word(
            clk_buf, clk_const, packed_bit, width);
        const uint64_t frame = tdm_load_packed_word(
            frame_buf, frame_const, packed_bit, width);
        const uint64_t data = tdm_load_packed_word(
            data_buf, data_const, packed_bit, width);
        tdm_process_word(di, s, clk, frame, data, width,
                         base_abs + pos, want_edge, &prev_clk);
    }

    s->edge_prev_clk = prev_clk;
}

/* ------------------------------------------------------------------ */
/* Decode                                                               */
/* ------------------------------------------------------------------ */

static void tdm_audio_fast_decode(struct srd_decoder_inst *di)
{
    tdm_audio_fast_state *s = (tdm_audio_fast_state *)c_decoder_get_private(di);
    const int channels[3] = {0, 1, 2};

    /* Preserve BCK history across v4 input chunks. */
    if (!s->have_prev_clk) {
        s->edge_prev_clk = s->edge; /* rising->0, falling->1 */
        s->have_prev_clk = 1;
    }

    while (1) {
        const uint8_t *buffers[3] = {NULL, NULL, NULL};
        uint8_t const_values[3] = {0, 0, 0};
        uint64_t base_abs = 0;
        uint8_t bit_offset = 0;

        const uint64_t n = c_fetch_packed_multi(
            di, channels, buffers, const_values, 3, s->fetch_block,
            &base_abs, &bit_offset);
        if (n == 0)
            break;

        process_packed(di, s,
                       buffers[0], const_values[0],
                       buffers[1], const_values[1],
                       buffers[2], const_values[2],
                       n, base_abs, bit_offset, s->edge);

        if (c_consume_samples(di, n) != SRD_OK)
            break;

        s->processed_samples += n;
        update_adaptive_batches(s);
    }

    flush_analog_all(di, s);
}

/* ------------------------------------------------------------------ */
/* Decoder registration                                                */
/* ------------------------------------------------------------------ */

struct srd_c_decoder tdm_audio_fast_c_decoder = {
    .id = "tdm_audio_fast",
    .name = "TDM audio (Fast)",
    .longname = "Time division multiplex audio (Fast/ASM)",
    .desc = "TDM/I2S multi-channel audio (adaptive batch+SIMD) with analog waveform",
    .license = "gplv2+",
    .channels = tdm_audio_fast_channels,
    .num_channels = 3,
    .optional_channels = NULL,
    .num_optional_channels = 0,
    .options = tdm_audio_fast_options,
    .num_options = 14 + TDM_AUDIO_MAX_CHANNELS * 7,
    .num_annotations = NUM_ANN,
    .ann_labels = tdm_audio_fast_ann_labels,
    .num_annotation_rows = 8,
    .annotation_rows = tdm_audio_fast_ann_rows,
    .inputs = tdm_audio_fast_inputs,
    .num_inputs = 1,
    .outputs = NULL,
    .num_outputs = 0,
    .binary = NULL,
    .num_binary = 0,
    .tags = tdm_audio_fast_tags,
    .num_tags = 1,
    .reset = tdm_audio_fast_reset,
    .start = tdm_audio_fast_start,
    .decode = tdm_audio_fast_decode,
    .destroy = tdm_audio_fast_destroy,
    .state_size = 0,
    .metadata = tdm_audio_fast_metadata,
};

SRD_C_DECODER_EXPORT struct srd_c_decoder *srd_c_decoder_entry(void)
{
    tdm_audio_fast_options[0].idn = "dec_tdm_audio_fast_opt_bps";
    tdm_audio_fast_options[0].def = g_variant_new_int64(32);

    tdm_audio_fast_options[1].idn = "dec_tdm_audio_fast_opt_channels";
    tdm_audio_fast_options[1].def = g_variant_new_int64(8);
    GSList *ch_vals = NULL;
    for (int i = 1; i <= 8; i++)
        ch_vals = g_slist_append(ch_vals, g_variant_new_int64(i));
    tdm_audio_fast_options[1].values = ch_vals;

    tdm_audio_fast_options[2].idn = "dec_tdm_audio_fast_opt_edge";
    tdm_audio_fast_options[2].def = g_variant_new_string("rising");
    GSList *edge_vals = NULL;
    edge_vals = g_slist_append(edge_vals, g_variant_new_string("rising"));
    edge_vals = g_slist_append(edge_vals, g_variant_new_string("falling"));
    tdm_audio_fast_options[2].values = edge_vals;

    tdm_audio_fast_options[3].idn = "dec_tdm_audio_fast_opt_frame_edge";
    tdm_audio_fast_options[3].def = g_variant_new_string("high");
    GSList *fe_vals = NULL;
    fe_vals = g_slist_append(fe_vals, g_variant_new_string("high"));
    fe_vals = g_slist_append(fe_vals, g_variant_new_string("low"));
    tdm_audio_fast_options[3].values = fe_vals;

    tdm_audio_fast_options[4].idn = "dec_tdm_audio_fast_opt_align";
    tdm_audio_fast_options[4].def = g_variant_new_string("I2S");
    GSList *al_vals = NULL;
    al_vals = g_slist_append(al_vals, g_variant_new_string("left-justified"));
    al_vals = g_slist_append(al_vals, g_variant_new_string("I2S"));
    tdm_audio_fast_options[4].values = al_vals;

    tdm_audio_fast_options[5].idn = "dec_tdm_audio_fast_opt_data_format";
    tdm_audio_fast_options[5].def = g_variant_new_string("signed");
    GSList *df_vals = NULL;
    df_vals = g_slist_append(df_vals, g_variant_new_string("signed"));
    df_vals = g_slist_append(df_vals, g_variant_new_string("unsigned"));
    tdm_audio_fast_options[5].values = df_vals;

    tdm_audio_fast_options[6].idn = "dec_tdm_audio_fast_opt_output";
    tdm_audio_fast_options[6].def = g_variant_new_string("waveform");
    GSList *output_vals = NULL;
    output_vals = g_slist_append(output_vals, g_variant_new_string("waveform"));
    output_vals = g_slist_append(output_vals, g_variant_new_string("both"));
    output_vals = g_slist_append(output_vals, g_variant_new_string("annotations"));
    tdm_audio_fast_options[6].values = output_vals;

    /* Per-channel display options grouped by function */
    for (int i = 0; i < TDM_AUDIO_MAX_CHANNELS; i++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "dec_tdm_audio_fast_opt_ch%d_vzoom", i);
        tdm_audio_fast_options[7 + i].idn = g_strdup(buf);
        tdm_audio_fast_options[7 + i].def = g_variant_new_double(1.0);
    }
    for (int i = 0; i < TDM_AUDIO_MAX_CHANNELS; i++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "dec_tdm_audio_fast_opt_ch%d_vpos", i);
        tdm_audio_fast_options[15 + i].idn = g_strdup(buf);
        tdm_audio_fast_options[15 + i].def = g_variant_new_double(1.0);
    }
    for (int i = 0; i < TDM_AUDIO_MAX_CHANNELS; i++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "dec_tdm_audio_fast_opt_ch%d_enable", i);
        tdm_audio_fast_options[23 + i].idn = g_strdup(buf);
        tdm_audio_fast_options[23 + i].def = g_variant_new_int64(1);
    }

    for (int i = 0; i < TDM_AUDIO_MAX_CHANNELS; i++) {
        char buf[64];
        GSList *range_vals = NULL;
        range_vals = g_slist_append(range_vals, g_variant_new_string("bipolar"));
        range_vals = g_slist_append(range_vals, g_variant_new_string("unipolar"));
        range_vals = g_slist_append(range_vals, g_variant_new_string("custom"));
        snprintf(buf, sizeof(buf), "dec_tdm_audio_fast_opt_ch%d_range_mode", i);
        tdm_audio_fast_options[31 + i].idn = g_strdup(buf);
        tdm_audio_fast_options[31 + i].def = g_variant_new_string("bipolar");
        tdm_audio_fast_options[31 + i].values = range_vals;
    }
    for (int i = 0; i < TDM_AUDIO_MAX_CHANNELS; i++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "dec_tdm_audio_fast_opt_ch%d_eng_min", i);
        tdm_audio_fast_options[39 + i].idn = g_strdup(buf);
        tdm_audio_fast_options[39 + i].def = g_variant_new_double(-1.0);
    }
    for (int i = 0; i < TDM_AUDIO_MAX_CHANNELS; i++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "dec_tdm_audio_fast_opt_ch%d_eng_max", i);
        tdm_audio_fast_options[47 + i].idn = g_strdup(buf);
        tdm_audio_fast_options[47 + i].def = g_variant_new_double(1.0);
    }
    for (int i = 0; i < TDM_AUDIO_MAX_CHANNELS; i++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "dec_tdm_audio_fast_opt_ch%d_unit", i);
        tdm_audio_fast_options[55 + i].idn = g_strdup(buf);
        tdm_audio_fast_options[55 + i].def = g_variant_new_string("V");
    }

    tdm_audio_fast_options[63].idn =
        "dec_tdm_audio_fast_opt_realtime_decode";
    tdm_audio_fast_options[63].def = g_variant_new_boolean(FALSE);

    tdm_audio_fast_options[64].idn =
        "dec_tdm_audio_fast_opt_display_trigger_enable";
    tdm_audio_fast_options[64].def = g_variant_new_boolean(FALSE);

    tdm_audio_fast_options[65].idn =
        "dec_tdm_audio_fast_opt_display_trigger_mode";
    tdm_audio_fast_options[65].def = g_variant_new_string("auto");
    GSList *display_trigger_mode_vals = NULL;
    display_trigger_mode_vals = g_slist_append(
        display_trigger_mode_vals, g_variant_new_string("auto"));
    display_trigger_mode_vals = g_slist_append(
        display_trigger_mode_vals, g_variant_new_string("normal"));
    tdm_audio_fast_options[65].values = display_trigger_mode_vals;

    tdm_audio_fast_options[66].idn =
        "dec_tdm_audio_fast_opt_display_trigger_channel";
    tdm_audio_fast_options[66].def = g_variant_new_int64(0);
    GSList *display_trigger_channel_vals = NULL;
    for (int i = 0; i < TDM_AUDIO_MAX_CHANNELS; ++i)
        display_trigger_channel_vals = g_slist_append(
            display_trigger_channel_vals, g_variant_new_int64(i));
    tdm_audio_fast_options[66].values = display_trigger_channel_vals;

    tdm_audio_fast_options[67].idn =
        "dec_tdm_audio_fast_opt_display_trigger_edge";
    tdm_audio_fast_options[67].def = g_variant_new_string("rising");
    GSList *display_trigger_edge_vals = NULL;
    display_trigger_edge_vals = g_slist_append(
        display_trigger_edge_vals, g_variant_new_string("rising"));
    display_trigger_edge_vals = g_slist_append(
        display_trigger_edge_vals, g_variant_new_string("falling"));
    display_trigger_edge_vals = g_slist_append(
        display_trigger_edge_vals, g_variant_new_string("either"));
    tdm_audio_fast_options[67].values = display_trigger_edge_vals;

    tdm_audio_fast_options[68].idn =
        "dec_tdm_audio_fast_opt_display_trigger_level";
    tdm_audio_fast_options[68].def = g_variant_new_double(0.0);

    tdm_audio_fast_options[69].idn =
        "dec_tdm_audio_fast_opt_display_trigger_position";
    tdm_audio_fast_options[69].def = g_variant_new_int64(50);

    return &tdm_audio_fast_c_decoder;
}

SRD_C_DECODER_EXPORT int srd_c_decoder_api_version(void)
{
    return SRD_C_DECODER_API_VERSION;
}
