/*
 * pwm_waveform_c — fast multi-mode PWM decoder with analog reconstruction.
 *
 * The original pwm_c decoder is intentionally left untouched. This sibling
 * is adapted for PXView 1.5.5 C decoder API v4: it borrows the native
 * bit-packed channel buffers directly (zero-copy), scans 64 samples per
 * machine word, and keeps exact-timestamp batched analog callbacks. It supports:
 *
 *   - Generic PWM duty-cycle display (0..100 %)
 *   - Power-supply/control PWM engineering display
 *   - Class-D single-ended AD reconstruction (50 % = zero)
 *   - Class-D differential BD reconstruction (PWM+ minus PWM-)
 *   - Per-cycle hold, moving-average, exponential, and RC low-pass methods
 *   - V-POS / V-ZOOM for four analog traces
 *   - Duty, frequency, pulse-width, common-mode, dead-time and overlap metrics
 *
 * Enhancements and Chinese documentation: ZB-FENG
 * SPDX-License-Identifier: GPL-2.0+
 */

#include "libsigrokdecode.h"

#include <glib.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(_MSC_VER) && defined(_M_X64)
#include <intrin.h>
#endif


#define PWM_FETCH_BLOCK 65536
#define PWM_ANALOG_BATCH_MIN    256
#define PWM_ANALOG_BATCH_MEDIUM 512
#define PWM_ANALOG_BATCH_LARGE  2048
#define PWM_ANALOG_BATCH_MAX    4096
#define PWM_FILTER_MAX  128
#define PWM_FILTER_CUTOFF_MIN_HZ 0.01
#define PWM_FILTER_CUTOFF_MAX_HZ 10000000.0
#define PWM_TWO_PI      6.283185307179586476925286766559

#define PWM_DEFAULT_DUTY_DECIMALS 4
#define PWM_DEFAULT_TIME_DECIMALS 3
#define PWM_DEFAULT_FREQ_DECIMALS 3
#define PWM_DECIMALS_MAX          8

/* Analog output channels. */
enum {
    ANALOG_MAIN = 0,   /* Reconstructed / engineering output. */
    ANALOG_DUTY_A,     /* Raw duty of PWM A/PWM+. */
    ANALOG_DUTY_B,     /* Raw duty of PWM B/PWM-. */
    ANALOG_COMMON,     /* (Duty A + Duty B) / 2. */
    NUM_ANALOG_CHANNELS,
};

/* Annotation classes. */
enum {
    ANN_OUTPUT = 0,
    ANN_DUTY_A,
    ANN_DUTY_B,
    ANN_DIFFERENTIAL,
    ANN_COMMON,
    ANN_PERIOD,
    ANN_FREQUENCY,
    ANN_ACTIVE_A,
    ANN_INACTIVE_A,
    ANN_ACTIVE_B,
    ANN_INACTIVE_B,
    ANN_BOTH_INACTIVE,
    ANN_BOTH_ACTIVE,
    ANN_WARNING,
    NUM_ANN,
};

enum pwm_filter_type {
    PWM_FILTER_NONE = 0,
    PWM_FILTER_MOVING_AVERAGE,
    PWM_FILTER_EXPONENTIAL,
    PWM_FILTER_RC_LOWPASS,
};

enum pwm_application_mode {
    PWM_MODE_GENERIC = 0,
    PWM_MODE_POWER_SUPPLY,
    PWM_MODE_CLASSD_AD,
    PWM_MODE_CLASSD_BD,
};

enum pwm_option_index {
    OPT_POLARITY_A = 0,
    OPT_POLARITY_B,
    OPT_OUTPUT,
    OPT_ANALOG_MODE,
    OPT_FILTER,
    OPT_FILTER_CYCLES,
    OPT_FILTER_CUTOFF_HZ,
    OPT_DUTY_DECIMALS,
    OPT_TIME_DECIMALS,
    OPT_FREQ_DECIMALS,

    OPT_CH0_VZOOM,
    OPT_CH0_VPOS,
    OPT_CH0_ENABLE,
    OPT_CH0_RANGE_MODE,
    OPT_CH0_ENG_MIN,
    OPT_CH0_ENG_MAX,
    OPT_CH0_UNIT,

    OPT_CH1_VZOOM,
    OPT_CH1_VPOS,
    OPT_CH1_ENABLE,
    OPT_CH1_RANGE_MODE,
    OPT_CH1_ENG_MIN,
    OPT_CH1_ENG_MAX,
    OPT_CH1_UNIT,

    OPT_CH2_VZOOM,
    OPT_CH2_VPOS,
    OPT_CH2_ENABLE,
    OPT_CH2_RANGE_MODE,
    OPT_CH2_ENG_MIN,
    OPT_CH2_ENG_MAX,
    OPT_CH2_UNIT,

    OPT_CH3_VZOOM,
    OPT_CH3_VPOS,
    OPT_CH3_ENABLE,
    OPT_CH3_RANGE_MODE,
    OPT_CH3_ENG_MIN,
    OPT_CH3_ENG_MAX,
    OPT_CH3_UNIT,

    OPT_REALTIME_DECODE,

    OPT_DISPLAY_TRIGGER_ENABLE,
    OPT_DISPLAY_TRIGGER_MODE,
    OPT_DISPLAY_TRIGGER_CHANNEL,
    OPT_DISPLAY_TRIGGER_EDGE,
    OPT_DISPLAY_TRIGGER_LEVEL,
    OPT_DISPLAY_TRIGGER_POSITION,

    NUM_OPTIONS,
};

typedef struct {
    uint64_t samplerate;
    uint64_t capture_samples;
    uint64_t processed_samples;
    uint16_t analog_batch;

    int active_level_a;
    int active_level_b;
    int previous_active_a;
    int have_previous_a;
    int has_b;
    int warned_missing_b;

    int cycle_started;
    uint64_t cycle_start;
    uint64_t count_a_active;
    uint64_t count_b_active;
    uint64_t count_both_inactive;
    uint64_t count_both_active;

    int out_ann;
    int out_binary;
    int out_analog;
    int out_average_output;
    int out_min_output;
    int out_max_output;
    int out_average_duty_a;
    int out_average_duty_b;
    int out_average_frequency;

    int emit_annotations;
    int emit_analog;
    int emit_parameters;
    uint8_t analog_enabled[NUM_ANALOG_CHANNELS];

    enum pwm_application_mode application_mode;
    enum pwm_filter_type filter_type;
    int filter_cycles;
    double filter_cutoff_hz;

    double moving_values[PWM_FILTER_MAX];
    int moving_count;
    int moving_pos;
    double moving_sum;
    double filtered_value;
    int filter_valid;

    int duty_decimals;
    int time_decimals;
    int freq_decimals;

    int64_t num_cycles;
    double output_sum_percent;
    double output_min_percent;
    double output_max_percent;
    double duty_a_sum_percent;
    double duty_b_sum_percent;
    double frequency_sum_hz;
    uint64_t first_cycle_start;
    int have_first_cycle;

    /* Batch analog callbacks like tdm_audio_fast, but keep exact PWM-cycle
     * timing through c_put_analog_timed(). */
    float analog_values[NUM_ANALOG_CHANNELS][PWM_ANALOG_BATCH_MAX];
    uint64_t analog_starts[NUM_ANALOG_CHANNELS][PWM_ANALOG_BATCH_MAX];
    uint64_t analog_ends[NUM_ANALOG_CHANNELS][PWM_ANALOG_BATCH_MAX];
    uint16_t analog_count[NUM_ANALOG_CHANNELS];
} pwm_waveform_state;

static struct srd_channel pwm_waveform_channels[] = {
    {"pwm_a", "PWM A / PWM+", "Primary PWM or positive Class-D output", 0,
     SRD_CHANNEL_SDATA, "dec_pwm_waveform_chan_a"},
};

static struct srd_channel pwm_waveform_optional_channels[] = {
    {"pwm_b", "PWM B / PWM-", "Optional negative/differential PWM output", 1,
     SRD_CHANNEL_SDATA, "dec_pwm_waveform_opt_chan_b"},
};

static struct srd_decoder_option pwm_waveform_options[NUM_OPTIONS] = {
    [OPT_POLARITY_A] = {"polarity", NULL, "PWM A active polarity", NULL, NULL},
    [OPT_POLARITY_B] = {"polarity_b", NULL, "PWM B active polarity", NULL, NULL},
    [OPT_OUTPUT] = {"output", NULL, "Output mode", NULL, NULL},
    [OPT_ANALOG_MODE] = {"analog_mode", NULL, "Application / analog reconstruction mode", NULL, NULL},
    [OPT_FILTER] = {"filter", NULL, "Analog reconstruction / filter method", NULL, NULL},
    [OPT_FILTER_CYCLES] = {"filter_cycles", NULL, "Filter length (PWM cycles)", NULL, NULL},
    [OPT_FILTER_CUTOFF_HZ] = {"filter_cutoff_hz", NULL, "RC low-pass cutoff frequency (Hz)", NULL, NULL},
    [OPT_DUTY_DECIMALS] = {"duty_decimals", NULL, "Duty / output decimal places", NULL, NULL},
    [OPT_TIME_DECIMALS] = {"time_decimals", NULL, "Time decimal places", NULL, NULL},
    [OPT_FREQ_DECIMALS] = {"freq_decimals", NULL, "Frequency decimal places", NULL, NULL},

    [OPT_CH0_VZOOM] = {"ch0_vzoom", NULL, "V-ZOOM Main output", NULL, NULL},
    [OPT_CH0_VPOS] = {"ch0_vpos", NULL, "V-POS Main output", NULL, NULL},
    [OPT_CH0_ENABLE] = {"ch0_enable", NULL, "SW Main output", NULL, NULL},
    [OPT_CH0_RANGE_MODE] = {"ch0_range_mode", NULL, "Range mode Main output", NULL, NULL},
    [OPT_CH0_ENG_MIN] = {"ch0_eng_min", NULL, "Engineering Min Main output", NULL, NULL},
    [OPT_CH0_ENG_MAX] = {"ch0_eng_max", NULL, "Engineering Max Main output", NULL, NULL},
    [OPT_CH0_UNIT] = {"ch0_unit", NULL, "Engineering unit Main output", NULL, NULL},

    [OPT_CH1_VZOOM] = {"ch1_vzoom", NULL, "V-ZOOM Duty A", NULL, NULL},
    [OPT_CH1_VPOS] = {"ch1_vpos", NULL, "V-POS Duty A", NULL, NULL},
    [OPT_CH1_ENABLE] = {"ch1_enable", NULL, "SW Duty A", NULL, NULL},
    [OPT_CH1_RANGE_MODE] = {"ch1_range_mode", NULL, "Range mode Duty A", NULL, NULL},
    [OPT_CH1_ENG_MIN] = {"ch1_eng_min", NULL, "Engineering Min Duty A", NULL, NULL},
    [OPT_CH1_ENG_MAX] = {"ch1_eng_max", NULL, "Engineering Max Duty A", NULL, NULL},
    [OPT_CH1_UNIT] = {"ch1_unit", NULL, "Engineering unit Duty A", NULL, NULL},

    [OPT_CH2_VZOOM] = {"ch2_vzoom", NULL, "V-ZOOM Duty B", NULL, NULL},
    [OPT_CH2_VPOS] = {"ch2_vpos", NULL, "V-POS Duty B", NULL, NULL},
    [OPT_CH2_ENABLE] = {"ch2_enable", NULL, "SW Duty B", NULL, NULL},
    [OPT_CH2_RANGE_MODE] = {"ch2_range_mode", NULL, "Range mode Duty B", NULL, NULL},
    [OPT_CH2_ENG_MIN] = {"ch2_eng_min", NULL, "Engineering Min Duty B", NULL, NULL},
    [OPT_CH2_ENG_MAX] = {"ch2_eng_max", NULL, "Engineering Max Duty B", NULL, NULL},
    [OPT_CH2_UNIT] = {"ch2_unit", NULL, "Engineering unit Duty B", NULL, NULL},

    [OPT_CH3_VZOOM] = {"ch3_vzoom", NULL, "V-ZOOM Common mode", NULL, NULL},
    [OPT_CH3_VPOS] = {"ch3_vpos", NULL, "V-POS Common mode", NULL, NULL},
    [OPT_CH3_ENABLE] = {"ch3_enable", NULL, "SW Common mode", NULL, NULL},
    [OPT_CH3_RANGE_MODE] = {"ch3_range_mode", NULL, "Range mode Common mode", NULL, NULL},
    [OPT_CH3_ENG_MIN] = {"ch3_eng_min", NULL, "Engineering Min Common mode", NULL, NULL},
    [OPT_CH3_ENG_MAX] = {"ch3_eng_max", NULL, "Engineering Max Common mode", NULL, NULL},
    [OPT_CH3_UNIT] = {"ch3_unit", NULL, "Engineering unit Common mode", NULL, NULL},

    [OPT_REALTIME_DECODE] = {"realtime_decode", NULL,
                             "Decode and display while capturing", NULL, NULL},

    [OPT_DISPLAY_TRIGGER_ENABLE] = {
        "display_trigger_enable", NULL,
        "Repeat capture: analog display trigger", NULL, NULL},
    [OPT_DISPLAY_TRIGGER_MODE] = {
        "display_trigger_mode", NULL, "Display trigger mode", NULL, NULL},
    [OPT_DISPLAY_TRIGGER_CHANNEL] = {
        "display_trigger_channel", NULL,
        "Display trigger analog channel", NULL, NULL},
    [OPT_DISPLAY_TRIGGER_EDGE] = {
        "display_trigger_edge", NULL, "Display trigger edge", NULL, NULL},
    [OPT_DISPLAY_TRIGGER_LEVEL] = {
        "display_trigger_level", NULL,
        "Display trigger level (engineering unit)", NULL, NULL},
    [OPT_DISPLAY_TRIGGER_POSITION] = {
        "display_trigger_position", NULL,
        "Display trigger horizontal position", NULL, NULL},
};

_Static_assert(G_N_ELEMENTS(pwm_waveform_options) == NUM_OPTIONS,
               "PWM option table size mismatch");

static const char *pwm_waveform_inputs[] = {"logic", NULL};
static const char *pwm_waveform_outputs[] = {NULL};
static const char *pwm_waveform_tags[] = {"Encoding", "Audio", NULL};

static const char *pwm_waveform_ann_labels[][3] = {
    {"", "output", "Reconstructed output"},
    {"", "duty-a", "Duty A"},
    {"", "duty-b", "Duty B"},
    {"", "differential", "Differential"},
    {"", "common-mode", "Common mode"},
    {"", "period", "Period"},
    {"", "frequency", "Frequency"},
    {"", "active-a", "A active width"},
    {"", "inactive-a", "A inactive width"},
    {"", "active-b", "B active width"},
    {"", "inactive-b", "B inactive width"},
    {"", "both-inactive", "Both inactive / dead-time total"},
    {"", "both-active", "Both active / overlap total"},
    {"", "warning", "Warning"},
};

#define ANN_ROW(name_, title_, cls_)                         \
    static const int row_##name_[] = {cls_};                 \
    /* declaration continues below */

ANN_ROW(output, output, ANN_OUTPUT)
ANN_ROW(duty_a, duty_a, ANN_DUTY_A)
ANN_ROW(duty_b, duty_b, ANN_DUTY_B)
ANN_ROW(differential, differential, ANN_DIFFERENTIAL)
ANN_ROW(common, common, ANN_COMMON)
ANN_ROW(period, period, ANN_PERIOD)
ANN_ROW(frequency, frequency, ANN_FREQUENCY)
ANN_ROW(active_a, active_a, ANN_ACTIVE_A)
ANN_ROW(inactive_a, inactive_a, ANN_INACTIVE_A)
ANN_ROW(active_b, active_b, ANN_ACTIVE_B)
ANN_ROW(inactive_b, inactive_b, ANN_INACTIVE_B)
ANN_ROW(both_inactive, both_inactive, ANN_BOTH_INACTIVE)
ANN_ROW(both_active, both_active, ANN_BOTH_ACTIVE)
ANN_ROW(warning, warning, ANN_WARNING)

static const struct srd_c_ann_row pwm_waveform_ann_rows[] = {
    {"output", "Reconstructed output", row_output, 1},
    {"duty-a", "Duty A", row_duty_a, 1},
    {"duty-b", "Duty B", row_duty_b, 1},
    {"differential", "Differential", row_differential, 1},
    {"common-mode", "Common mode", row_common, 1},
    {"period", "Period", row_period, 1},
    {"frequency", "Frequency", row_frequency, 1},
    {"active-a", "A active width", row_active_a, 1},
    {"inactive-a", "A inactive width", row_inactive_a, 1},
    {"active-b", "B active width", row_active_b, 1},
    {"inactive-b", "B inactive width", row_inactive_b, 1},
    {"both-inactive", "Both inactive / dead-time total", row_both_inactive, 1},
    {"both-active", "Both active / overlap total", row_both_active, 1},
    {"warning", "Warning", row_warning, 1},
};

static const struct srd_decoder_binary pwm_waveform_binary[] = {
    {0, "raw", "Duty A and Duty B bytes"},
};

static int clamp_decimals(int decimals, int fallback)
{
    if (decimals < 0)
        decimals = fallback;
    if (decimals > PWM_DECIMALS_MAX)
        decimals = PWM_DECIMALS_MAX;
    return decimals;
}

static double clamp_double(double value, double minimum, double maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static void pwm_waveform_reset(struct srd_decoder_inst *di)
{
    if (!c_decoder_get_private(di))
        c_decoder_set_private(di, g_malloc0(sizeof(pwm_waveform_state)));

    pwm_waveform_state *s =
        (pwm_waveform_state *)c_decoder_get_private(di);
    memset(s, 0, sizeof(*s));

    s->out_ann = -1;
    s->out_binary = -1;
    s->out_analog = -1;
    s->out_average_output = -1;
    s->out_min_output = -1;
    s->out_max_output = -1;
    s->out_average_duty_a = -1;
    s->out_average_duty_b = -1;
    s->out_average_frequency = -1;

    s->filter_cycles = 8;
    s->filter_cutoff_hz = 20000.0;
    s->application_mode = PWM_MODE_GENERIC;
    s->filter_type = PWM_FILTER_NONE;
    s->duty_decimals = PWM_DEFAULT_DUTY_DECIMALS;
    s->time_decimals = PWM_DEFAULT_TIME_DECIMALS;
    s->freq_decimals = PWM_DEFAULT_FREQ_DECIMALS;
    s->analog_batch = PWM_ANALOG_BATCH_MIN;
}

static void pwm_waveform_metadata(struct srd_decoder_inst *di, int key,
                                  uint64_t value)
{
    pwm_waveform_state *s =
        (pwm_waveform_state *)c_decoder_get_private(di);
    if (key == SRD_CONF_SAMPLERATE)
        s->samplerate = value;
    else if (key == SRD_CONF_CAPTURE_SAMPLES)
        s->capture_samples = value;
}

static void pwm_update_adaptive_batches(pwm_waveform_state *s)
{
    uint64_t samples = s->capture_samples;
    if (s->processed_samples > samples)
        samples = s->processed_samples;

    const uint64_t seconds = s->samplerate ? samples / s->samplerate : 0;
    uint16_t batch = PWM_ANALOG_BATCH_MIN;
    if (seconds >= 60)
        batch = PWM_ANALOG_BATCH_MAX;
    else if (seconds >= 10)
        batch = PWM_ANALOG_BATCH_LARGE;
    else if (seconds >= 1)
        batch = PWM_ANALOG_BATCH_MEDIUM;

    if (batch > s->analog_batch)
        s->analog_batch = batch;
}

static void pwm_flush_analog_channel(struct srd_decoder_inst *di,
                                     pwm_waveform_state *s, int channel)
{
    const uint16_t count = s->analog_count[channel];
    if (count == 0)
        return;

    c_put_analog_timed(di, s->out_analog, channel, NUM_ANALOG_CHANNELS,
                       s->analog_values[channel],
                       s->analog_starts[channel], s->analog_ends[channel],
                       count, 1.0);
    s->analog_count[channel] = 0;
}

static void pwm_flush_analog_all(struct srd_decoder_inst *di,
                                 pwm_waveform_state *s)
{
    for (int channel = 0; channel < NUM_ANALOG_CHANNELS; ++channel)
        pwm_flush_analog_channel(di, s, channel);
}

static inline void pwm_buffer_analog(struct srd_decoder_inst *di,
                                     pwm_waveform_state *s, int channel,
                                     uint64_t start_sample,
                                     uint64_t end_sample, float value)
{
    if (channel < 0 || channel >= NUM_ANALOG_CHANNELS ||
        !s->analog_enabled[channel])
        return;

    uint16_t count = s->analog_count[channel];
    if (count >= PWM_ANALOG_BATCH_MAX) {
        pwm_flush_analog_channel(di, s, channel);
        count = 0;
    }

    s->analog_values[channel][count] = value;
    s->analog_starts[channel][count] = start_sample;
    s->analog_ends[channel][count] = end_sample;
    s->analog_count[channel] = ++count;

    if (count >= s->analog_batch)
        pwm_flush_analog_channel(di, s, channel);
}

static enum pwm_application_mode parse_application_mode(const char *mode)
{
    if (!mode)
        return PWM_MODE_GENERIC;
    if (strcmp(mode, "Power-supply control 0-100%") == 0)
        return PWM_MODE_POWER_SUPPLY;
    if (strcmp(mode, "Class-D centered -100..+100%") == 0)
        return PWM_MODE_CLASSD_AD;
    if (strcmp(mode, "Class-D differential (PWM+ - PWM-)") == 0)
        return PWM_MODE_CLASSD_BD;
    return PWM_MODE_GENERIC;
}

static enum pwm_filter_type parse_filter_type(const char *filter)
{
    if (!filter)
        return PWM_FILTER_NONE;
    if (strcmp(filter, "Moving average") == 0)
        return PWM_FILTER_MOVING_AVERAGE;
    if (strcmp(filter, "Exponential smoothing") == 0)
        return PWM_FILTER_EXPONENTIAL;
    if (strcmp(filter, "RC low-pass") == 0)
        return PWM_FILTER_RC_LOWPASS;
    return PWM_FILTER_NONE;
}

static void pwm_waveform_start(struct srd_decoder_inst *di)
{
    pwm_waveform_state *s =
        (pwm_waveform_state *)c_decoder_get_private(di);

    s->out_ann = c_reg_out(di, SRD_OUTPUT_ANN, "pwm_waveform");
    s->out_binary = c_reg_out(di, SRD_OUTPUT_BINARY, "pwm_waveform");
    s->out_analog = c_reg_out(di, SRD_OUTPUT_ANALOG, "pwm_waveform_analog");

    s->out_average_output =
        c_reg_meta(di, SRD_OUTPUT_META, "pwm_waveform", "float",
                   "Average reconstructed output",
                   "Average reconstructed output in percent/full-scale");
    s->out_min_output =
        c_reg_meta(di, SRD_OUTPUT_META, "pwm_waveform", "float",
                   "Minimum reconstructed output",
                   "Minimum reconstructed output in percent/full-scale");
    s->out_max_output =
        c_reg_meta(di, SRD_OUTPUT_META, "pwm_waveform", "float",
                   "Maximum reconstructed output",
                   "Maximum reconstructed output in percent/full-scale");
    s->out_average_duty_a =
        c_reg_meta(di, SRD_OUTPUT_META, "pwm_waveform", "float",
                   "Average duty A", "Average PWM A duty cycle in percent");
    s->out_average_duty_b =
        c_reg_meta(di, SRD_OUTPUT_META, "pwm_waveform", "float",
                   "Average duty B", "Average PWM B duty cycle in percent");
    s->out_average_frequency =
        c_reg_meta(di, SRD_OUTPUT_META, "pwm_waveform", "float",
                   "Average carrier frequency",
                   "Average PWM A carrier frequency in hertz");

    s->samplerate = c_samplerate(di);
    pwm_update_adaptive_batches(s);

    const char *polarity_a = c_opt_str(di, "polarity", "active-high");
    const char *polarity_b = c_opt_str(di, "polarity_b", "active-high");
    s->active_level_a = strcmp(polarity_a, "active-low") == 0 ? 0 : 1;
    s->active_level_b = strcmp(polarity_b, "active-low") == 0 ? 0 : 1;

    const char *output = c_opt_str(di, "output", "Waveform");
    s->emit_analog = strcmp(output, "Annotations only") != 0 &&
                     strcmp(output, "Parameters only") != 0;
    s->emit_annotations = strcmp(output, "Waveform") != 0;
    s->emit_parameters = strcmp(output, "Waveform + parameters") == 0 ||
                         strcmp(output, "Parameters only") == 0;

    /* Do not decode/store analog rows the user switched off.  The old path
     * still generated hidden Duty-A/B/Common traces, wasting callbacks, locks
     * and vector memory. Decoder option changes trigger a re-decode, so an
     * enabled row is populated on demand. */
    for (int channel = 0; channel < NUM_ANALOG_CHANNELS; ++channel) {
        char key[24];
        snprintf(key, sizeof(key), "ch%d_enable", channel);
        s->analog_enabled[channel] =
            c_opt_int(di, key, channel == ANALOG_MAIN ? 1 : 0) != 0;
    }

    s->application_mode = parse_application_mode(
        c_opt_str(di, "analog_mode", "Duty 0-100%"));
    s->filter_type = parse_filter_type(c_opt_str(di, "filter", "None"));

    s->filter_cycles = (int)c_opt_int(di, "filter_cycles", 8);
    if (s->filter_cycles < 1)
        s->filter_cycles = 1;
    if (s->filter_cycles > PWM_FILTER_MAX)
        s->filter_cycles = PWM_FILTER_MAX;

    s->filter_cutoff_hz = c_opt_dbl(di, "filter_cutoff_hz", 20000.0);
    if (!isfinite(s->filter_cutoff_hz) || s->filter_cutoff_hz <= 0.0)
        s->filter_cutoff_hz = 20000.0;
    else if (s->filter_cutoff_hz < PWM_FILTER_CUTOFF_MIN_HZ)
        s->filter_cutoff_hz = PWM_FILTER_CUTOFF_MIN_HZ;
    else if (s->filter_cutoff_hz > PWM_FILTER_CUTOFF_MAX_HZ)
        s->filter_cutoff_hz = PWM_FILTER_CUTOFF_MAX_HZ;

    s->duty_decimals = clamp_decimals(
        (int)c_opt_int(di, "duty_decimals", PWM_DEFAULT_DUTY_DECIMALS),
        PWM_DEFAULT_DUTY_DECIMALS);
    s->time_decimals = clamp_decimals(
        (int)c_opt_int(di, "time_decimals", PWM_DEFAULT_TIME_DECIMALS),
        PWM_DEFAULT_TIME_DECIMALS);
    s->freq_decimals = clamp_decimals(
        (int)c_opt_int(di, "freq_decimals", PWM_DEFAULT_FREQ_DECIMALS),
        PWM_DEFAULT_FREQ_DECIMALS);

    s->has_b = di->dec_num_channels > 1 && di->dec_channelmap &&
               di->dec_channelmap[1] >= 0;
}

static void format_scaled(double value, const char *unit, int decimals,
                          char *out, size_t out_size)
{
    const double abs_value = fabs(value);
    const char *prefix = "";
    double scaled = value;

    if (abs_value == 0.0) {
        scaled = 0.0;
    } else if (abs_value >= 1e9) {
        prefix = "G";
        scaled = value / 1e9;
    } else if (abs_value >= 1e6) {
        prefix = "M";
        scaled = value / 1e6;
    } else if (abs_value >= 1e3) {
        prefix = "k";
        scaled = value / 1e3;
    } else if (abs_value < 1e-12) {
        prefix = "f";
        scaled = value * 1e15;
    } else if (abs_value < 1e-9) {
        prefix = "p";
        scaled = value * 1e12;
    } else if (abs_value < 1e-6) {
        prefix = "n";
        scaled = value * 1e9;
    } else if (abs_value < 1e-3) {
        prefix = "\xce\xbc";
        scaled = value * 1e6;
    } else if (abs_value < 1.0) {
        prefix = "m";
        scaled = value * 1e3;
    }

    snprintf(out, out_size, "%.*f %s%s", decimals, scaled, prefix, unit);
}

static double pwm_filter_value(pwm_waveform_state *s, double value,
                               double period_seconds)
{
    if (s->filter_type == PWM_FILTER_NONE)
        return value;

    if (s->filter_type == PWM_FILTER_RC_LOWPASS) {
        double alpha = 1.0;
        if (period_seconds > 0.0 && s->filter_cutoff_hz > 0.0)
            alpha = 1.0 - exp(-PWM_TWO_PI * s->filter_cutoff_hz * period_seconds);
        alpha = clamp_double(alpha, 0.0, 1.0);
        if (!s->filter_valid) {
            s->filtered_value = value;
            s->filter_valid = 1;
        } else {
            s->filtered_value += alpha * (value - s->filtered_value);
        }
        return s->filtered_value;
    }

    if (s->filter_type == PWM_FILTER_EXPONENTIAL) {
        const double alpha = 2.0 / ((double)s->filter_cycles + 1.0);
        if (!s->filter_valid) {
            s->filtered_value = value;
            s->filter_valid = 1;
        } else {
            s->filtered_value += alpha * (value - s->filtered_value);
        }
        return s->filtered_value;
    }

    if (s->filter_cycles <= 1)
        return value;

    if (s->moving_count < s->filter_cycles) {
        s->moving_values[s->moving_pos] = value;
        s->moving_sum += value;
        s->moving_count++;
        s->moving_pos = (s->moving_pos + 1) % s->filter_cycles;
    } else {
        s->moving_sum -= s->moving_values[s->moving_pos];
        s->moving_values[s->moving_pos] = value;
        s->moving_sum += value;
        s->moving_pos = (s->moving_pos + 1) % s->filter_cycles;
    }

    return s->moving_sum / (double)s->moving_count;
}

static double reconstructed_value(pwm_waveform_state *s,
                                  double duty_a, double duty_b,
                                  int *requires_b)
{
    *requires_b = 0;
    switch (s->application_mode) {
    case PWM_MODE_POWER_SUPPLY:
    case PWM_MODE_GENERIC:
        return duty_a;
    case PWM_MODE_CLASSD_AD:
        return clamp_double(2.0 * duty_a - 1.0, -1.0, 1.0);
    case PWM_MODE_CLASSD_BD:
        *requires_b = 1;
        if (!s->has_b)
            return 0.0;
        return clamp_double(duty_a - duty_b, -1.0, 1.0);
    }
    return duty_a;
}

static double main_to_normalized(pwm_waveform_state *s, double main_value)
{
    if (s->application_mode == PWM_MODE_GENERIC ||
        s->application_mode == PWM_MODE_POWER_SUPPLY)
        return clamp_double(2.0 * main_value - 1.0, -1.0, 1.0);
    return clamp_double(main_value, -1.0, 1.0);
}

static double main_to_percent(pwm_waveform_state *s, double main_value)
{
    (void)s;
    return main_value * 100.0;
}

static void pwm_output_meta(struct srd_decoder_inst *di,
                            pwm_waveform_state *s, uint64_t cycle_end,
                            double output_percent)
{
    if (!s->have_first_cycle || s->num_cycles <= 0)
        return;

    c_put_meta_dbl(di, s->first_cycle_start, cycle_end,
                   s->out_average_output,
                   s->output_sum_percent / (double)s->num_cycles);
    c_put_meta_dbl(di, s->first_cycle_start, cycle_end,
                   s->out_min_output, s->output_min_percent);
    c_put_meta_dbl(di, s->first_cycle_start, cycle_end,
                   s->out_max_output, s->output_max_percent);
    c_put_meta_dbl(di, s->first_cycle_start, cycle_end,
                   s->out_average_duty_a,
                   s->duty_a_sum_percent / (double)s->num_cycles);
    if (s->has_b)
        c_put_meta_dbl(di, s->first_cycle_start, cycle_end,
                       s->out_average_duty_b,
                       s->duty_b_sum_percent / (double)s->num_cycles);
    c_put_meta_dbl(di, s->first_cycle_start, cycle_end,
                   s->out_average_frequency,
                   s->frequency_sum_hz / (double)s->num_cycles);
    (void)output_percent;
}

static void put_percent_annotation(struct srd_decoder_inst *di,
                                   pwm_waveform_state *s,
                                   uint64_t start, uint64_t end,
                                   int ann_class, const char *prefix,
                                   double value)
{
    char text[96];
    snprintf(text, sizeof(text), "%s%.*f%%", prefix,
             s->duty_decimals, value);
    c_put(di, start, end, s->out_ann, ann_class, text);
}

static void pwm_output_cycle(struct srd_decoder_inst *di,
                             pwm_waveform_state *s, uint64_t cycle_end)
{
    if (!s->cycle_started || cycle_end <= s->cycle_start)
        return;

    const uint64_t period_samples = cycle_end - s->cycle_start;
    if (period_samples == 0)
        return;

    const double duty_a = clamp_double(
        (double)s->count_a_active / (double)period_samples, 0.0, 1.0);
    const double duty_b = s->has_b
                              ? clamp_double((double)s->count_b_active /
                                                 (double)period_samples,
                                             0.0, 1.0)
                              : 0.0;
    const double common = s->has_b ? 0.5 * (duty_a + duty_b) : duty_a;
    const double differential = s->has_b ? duty_a - duty_b : 0.0;

    const double period_seconds = s->samplerate
                                      ? (double)period_samples /
                                            (double)s->samplerate
                                      : 0.0;
    const double frequency_hz = period_seconds > 0.0
                                    ? 1.0 / period_seconds
                                    : 0.0;

    int requires_b = 0;
    const double raw_main = reconstructed_value(s, duty_a, duty_b, &requires_b);
    const double filtered_main = pwm_filter_value(s, raw_main, period_seconds);
    const double output_percent = main_to_percent(s, filtered_main);

    if (s->emit_annotations) {
        const char *prefix = "OUT=";
        if (s->application_mode == PWM_MODE_POWER_SUPPLY)
            prefix = "CTRL=";
        else if (s->application_mode == PWM_MODE_CLASSD_AD)
            prefix = "AD=";
        else if (s->application_mode == PWM_MODE_CLASSD_BD)
            prefix = "BD=";

        put_percent_annotation(di, s, s->cycle_start, cycle_end,
                               ANN_OUTPUT, prefix, output_percent);
        put_percent_annotation(di, s, s->cycle_start, cycle_end,
                               ANN_DUTY_A, "A=", duty_a * 100.0);
        if (s->has_b)
            put_percent_annotation(di, s, s->cycle_start, cycle_end,
                                   ANN_DUTY_B, "B=", duty_b * 100.0);

        if (requires_b && !s->has_b && !s->warned_missing_b) {
            c_put(di, s->cycle_start, cycle_end, s->out_ann, ANN_WARNING,
                  "Class-D differential mode requires PWM B / PWM-");
            s->warned_missing_b = 1;
        }

        if (s->emit_parameters) {
            char text[96];
            format_scaled(period_seconds, "s", s->time_decimals,
                          text, sizeof(text));
            c_put(di, s->cycle_start, cycle_end, s->out_ann,
                  ANN_PERIOD, text);

            format_scaled(frequency_hz, "Hz", s->freq_decimals,
                          text, sizeof(text));
            c_put(di, s->cycle_start, cycle_end, s->out_ann,
                  ANN_FREQUENCY, text);

            const double active_a_s = s->samplerate
                                          ? (double)s->count_a_active /
                                                (double)s->samplerate
                                          : 0.0;
            const double inactive_a_s = period_seconds - active_a_s;
            format_scaled(active_a_s, "s", s->time_decimals,
                          text, sizeof(text));
            c_put(di, s->cycle_start, cycle_end, s->out_ann,
                  ANN_ACTIVE_A, text);
            format_scaled(inactive_a_s, "s", s->time_decimals,
                          text, sizeof(text));
            c_put(di, s->cycle_start, cycle_end, s->out_ann,
                  ANN_INACTIVE_A, text);

            if (s->has_b) {
                const double active_b_s = s->samplerate
                                              ? (double)s->count_b_active /
                                                    (double)s->samplerate
                                              : 0.0;
                const double inactive_b_s = period_seconds - active_b_s;
                format_scaled(active_b_s, "s", s->time_decimals,
                              text, sizeof(text));
                c_put(di, s->cycle_start, cycle_end, s->out_ann,
                      ANN_ACTIVE_B, text);
                format_scaled(inactive_b_s, "s", s->time_decimals,
                              text, sizeof(text));
                c_put(di, s->cycle_start, cycle_end, s->out_ann,
                      ANN_INACTIVE_B, text);

                put_percent_annotation(di, s, s->cycle_start, cycle_end,
                                       ANN_DIFFERENTIAL, "A-B=",
                                       differential * 100.0);
                put_percent_annotation(di, s, s->cycle_start, cycle_end,
                                       ANN_COMMON, "CM=", common * 100.0);

                const double both_inactive_s = s->samplerate
                    ? (double)s->count_both_inactive / (double)s->samplerate
                    : 0.0;
                const double both_active_s = s->samplerate
                    ? (double)s->count_both_active / (double)s->samplerate
                    : 0.0;
                format_scaled(both_inactive_s, "s", s->time_decimals,
                              text, sizeof(text));
                c_put(di, s->cycle_start, cycle_end, s->out_ann,
                      ANN_BOTH_INACTIVE, text);
                format_scaled(both_active_s, "s", s->time_decimals,
                              text, sizeof(text));
                c_put(di, s->cycle_start, cycle_end, s->out_ann,
                      ANN_BOTH_ACTIVE, text);
            }
        }
    }

    /* Waveform-only is the fast default.  Binary output is not consumed by
     * PXView's decoder stack and used to cost one callback lookup per PWM
     * carrier cycle. Keep it for modes that explicitly request decoded text
     * or parameters, but skip it for pure waveform rendering. */
    if (s->emit_annotations || s->emit_parameters) {
        uint8_t bytes[2];
        bytes[0] = (uint8_t)clamp_double(duty_a * 255.0, 0.0, 255.0);
        bytes[1] = (uint8_t)clamp_double(duty_b * 255.0, 0.0, 255.0);
        c_put_bin(di, s->cycle_start, cycle_end, s->out_binary, 0,
                  s->has_b ? 2 : 1, bytes);
    }

    if (s->emit_analog) {
        pwm_buffer_analog(di, s, ANALOG_MAIN, s->cycle_start, cycle_end,
                          (float)main_to_normalized(s, filtered_main));
        pwm_buffer_analog(di, s, ANALOG_DUTY_A, s->cycle_start, cycle_end,
                          (float)(2.0 * duty_a - 1.0));

        if (s->has_b) {
            pwm_buffer_analog(di, s, ANALOG_DUTY_B, s->cycle_start, cycle_end,
                              (float)(2.0 * duty_b - 1.0));
            pwm_buffer_analog(di, s, ANALOG_COMMON, s->cycle_start, cycle_end,
                              (float)(2.0 * common - 1.0));
        }
    }

    if (!s->have_first_cycle) {
        s->first_cycle_start = s->cycle_start;
        s->have_first_cycle = 1;
        s->output_min_percent = output_percent;
        s->output_max_percent = output_percent;
    } else {
        if (output_percent < s->output_min_percent)
            s->output_min_percent = output_percent;
        if (output_percent > s->output_max_percent)
            s->output_max_percent = output_percent;
    }

    s->num_cycles++;
    s->output_sum_percent += output_percent;
    s->duty_a_sum_percent += duty_a * 100.0;
    s->duty_b_sum_percent += duty_b * 100.0;
    s->frequency_sum_hz += frequency_hz;
    if (s->emit_parameters)
        pwm_output_meta(di, s, cycle_end, output_percent);
}

static void reset_cycle_counts(pwm_waveform_state *s)
{
    s->count_a_active = 0;
    s->count_b_active = 0;
    s->count_both_inactive = 0;
    s->count_both_active = 0;
}

static inline unsigned pwm_ctz64(uint64_t value)
{
#if defined(_MSC_VER) && defined(_M_X64)
    unsigned long index = 0;
    _BitScanForward64(&index, value);
    return (unsigned)index;
#else
    return (unsigned)__builtin_ctzll(value);
#endif
}

static inline unsigned pwm_popcount64(uint64_t value)
{
#if defined(_MSC_VER) && defined(_M_X64)
    return (unsigned)__popcnt64(value);
#else
    return (unsigned)__builtin_popcountll(value);
#endif
}

static inline uint64_t pwm_low_mask64(unsigned bits)
{
    return bits >= 64 ? UINT64_MAX
                      : (bits == 0 ? 0ULL : ((1ULL << bits) - 1ULL));
}

static inline int pwm_packed_bit(const uint8_t *buffer, uint8_t const_value,
                                 uint64_t bit_index)
{
    if (!buffer)
        return const_value ? 1 : 0;
    return (buffer[bit_index >> 3] >> (bit_index & 7u)) & 1u;
}

static inline uint64_t pwm_load_packed_word(const uint8_t *buffer,
                                            uint8_t const_value,
                                            uint64_t bit_index,
                                            unsigned width)
{
    const uint64_t valid = pwm_low_mask64(width);
    if (!buffer)
        return const_value ? valid : 0ULL;

    const uint8_t *src = buffer + (bit_index >> 3);
    uint64_t word = 0;
    const size_t bytes = (size_t)((width + 7u) >> 3);
    memcpy(&word, src, bytes);
    return word & valid;
}

static inline void pwm_process_one(struct srd_decoder_inst *di,
                                   pwm_waveform_state *s,
                                   int raw_a, int raw_b,
                                   uint64_t sample_abs)
{
    const int active_a = raw_a == s->active_level_a;
    const int active_b = s->has_b ? (raw_b == s->active_level_b) : 0;

    if (!s->have_previous_a) {
        s->previous_active_a = active_a;
        s->have_previous_a = 1;
    }

    const int active_edge_a = active_a && !s->previous_active_a;
    if (active_edge_a) {
        if (s->cycle_started)
            pwm_output_cycle(di, s, sample_abs);
        s->cycle_started = 1;
        s->cycle_start = sample_abs;
        reset_cycle_counts(s);
    }

    if (s->cycle_started) {
        if (active_a)
            s->count_a_active++;
        if (s->has_b && active_b)
            s->count_b_active++;
        if (s->has_b) {
            if (!active_a && !active_b)
                s->count_both_inactive++;
            if (active_a && active_b)
                s->count_both_active++;
        }
    }
    s->previous_active_a = active_a;
}

static inline void pwm_accumulate_mask_range64(pwm_waveform_state *s,
                                               uint64_t active_a,
                                               uint64_t active_b,
                                               unsigned begin, unsigned end)
{
    if (!s->cycle_started || end <= begin)
        return;

    const uint64_t range = pwm_low_mask64(end) & ~pwm_low_mask64(begin);
    s->count_a_active += (uint64_t)pwm_popcount64(active_a & range);
    if (s->has_b) {
        s->count_b_active += (uint64_t)pwm_popcount64(active_b & range);
        s->count_both_active +=
            (uint64_t)pwm_popcount64(active_a & active_b & range);
        s->count_both_inactive +=
            (uint64_t)pwm_popcount64((~(active_a | active_b)) & range);
    }
}

static inline void pwm_process_masks64(struct srd_decoder_inst *di,
                                       pwm_waveform_state *s,
                                       uint64_t active_a,
                                       uint64_t active_b,
                                       unsigned width,
                                       uint64_t base_abs)
{
    const uint64_t valid = pwm_low_mask64(width);
    active_a &= valid;
    active_b &= valid;

    int prev_active = s->previous_active_a;
    if (!s->have_previous_a) {
        prev_active = (active_a & 1ULL) != 0;
        s->previous_active_a = prev_active;
        s->have_previous_a = 1;
    }

    uint64_t edges = active_a & ~(active_a << 1);
    edges &= ~1ULL;
    if ((active_a & 1ULL) && !prev_active)
        edges |= 1ULL;
    edges &= valid;

    unsigned segment_begin = 0;
    while (edges) {
        const unsigned bit = pwm_ctz64(edges);
        pwm_accumulate_mask_range64(s, active_a, active_b,
                                    segment_begin, bit);

        const uint64_t sample_abs = base_abs + bit;
        if (s->cycle_started)
            pwm_output_cycle(di, s, sample_abs);

        s->cycle_started = 1;
        s->cycle_start = sample_abs;
        reset_cycle_counts(s);
        segment_begin = bit;
        edges &= edges - 1ULL;
    }

    pwm_accumulate_mask_range64(s, active_a, active_b,
                                segment_begin, width);
    s->previous_active_a = (int)((active_a >> (width - 1u)) & 1ULL);
}

static void pwm_process_packed(struct srd_decoder_inst *di,
                               pwm_waveform_state *s,
                               const uint8_t *data_a, uint8_t const_a,
                               const uint8_t *data_b, uint8_t const_b,
                               uint64_t count, uint64_t base_abs,
                               uint8_t bit_offset)
{
    uint64_t pos = 0;

    /* Consume at most seven samples until the borrowed packed view reaches a
     * byte boundary.  All large work below stays 64-bit packed. */
    if (bit_offset && count) {
        uint64_t head = 8u - bit_offset;
        if (head > count)
            head = count;
        for (uint64_t i = 0; i < head; ++i) {
            const uint64_t bi = (uint64_t)bit_offset + i;
            pwm_process_one(di, s,
                            pwm_packed_bit(data_a, const_a, bi),
                            s->has_b ? pwm_packed_bit(data_b, const_b, bi) : 0,
                            base_abs + i);
        }
        pos += head;
    }

    while (count - pos >= 64) {
        const uint64_t packed_bit = (uint64_t)bit_offset + pos;
        const uint64_t raw_a = pwm_load_packed_word(
            data_a, const_a, packed_bit, 64);
        const uint64_t raw_b = s->has_b
            ? pwm_load_packed_word(data_b, const_b, packed_bit, 64)
            : 0ULL;
        const uint64_t active_a = s->active_level_a ? raw_a : ~raw_a;
        const uint64_t active_b = s->has_b
            ? (s->active_level_b ? raw_b : ~raw_b)
            : 0ULL;
        pwm_process_masks64(di, s, active_a, active_b, 64,
                            base_abs + pos);
        pos += 64;
    }

    if (pos < count) {
        const unsigned width = (unsigned)(count - pos);
        const uint64_t packed_bit = (uint64_t)bit_offset + pos;
        const uint64_t valid = pwm_low_mask64(width);
        const uint64_t raw_a = pwm_load_packed_word(
            data_a, const_a, packed_bit, width);
        const uint64_t raw_b = s->has_b
            ? pwm_load_packed_word(data_b, const_b, packed_bit, width)
            : 0ULL;
        const uint64_t active_a = s->active_level_a
            ? raw_a : ((~raw_a) & valid);
        const uint64_t active_b = s->has_b
            ? (s->active_level_b ? raw_b : ((~raw_b) & valid))
            : 0ULL;
        pwm_process_masks64(di, s, active_a, active_b, width,
                            base_abs + pos);
    }
}

static void pwm_waveform_decode(struct srd_decoder_inst *di)
{
    pwm_waveform_state *s =
        (pwm_waveform_state *)c_decoder_get_private(di);

    if (!s->samplerate)
        s->samplerate = c_samplerate(di);

    while (1) {
        const int channels[2] = {0, 1};
        const uint8_t *buffers[2] = {NULL, NULL};
        uint8_t const_values[2] = {0, 0};
        uint64_t base_abs = 0;
        uint8_t bit_offset = 0;
        const int num_channels = s->has_b ? 2 : 1;

        const uint64_t count = c_fetch_packed_multi(
            di, channels, buffers, const_values, num_channels,
            PWM_FETCH_BLOCK, &base_abs, &bit_offset);
        if (count == 0)
            break;

        pwm_process_packed(di, s,
                           buffers[0], const_values[0],
                           s->has_b ? buffers[1] : NULL,
                           s->has_b ? const_values[1] : 0,
                           count, base_abs, bit_offset);

        if (c_consume_samples(di, count) != SRD_OK)
            break;

        s->processed_samples += count;
        pwm_update_adaptive_batches(s);
    }

    /* Preserve real-time responsiveness: flush any partial batch after the
     * current v4 input stream terminates. */
    pwm_flush_analog_all(di, s);
}

static void pwm_waveform_destroy(struct srd_decoder_inst *di)
{
    void *private_data = c_decoder_get_private(di);
    if (private_data) {
        g_free(private_data);
        c_decoder_set_private(di, NULL);
    }
}

struct srd_c_decoder pwm_waveform_c_decoder = {
    .id = "pwm_waveform_c",
    .name = "PWM waveform (Fast)",
    .longname = "Multi-mode PWM analog reconstruction (Fast/C)",
    .desc = "Generic, power-supply and Class-D PWM analog reconstruction with filtering and measurements",
    .license = "gplv2+",
    .channels = pwm_waveform_channels,
    .num_channels = 1,
    .optional_channels = pwm_waveform_optional_channels,
    .num_optional_channels = 1,
    .options = pwm_waveform_options,
    .num_options = NUM_OPTIONS,
    .num_annotations = NUM_ANN,
    .ann_labels = pwm_waveform_ann_labels,
    .num_annotation_rows = NUM_ANN,
    .annotation_rows = pwm_waveform_ann_rows,
    .inputs = pwm_waveform_inputs,
    .num_inputs = 1,
    .outputs = pwm_waveform_outputs,
    .num_outputs = 0,
    .binary = pwm_waveform_binary,
    .num_binary = 1,
    .tags = pwm_waveform_tags,
    .num_tags = 2,
    .reset = pwm_waveform_reset,
    .start = pwm_waveform_start,
    .decode = pwm_waveform_decode,
    .destroy = pwm_waveform_destroy,
    .state_size = 0,
    .metadata = pwm_waveform_metadata,
};

static GSList *string_values(const char *const *items, size_t count)
{
    GSList *values = NULL;
    for (size_t i = 0; i < count; ++i)
        values = g_slist_append(values, g_variant_new_string(items[i]));
    return values;
}

static GSList *integer_values(const int64_t *items, size_t count)
{
    GSList *values = NULL;
    for (size_t i = 0; i < count; ++i)
        values = g_slist_append(values, g_variant_new_int64(items[i]));
    return values;
}

static void setup_channel_display_options(int base, int channel,
                                          double vpos, int enabled,
                                          const char *unit)
{
    char idn[96];

    snprintf(idn, sizeof(idn), "dec_pwm_waveform_opt_ch%d_vzoom", channel);
    pwm_waveform_options[base + 0].idn = g_strdup(idn);
    pwm_waveform_options[base + 0].def = g_variant_new_double(1.0);

    snprintf(idn, sizeof(idn), "dec_pwm_waveform_opt_ch%d_vpos", channel);
    pwm_waveform_options[base + 1].idn = g_strdup(idn);
    pwm_waveform_options[base + 1].def = g_variant_new_double(vpos);

    snprintf(idn, sizeof(idn), "dec_pwm_waveform_opt_ch%d_enable", channel);
    pwm_waveform_options[base + 2].idn = g_strdup(idn);
    pwm_waveform_options[base + 2].def = g_variant_new_int64(enabled);

    snprintf(idn, sizeof(idn), "dec_pwm_waveform_opt_ch%d_range_mode", channel);
    pwm_waveform_options[base + 3].idn = g_strdup(idn);
    pwm_waveform_options[base + 3].def = g_variant_new_string("unipolar");
    {
        static const char *const range_values[] = {
            "unipolar", "bipolar", "custom"
        };
        pwm_waveform_options[base + 3].values =
            string_values(range_values, G_N_ELEMENTS(range_values));
    }

    snprintf(idn, sizeof(idn), "dec_pwm_waveform_opt_ch%d_eng_min", channel);
    pwm_waveform_options[base + 4].idn = g_strdup(idn);
    pwm_waveform_options[base + 4].def = g_variant_new_double(0.0);

    snprintf(idn, sizeof(idn), "dec_pwm_waveform_opt_ch%d_eng_max", channel);
    pwm_waveform_options[base + 5].idn = g_strdup(idn);
    pwm_waveform_options[base + 5].def = g_variant_new_double(100.0);

    snprintf(idn, sizeof(idn), "dec_pwm_waveform_opt_ch%d_unit", channel);
    pwm_waveform_options[base + 6].idn = g_strdup(idn);
    pwm_waveform_options[base + 6].def = g_variant_new_string(unit);
}

SRD_C_DECODER_EXPORT struct srd_c_decoder *srd_c_decoder_entry(void)
{
    static const char *const polarity_values[] = {
        "active-low", "active-high"
    };
    static const char *const output_values[] = {
        "Waveform",
        "Waveform + annotations",
        "Waveform + parameters",
        "Parameters only",
        "Annotations only"
    };
    static const char *const mode_values[] = {
        "Duty 0-100%",
        "Power-supply control 0-100%",
        "Class-D centered -100..+100%",
        "Class-D differential (PWM+ - PWM-)"
    };
    static const char *const filter_values[] = {
        "None",
        "Moving average",
        "Exponential smoothing",
        "RC low-pass"
    };
    static const int64_t cycle_values[] = {1, 2, 4, 8, 16, 32, 64, 128};
    static const int64_t decimal_values[] = {0, 1, 2, 3, 4, 5, 6, 8};
    static const char *const display_trigger_mode_values[] = {
        "auto", "normal"
    };
    static const int64_t display_trigger_channel_values[] = {0, 1, 2, 3};
    static const char *const display_trigger_edge_values[] = {
        "rising", "falling", "either"
    };

    pwm_waveform_options[OPT_POLARITY_A].idn =
        "dec_pwm_waveform_opt_polarity_a";
    pwm_waveform_options[OPT_POLARITY_A].def =
        g_variant_new_string("active-high");
    pwm_waveform_options[OPT_POLARITY_A].values =
        string_values(polarity_values, G_N_ELEMENTS(polarity_values));

    pwm_waveform_options[OPT_POLARITY_B].idn =
        "dec_pwm_waveform_opt_polarity_b";
    pwm_waveform_options[OPT_POLARITY_B].def =
        g_variant_new_string("active-high");
    pwm_waveform_options[OPT_POLARITY_B].values =
        string_values(polarity_values, G_N_ELEMENTS(polarity_values));

    pwm_waveform_options[OPT_OUTPUT].idn =
        "dec_pwm_waveform_opt_output";
    pwm_waveform_options[OPT_OUTPUT].def = g_variant_new_string("Waveform");
    pwm_waveform_options[OPT_OUTPUT].values =
        string_values(output_values, G_N_ELEMENTS(output_values));

    pwm_waveform_options[OPT_ANALOG_MODE].idn =
        "dec_pwm_waveform_opt_analog_mode";
    pwm_waveform_options[OPT_ANALOG_MODE].def =
        g_variant_new_string("Duty 0-100%");
    pwm_waveform_options[OPT_ANALOG_MODE].values =
        string_values(mode_values, G_N_ELEMENTS(mode_values));

    pwm_waveform_options[OPT_FILTER].idn =
        "dec_pwm_waveform_opt_filter";
    pwm_waveform_options[OPT_FILTER].def = g_variant_new_string("None");
    pwm_waveform_options[OPT_FILTER].values =
        string_values(filter_values, G_N_ELEMENTS(filter_values));

    pwm_waveform_options[OPT_FILTER_CYCLES].idn =
        "dec_pwm_waveform_opt_filter_cycles";
    pwm_waveform_options[OPT_FILTER_CYCLES].def = g_variant_new_int64(8);
    pwm_waveform_options[OPT_FILTER_CYCLES].values =
        integer_values(cycle_values, G_N_ELEMENTS(cycle_values));

    pwm_waveform_options[OPT_FILTER_CUTOFF_HZ].idn =
        "dec_pwm_waveform_opt_filter_cutoff_hz";
    pwm_waveform_options[OPT_FILTER_CUTOFF_HZ].def =
        g_variant_new_double(20000.0);

    pwm_waveform_options[OPT_DUTY_DECIMALS].idn =
        "dec_pwm_waveform_opt_duty_decimals";
    pwm_waveform_options[OPT_DUTY_DECIMALS].def =
        g_variant_new_int64(PWM_DEFAULT_DUTY_DECIMALS);
    pwm_waveform_options[OPT_DUTY_DECIMALS].values =
        integer_values(decimal_values, G_N_ELEMENTS(decimal_values));

    pwm_waveform_options[OPT_TIME_DECIMALS].idn =
        "dec_pwm_waveform_opt_time_decimals";
    pwm_waveform_options[OPT_TIME_DECIMALS].def =
        g_variant_new_int64(PWM_DEFAULT_TIME_DECIMALS);
    pwm_waveform_options[OPT_TIME_DECIMALS].values =
        integer_values(decimal_values, G_N_ELEMENTS(decimal_values));

    pwm_waveform_options[OPT_FREQ_DECIMALS].idn =
        "dec_pwm_waveform_opt_freq_decimals";
    pwm_waveform_options[OPT_FREQ_DECIMALS].def =
        g_variant_new_int64(PWM_DEFAULT_FREQ_DECIMALS);
    pwm_waveform_options[OPT_FREQ_DECIMALS].values =
        integer_values(decimal_values, G_N_ELEMENTS(decimal_values));

    setup_channel_display_options(OPT_CH0_VZOOM, 0, 0.0, 1, "%");
    setup_channel_display_options(OPT_CH1_VZOOM, 1, -2.0, 0, "%");
    setup_channel_display_options(OPT_CH2_VZOOM, 2, -4.0, 0, "%");
    setup_channel_display_options(OPT_CH3_VZOOM, 3, -6.0, 0, "%");

    pwm_waveform_options[OPT_REALTIME_DECODE].idn =
        "dec_pwm_waveform_opt_realtime_decode";
    pwm_waveform_options[OPT_REALTIME_DECODE].def =
        g_variant_new_boolean(FALSE);

    pwm_waveform_options[OPT_DISPLAY_TRIGGER_ENABLE].idn =
        "dec_pwm_waveform_opt_display_trigger_enable";
    pwm_waveform_options[OPT_DISPLAY_TRIGGER_ENABLE].def =
        g_variant_new_boolean(FALSE);

    pwm_waveform_options[OPT_DISPLAY_TRIGGER_MODE].idn =
        "dec_pwm_waveform_opt_display_trigger_mode";
    pwm_waveform_options[OPT_DISPLAY_TRIGGER_MODE].def =
        g_variant_new_string("auto");
    pwm_waveform_options[OPT_DISPLAY_TRIGGER_MODE].values =
        string_values(display_trigger_mode_values,
                      G_N_ELEMENTS(display_trigger_mode_values));

    pwm_waveform_options[OPT_DISPLAY_TRIGGER_CHANNEL].idn =
        "dec_pwm_waveform_opt_display_trigger_channel";
    pwm_waveform_options[OPT_DISPLAY_TRIGGER_CHANNEL].def =
        g_variant_new_int64(0);
    pwm_waveform_options[OPT_DISPLAY_TRIGGER_CHANNEL].values =
        integer_values(display_trigger_channel_values,
                       G_N_ELEMENTS(display_trigger_channel_values));

    pwm_waveform_options[OPT_DISPLAY_TRIGGER_EDGE].idn =
        "dec_pwm_waveform_opt_display_trigger_edge";
    pwm_waveform_options[OPT_DISPLAY_TRIGGER_EDGE].def =
        g_variant_new_string("rising");
    pwm_waveform_options[OPT_DISPLAY_TRIGGER_EDGE].values =
        string_values(display_trigger_edge_values,
                      G_N_ELEMENTS(display_trigger_edge_values));

    pwm_waveform_options[OPT_DISPLAY_TRIGGER_LEVEL].idn =
        "dec_pwm_waveform_opt_display_trigger_level";
    pwm_waveform_options[OPT_DISPLAY_TRIGGER_LEVEL].def =
        g_variant_new_double(50.0);

    pwm_waveform_options[OPT_DISPLAY_TRIGGER_POSITION].idn =
        "dec_pwm_waveform_opt_display_trigger_position";
    pwm_waveform_options[OPT_DISPLAY_TRIGGER_POSITION].def =
        g_variant_new_int64(50);

    return &pwm_waveform_c_decoder;
}

SRD_C_DECODER_EXPORT int srd_c_decoder_api_version(void)
{
    return SRD_C_DECODER_API_VERSION;
}
