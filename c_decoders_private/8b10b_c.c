#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <glib.h>
#include "libsigrokdecode.h"
#include "8b10b_table.h"

#define RX 0

enum ann_type {
    ANN_DATA,
    ANN_CTRL,
    ANN_ERROR,
    NUM_ANN
};

static const char *ann_labels[NUM_ANN][3] = {
    {"Data", "D"},
    {"Control", "K"},
    {"Error", "Err"},
};

static const struct srd_c_ann_row ann_rows[] = {
    {"data", "Data", (int[]){ANN_DATA, ANN_CTRL, ANN_ERROR, -1}, 3},
};

static struct srd_channel channels[] = {
    {"rx", "RX", "Data line", 0, SRD_CHANNEL_SCLK, "dec_8b10b_c_chan_rx"},
};

static struct srd_decoder_option options[] = {
    {"baudrate", "Baudrate", "Bitrate (baud)", NULL, NULL},
    {"polarity", "Polarity", "Data polarity", NULL, NULL},
    {"bit_order", "Bit Order", "Bit Order", NULL, NULL},
};

static const char *inputs[] = {"logic", NULL};
static const char *outputs[] = {"8b10b", NULL};

C_DECODER_STATE(decoder_8b10b, {
    double bit_time;
    int64_t baudrate;
    bool invert;
    bool lsb_first;
    uint64_t samplerate;
    
    uint64_t last_edge_samp;
    int last_pin_val;
    
    uint32_t shift_reg;
    int bits_in_reg;
    
    uint64_t start_samp;
    int disparity;
    
    struct srd_decoder_inst *di;
    int out_ann;
    int out_proto;
});

// Removed custom dec_reset, use macro-generated decoder_8b10b_reset

static void dec_start(struct srd_decoder_inst *di)
{
    decoder_8b10b_s *s = (decoder_8b10b_s *)c_decoder_get_private(di);
    GVariant *gvar;

    gvar = g_hash_table_lookup(di->c_options, "baudrate");
    s->baudrate = g_variant_get_int64(gvar);
    gvar = g_hash_table_lookup(di->c_options, "polarity");
    s->invert = strcmp(g_variant_get_string(gvar, NULL), "Inverted") == 0;
    gvar = g_hash_table_lookup(di->c_options, "bit_order");
    s->lsb_first = strcmp(g_variant_get_string(gvar, NULL), "lsb-first") == 0;

    s->samplerate = c_samplerate(di);

    if (s->samplerate > 0 && s->baudrate > 0)
        s->bit_time = (double)s->samplerate / s->baudrate;
    else
        s->bit_time = 0;

    s->di = di;
    s->out_ann = c_reg_out(di, SRD_OUTPUT_ANN, "8b10b");
    s->out_proto = c_reg_out(di, SRD_OUTPUT_PROTO, "8b10b");

    s->last_edge_samp = 0;
    s->last_pin_val = c_pin(di, RX);
    s->shift_reg = 0;
    s->bits_in_reg = 0;
    s->start_samp = 0;
    s->disparity = 0;
}

static void dec_metadata(struct srd_decoder_inst *di, int key, uint64_t value)
{
    decoder_8b10b_s *s = (decoder_8b10b_s *)c_decoder_get_private(di);
    if (key == SRD_CONF_SAMPLERATE) {
        s->samplerate = value;
        if (s->samplerate > 0 && s->baudrate > 0)
            s->bit_time = (double)s->samplerate / s->baudrate;
    }
}

static uint32_t reverse_10bit(uint32_t val)
{
    uint32_t res = 0;
    for (int i = 0; i < 10; i++) {
        res = (res << 1) | (val & 1);
        val >>= 1;
    }
    return res;
}

static void process_symbol(decoder_8b10b_s *s, uint64_t current_samp)
{
    uint32_t code = s->shift_reg & 0x3FF;
    if (s->lsb_first) {
        code = reverse_10bit(code);
    }
    
    int match_idx = -1;
    int is_rd_plus = 0;
    
    for (size_t i = 0; i < sizeof(table_8b10b) / sizeof(table_8b10b[0]); i++) {
        if (code == table_8b10b[i].val1) {
            match_idx = (int)i;
            is_rd_plus = 1;
            break;
        } else if (code == table_8b10b[i].val3) {
            match_idx = (int)i;
            is_rd_plus = 0;
            break;
        }
    }
    
    if (match_idx >= 0) {
        int is_ctrl = (table_8b10b[match_idx].name[0] == 'K');
        int ann_type = is_ctrl ? ANN_CTRL : ANN_DATA;
        
        int current_disp = is_rd_plus ? 1 : -1;
        if (s->disparity != 0 && s->disparity == current_disp) {
            char err_buf[64];
            snprintf(err_buf, sizeof(err_buf), "%s (DispErr)", table_8b10b[match_idx].name);
            c_put(s->di, s->start_samp, current_samp, s->out_ann, ANN_ERROR, err_buf, "E");
        } else {
            c_put(s->di, s->start_samp, current_samp, s->out_ann, ann_type, table_8b10b[match_idx].name, table_8b10b[match_idx].name);
        }
        
        s->disparity = is_rd_plus ? -1 : 1;
        
        s->bits_in_reg = 0;
        s->shift_reg = 0;
    } else {
        s->bits_in_reg = 9;
        s->shift_reg &= 0x1FF;
        s->start_samp = current_samp - (uint64_t)(s->bit_time * 9);
    }
}

static void dec_decode(struct srd_decoder_inst *di)
{
    decoder_8b10b_s *s = (decoder_8b10b_s *)c_decoder_get_private(di);
    
    if (s->bit_time == 0) return;

    while (1) {
        int ret = c_wait(di, CW_E(RX), CW_END);
        if (ret != SRD_OK) return;

        uint64_t current_samp = di_samplenum(di);
        int current_pin = c_pin(di, RX);
        
        if (s->invert)
            current_pin = !current_pin;

        if (s->last_pin_val == -1) {
            s->last_pin_val = current_pin;
            s->last_edge_samp = current_samp;
            continue;
        }

        double duration = current_samp - s->last_edge_samp;
        double bits_f = duration / s->bit_time;
        int num_bits = (int)(bits_f + 0.5);

        if (num_bits > 0) {
            if (num_bits > 20) {
                s->bits_in_reg = 0;
                s->shift_reg = 0;
                s->last_pin_val = current_pin;
                s->last_edge_samp = current_samp;
                continue;
            }

            int val = s->last_pin_val;
            for (int i = 0; i < num_bits; i++) {
                if (s->bits_in_reg == 0) {
                    s->start_samp = s->last_edge_samp + (uint64_t)(i * s->bit_time);
                }
                
                s->shift_reg = (s->shift_reg << 1) | val;
                s->bits_in_reg++;
                
                if (s->bits_in_reg == 10) {
                    process_symbol(s, s->last_edge_samp + (uint64_t)((i+1) * s->bit_time));
                }
            }
        }
        
        s->last_pin_val = current_pin;
        s->last_edge_samp = current_samp;
    }
}

static struct srd_c_decoder decoder = {
    .id = "8b10b_c",
    .name = "8b/10b (Native C)",
    .longname = "8b/10b protocol (C)",
    .desc = "Decodes 8b/10b line coding.",
    .license = "gplv2+",
    .channels = channels,
    .num_channels = 1,
    .options = options,
    .num_options = 3,
    .num_annotations = NUM_ANN,
    .ann_labels = ann_labels,
    .num_annotation_rows = 1,
    .annotation_rows = ann_rows,
    .inputs = inputs,
    .num_inputs = 1,
    .outputs = outputs,
    .num_outputs = 1,
    .reset = decoder_8b10b_reset,
    .start = dec_start,
    .decode = dec_decode,
    .metadata = dec_metadata,
    .destroy = decoder_8b10b_destroy,
};

SRD_C_DECODER_EXPORT struct srd_c_decoder *srd_c_decoder_entry(void)
{
    GSList *pol_vals = NULL;
    pol_vals = g_slist_append(pol_vals, g_variant_new_string("Normal"));
    pol_vals = g_slist_append(pol_vals, g_variant_new_string("Inverted"));
    options[1].def = g_variant_new_string("Normal");
    options[1].values = pol_vals;
    
    GSList *bit_vals = NULL;
    bit_vals = g_slist_append(bit_vals, g_variant_new_string("lsb-first"));
    bit_vals = g_slist_append(bit_vals, g_variant_new_string("msb-first"));
    options[2].def = g_variant_new_string("lsb-first");
    options[2].values = bit_vals;

    options[0].def = g_variant_new_int64(2500000000);
    
    return &decoder;
}

SRD_C_DECODER_EXPORT int srd_c_decoder_api_version(void)
{
    return SRD_C_DECODER_API_VERSION;
}
