#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <glib.h>
#include "libsigrokdecode.h"

#define SB_CHAN 0

enum sb_ann {
    ANN_SYNC = 0,
    ANN_TRANSACTION = 1,
    ANN_PAYLOAD = 2,
    ANN_ERROR = 3,
    ANN_WARNING = 4,
    NUM_ANN,
};

C_DECODER_STATE(sb_channel_c, {
    int state;
    
    int prev_pin;
    int curr_pin;
    
    uint64_t last_edge;
    uint64_t bit_start_edge;
    uint64_t packet_start_edge;
    
    int is_half_bit;
    
    uint32_t shift_reg;
    int bit_count;
    
    uint8_t payload[256];
    int payload_len;
    uint16_t crc;
    
    uint64_t samplerate;
    int out_ann;
    int out_proto;
});

static struct srd_channel sb_channel_c_channels[] = {
    {"sb", "SB", "Sideband channel", 0, SRD_CHANNEL_SCLK, "dec_sb_channel_c_chan_sb"},
};

static const char *sb_channel_c_ann_labels[][3] = {
    {"S",  "Sync",         "Sync (0xFE)"},
    {"T",  "Transaction",  "Transaction"},
    {"P",  "Payload",      "Payload Data"},
    {"!",  "Error",        "Protocol error"},
    {"W",  "Warning",      "Protocol warning"},
};

static const int sb_channel_c_row_ctrl_classes[] = {ANN_SYNC, ANN_TRANSACTION, ANN_ERROR, ANN_WARNING, -1};
static const int sb_channel_c_row_data_classes[] = {ANN_PAYLOAD, -1};
static const struct srd_c_ann_row sb_channel_c_ann_rows[] = {
    {"control", "Control", sb_channel_c_row_ctrl_classes, 4},
    {"data",    "Data",    sb_channel_c_row_data_classes, 1},
};

static const char *sb_channel_c_inputs[] = {"logic", NULL};
static const char *sb_channel_c_outputs[] = {"sb_channel", NULL};

static void sb_channel_c_start(struct srd_decoder_inst *di)
{
    sb_channel_c_s *s = (sb_channel_c_s *)c_decoder_get_private(di);
    s->out_ann   = c_reg_out(di, SRD_OUTPUT_ANN, "sb_channel");
    s->out_proto = c_reg_out(di, SRD_OUTPUT_PROTO, "sb_channel");
    s->samplerate = c_samplerate(di);
    
    s->state = 0;
    s->last_edge = 0;
    s->bit_start_edge = 0;
    s->is_half_bit = 0;
    s->shift_reg = 0;
    s->bit_count = 0;
    s->payload_len = 0;
    s->crc = 0xFFFF;
    
    s->prev_pin = c_pin(di, SB_CHAN);
    s->curr_pin = s->prev_pin;
}

static void sb_channel_c_metadata(struct srd_decoder_inst *di, int key, uint64_t value)
{
    sb_channel_c_s *s = (sb_channel_c_s *)c_decoder_get_private(di);
    if (key == SRD_CONF_SAMPLERATE)
        s->samplerate = value;
}

static void update_crc(sb_channel_c_s *s, uint8_t byte) {
    s->crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (s->crc & 1) {
            s->crc = (s->crc >> 1) ^ 0xA001;
        } else {
            s->crc >>= 1;
        }
    }
}

static void parse_sb_payload(struct srd_decoder_inst *di, sb_channel_c_s *s, uint64_t end_samplenum) {
    if (s->payload_len < 3) return; // Need at least Type + 2 bytes CRC
    int data_len = s->payload_len - 3; // Type byte + 2 bytes CRC
    uint8_t type = s->payload[0];
    uint8_t *data = &s->payload[1];
    
    // Check if it is LT Transaction (0x80~0xBF)
    if ((type & 0xC0) == 0x80) {
        if (data_len >= 1) {
            char buf[64];
            snprintf(buf, sizeof(buf), "LT Cmd: 0x%02X", data[0]);
            c_put(di, s->packet_start_edge, end_samplenum, s->out_ann, ANN_PAYLOAD, buf, "LT Cmd", "LT");
        }
    } 
    // AT Command/Response (0x40~0x5F) or RT Command/Response (0x00~0x3F)
    else if (((type & 0xE0) == 0x40) || ((type & 0xC0) == 0x00)) {
        if (data_len >= 2) {
            uint8_t opcode = data[0];
            // data[1] seems to be length/reserved, payload starts at data[2]
            uint8_t *payload_ptr = &data[2];
            int payload_len = data_len - 2;
            
            if (opcode == 5 && payload_len >= 1) {
                // Scrambler Re-Sync Support (Opcode 5)
                char buf[256];
                uint8_t b0 = payload_ptr[0];
                snprintf(buf, sizeof(buf),
                    "Scrambler Re-Sync | ESRS G4:%d ESRS G2/3:%d | SRS G4:%d SRS G2/3:%d",
                    (b0 >> 6) & 3, (b0 >> 4) & 3, (b0 >> 2) & 3, b0 & 3
                );
                c_put(di, s->packet_start_edge, end_samplenum, s->out_ann, ANN_PAYLOAD, buf, "Scrambler Re-Sync", "SRS");
            }
            else if (opcode == 12 && payload_len >= 3) {
                // Link Configuration (Opcode 12)
                char buf[256];
                uint8_t b0 = payload_ptr[0];
                uint8_t b1 = payload_ptr[1];
                uint8_t b2 = payload_ptr[2];
                snprintf(buf, sizeof(buf),
                    "Link Config | EnL0:%d EnL1:%d | EnReqL0:%d EnReqL1:%d Bond:%d Gen3:%d RS-FEC(G2):%d RS-FEC(G3):%d | USB4:%d TBT3:%d",
                    b0 & 1, (b0 >> 1) & 1,
                    b1 & 1, (b1 >> 1) & 1, (b1 >> 4) & 1, (b1 >> 5) & 1, (b1 >> 6) & 1, (b1 >> 7) & 1,
                    b2 & 1, (b2 >> 1) & 1
                );
                c_put(di, s->packet_start_edge, end_samplenum, s->out_ann, ANN_PAYLOAD, buf, "Link Config", "LC");
            } 
            else if (opcode == 13 && payload_len >= 4) {
                // TxFFE Gen 2/3 (Opcode 13)
                char buf[256];
                uint8_t b0 = payload_ptr[0];
                uint8_t b1 = payload_ptr[1];
                uint8_t b2 = payload_ptr[2];
                uint8_t b3 = payload_ptr[3];
                snprintf(buf, sizeof(buf),
                    "Gen2/3 TxFFE | L0 Req:0x%X RxLck:%d RxAct:%d ClkSw:%d New:%d Set:0x%X ReqDn:%d TxAct:%d | L1 Req:0x%X RxLck:%d RxAct:%d ClkSw:%d New:%d Set:0x%X ReqDn:%d TxAct:%d",
                    b0 & 0xF, (b0 >> 4) & 1, (b0 >> 5) & 1, (b0 >> 6) & 1, (b0 >> 7) & 1,
                    b2 & 0xF, (b2 >> 6) & 1, (b2 >> 7) & 1,
                    b1 & 0xF, (b1 >> 4) & 1, (b1 >> 5) & 1, (b1 >> 6) & 1, (b1 >> 7) & 1,
                    b3 & 0xF, (b3 >> 6) & 1, (b3 >> 7) & 1
                );
                c_put(di, s->packet_start_edge, end_samplenum, s->out_ann, ANN_PAYLOAD, buf, "Gen2/3 TxFFE", "TxFFE");
            }
            else if (opcode == 14 && payload_len >= 4) {
                // TxFFE Gen 4 (Opcode 14)
                char buf[256];
                uint8_t b0 = payload_ptr[0];
                uint8_t b1 = payload_ptr[1];
                uint8_t b2 = payload_ptr[2];
                uint8_t b3 = payload_ptr[3];
                snprintf(buf, sizeof(buf), 
                    "Gen4 TxFFE | L0 Req: 0x%02X(New:%d) L0 Set: 0x%02X(Done:%d) | L1 Req: 0x%02X(New:%d) L1 Set: 0x%02X(Done:%d)",
                    b0 & 0x3F, (b0 >> 7) & 1,
                    b1 & 0x3F, (b1 >> 6) & 1,
                    b2 & 0x3F, (b2 >> 7) & 1,
                    b3 & 0x3F, (b3 >> 6) & 1
                );
                c_put(di, s->packet_start_edge, end_samplenum, s->out_ann, ANN_PAYLOAD, buf, "Gen4 TxFFE", "TxFFE");
            }
            else if (opcode == 15 && payload_len >= 4) {
                // Sideband Channel Version (Opcode 15)
                char buf[256];
                uint8_t sub_ver = payload_ptr[0];
                uint8_t minor_ver = payload_ptr[1];
                uint16_t major_ver = payload_ptr[2] | (payload_ptr[3] << 8);
                snprintf(buf, sizeof(buf), "SB Version: %d.%d.%d", major_ver, minor_ver, sub_ver);
                c_put(di, s->packet_start_edge, end_samplenum, s->out_ann, ANN_PAYLOAD, buf, "SB Version", "Ver");
            }
        }
    }
}

static void sb_channel_c_decode(struct srd_decoder_inst *di)
{
    sb_channel_c_s *s = (sb_channel_c_s *)c_decoder_get_private(di);
    
    double ui_samples = (double)s->samplerate / 1000000.0;
    
    while (1) {
        int ret = c_wait(di, CW_E(SB_CHAN), CW_END);
        if (ret != SRD_OK) return;
        
        uint64_t samplenum = di_samplenum(di);
        s->curr_pin = c_pin(di, SB_CHAN);
        
        if (s->last_edge == 0) {
            s->last_edge = samplenum;
            s->prev_pin = s->curr_pin;
            continue;
        }
        
        uint64_t delta = samplenum - s->last_edge;
        double diff_ui = (double)delta / ui_samples;
        
        if (diff_ui > 2.5) {
            s->state = 0;
            s->is_half_bit = 0;
            s->bit_count = 0;
            s->shift_reg = 0;
        }
        
        int bit_completed = 0;
        int bit_val = 0;
        
        if (diff_ui > 0.25 && diff_ui <= 0.75) {
            if (!s->is_half_bit) {
                s->is_half_bit = 1;
            } else {
                s->is_half_bit = 0;
                bit_completed = 1;
                bit_val = 1;
            }
        } else if (diff_ui > 0.75 && diff_ui <= 1.25) {
            if (s->is_half_bit) {
                // Sync error
                s->is_half_bit = 0;
                c_put(di, s->last_edge, samplenum, s->out_ann, ANN_ERROR, "Error", "E", "E");
            } else {
                bit_completed = 1;
                bit_val = 0;
            }
        }
        
        if (bit_completed) {
            if (s->bit_count == 0) s->bit_start_edge = s->last_edge;
            
            s->shift_reg = (s->shift_reg >> 1) | (bit_val ? 0x80 : 0);
            s->bit_count++;
            
            if (s->bit_count == 8) {
                uint8_t byte = s->shift_reg;
                
                if (byte == 0xFE) {
                    if (s->state == 2) {
                        if (s->payload_len >= 2) {
                            if (s->crc == 0) {
                                c_put(di, s->bit_start_edge, samplenum, s->out_ann, ANN_TRANSACTION, "Valid CRC", "CRC OK", "CRC");
                                parse_sb_payload(di, s, samplenum);
                            } else {
                                c_put(di, s->bit_start_edge, samplenum, s->out_ann, ANN_ERROR, "CRC Error", "CRC Err", "CE");
                            }
                        }
                    }
                    s->state = 1; // Wait for transaction
                    s->packet_start_edge = s->bit_start_edge;
                    c_put(di, s->bit_start_edge, samplenum, s->out_ann, ANN_SYNC, "Sync (0xFE)", "S", "S");
                    s->payload_len = 0;
                    s->crc = 0xFFFF;
                } else if (s->state == 1) {
                    char buf[128];
                    if ((byte & 0xC0) == 0x80) {
                        snprintf(buf, sizeof(buf), "LT Transaction (%s)", (byte & 0x80) ? "Read" : "Write");
                    } else if ((byte & 0xE0) == 0x60) {
                        snprintf(buf, sizeof(buf), "NVM Data");
                    } else if ((byte & 0xE1) == 0x41) {
                        snprintf(buf, sizeof(buf), "AT Command");
                    } else if ((byte & 0xE1) == 0x40) {
                        snprintf(buf, sizeof(buf), "AT Response");
                    } else if ((byte & 0xC1) == 0x01) {
                        snprintf(buf, sizeof(buf), "RT Command");
                    } else if ((byte & 0xC1) == 0x00) {
                        snprintf(buf, sizeof(buf), "RT Response");
                    } else {
                        snprintf(buf, sizeof(buf), "Unknown (0x%02X)", byte);
                    }
                    
                    c_put(di, s->bit_start_edge, samplenum, s->out_ann, ANN_TRANSACTION, buf, "T", "Trans");
                    
                    s->payload[s->payload_len++] = byte;
                    update_crc(s, byte);
                    s->state = ((byte & 0xC0) == 0x80) ? 3 : 2; // Receiving payload or LT continuation
                } else if (s->state == 3) {
                    s->payload[s->payload_len++] = byte;
                    if (byte == (uint8_t)(~s->payload[0])) {
                        c_put(di, s->bit_start_edge, samplenum, s->out_ann, ANN_PAYLOAD, "~Cmd", "D", "Data");
                        parse_sb_payload(di, s, samplenum);
                    } else {
                        c_put(di, s->bit_start_edge, samplenum, s->out_ann, ANN_ERROR, "LT Inv", "LT", "E");
                    }
                    s->state = 1;
                } else if (s->state == 2) {
                    s->payload[s->payload_len++] = byte;
                    update_crc(s, byte);
                    
                    char buf[32];
                    snprintf(buf, sizeof(buf), "%02X", byte);
                    c_put(di, s->bit_start_edge, samplenum, s->out_ann, ANN_PAYLOAD, buf, "D", "Data");
                }
                
                s->bit_count = 0;
            }
        }
        
        s->last_edge = samplenum;
        s->prev_pin = s->curr_pin;
    }
}

static struct srd_decoder_option sb_channel_c_options[] = {
    {"invert", "dec_sb_channel_c_opt_invert", "Invert polarity", NULL, NULL},
};

static struct srd_c_decoder sb_channel_c_decoder = {
    .id = "sb_channel_c",
    .name = "Thunderbolt SB Channel (C)",
    .longname = "Thunderbolt / USB4 Sideband Channel",
    .desc = "Decodes Thunderbolt/USB4 Sideband Channel.",
    .license = "gplv2+",
    .channels = sb_channel_c_channels,
    .num_channels = 1,
    .options = sb_channel_c_options,
    .num_options = 1,
    .num_annotations = NUM_ANN,
    .ann_labels = sb_channel_c_ann_labels,
    .num_annotation_rows = 2,
    .annotation_rows = sb_channel_c_ann_rows,
    .inputs = sb_channel_c_inputs,
    .num_inputs = 1,
    .outputs = sb_channel_c_outputs,
    .num_outputs = 1,
    .reset = sb_channel_c_reset,
    .start = sb_channel_c_start,
    .decode = sb_channel_c_decode,
    .metadata = sb_channel_c_metadata,
    .destroy = sb_channel_c_destroy,
};

SRD_C_DECODER_EXPORT struct srd_c_decoder *srd_c_decoder_entry(void)
{
    GSList *inv_vals = NULL;
    inv_vals = g_slist_append(inv_vals, g_variant_new_boolean(FALSE));
    inv_vals = g_slist_append(inv_vals, g_variant_new_boolean(TRUE));
    sb_channel_c_options[0].def = g_variant_new_boolean(FALSE);
    sb_channel_c_options[0].values = inv_vals;

    return &sb_channel_c_decoder;
}

SRD_C_DECODER_EXPORT int srd_c_decoder_api_version(void)
{
    return SRD_C_DECODER_API_VERSION;
}
