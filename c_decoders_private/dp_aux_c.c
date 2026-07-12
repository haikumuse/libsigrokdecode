#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <glib.h>
#include "libsigrokdecode.h"

/* ===== 通道索引定义 ===== */
#define AUX 0

/* ===== 状态变量与常量 ===== */
typedef struct {
    uint32_t addr;
    const char *name;
} dpcd_reg_t;

static const dpcd_reg_t dpcd_regs[] = {
    {0x00000, "DPCD_REV"},
    {0x00001, "MAX_LINK_RATE"},
    {0x00002, "MAX_LANE_COUNT"},
    {0x00003, "MAX_DOWNSPREAD"},
    {0x00004, "NORP"},
    {0x00005, "DOWNSTREAMPORT_PRESENT"},
    {0x00006, "MAIN_LINK_CHANNEL_CODING"},
    {0x00007, "DOWN_STREAM_PORT_COUNT"},
    {0x00008, "RECEIVE_PORT0_CAP_0"},
    {0x00009, "RECEIVE_PORT0_CAP_1"},
    {0x0000A, "RECEIVE_PORT1_CAP_0"},
    {0x0000B, "RECEIVE_PORT1_CAP_1"},
    {0x0000C, "I2C_SPEED_CONTROL_CAP"},
    {0x0000D, "eDP_CONFIGURATION_CAP"},
    {0x0000E, "TRAINING_AUX_RD_INTERVAL"},
    {0x0000F, "ADAPTER_CAP"},
    {0x00100, "LINK_BW_SET"},
    {0x00101, "LANE_COUNT_SET"},
    {0x00102, "TRAINING_PATTERN_SET"},
    {0x00103, "TRAINING_LANE0_SET"},
    {0x00104, "TRAINING_LANE1_SET"},
    {0x00105, "TRAINING_LANE2_SET"},
    {0x00106, "TRAINING_LANE3_SET"},
    {0x00107, "DOWNSPREAD_CTRL"},
    {0x00108, "MAIN_LINK_CHANNEL_CODING_SET"},
    {0x00109, "I2C_SPEED_CONTROL_SET"},
    {0x0010A, "eDP_CONFIGURATION_SET"},
    {0x0010B, "LINK_QUAL_LANE0_SET"},
    {0x0010C, "LINK_QUAL_LANE1_SET"},
    {0x0010D, "LINK_QUAL_LANE2_SET"},
    {0x0010E, "LINK_QUAL_LANE3_SET"},
    {0x0010F, "TRAINING_LANE0_1_SET2"},
    {0x00110, "TRAINING_LANE2_3_SET2"},
    {0x00200, "SINK_COUNT"},
    {0x00201, "DEVICE_SERVICE_IRQ_VECTOR"},
    {0x00202, "LANE0_1_STATUS"},
    {0x00203, "LANE2_3_STATUS"},
    {0x00204, "LANE_ALIGN_STATUS_UPDATED"},
    {0x00205, "SINK_STATUS"},
    {0x00206, "ADJUST_REQUEST_LANE0_1"},
    {0x00207, "ADJUST_REQUEST_LANE2_3"},
    {0x00208, "TRAINING_SCORE_LANE0"},
    {0x00209, "TRAINING_SCORE_LANE1"},
    {0x0020A, "TRAINING_SCORE_LANE2"},
    {0x0020B, "TRAINING_SCORE_LANE3"},
    {0x0020C, "TEST_REQUEST"},
    {0x0020D, "TEST_LINK_RATE"},
    {0x0020E, "TEST_LANE_COUNT"},
    {0x0020F, "TEST_PATTERN"},
    {0x00210, "TEST_H_TOTAL"},
    {0x00212, "TEST_V_TOTAL"},
    {0x00214, "TEST_H_START"},
    {0x00216, "TEST_V_START"},
    {0x00218, "TEST_HSYNC"},
    {0x0021A, "TEST_VSYNC"},
    {0x0021C, "TEST_H_WIDTH"},
    {0x0021E, "TEST_V_HEIGHT"},
    {0x00220, "TEST_MISC"},
    {0x00221, "TEST_REFRESH_RATE_NUMERATOR"},
    {0x00222, "TEST_CRC_R_CR"},
    {0x00224, "TEST_CRC_G_Y"},
    {0x00226, "TEST_CRC_B_CB"},
    {0x00260, "TEST_RESPONSE"},
    {0x00261, "TEST_EDID_CHECKSUM"},
    {0x00270, "TEST_SINK_MISC"},
    {0x00280, "PAYLOAD_ALLOCATE_SET"},
    {0x002C0, "PAYLOAD_ALLOCATE_TIME_SLOT_COUNT_SET"},
    {0x00700, "EDP_DPCD_REV"},
    // 用户可以在此处继续添加更多寄存器映射
};

static const char *get_dpcd_reg_name(uint32_t addr) {
    for (size_t i = 0; i < sizeof(dpcd_regs) / sizeof(dpcd_regs[0]); i++) {
        if (dpcd_regs[i].addr == addr) {
            return dpcd_regs[i].name;
        }
    }
    return NULL;
}

static const char *get_aux_cmd_name(uint8_t cmd) {
    switch (cmd) {
        case 0x0: return "I2C Write";
        case 0x1: return "I2C Read";
        case 0x2: return "I2C Write Status Req";
        case 0x3: return "I2C Read Status Req";
        case 0x4: return "I2C Write MOT";
        case 0x5: return "I2C Read MOT";
        case 0x6: return "I2C Write Status Req MOT";
        case 0x7: return "I2C Read Status Req MOT";
        case 0x8: return "Native AUX Write";
        case 0x9: return "Native AUX Read";
        default: return "Reserved";
    }
}

static void format_reply_cmd(uint8_t cmd, char *out_str, size_t max_len) {
    uint8_t aux_reply = (cmd >> 4) & 0x0F;
    uint8_t i2c_reply = cmd & 0x0F;
    const char *aux_str = "Reserved";
    const char *i2c_str = "";

    switch (aux_reply) {
        case 0x0: aux_str = "AUX ACK"; break;
        case 0x1: aux_str = "AUX NACK"; break;
        case 0x2: aux_str = "AUX DEFER"; break;
    }
    
    // I2C reply only valid if AUX ACK
    if (aux_reply == 0x0) {
        switch (i2c_reply) {
            case 0x0: i2c_str = " (I2C ACK)"; break;
            case 0x1: i2c_str = " (I2C NACK)"; break;
            case 0x2: i2c_str = " (I2C DEFER)"; break;
        }
    }
    snprintf(out_str, max_len, "Reply: %s%s", aux_str, i2c_str);
}


/* ===== 注解类枚举 ===== */
enum dp_aux_ann {
    ANN_SYNC = 0,
    ANN_SYNC_END = 1,
    ANN_COMMAND = 2,
    ANN_ADDRESS = 3,
    ANN_LENGTH = 4,
    ANN_DATA = 5,
    ANN_ERROR = 6,
    ANN_WARNING = 7,
    ANN_STOP = 8,
    NUM_ANN,
};

/* ===== 状态结构体 ===== */
C_DECODER_STATE(dp_aux_c, {
    int state;               // state
    
    int prev_pin_val;                 // prev pin value
    int curr_pin_val;                // current pin value for next iter
    
    double half_ui_samples;
    double timing_err_accum;
    int sync_zero_count;
    int sync_glitch_count;
    
    uint8_t is_alt_format;
    uint8_t sync_end_valid;
    uint8_t addr_phase_start;
    uint8_t abort_flag;
    uint64_t sync_duration;
    uint64_t sync_pulse_count;
    uint64_t data_byte_idx;
    uint8_t sync_edge_count;
    uint8_t sync_glitch_flag;
    uint8_t is_request;
    int expected_len;
    int temp_v94;
    int edge_type;
    int byte_counter;
    uint64_t half_ui_accum;             // accumulated half UIs
    uint64_t last_edge_samp;            // last edge sample (last_edge_samp)
    
    uint32_t temp_v111[52];
    uint32_t temp_v112[52];
    
    uint64_t temp_v114;
    uint64_t timing_baseline;
    uint64_t cmd_byte;
    uint64_t address_val;
    uint64_t len_minus_1;
    uint64_t data_buf[4087];
    uint64_t temp_v113[3];
    uint64_t ann_start_samps[3];
    
    uint64_t samplerate;
    int out_ann;
    int out_proto;
});

/* ===== 通道定义 ===== */
static struct srd_channel dp_aux_c_channels[] = {
    {"aux", "AUX", "DP AUX channel", 0, SRD_CHANNEL_SCLK, "dec_dp_aux_c_chan_aux"},
};

/* ===== 注解标签 ===== */
static const char *dp_aux_c_ann_labels[][3] = {
    {"S",  "Sync",         "Sync"},
    {"SE", "Sync End",     "Sync End"},
    {"C",  "Command",      "Command"},
    {"A",  "Address",      "Address"},
    {"L",  "Length",       "Length"},
    {"D",  "Data",         "Data byte"},
    {"!",  "Error",        "Protocol error"},
    {"W",  "Warning",      "Protocol warning"},
    {"P",  "Stop",         "Stop condition"},
};

/* ===== 注解行 ===== */
static const int dp_aux_c_row_ctrl_classes[] = {ANN_SYNC, ANN_SYNC_END, ANN_COMMAND, ANN_LENGTH, ANN_STOP, ANN_ERROR, ANN_WARNING, -1};
static const int dp_aux_c_row_data_classes[] = {ANN_ADDRESS, ANN_DATA, -1};
static const struct srd_c_ann_row dp_aux_c_ann_rows[] = {
    {"control", "Control", dp_aux_c_row_ctrl_classes, 7},
    {"data",    "Data",    dp_aux_c_row_data_classes, 2},
};

static const char *dp_aux_c_inputs[] = {"logic", NULL};
static const char *dp_aux_c_outputs[] = {"dp_aux", NULL};



static void dp_aux_c_start(struct srd_decoder_inst *di)
{
    dp_aux_c_s *s = (dp_aux_c_s *)c_decoder_get_private(di);
    s->out_ann   = c_reg_out(di, SRD_OUTPUT_ANN, "dp_aux");
    s->out_proto = c_reg_out(di, SRD_OUTPUT_PROTO, "dp_aux");
    s->samplerate = c_samplerate(di);
    
    // Initial state setup like sub_18000308C
    s->edge_type = 1;
    s->is_request = 1;
    s->sync_edge_count = 1;
    s->sync_glitch_flag = 1;
    s->state = 0;
    s->last_edge_samp = 0;
    
    // To grab the initial pin state
    int initial_pin = c_pin(di, AUX);
    s->curr_pin_val = initial_pin;
    s->prev_pin_val = initial_pin;
}

static void dp_aux_c_metadata(struct srd_decoder_inst *di, int key, uint64_t value)
{
    dp_aux_c_s *s = (dp_aux_c_s *)c_decoder_get_private(di);
    if (key == SRD_CONF_SAMPLERATE)
        s->samplerate = value;
}

static void dp_aux_c_decode(struct srd_decoder_inst *di)
{
    dp_aux_c_s *s = (dp_aux_c_s *)c_decoder_get_private(di);
    bool is_valid = false;

    while (1) {
        int ret = c_wait(di, CW_E(AUX), CW_END);
        if (ret != SRD_OK) return;
        
        uint64_t edge_samp_tmp = di_samplenum(di);
        int current_pin = c_pin(di, AUX);
        
        uint64_t current_samp = edge_samp_tmp;
        double delta_samp = (double)(current_samp - s->last_edge_samp);
        
        s->prev_pin_val = s->curr_pin_val;
        int tmp_30 = current_pin;
        s->curr_pin_val = tmp_30;
        
        int current_bit = tmp_30;
        if (!s->edge_type) {
            current_bit = tmp_30 ^ 1;
        }
        
        uint8_t tmp_32 = 0;
        uint8_t has_sync = 0;
        uint64_t prev_edge_samp = 0;
        
        if (s->state > 3) {
            if (s->prev_pin_val == 0) {
                if (tmp_30 == 1) {
                    prev_edge_samp = s->last_edge_samp;
                    tmp_32 = 1;
                    goto LABEL_68;
                }
LABEL_67:
                prev_edge_samp = s->last_edge_samp;
                goto LABEL_68;
            }
LABEL_64:
            if (s->prev_pin_val == 1 && tmp_30 == 0) {
                prev_edge_samp = s->last_edge_samp;
                has_sync = 1;
                goto LABEL_68;
            }
            goto LABEL_67;
        }
        
        // Timeout logic
        if (delta_samp > (double)s->samplerate * 400000.0 / 1000000000.0) {
            s->is_request = 1;
        }
        
        uint64_t tmp_36 = 0;
        uint64_t tmp_35 = 0;
        if (s->sync_glitch_count < 50 && s->sync_zero_count < 50) {
            tmp_36 = s->sync_duration;
            tmp_35 = s->sync_pulse_count;
        } else {
            s->sync_glitch_count = 0;
            s->sync_pulse_count = 0;
            s->sync_zero_count = 0;
            s->sync_duration = 0;
            tmp_35 = 0;
            tmp_36 = 0;
        }
        
        if (s->prev_pin_val == 1) {
            if (s->curr_pin_val) goto LABEL_67;
            
            double tmp_37 = 0.0;
            for (uint64_t tmp_38 = 0; tmp_38 < tmp_36; tmp_38++) {
                tmp_37 += (double)s->temp_v111[tmp_38];
            }
            s->half_ui_samples = (s->sync_zero_count > 0) ? (tmp_37 / (double)s->sync_zero_count) : 0.0;
            
            if (s->half_ui_samples <= 0.0) goto LABEL_43;
            if (s->sync_zero_count >= 8) {
                if (delta_samp >= s->half_ui_samples * 1.5) {
                    prev_edge_samp = s->last_edge_samp;
                    s->sync_zero_count = 0;
                    s->sync_glitch_count = 0;
                    s->state = 4;
                    s->sync_pulse_count = 0;
                    s->timing_baseline = 0;
                    has_sync = 1;
                    s->sync_duration = 0;
                    goto LABEL_68;
                }
LABEL_43:
                prev_edge_samp = s->last_edge_samp;
                s->temp_v111[tmp_36 + 1] = current_samp - s->last_edge_samp;
                s->sync_duration = tmp_36 + 1;
                s->sync_zero_count++;
                has_sync = 1;
                goto LABEL_68;
            }
            
            if (delta_samp < s->half_ui_samples * 1.5 && s->half_ui_samples * 0.5 <= delta_samp) {
                goto LABEL_43;
            }
            
            if (s->sync_glitch_flag) {
                s->sync_glitch_flag = 0;
            }
            current_samp = edge_samp_tmp;
            prev_edge_samp = s->last_edge_samp;
            s->sync_glitch_count = 0;
            s->sync_pulse_count = 0;
            s->temp_v111[0] = edge_samp_tmp - s->last_edge_samp;
            s->sync_zero_count = 1;
            s->sync_duration = 1;
            has_sync = 1;
        } else {
            if (s->prev_pin_val != 0) {
                // prev_pin_val is 0 or 1.
                tmp_30 = s->curr_pin_val;
                goto LABEL_64;
            }
            if (s->curr_pin_val != 1) goto LABEL_67;
            
            double tmp_44 = 0.0;
            for (uint64_t tmp_45 = 0; tmp_45 < tmp_35; tmp_45++) {
                tmp_44 += (double)s->temp_v112[tmp_45];
            }
            s->timing_err_accum = (s->sync_glitch_count > 0) ? (tmp_44 / (double)s->sync_glitch_count) : 0.0;
            
            if (s->timing_err_accum <= 0.0) goto LABEL_59;
            
            if (s->sync_glitch_count >= 8) {
                if (delta_samp >= s->timing_err_accum * 1.5) {
                    prev_edge_samp = s->last_edge_samp;
                    s->sync_zero_count = 0;
                    s->sync_duration = 0;
                    s->sync_glitch_count = 0;
                    s->sync_pulse_count = 0;
                    s->state = 4;
                    s->timing_baseline = 0;
                    tmp_32 = 1;
                    goto LABEL_68;
                }
LABEL_59:
                prev_edge_samp = s->last_edge_samp;
                s->temp_v112[tmp_35 + 1] = current_samp - s->last_edge_samp;
                s->sync_pulse_count = tmp_35 + 1;
                s->sync_glitch_count++;
                tmp_32 = 1;
                goto LABEL_68;
            }
            
            if (delta_samp < s->timing_err_accum * 1.5 && s->timing_err_accum * 0.5 <= delta_samp) {
                goto LABEL_59;
            }
            
            if (s->sync_glitch_flag) {
                s->sync_glitch_flag = 0;
            }
            current_samp = edge_samp_tmp;
            prev_edge_samp = s->last_edge_samp;
            s->sync_zero_count = 0;
            s->sync_duration = 0;
            s->temp_v112[0] = edge_samp_tmp - s->last_edge_samp;
            s->sync_glitch_count = 1;
            s->sync_pulse_count = 1;
            tmp_32 = 1;
        }

LABEL_68:
        if (s->state <= 3) {
            s->last_edge_samp = current_samp;
            continue;
        }
        if (s->state >= 4096) s->state = 0;
        
        double phase_base = (double)current_samp;
        double phase_step = (s->timing_err_accum + s->half_ui_samples) * 0.5;
        double phase_err = (phase_base - (double)prev_edge_samp) / phase_step;
        
        uint64_t edge_half_uis = 0;
        if (phase_err >= 6.0) {
            edge_half_uis = (uint64_t)(phase_err + 0.5);
        } else {
            int phase_diff = 1;
            double tmp_54 = fabs(1.0 - phase_err);
            double tmp_55 = fabs(2.0 - phase_err);
            if (tmp_54 > tmp_55) {
                tmp_54 = tmp_55;
                phase_diff = 2;
            }
            double tmp_56 = fabs(3.0 - phase_err);
            if (tmp_54 > tmp_56) {
                tmp_54 = tmp_56;
                phase_diff = 3;
            }
            double tmp_57 = fabs(4.0 - phase_err);
            if (tmp_54 > tmp_57) {
                tmp_54 = tmp_57;
                phase_diff = 4;
            }
            if (tmp_54 > fabs(5.0 - phase_err)) {
                phase_diff = 5;
            }
            edge_half_uis = phase_diff;
        }
        
        if (s->state == 4) {
            s->half_ui_accum = 0;
        }
        s->half_ui_accum += edge_half_uis;
        

        switch (s->state) {
            case 4:
                memset(s->temp_v113, 0, sizeof(s->temp_v113));
                memset(s->data_buf, 0, sizeof(s->data_buf));
                s->temp_v114 = 0; s->timing_baseline = 0; s->cmd_byte = 0; s->address_val = 0; s->len_minus_1 = 0;
                
                if (s->is_request) {
                    is_valid = (s->half_ui_accum == 2);
                } else {
                    is_valid = (s->half_ui_accum == 5);
                }
                
                s->half_ui_accum = 0;
                if (has_sync) {
                    s->edge_type = 0;
                } else {
                    int tmp_62 = s->edge_type;
                    if (tmp_32) tmp_62 = 1;
                    s->edge_type = tmp_62;
                }
                
                if (s->sync_edge_count) {
                    s->is_request = 1; // Simplification, DSView checks if option bit is set
                    s->sync_edge_count = 0;
                }
                s->state = 5;
                break;
                
            case 5:
                s->half_ui_accum -= 4;
                s->state = 6;
                s->ann_start_samps[0] = current_samp; // Start of Command
                
                if (is_valid && s->half_ui_accum == 0) {
                    s->sync_end_valid = 1;
                    goto LABEL_109;
                }
                s->sync_end_valid = 0;
                
                if (s->half_ui_accum == 1) {
                    if (s->is_request) {
                        s->is_alt_format = 1;
                        goto LABEL_114;
                    }
                } else if (s->half_ui_accum != 0) {
                    if (s->half_ui_accum == 4) s->temp_v94 = 2;
LABEL_114:
                    if (s->is_request) {
                        if (s->temp_v94 != 0) s->temp_v94 = 2;
                    }
                } else {
LABEL_109:
                    if (s->is_request) {
                        s->is_alt_format = 0;
                        goto LABEL_114;
                    }
                }
                s->temp_v114 |= s->temp_v94;
                break;
                
            case 6: {
                uint64_t bits_to_read = 0;
                if (s->is_request) {
                    if (s->sync_end_valid && s->half_ui_accum == 2) {
                        s->addr_phase_start = 1;
                        s->half_ui_accum = 0;
                        s->state = 7;
                        break;
                    }
                    s->addr_phase_start = 0;
                    if (s->half_ui_accum < 8 || s->is_alt_format) {
                        if (s->half_ui_accum < 16 || !s->is_alt_format) {
                            if ( (((s->half_ui_accum >> 63) ^ (s->half_ui_accum & 1)) - (s->half_ui_accum >> 63)) != 1 ) break;
                            uint64_t bits_to_read = s->is_alt_format ? 16 : 8;
                            s->cmd_byte |= (uint64_t)current_bit << ((bits_to_read - s->half_ui_accum) / 2);
                            break;
                        }
                        
                        // Output command
                        char cmd_str[128];
                        char cmd_short[32];
                        snprintf(cmd_str, sizeof(cmd_str), "Request: %s (0x%X)", get_aux_cmd_name((uint8_t)s->cmd_byte), (unsigned int)s->cmd_byte);
                        snprintf(cmd_short, sizeof(cmd_short), "CMD: %X", (unsigned int)s->cmd_byte);
                        c_put_v(di, s->ann_start_samps[0], current_samp, s->out_ann, ANN_COMMAND, s->cmd_byte, "Command", cmd_str, cmd_short);
                        
                        s->half_ui_accum -= 16;
                        s->byte_counter = 0;
                        s->data_byte_idx = 0;
                        s->state = 9;
                        s->ann_start_samps[1] = current_samp; // Data start sample
                        if (s->half_ui_accum == 1) s->data_buf[0] |= (uint64_t)current_bit << 19;
                    } else {
                        // Output command
                        char cmd_str[128];
                        char cmd_short[32];
                        snprintf(cmd_str, sizeof(cmd_str), "Request: %s (0x%X)", get_aux_cmd_name((uint8_t)s->cmd_byte), (unsigned int)s->cmd_byte);
                        snprintf(cmd_short, sizeof(cmd_short), "CMD: %X", (unsigned int)s->cmd_byte);
                        c_put_v(di, s->ann_start_samps[0], current_samp, s->out_ann, ANN_COMMAND, s->cmd_byte, "Command", cmd_str, cmd_short);
                        
                        s->half_ui_accum -= 8;
                        s->state = 7;
                        s->ann_start_samps[0] = current_samp; // Address start sample
                        if (s->half_ui_accum == 1) s->address_val |= (uint64_t)current_bit << 19;
                    }
                } else {
                    if (s->half_ui_accum >= 16) {
                        // Output reply command before transitioning
                        char cmd_str[128];
                        char cmd_short[32];
                        format_reply_cmd((uint8_t)s->cmd_byte, cmd_str, sizeof(cmd_str));
                        snprintf(cmd_short, sizeof(cmd_short), "CMD: %02X", (unsigned int)s->cmd_byte);
                        c_put_v(di, s->ann_start_samps[0], current_samp, s->out_ann, ANN_COMMAND, s->cmd_byte, "Command", cmd_str, cmd_short);

                        s->half_ui_accum -= 16;
                        // Start Data phase directly for Reply
                        s->data_byte_idx = 0;
                        s->byte_counter = 0;
                        s->state = 9;
                        s->ann_start_samps[1] = current_samp; // Data start sample
                        if (s->half_ui_accum == 1) s->data_buf[0] |= (uint64_t)current_bit << 7;
                        break;
                    }
                    if ( (((s->half_ui_accum >> 63) ^ (s->half_ui_accum & 1)) - (s->half_ui_accum >> 63)) == 1 && s->half_ui_accum < 16) {
                        s->cmd_byte |= (uint64_t)current_bit << ((16 - s->half_ui_accum) / 2);
                    }
                }
                break;
            }
                
            case 7:
                if (s->addr_phase_start && s->half_ui_accum == 4) {
                    s->abort_flag = 1;
                    s->state = 5;
                    s->temp_v114 |= 2;
                    s->temp_v94 = 2;
                    break;
                }
                s->abort_flag = 0;
                if (s->half_ui_accum < 40) {
                    if ( (((s->half_ui_accum >> 63) ^ (s->half_ui_accum & 1)) - (s->half_ui_accum >> 63)) == 1 ) {
                        s->address_val |= (uint64_t)current_bit << ((40 - s->half_ui_accum) / 2);
                    }
                } else {
                    char addr_str[128];
                    char addr_short[32];
                    const char *reg_name = get_dpcd_reg_name((uint32_t)s->address_val);
                    if (reg_name) {
                        snprintf(addr_str, sizeof(addr_str), "ADDR: %05lX (%s)", (unsigned long)s->address_val, reg_name);
                    } else {
                        snprintf(addr_str, sizeof(addr_str), "ADDR: %05lX", (unsigned long)s->address_val);
                    }
                    snprintf(addr_short, sizeof(addr_short), "%05lX", (unsigned long)s->address_val);
                    c_put_v(di, s->ann_start_samps[0], current_samp, s->out_ann, ANN_ADDRESS, s->address_val, "Address", addr_str, addr_short);
                    
                    s->half_ui_accum -= 40;
                    s->state = 8;
                    s->ann_start_samps[0] = current_samp; // Length start sample
                    if (s->half_ui_accum == 1) s->len_minus_1 |= (uint64_t)current_bit << 7;
                }
                break;
                
            case 8:
                if (s->half_ui_accum >= 16) {
                    char len_str[64];
                    char len_short[32];
                    snprintf(len_str, sizeof(len_str), "Length: %lu", (unsigned long)s->len_minus_1 + 1); // Length is 0-based in protocol
                    snprintf(len_short, sizeof(len_short), "LEN: %02lX", (unsigned long)s->len_minus_1);
                    c_put_v(di, s->ann_start_samps[0], current_samp, s->out_ann, ANN_LENGTH, s->len_minus_1, "Length", len_str, len_short);
                    
                    s->half_ui_accum -= 16;
                    // Start Data phase
                    s->data_byte_idx = 0;
                    s->byte_counter = 0;
                    s->state = 9;
                    s->ann_start_samps[1] = current_samp; // Data start sample
                    if (s->half_ui_accum == 1) s->data_buf[0] |= (uint64_t)current_bit << 7;
                    break;
                }
                if ( (((s->half_ui_accum >> 63) ^ (s->half_ui_accum & 1)) - (s->half_ui_accum >> 63)) == 1 ) {
                    s->len_minus_1 |= (uint64_t)current_bit << ((16 - s->half_ui_accum) / 2);
                }
                break;
                
            default:
                if (s->state == 16 * s->byte_counter + 9) {
                    if (s->half_ui_accum < 16) {
                        if ( (((s->half_ui_accum >> 63) ^ (s->half_ui_accum & 1)) - (s->half_ui_accum >> 63)) == 1 ) {
                            // Wait, the original code had temp_v113[state], which maps to data_buf[16 * byte_counter]
                            s->data_buf[16 * s->byte_counter] |= (uint64_t)current_bit << ((16 - s->half_ui_accum) / 2);
                        }
                    } else {
                        // Byte finished
                        char data_str[32];
                        snprintf(data_str, sizeof(data_str), "D: %02X", (unsigned int)s->data_buf[16 * s->byte_counter]);
                        c_put_v(di, s->ann_start_samps[9 + s->byte_counter], current_samp, s->out_ann, ANN_DATA, s->data_buf[16 * s->byte_counter], "Data", data_str);
                        s->half_ui_accum -= 16;
                        s->byte_counter++;
                        s->data_byte_idx++;
                        s->state = 16 * s->byte_counter + 9;
                        s->ann_start_samps[9 + s->byte_counter] = current_samp;
                        if (s->half_ui_accum == 1) s->data_buf[16 * s->data_byte_idx] |= (uint64_t)current_bit << 7;
                    }
                }
                break;
        }
        
        // Post-state-machine edge-duration validation
        bool abort = false;
        
        if (s->abort_flag) {
            s->state = 5;
            abort = true;
        } else if (s->state != 0) {
            if (s->state == 4 || s->state == 5) {
                if (edge_half_uis > 8) abort = true;
            } else if (s->state == 6) {
                if (edge_half_uis > 41) abort = true;
                if (!abort && edge_half_uis > 2 && s->half_ui_accum > 1) abort = true;
            } else { // state > 6
                if (edge_half_uis > 41) abort = true;
                if (!abort && edge_half_uis > 2) abort = true;
            }
            
            if (s->byte_counter > 255) {
                abort = true;
            }
        }
        
        if (abort) {
            if (s->is_request && s->state == 6) {
                c_put(di, s->ann_start_samps[0], current_samp, s->out_ann, ANN_ERROR, "Error", "E");
            } else if (s->state > 4) {
                c_put(di, s->last_edge_samp, current_samp, s->out_ann, ANN_STOP, "Stop condition", "P");
            }
            s->state = 0;
        }
        
        s->last_edge_samp = current_samp; // Remember last edge
    }
}



static struct srd_decoder_option dp_aux_c_options[] = {
    {"invert", "dec_dp_aux_c_opt_invert", "Invert polarity", NULL, NULL},
};

static struct srd_c_decoder dp_aux_c_decoder = {
    .id = "dp_aux_c",
    .name = "DP AUX (Native C)",
    .longname = "DisplayPort AUX channel (DSView replication)",
    .desc = "Decodes DisplayPort AUX channel, matching AqDPAuxCh64 logic.",
    .license = "gplv2+",
    .channels = dp_aux_c_channels,
    .num_channels = 1,
    .options = dp_aux_c_options,
    .num_options = 1,
    .num_annotations = NUM_ANN,
    .ann_labels = dp_aux_c_ann_labels,
    .num_annotation_rows = 2,
    .annotation_rows = dp_aux_c_ann_rows,
    .inputs = dp_aux_c_inputs,
    .num_inputs = 1,
    .outputs = dp_aux_c_outputs,
    .num_outputs = 1,
    .reset = dp_aux_c_reset,
    .start = dp_aux_c_start,
    .decode = dp_aux_c_decode,
    .metadata = dp_aux_c_metadata,
    .destroy = dp_aux_c_destroy,
};

SRD_C_DECODER_EXPORT struct srd_c_decoder *srd_c_decoder_entry(void)
{
    GSList *inv_vals = NULL;
    inv_vals = g_slist_append(inv_vals, g_variant_new_boolean(FALSE));
    inv_vals = g_slist_append(inv_vals, g_variant_new_boolean(TRUE));
    dp_aux_c_options[0].def = g_variant_new_boolean(FALSE);
    dp_aux_c_options[0].values = inv_vals;

    return &dp_aux_c_decoder;
}

SRD_C_DECODER_EXPORT int srd_c_decoder_api_version(void)
{
    return SRD_C_DECODER_API_VERSION;
}
