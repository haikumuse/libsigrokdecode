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
    int state;               // v17
    
    int v29;                 // prev pin value
    int v100;                // current pin value for next iter
    
    double v23;
    double v24;
    int v98;
    int v96;
    
    uint8_t v88;
    uint8_t v90;
    uint8_t v92;
    uint8_t v89;
    uint64_t v105;
    uint64_t v104;
    uint64_t v107;
    uint8_t v93;
    uint8_t v91;
    uint8_t v87;
    int v101;
    int v94;
    int v102;
    int v99;
    uint64_t v9;             // accumulated half UIs
    uint64_t v97;            // last edge sample (v97)
    
    uint32_t v111[52];
    uint32_t v112[52];
    
    uint64_t v114;
    uint64_t v115;
    uint64_t v116;
    uint64_t v117;
    uint64_t v118;
    uint64_t v119[4087];
    uint64_t v113[3];
    uint64_t v120[3];
    
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
    s->v102 = 1;
    s->v87 = 1;
    s->v93 = 1;
    s->v91 = 1;
    s->state = 0;
    s->v97 = 0;
    
    // To grab the initial pin state
    int initial_pin = c_pin(di, AUX);
    s->v100 = initial_pin;
    s->v29 = initial_pin;
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
    bool v61 = false;

    while (1) {
        int ret = c_wait(di, CW_E(AUX), CW_END);
        if (ret != SRD_OK) return;
        
        uint64_t v95 = di_samplenum(di);
        int current_pin = c_pin(di, AUX);
        
        uint64_t v28 = v95;
        double v34 = (double)(v28 - s->v97);
        
        s->v29 = s->v100;
        int v30 = current_pin;
        s->v100 = v30;
        
        int v31 = v30;
        if (!s->v102) {
            v31 = v30 ^ 1;
        }
        
        uint8_t v32 = 0;
        uint8_t v33 = 0;
        uint64_t v42 = 0;
        
        if (s->state > 3) {
            if (s->v29 == 0) {
                if (v30 == 1) {
                    v42 = s->v97;
                    v32 = 1;
                    goto LABEL_68;
                }
LABEL_67:
                v42 = s->v97;
                goto LABEL_68;
            }
LABEL_64:
            if (s->v29 == 1 && v30 == 0) {
                v42 = s->v97;
                v33 = 1;
                goto LABEL_68;
            }
            goto LABEL_67;
        }
        
        // Timeout logic
        if (v34 > (double)s->samplerate * 400000.0 / 1000000000.0) {
            s->v87 = 1;
        }
        
        uint64_t v36 = 0;
        uint64_t v35 = 0;
        if (s->v96 < 50 && s->v98 < 50) {
            v36 = s->v105;
            v35 = s->v104;
        } else {
            s->v96 = 0;
            s->v104 = 0;
            s->v98 = 0;
            s->v105 = 0;
            v35 = 0;
            v36 = 0;
        }
        
        if (s->v29 == 1) {
            if (s->v100) goto LABEL_67;
            
            double v37 = 0.0;
            for (uint64_t v38 = 0; v38 < v36; v38++) {
                v37 += (double)s->v111[v38];
            }
            s->v23 = (s->v98 > 0) ? (v37 / (double)s->v98) : 0.0;
            
            if (s->v23 <= 0.0) goto LABEL_43;
            if (s->v98 >= 8) {
                if (v34 >= s->v23 * 1.5) {
                    v42 = s->v97;
                    s->v98 = 0;
                    s->v96 = 0;
                    s->state = 4;
                    s->v104 = 0;
                    s->v115 = 0;
                    v33 = 1;
                    s->v105 = 0;
                    goto LABEL_68;
                }
LABEL_43:
                v42 = s->v97;
                s->v111[v36 + 1] = v28 - s->v97;
                s->v105 = v36 + 1;
                s->v98++;
                v33 = 1;
                goto LABEL_68;
            }
            
            if (v34 < s->v23 * 1.5 && s->v23 * 0.5 <= v34) {
                goto LABEL_43;
            }
            
            if (s->v91) {
                s->v91 = 0;
            }
            v28 = v95;
            v42 = s->v97;
            s->v96 = 0;
            s->v104 = 0;
            s->v111[0] = v95 - s->v97;
            s->v98 = 1;
            s->v105 = 1;
            v33 = 1;
        } else {
            if (s->v29 != 0) {
                // v29 is 0 or 1.
                v30 = s->v100;
                goto LABEL_64;
            }
            if (s->v100 != 1) goto LABEL_67;
            
            double v44 = 0.0;
            for (uint64_t v45 = 0; v45 < v35; v45++) {
                v44 += (double)s->v112[v45];
            }
            s->v24 = (s->v96 > 0) ? (v44 / (double)s->v96) : 0.0;
            
            if (s->v24 <= 0.0) goto LABEL_59;
            
            if (s->v96 >= 8) {
                if (v34 >= s->v24 * 1.5) {
                    v42 = s->v97;
                    s->v98 = 0;
                    s->v105 = 0;
                    s->v96 = 0;
                    s->v104 = 0;
                    s->state = 4;
                    s->v115 = 0;
                    v32 = 1;
                    goto LABEL_68;
                }
LABEL_59:
                v42 = s->v97;
                s->v112[v35 + 1] = v28 - s->v97;
                s->v104 = v35 + 1;
                s->v96++;
                v32 = 1;
                goto LABEL_68;
            }
            
            if (v34 < s->v24 * 1.5 && s->v24 * 0.5 <= v34) {
                goto LABEL_59;
            }
            
            if (s->v91) {
                s->v91 = 0;
            }
            v28 = v95;
            v42 = s->v97;
            s->v98 = 0;
            s->v105 = 0;
            s->v112[0] = v95 - s->v97;
            s->v96 = 1;
            s->v104 = 1;
            v32 = 1;
        }

LABEL_68:
        if (s->state <= 3) {
            s->v97 = v28;
            continue;
        }
        if (s->state >= 4096) s->state = 0;
        
        double v50 = (double)v28;
        double v51 = (s->v24 + s->v23) * 0.5;
        double v52 = (v50 - (double)v42) / v51;
        
        uint64_t v58 = 0;
        if (v52 >= 6.0) {
            v58 = (uint64_t)(v52 + 0.5);
        } else {
            int v53 = 1;
            double v54 = fabs(1.0 - v52);
            double v55 = fabs(2.0 - v52);
            if (v54 > v55) {
                v54 = v55;
                v53 = 2;
            }
            double v56 = fabs(3.0 - v52);
            if (v54 > v56) {
                v54 = v56;
                v53 = 3;
            }
            double v57 = fabs(4.0 - v52);
            if (v54 > v57) {
                v54 = v57;
                v53 = 4;
            }
            if (v54 > fabs(5.0 - v52)) {
                v53 = 5;
            }
            v58 = v53;
        }
        
        if (s->state == 4) {
            s->v9 = 0;
        }
        s->v9 += v58;
        

        switch (s->state) {
            case 4:
                memset(s->v113, 0, sizeof(s->v113));
                memset(s->v119, 0, sizeof(s->v119));
                s->v114 = 0; s->v115 = 0; s->v116 = 0; s->v117 = 0; s->v118 = 0;
                
                if (s->v87) {
                    v61 = (s->v9 == 2);
                } else {
                    v61 = (s->v9 == 5);
                }
                
                s->v9 = 0;
                if (v33) {
                    s->v102 = 0;
                } else {
                    int v62 = s->v102;
                    if (v32) v62 = 1;
                    s->v102 = v62;
                }
                
                if (s->v93) {
                    s->v87 = 1; // Simplification, DSView checks if option bit is set
                    s->v93 = 0;
                }
                s->state = 5;
                break;
                
            case 5:
                s->v9 -= 4;
                s->state = 6;
                s->v120[0] = v28; // Start of Command
                
                if (v61 && s->v9 == 0) {
                    s->v90 = 1;
                    goto LABEL_109;
                }
                s->v90 = 0;
                
                if (s->v9 == 1) {
                    if (s->v87) {
                        s->v88 = 1;
                        goto LABEL_114;
                    }
                } else if (s->v9 != 0) {
                    if (s->v9 == 4) s->v94 = 2;
LABEL_114:
                    if (s->v87) {
                        if (s->v94 != 0) s->v94 = 2;
                    }
                } else {
LABEL_109:
                    if (s->v87) {
                        s->v88 = 0;
                        goto LABEL_114;
                    }
                }
                s->v114 |= s->v94;
                break;
                
            case 6: {
                uint64_t v65 = 0;
                if (s->v87) {
                    if (s->v90 && s->v9 == 2) {
                        s->v92 = 1;
                        s->v9 = 0;
                        s->state = 7;
                        break;
                    }
                    s->v92 = 0;
                    if (s->v9 < 8 || s->v88) {
                        if (s->v9 < 16 || !s->v88) {
                            if ( (((s->v9 >> 63) ^ (s->v9 & 1)) - (s->v9 >> 63)) != 1 ) break;
                            uint64_t v65 = s->v88 ? 16 : 8;
                            s->v116 |= (uint64_t)v31 << ((v65 - s->v9) / 2);
                            break;
                        }
                        
                        // Output command
                        char cmd_str[128];
                        char cmd_short[32];
                        snprintf(cmd_str, sizeof(cmd_str), "Request: %s (0x%X)", get_aux_cmd_name((uint8_t)s->v116), (unsigned int)s->v116);
                        snprintf(cmd_short, sizeof(cmd_short), "CMD: %X", (unsigned int)s->v116);
                        c_put_v(di, s->v120[0], v28, s->out_ann, ANN_COMMAND, s->v116, "Command", cmd_str, cmd_short);
                        
                        s->v9 -= 16;
                        s->v99 = 0;
                        s->v107 = 0;
                        s->state = 9;
                        s->v120[1] = v28; // Data start sample
                        if (s->v9 == 1) s->v119[0] |= (uint64_t)v31 << 19;
                    } else {
                        // Output command
                        char cmd_str[128];
                        char cmd_short[32];
                        snprintf(cmd_str, sizeof(cmd_str), "Request: %s (0x%X)", get_aux_cmd_name((uint8_t)s->v116), (unsigned int)s->v116);
                        snprintf(cmd_short, sizeof(cmd_short), "CMD: %X", (unsigned int)s->v116);
                        c_put_v(di, s->v120[0], v28, s->out_ann, ANN_COMMAND, s->v116, "Command", cmd_str, cmd_short);
                        
                        s->v9 -= 8;
                        s->state = 7;
                        s->v120[0] = v28; // Address start sample
                        if (s->v9 == 1) s->v117 |= (uint64_t)v31 << 19;
                    }
                } else {
                    if (s->v9 >= 16) {
                        // Output reply command before transitioning
                        char cmd_str[128];
                        char cmd_short[32];
                        format_reply_cmd((uint8_t)s->v116, cmd_str, sizeof(cmd_str));
                        snprintf(cmd_short, sizeof(cmd_short), "CMD: %02X", (unsigned int)s->v116);
                        c_put_v(di, s->v120[0], v28, s->out_ann, ANN_COMMAND, s->v116, "Command", cmd_str, cmd_short);

                        s->v9 -= 16;
                        // Start Data phase directly for Reply
                        s->v107 = 0;
                        s->v99 = 0;
                        s->state = 9;
                        s->v120[1] = v28; // Data start sample
                        if (s->v9 == 1) s->v119[0] |= (uint64_t)v31 << 7;
                        break;
                    }
                    if ( (((s->v9 >> 63) ^ (s->v9 & 1)) - (s->v9 >> 63)) == 1 && s->v9 < 16) {
                        s->v116 |= (uint64_t)v31 << ((16 - s->v9) / 2);
                    }
                }
                break;
            }
                
            case 7:
                if (s->v92 && s->v9 == 4) {
                    s->v89 = 1;
                    s->state = 5;
                    s->v114 |= 2;
                    s->v94 = 2;
                    break;
                }
                s->v89 = 0;
                if (s->v9 < 40) {
                    if ( (((s->v9 >> 63) ^ (s->v9 & 1)) - (s->v9 >> 63)) == 1 ) {
                        s->v117 |= (uint64_t)v31 << ((40 - s->v9) / 2);
                    }
                } else {
                    char addr_str[128];
                    char addr_short[32];
                    const char *reg_name = get_dpcd_reg_name((uint32_t)s->v117);
                    if (reg_name) {
                        snprintf(addr_str, sizeof(addr_str), "ADDR: %05lX (%s)", (unsigned long)s->v117, reg_name);
                    } else {
                        snprintf(addr_str, sizeof(addr_str), "ADDR: %05lX", (unsigned long)s->v117);
                    }
                    snprintf(addr_short, sizeof(addr_short), "%05lX", (unsigned long)s->v117);
                    c_put_v(di, s->v120[0], v28, s->out_ann, ANN_ADDRESS, s->v117, "Address", addr_str, addr_short);
                    
                    s->v9 -= 40;
                    s->state = 8;
                    s->v120[0] = v28; // Length start sample
                    if (s->v9 == 1) s->v118 |= (uint64_t)v31 << 7;
                }
                break;
                
            case 8:
                if (s->v9 >= 16) {
                    char len_str[64];
                    char len_short[32];
                    snprintf(len_str, sizeof(len_str), "Length: %lu", (unsigned long)s->v118 + 1); // Length is 0-based in protocol
                    snprintf(len_short, sizeof(len_short), "LEN: %02lX", (unsigned long)s->v118);
                    c_put_v(di, s->v120[0], v28, s->out_ann, ANN_LENGTH, s->v118, "Length", len_str, len_short);
                    
                    s->v9 -= 16;
                    // Start Data phase
                    s->v107 = 0;
                    s->v99 = 0;
                    s->state = 9;
                    s->v120[1] = v28; // Data start sample
                    if (s->v9 == 1) s->v119[0] |= (uint64_t)v31 << 7;
                    break;
                }
                if ( (((s->v9 >> 63) ^ (s->v9 & 1)) - (s->v9 >> 63)) == 1 ) {
                    s->v118 |= (uint64_t)v31 << ((16 - s->v9) / 2);
                }
                break;
                
            default:
                if (s->state == 16 * s->v99 + 9) {
                    if (s->v9 < 16) {
                        if ( (((s->v9 >> 63) ^ (s->v9 & 1)) - (s->v9 >> 63)) == 1 ) {
                            // Wait, the original code had v113[v17], which maps to v119[16 * v99]
                            s->v119[16 * s->v99] |= (uint64_t)v31 << ((16 - s->v9) / 2);
                        }
                    } else {
                        // Byte finished
                        char data_str[32];
                        snprintf(data_str, sizeof(data_str), "D: %02X", (unsigned int)s->v119[16 * s->v99]);
                        c_put_v(di, s->v120[9 + s->v99], v28, s->out_ann, ANN_DATA, s->v119[16 * s->v99], "Data", data_str);
                        s->v9 -= 16;
                        s->v99++;
                        s->v107++;
                        s->state = 16 * s->v99 + 9;
                        s->v120[9 + s->v99] = v28;
                        if (s->v9 == 1) s->v119[16 * s->v107] |= (uint64_t)v31 << 7;
                    }
                }
                break;
        }
        
        // Post-state-machine edge-duration validation
        bool abort = false;
        
        if (s->v89) {
            s->state = 5;
            abort = true;
        } else if (s->state != 0) {
            if (s->state == 4 || s->state == 5) {
                if (v58 > 8) abort = true;
            } else if (s->state == 6) {
                if (v58 > 41) abort = true;
                if (!abort && v58 > 2 && s->v9 > 1) abort = true;
            } else { // state > 6
                if (v58 > 41) abort = true;
                if (!abort && v58 > 2) abort = true;
            }
            
            if (s->v99 > 255) {
                abort = true;
            }
        }
        
        if (abort) {
            if (s->v87 && s->state == 6) {
                c_put(di, s->v120[0], v28, s->out_ann, ANN_ERROR, "Error", "E");
            } else if (s->state > 4) {
                c_put(di, s->v97, v28, s->out_ann, ANN_STOP, "Stop condition", "P");
            }
            s->state = 0;
        }
        
        s->v97 = v28; // Remember last edge
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
