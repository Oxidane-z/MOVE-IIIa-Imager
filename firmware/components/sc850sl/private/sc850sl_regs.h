/*
 * SC850SL register address map (private to the driver).
 * Sources: SmartSens datasheet V1.10 + M5Stack libsns_sc850sl.so .data dump.
 * See ../../../PROJECT_MEMORY.md section 3 for the full story.
 */
#pragma once

#include <stdint.h>

/* Init-table record format. Shared by all four sc850sl_table_*.c arrays. */
typedef struct {
    uint16_t reg;
    uint8_t  val;
} sc850sl_reg_pair_t;

/* Tables exported from init_tables/sc850sl_table_N.c. */
extern const sc850sl_reg_pair_t sc850sl_table_0[];
extern const size_t             sc850sl_table_0_len;
extern const sc850sl_reg_pair_t sc850sl_table_1[];
extern const size_t             sc850sl_table_1_len;
extern const sc850sl_reg_pair_t sc850sl_table_2[];
extern const size_t             sc850sl_table_2_len;
extern const sc850sl_reg_pair_t sc850sl_table_3[];
extern const size_t             sc850sl_table_3_len;

/* ---- ID / lifecycle ---------------------------------------------------- */
#define SC850SL_REG_SOFT_RESET     0x0103   /* bit0: 1 = reset (hold ≥150 ns) */
#define SC850SL_REG_STREAM         0x0100   /* bit0: 1 = stream on            */
#define SC850SL_REG_SLEEP_AUX      0x302c   /* 0x0f → enter low-power sleep   */
#define SC850SL_REG_CHIP_ID_HI     0x3107   /* expect 0x9d                    */
#define SC850SL_REG_CHIP_ID_LO     0x3108   /* expect 0x1e                    */

/* ---- MIPI -------------------------------------------------------------- */
#define SC850SL_REG_MIPI_LANE      0x3018   /* bit[7:5] lane count            */
#define SC850SL_REG_MIPI_DATA_FMT  0x3031   /* 0x0a=RAW10, 0x0c=RAW12         */
#define SC850SL_REG_MIPI_PHY_BITS  0x3037   /* leave at 0 (datasheet decode bogus) */
#define SC850SL_REG_MIPI_2x4_FMT   0x3033

/* ---- Frame timing ------------------------------------------------------ */
#define SC850SL_REG_HTS_HI         0x320c
#define SC850SL_REG_HTS_LO         0x320d
#define SC850SL_REG_VTS_HI         0x320e   /* bit[6:0] high                  */
#define SC850SL_REG_VTS_LO         0x320f

/* ---- Output window ----------------------------------------------------- */
#define SC850SL_REG_OUT_W_HI       0x3208
#define SC850SL_REG_OUT_W_LO       0x3209
#define SC850SL_REG_OUT_H_HI       0x320a
#define SC850SL_REG_OUT_H_LO       0x320b
#define SC850SL_REG_OUT_X_HI       0x3210
#define SC850SL_REG_OUT_X_LO       0x3211
#define SC850SL_REG_OUT_Y_HI       0x3212
#define SC850SL_REG_OUT_Y_LO       0x3213

/* ---- Mirror / flip ----------------------------------------------------- */
#define SC850SL_REG_MIRROR_FLIP    0x3221   /* bit[2:1] mirror, bit[6:5] flip */

/* ---- Test pattern ------------------------------------------------------ */
#define SC850SL_REG_TEST_PATTERN   0x4501   /* bit3: 1 = incremental pattern  */

/* ---- AEC / AGC (subset — full set in datasheet 2.3) ------------------- */
#define SC850SL_REG_EXP_HI         0x3e00   /* 24-bit exposure across 3e00/01/02 */
#define SC850SL_REG_EXP_MID        0x3e01
#define SC850SL_REG_EXP_LO         0x3e02
#define SC850SL_REG_DGAIN          0x3e06
#define SC850SL_REG_AGAIN_COARSE   0x3e08
#define SC850SL_REG_AGAIN_FINE     0x3e09

/* ---- DPC --------------------------------------------------------------- */
#define SC850SL_REG_DPC_EN_A       0x5000   /* bit2=white px cancel, bit1=black */
#define SC850SL_REG_DPC_EN_B       0x5002
