/*
 * SC850SL register reference, distilled from datasheet V1.10 (2024-09-04).
 * 16-bit register address, 8-bit data, I2C.
 *
 * Slave address: SID pin = 0 -> 0x30 (7-bit) / 0x60 wr / 0x61 rd
 *                SID pin = 1 -> 0x32 (7-bit) / 0x64 wr / 0x65 rd
 */
#pragma once

#define SC850SL_I2C_ADDR_SID0       0x30
#define SC850SL_I2C_ADDR_SID1       0x32

/* Identification (read back to verify) ---------------------------------- */
#define SC850SL_REG_CHIP_ID_HI      0x3107   /* expected 0x9d */
#define SC850SL_REG_CHIP_ID_LO      0x3108   /* expected 0x1e */

/* Stream / power control ------------------------------------------------ */
#define SC850SL_REG_STREAM          0x0100   /* bit0: 1=stream on, 0=off+sleep */
#define SC850SL_REG_SLEEP_AUX       0x302c   /* 0x0f to fully enter sleep, 0x00 active */
#define SC850SL_REG_SOFT_RESET      0x0103   /* bit0: 1=reset (hold >=150ns) */

/* MIPI configuration ---------------------------------------------------- */
#define SC850SL_REG_MIPI_LANE_NUM   0x3018   /* default 0xf2 -> bit[7:5]=7 = 2x4L
                                                bit[7:5]: 0=1L 1=2L 3=4L 7=2x4L
                                                ** MUST be overwritten for 2-lane:
                                                ** read-modify-write to 0x20 | (old & 0x1F) */
#define SC850SL_REG_MIPI_2x4_FMT    0x3033   /* default 0xa2; bit[1]: 2x4 even-data fmt */
#define SC850SL_REG_MIPI_DATA_MODE  0x3031   /* default 0x0a; bit[3:0]: 8=RAW8 a=RAW10 c=RAW12
                                                ** This sets the MIPI DT (0x2A/0x2B/0x2C) */
#define SC850SL_REG_MIPI_PHY_MODE   0x3037   /* default 0x00; bit[6:5]: 0=8b 1=10b 2=12b 3=16b
                                                ** Must match data mode (e.g. RAW10 -> 0x20) */
#define SC850SL_REG_MIPI0_LP_DRV    0x3650   /* default 0xc0; bit[1:0] */
#define SC850SL_REG_MIPI1_LP_DRV    0x3658   /* default 0x31; bit[1:0] */
#define SC850SL_REG_MIPI0_HS_DRV    0x3651   /* default 0x7d; bit[3:0] */
#define SC850SL_REG_MIPI1_HS_DRV    0x3659   /* default 0x7d; bit[3:0] */
/* Per-lane skew adjust (40 ps/step) — only touch if eye is bad:
 *   Lane0/1 delay -> 0x3652, Lane2/3 -> 0x3653, Clk0 -> 0x3654
 *   Lane4/5       -> 0x365a, Lane6/7 -> 0x365b, Clk1 -> 0x365c            */
/* Max per-lane HS rate: 1.5 Gbps                                          */

/* MIPI CSI-2 Data Type values the SC850SL emits (VC always 0 by default):
 *   0x00=frame start, 0x01=frame end, 0x02=line start, 0x03=line end
 *   0x2A=RAW8 payload, 0x2B=RAW10 payload, 0x2C=RAW12 payload             */

/* Output window / image size ------------------------------------------- */
#define SC850SL_REG_OUT_WIDTH_HI    0x3208   /* default 0x0f00 = 3840 */
#define SC850SL_REG_OUT_WIDTH_LO    0x3209
#define SC850SL_REG_OUT_HEIGHT_HI   0x320a   /* default 0x0870 = 2160 */
#define SC850SL_REG_OUT_HEIGHT_LO  0x320b
#define SC850SL_REG_X_START_HI      0x3210   /* default 0x0010 */
#define SC850SL_REG_X_START_LO      0x3211
#define SC850SL_REG_Y_START_HI      0x3212   /* default 0x0008 */
#define SC850SL_REG_Y_START_LO      0x3213

/* Frame rate: VTS lines per frame -------------------------------------- */
#define SC850SL_REG_VTS_HI          0x320e   /* bit[6:0] high */
#define SC850SL_REG_VTS_LO          0x320f
/* fps = pixclk / (HTS * VTS); default VTS=0x08ca for 30 fps linear */

/* Mirror / flip -------------------------------------------------------- */
#define SC850SL_REG_MIRROR_FLIP     0x3221
/* bit[2:1] mirror (00=off, 11=on), bit[6:5] flip */

/* AEC / AGC: see datasheet tables 2-4..2-8 (long; not duplicated here) - */

/* Test pattern --------------------------------------------------------- */
#define SC850SL_REG_TEST_PATTERN    0x4501   /* bit3: 1=incremental test, 0=normal */

/* DPC (defective pixel correction) ------------------------------------- */
#define SC850SL_REG_DPC_EN_A        0x5000   /* bit2=white, bit1=black */
#define SC850SL_REG_DPC_EN_B        0x5002   /* mirror of 0x5000 */

/* Notes ----------------------------------------------------------------
 *
 * 1) The full SC850SL register init table (PLL, analog bias, ISP-related
 *    settings) is NOT in the datasheet. SmartSens distributes it under
 *    NDA only. Without it, the sensor will not produce a usable image.
 *
 * 2) 2x2 binning is documented only inside the "ColGain HDR" feature,
 *    not as a standalone full-array binning mode. Most likely the chip
 *    supports binning through a complete mode-table swap rather than a
 *    single register bit. Plan to obtain a 2-lane 1080p binned mode
 *    table from SmartSens FAE.
 *
 * 3) Power (datasheet section 3):
 *      60 fps active: 447.8 mW typ (511 mW max)
 *      30 fps active: 394.7 mW typ (493 mW max)
 *      Sleep (0x0100=0, 0x302c=0x0f): not characterized, expect <10 mW
 *      XSHUTDN low: near zero. Keep XSHUTDN low between captures.
 */
