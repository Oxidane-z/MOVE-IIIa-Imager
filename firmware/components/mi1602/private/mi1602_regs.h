/*
 * MI48xx register map and bit definitions.
 *
 * Source: pysenxor-1.6.7/senxor/mi48.py:99-228. Address values are bytes
 * on the I²C bus; status / mode values are bit masks for the respective
 * register reads.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Register addresses (subset; full set is in datasheet) ------- */

/* Temperature error compensation */
#define MI48_REG_COMP_CTRL        0x02  /* Temperature Error Compensation Control */

/* Identification / status */
#define MI48_REG_SENXOR_POWERUP   0xB0  /* Power up for SenXor */
#define MI48_REG_FRAME_MODE       0xB1  /* Capture/readout control */
#define MI48_REG_FW_VERSION_1     0xB2  /* Firmware major/minor */
#define MI48_REG_FW_VERSION_2     0xB3  /* Firmware build */
#define MI48_REG_FRAME_RATE       0xB4  /* FPS divisor (fps = 25.5 / div) */
#define MI48_REG_POWER_DOWN_1     0xB5
#define MI48_REG_STATUS           0xB6  /* RO status flags */
#define MI48_REG_POWER_DOWN_2     0xB7
#define MI48_REG_SENXOR_TYPE      0xBA  /* SenXor chip type */
#define MI48_REG_MODULE_TYPE      0xBB  /* Module/lens type */
#define MI48_REG_LUT_SELECT       0xBC

/* Calibration / sensitivity */
#define MI48_REG_OFFS_CALIB       0xC0
#define MI48_REG_CALIB_CTRL       0xC1
#define MI48_REG_SENS_FACTOR      0xC2  /* 0x64 = 1.00 */
#define MI48_REG_BLIND_CALIB      0xC5
#define MI48_REG_EMISSIVITY       0xCA  /* int 0..100 (%) — pysenxor uses 0x5F = 95% */
#define MI48_REG_OFFSET_CORR      0xCB  /* signed byte; ±12.7 K, unit 0.05 K (FW<4.2.3) or 0.10 K */
#define MI48_REG_OBJ_TEMP_FACTOR  0xCD  /* signed byte; unit 0.01 */

/* Filters */
#define MI48_REG_FILTER_1         0xD0  /* Temporal filter ctrl (0x00 off / 0x03 on) */
#define MI48_REG_FILTER_1_LSB     0xD1  /* Temporal filter strength LSB */
#define MI48_REG_FILTER_1_MSB     0xD2  /* Temporal filter strength MSB */
#define MI48_REG_FILTER_2         0x20  /* STARK ctrl (0x00 off / 0x03 on) */
#define MI48_REG_FILTER_3         0x30  /* Median ctrl (0x00 off / 0x01 on) */
#define MI48_REG_MMS_CTRL         0x25  /* Min/Max stabilization (0x00 off / 0x01 on) */
#define MI48_REG_MCU_TYPE         0x33
#define MI48_REG_FLASH_CTRL       0xD8

/* SenXor serial number (6 bytes, big-endian) */
#define MI48_REG_SENXOR_ID_0      0xE0
#define MI48_REG_SENXOR_ID_1      0xE1
#define MI48_REG_SENXOR_ID_2      0xE2
#define MI48_REG_SENXOR_ID_3      0xE3
#define MI48_REG_SENXOR_ID_4      0xE4
#define MI48_REG_SENXOR_ID_5      0xE5

/* ---------- STATUS register (0xB6) bit masks ---------------------------- */
#define MI48_STATUS_READOUT_TOO_SLOW  0x02
#define MI48_STATUS_SXIF_ERROR        0x04
#define MI48_STATUS_CAPTURE_ERROR     0x08
#define MI48_STATUS_DATA_READY        0x10
#define MI48_STATUS_BOOTING_UP        0x20

/* ---------- FRAME_MODE register (0xB1) bit masks ------------------------ */
#define MI48_MODE_GET_SINGLE_FRAME    0x01
#define MI48_MODE_CONTINUOUS_STREAM   0x02
#define MI48_MODE_NO_HEADER           0x20  /* skip 1-row header on SPI */
#define MI48_MODE_DATA_TYPE_ADC       0x80  /* raw counts instead of dK */

/* ---------- SPI frame header word indices (16-bit words, big-endian) ----
 * Refs pysenxor mi48.py:40-55. Only meaningful words are listed; the
 * header row carries ncols words total (= MI1602_FRAME_HDR_WORDS).      */
#define MI48_HDR_FRCNT       0   /* frame counter */
#define MI48_HDR_SXVDD       1   /* SenXor V_dd × 10000 */
#define MI48_HDR_SXTA        2   /* SenXor temperature × 100 (in K) */
#define MI48_HDR_TIME_LO     3   /* timestamp low 16 bits */
#define MI48_HDR_TIME_HI     4   /* timestamp high 16 bits */
#define MI48_HDR_MAXV        5   /* max pixel temp (dK) */
#define MI48_HDR_MINV        6   /* min pixel temp (dK) */
#define MI48_HDR_CRC         7   /* CRC-16/CCITT-FALSE over pixel payload */
#define MI48_HDR_IPLOCK1     10
#define MI48_HDR_IPLOCK2     11
#define MI48_HDR_NGTMEAN     22
#define MI48_HDR_NGTMIDDLE   23
#define MI48_HDR_UCMEAN      24
#define MI48_HDR_UCMIDDLE    25
#define MI48_HDR_NUNMASKED   26
#define MI48_HDR_NORM        27
#define MI48_HDR_UCMIN       28
#define MI48_HDR_UCMAX       29
#define MI48_HDR_RHO         30

/* ---------- Misc -------------------------------------------------------- */
#define MI48_KELVIN_0_X10    2731  /* deci-Kelvin offset for 0 °C: dK = 2731 */

#ifdef __cplusplus
}
#endif
