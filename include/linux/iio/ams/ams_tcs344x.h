/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
 * USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
 * EXCLUDED.                                                                 *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */

/*! \file
 * \brief Device driver for monitoring ambient light intensity in (lux)
 * proximity detection (prox), Gesture, and Beam functionality within the
 * AMS-TAOS TCS344x family of devices.
 */

#ifndef __TCS344x_H
#define __TCS344x_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#ifdef CONFIG_QUALCOMM_AP
#include <linux/sensors.h>
#endif

#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) { \
                pr_info("%s: Mutex Lock\n", __func__); \
                mutex_lock(m); \
        }
#define AMS_MUTEX_UNLOCK(m) { \
                pr_info("%s: Mutex Unlock\n", __func__); \
                mutex_unlock(m); \
        }
#else
#define AMS_MUTEX_LOCK(m) { \
                mutex_lock(m); \
        }
#define AMS_MUTEX_UNLOCK(m) { \
                mutex_unlock(m); \
        }
#endif

// fraction out of 10
#define TENTH_FRACTION_OF_VAL(v, x) ({ \
  int __frac = v; \
  if (((x) > 0) && ((x) < 10)) __frac = (__frac*(x)) / 10 ; \
  __frac; \
})

// these are the numbers used for the one-shot spectral analysis. The spectral library processes 10 32 bit
// sensor values F1..F8, NIR and CLR. The Channel output data is 6 16 bit values at the end of each spectral
// cycle. For a spectral one-shot, one page or 4096 bytes or 1024 32 bit values are written. The continuous mode
// will continue to the next page until the user stops or the device is powered off

#define MAX_TH_VALUE 			65535
#define MAX_ASTEP 				65535
#define MAX_ATIME 				255
#define MAX_PERSIST 			15
#define MAX_WTIME 				255
#define MAX_AZCONFIG 			255
#define MAX_AGAIN 				9
#define MAX_FD_TIME 			1023
#define MAX_FD_CFG 				255
#define MAX_INTENAB 			255
#define MAX_REG_VAL 			255
/* ALS Gain uses 5 bits with a max of 256x which is 9 - data sheet */
#define MAX_AGAIN 				9
#define ONE_PAGE_CHANNEL_DATA 	1024
// The spectral library expects rows of sensor data with each row being 10 32-bit values
#define SENSOR_ROW_SIZE 		10
// Two consecutive spectral integration cycles yields 12 16-bit values , F1..F4, NIR and CLR and F5..F8, NIR and CLR
#define SPEC_CYCLE_SIZE 		12
//Fifo
#define MAX_FIFO_LEN 			512
#define MAX_FIFO_REPORT_SZ 		8192
// Mode string length
#define MODE_STR_LEN 			30
// Map linux events to TCS344x events
#define FIFO_DEPTH_EVENT ABS_VOLUME
#define SINT_EVENT       ABS_DISTANCE

#define ENABLE_FIFO_WRITE_FD 	(0x01 << 7)
#define DISABLE_FIFO_WRITE_FD 	(0x00<< 7)
#define FDCFG0_REV              (0x21)
#define NUM_FD_SAMPLES_512		(0x30)		// 512 FD samples to be collected
#define NUM_FD_SAMPLES_1024		(0x31)		// 1024 FD samples to be collected
#define NUM_FD_SAMPLES_256		(0x28)		// 256 FD samples to be collected
#define NUM_FD_SAMPLES_128		(0x20)		// 128 FD samples to be collected
#define NUM_FD_SAMPLES_64		(0x18)		// 64 FD samples to be collected
#define NUM_FD_SAMPLES_32		(0x10)		// 32 FD samples to be collected
#define NUM_FD_SAMPLES_16		(0x01)		// 16 FD samples to be collected
#define FD_COMPARE_LIMIT		(0x01)		// Compare limit for frequency detection
										    // 1 => 0.75 = ((1/2) + (1/4))
#define DEFAULT_FD_TIME_1		(0x5E)		// 1 ms
#define DEFAULT_FD_TIME_2		(0x21)		// 1ms, default fd_gain = 8x
#define DEFAULT_FIFO_MAP		(0x00)

enum tcs344x_regs {
    TCS344x_REGADDR_RAM_START 				= 0x00,

    TCS344x_REGADDR_P2RAM_OTP_1 			= 0x01,	// to set the I2C operating voltage
    TCS344x_REGADDR_P2RAM_OTP_2 			= 0x02, // to set the I2C slave address	
    TCS344x_REGADDR_P2RAM_OTP_3 			= 0x03, 
    TCS344x_REGADDR_CHIP_SERIAL_NUMBER_1 	= 0x04, // Chip serial number low byte
    TCS344x_REGADDR_CHIP_SERIAL_NUMBER_2 	= 0x05, // Chip serial number high byte

    TCS344x_REGADDR_CHIP_ID 		= 0x0B,
    TCS344x_REGADDR_AOFFSET0_H 		= 0x70,		//AOFFSET0_H
    TCS344x_REGADDR_AOFFSET1_H 		= 0x71,		//AOFFSET1_H
    TCS344x_REGADDR_AOFFSET2_H 		= 0x72,		//AOFFSET2_H
    TCS344x_REGADDR_AOFFSET3_H 		= 0x73,		//AOFFSET3_H
    TCS344x_REGADDR_AOFFSET4_H 		= 0x74,		//AOFFSET4_H
    TCS344x_REGADDR_AOFFSET5_H 		= 0x75,		//AOFFSET5_H
    //TCS344x_REGADDR_EDGE 			= 0x72,
    TCS344x_REGADDR_GPIO 			= 0x6B,
    TCS344x_REGADDR_LED 			= 0xCD,		//PCFG2
    TCS344x_REGADDR_ENABLE 			= 0x80,
    //TCS344x_REGADDR_ITIME 		= 0x63,
    TCS344x_REGADDR_CALIBSTAT 		= 0x63,		//CALIBSTAT
    TCS344x_REGADDR_ATIME 			= 0x81,
    TCS344x_REGADDR_PTIME 			= 0x82,
    TCS344x_REGADDR_WTIME 			= 0x83,
    TCS344x_REGADDR_SPEC_L 			= 0x84,		//AILTL
    TCS344x_REGADDR_SPEC_H 			= 0x86,		//AIHTL

    TCS344x_REGADDR_PILT0L			= 0x88,
    TCS344x_REGADDR_PILT0H			= 0x89,
	TCS344x_REGADDR_PILT1L			= 0x8A,
	TCS344x_REGADDR_PILT1H			= 0x8B,
	TCS344x_REGADDR_PIHT0L			= 0x8C,
	TCS344x_REGADDR_PIHT0H			= 0x8D,
	TCS344x_REGADDR_PIHT1L			= 0x8E,
	TCS344x_REGADDR_PIHT1H			= 0x8F,

    TCS344x_REGADDR_AUXID 			= 0x58,
    TCS344x_REGADDR_REVID 			= 0x59,
    TCS344x_REGADDR_ID 				= 0x5A,

    TCS344x_REGADDR_STATUS_2 		= 0x90,
    TCS344x_REGADDR_STATUS_3		= 0x91,
    TCS344x_REGADDR_STATUS_6 		= 0x92,
    TCS344x_REGADDR_STATUS 			= 0x93,
    TCS344x_REGADDR_ASTATUS 		= 0x94,
    TCS344x_REGADDR_CH0_DATA 		= 0x95,		//ADATA0L
    TCS344x_REGADDR_CH1_DATA 		= 0x97,		//ADATA1L
    TCS344x_REGADDR_CH2_DATA 		= 0x99,		//ADATA2L
    TCS344x_REGADDR_CH3_DATA 		= 0x9B,		//ADATA3L
    TCS344x_REGADDR_CH4_DATA 		= 0x9D,		//ADATA4L
    TCS344x_REGADDR_CH5_DATA 		= 0x9F,		//ADATA5L
    TCS344x_REGADDR_CH6_DATA 		= 0xA1,		//ADATA6L
    TCS344x_REGADDR_CH7_DATA 		= 0xA3,		//ADATA7L
    TCS344x_REGADDR_CH8_DATA 		= 0xA5,		//ADATA8L
    TCS344x_REGADDR_CH9_DATA 		= 0xA7,		//ADATA9L
    TCS344x_REGADDR_CH10_DATA 		= 0xA9,		//ADATA10L
    TCS344x_REGADDR_CH11_DATA 		= 0xAB,		//ADATA11L
    TCS344x_REGADDR_CH12_DATA 		= 0xAD,		//ADATA12L
    TCS344x_REGADDR_CH13_DATA 		= 0xAF,		//ADATA13L
    TCS344x_REGADDR_CH14_DATA 		= 0xB1,		//ADATA14L
    TCS344x_REGADDR_CH15_DATA 		= 0xB3,		//ADATA15L
    TCS344x_REGADDR_CH16_DATA 		= 0xB5,		//ADATA16L
    TCS344x_REGADDR_CH17_DATA 		= 0xB7,		//ADATA17L
    TCS344x_REGADDR_PDATA0L			= 0xB9,
	TCS344x_REGADDR_STATUS_5 		= 0xBB,
    TCS344x_REGADDR_STATUS_4		= 0xBC,
    TCS344x_REGADDR_ASTATUS6 		= 0xBD,
	TCS344x_REGADDR_RAM_BIST		= 0xBE,


    TCS344x_REGADDR_AOFFSET1_L 		= 0xC0,		//AOFFSET5_L
    TCS344x_REGADDR_AOFFSET2_L 		= 0xC1,		//AOFFSET5_L
    TCS344x_REGADDR_AOFFSET3_L 		= 0xC2,		//AOFFSET5_L
    TCS344x_REGADDR_AOFFSET4_L 		= 0xC3,		//AOFFSET5_L
    TCS344x_REGADDR_AOFFSET5_L 		= 0xC4,		//AOFFSET5_L
    TCS344x_REGADDR_AOFFSET6_L 		= 0xC5,		//AOFFSET5_L

    TCS344x_REGADDR_CFG_0 			= 0xBF,
    TCS344x_REGADDR_CFG_1 			= 0xC6,
    TCS344x_REGADDR_CFG_3 			= 0xC7,
    TCS344x_REGADDR_CFG_4 			= 0x62,
    TCS344x_REGADDR_CFG_5 			= 0xC8,
    TCS344x_REGADDR_CFG_6 			= 0xF5,
    TCS344x_REGADDR_CFG_8			= 0xC9,
    TCS344x_REGADDR_CFG_9 			= 0xCA,
    TCS344x_REGADDR_CFG_10 			= 0x65,
    TCS344x_REGADDR_CFG_11 			= 0xCB,
    TCS344x_REGADDR_PCFG_1 			= 0xCC,
    TCS344x_REGADDR_CFG_12 			= 0x66,
    TCS344x_REGADDR_CFG_13 			= 0x67,
    TCS344x_REGADDR_CFG_14 			= 0x68,
    TCS344x_REGADDR_CFG_15 			= 0x40,
    TCS344x_REGADDR_CFG_16 			= 0x45,
    TCS344x_REGADDR_CFG_17 			= 0x46,
    TCS344x_REGADDR_CFG_18 			= 0x47,
    TCS344x_REGADDR_CFG_19 			= 0x49,
    TCS344x_REGADDR_CFG_20 			= 0xD6,
    TCS344x_REGADDR_CFG_21 			= 0x4A,
    TCS344x_REGADDR_CFG_22 			= 0x4B,
    TCS344x_REGADDR_PERS 			= 0xCF,		//ALS and PROXIMITY Persistence value

    TCS344x_REGADDR_GPIO2 			= 0x6B,

	TCS344x_REGADDR_ADC_CFG			= 0xD0,
	TCS344x_REGADDR_POFFSET0_L		= 0xD1,
	TCS344x_REGADDR_POFFSET0_H		= 0xD2,
	TCS344x_REGADDR_IDAC			= 0xD3,
    TCS344x_REGADDR_ASTEP_L			= 0xD4,
    TCS344x_REGADDR_ASTEP_H			= 0xD5,
    TCS344x_REGADDR_AGC_GAIN_MAX 	= 0xD7,
    TCS344x_REGADDR_PBSLN0_MEAS_L 	= 0xD8,
    TCS344x_REGADDR_PBSLN0_MEAS_H 	= 0xD9,
    TCS344x_REGADDR_PBSLN0_L	 	= 0xDA,
    TCS344x_REGADDR_PBSLN0_H		= 0xDB,
    TCS344x_REGADDR_NR_PRX_PULSES 	= 0xDC,
    TCS344x_REGADDR_ALS_CHANNEL_CTRL= 0xDD,

    TCS344x_REGADDR_AZCONFIG 		= 0xDE,
    TCS344x_REGADDR_FD_CFG_0 		= 0xDF,
    TCS344x_REGADDR_FD_CFG_1 		= 0xE0,		//For setting FD TIME
    TCS344x_REGADDR_FD_CFG_2 		= 0xE1,		//FD filter size
    TCS344x_REGADDR_FD_CFG_3 		= 0xE2,		
    //TCS344x_REGADDR_FD_TIME_L 		= 0xD8,
    //TCS344x_REGADDR_FD_TIME_H 		= 0xDA,
    TCS344x_REGADDR_FLICKR_STATUS 	= 0xE3,
    TCS344x_REGADDR_CHAIN_CMD	 	= 0xE4,
    TCS344x_REGADDR_CHAINL			= 0xE5,
    TCS344x_REGADDR_CHAINH			= 0xE6,
    TCS344x_REGADDR_CHAIN_SMUX		= 0xE7,
    TCS344x_REGADDR_ICONFIG			= 0xE8,
    TCS344x_REGADDR_ICONFIG_2		= 0xE9,
    TCS344x_REGADDR_ISNL 			= 0xEA,
    TCS344x_REGADDR_ISOFF 			= 0xEB,
    TCS344x_REGADDR_IPNL 			= 0xEC,
    TCS344x_REGADDR_IPOFF		 	= 0xED,
    TCS344x_REGADDR_ISLEN		 	= 0xEE,
    TCS344x_REGADDR_ISTART		 	= 0xEF,


	TCS344x_REGADDR_ALS_PHASES_CTRL = 0xF0,

	TCS344x_REGADDR_CALIB 			= 0xF1,
	TCS344x_REGADDR_CALIB_CFG_0		= 0xF2,
	TCS344x_REGADDR_CALIB_CFG_1		= 0xF3,
	TCS344x_REGADDR_CALIB_CFG_2		= 0xF4,

    TCS344x_REGADDR_INTENAB 		= 0xF9,
    TCS344x_REGADDR_CONTROL 		= 0xFA,
    TCS344x_REGADDR_FIFO_MAP 		= 0xFC,
    TCS344x_REGADDR_FIFO_LVL 		= 0xFD,
    TCS344x_REGADDR_FDATA_L 		= 0xFE,
    TCS344x_REGADDR_FDATA_H 		= 0xFF
};

#define STR1_MAX 16
#define STR2_MAX 256
#define MAX_REGS 256
struct device;

enum tcs344x_pwr_state {
        POWER_ON, POWER_OFF, POWER_STANDBY,
};

enum tcs344x_i2C_operating_voltage{
		use_i2c_reg, vol_1_8, vol_1_2, auto_mode,
};

/* pldrive */
#define PDRIVE_MA(p)   (((u8)((p) / 6) - 1) & 0x1f)
#define P_TIME_US(p)   ((((p) / 88) - 1.0) + 0.5)
#define PRX_PERSIST(p) (((p) & 0xf) << 4)

#define MULT_MS_US 				1000
#define INTEGRATION_CYCLE 		2800
#define AW_TIME_MS(p)  			((((p) * 1000) +\
								(INTEGRATION_CYCLE - 1)) / INTEGRATION_CYCLE)
#define ALS_PERSIST(p) (((p) & 0xf) << 0)

/* lux */
#define INDOOR_LUX_TRIGGER		6000
#define OUTDOOR_LUX_TRIGGER		10000
#define TCS344x_MAX_LUX			0xffff
#define TCS344x_MAX_ALS_VALUE	0xffff
#define TCS344x_MIN_ALS_VALUE	1

/**
 * struct aos_sensor_item_data - stores the data of an item which is processed by the chiplib
 * @p_data: - buffer holding the data of the item
 * @size: - size of the buffer
 */
struct aos_sensor_item_data {
    uint8_t *p_data;
    uint32_t size;
};

/**
 * struct aos_sensor_item - defines an item and its data which is processed by the chiplib
 * @item_id: - unique ID of the item for identification (must be equal to the item ids processed by the chiplib)
 * @data: - the data of the item
 */
struct aos_sensor_item {
    uint32_t item_id;
    struct aos_sensor_item_data data;
};

/**
 * struct aos_temp_node - A temperature node to read/write from
 * @id: - unique ID of the node for identification (shall be equal to the id processed by the chiplib)
 * @value: - the temperature of the node to read/write
 */
struct aos_temp_node {
    uint16_t id;
    int32_t value;
};

/**
 * Enumerations for setting the I2C working voltage
 */
enum tcs344x_i2C_voltage{
		TCS344x_USE_I2C_REG_VALUE = (0),
		TCS344x_I2C_1_8_VOLTAGE   = (1 << 4),
		TCS344x_I2C_1_2_VOLTAGE   = (1 << 5),
		TCS344x_I2C_AUTO		  = (3 << 5),
};

/**
 *Enumeration for setting the I2C Slave address
 */
enum tcs344x_select_i2c_addr{
		TCS344x_addr_59 	= (0),
		TCS344x_addr_29 	= (1 << 5),
		TCS344x_addr_49 	= (1 << 6),
		TCS344x_addr_39 	= (3 << 6),
};	

enum tcs344x_status{
        TCS344x_SINT = (1 << 0),
        TCS344x_CINT = (1 << 1),
        TCS344x_FINT = (1 << 2),
        TCS344x_AINT = (1 << 3),
        TCS344x_ASAT = (1 << 7)
};

enum tcs344x_auxid{
        TCS344x_AUXID = (1 << 1),
        TCS344x_OTP1 = (1 << 7)
};

enum tcs344x__reg {
        TCS344x_MASK_START_OFFSET_CALIB = 0x01,
        TCS344x_SHIFT_START_OFFSET_CALIB = 0,
        /*Note - MASK_AGAIN is for AGC_GAIN_MAX */
        TCS344x_MASK_AGC_AGAIN = 0x0f,
        TCS344x_MASK_AGAIN = 0x1f,
        TCS344x_MASK_CFG1 = 0x1f,
        TCS344x_MASK_FDTIME2 = 0x03,
        TCS344x_SHIFT_AGAIN = 0,

        TCS344x_MASK_APERS = 0x0f,
        TCS344x_SHIFT_APERS = 0,

        TCS344x_MASK_WLONG = 0x04,
        TCS344x_MASK_LOW_POWER = (1 << 5),
        TCS344x_SHIFT_WLONG = 2,
};

enum tcs344x_en_reg {
        TCS344x_PON = (1 << 0),
        TCS344x_AEN = (1 << 1),
        TCS344x_WEN = (1 << 3),
        TCS344x_MUXEN = (1 << 4),
        TCS344x_FDEN = (1 << 6),
        TCS344x_FD_ALS = (TCS344x_FDEN | TCS344x_MUXEN | TCS344x_AEN | TCS344x_WEN | TCS344x_PON),
        TCS344x_MUX_ON = (TCS344x_MUXEN | TCS344x_PON),
        TCS344x_MUX_ALS = (TCS344x_MUXEN | TCS344x_AEN | TCS344x_WEN | TCS344x_PON)
};

enum tcs344x_cfg_reg {
        TCS344x_SPECTRAL_MODE,
        TCS344x_SYNS_MODE,
        TCS344x_MODE_RESERVED,
        TCS344x_SYND_MODE,
        TCS344x_SYNC_INT = (1 << 2),
        TCS344x_LED_SEL = (1 << 3)
};

enum tcs344x_gpio_reg {
        TCS344x_PD_INT = (1 << 0),
        TCS344x_PD_GPIO = (1 << 1)
};

enum tcs344x_gpio2_reg {
        TCS344x_GPIO_OUT = (1 << 1),
        TCS344x_GPIO_IN_EN = (1 << 2),
        TCS344x_GPIO_INV = (1 << 3)
};

enum tcs344x_intenab_reg {
        TCS344x_INTENAB_SIEN = (1 << 0),
        TCS344x_INTENAB_FIEN = (1 << 2),
        TCS344x_INTENAB_AIEN = (1 << 3),
        TCS344x_INTENAB_ASIEN = (1 << 7),
        TCS344x_ANY_INTENAB = (TCS344x_INTENAB_SIEN | TCS344x_INTENAB_FIEN | TCS344x_INTENAB_AIEN | TCS344x_INTENAB_ASIEN)
};

enum tcs344x_cfg8_reg {
        TCS344x_SP_AGC = (1 << 2),
        TCS344x_FD_AGC = (1 << 3)
};

enum tcs344x_control_reg {
        TCS344x_CLEAR_SAI_ACT = (1 << 0),
        TCS344x_FIFO_CLR = (1 << 1),
        TCS344x_SP_MAN_AZ = (1 << 2)
};

enum tcs344x_status2_reg {
        TCS344x_FDSTAT_DIG = (1 << 0),
        TCS344x_FDSTAT_ANA = (1 << 1),
        TCS344x_ANA_SAT = (1 << 3),
        TCS344x_DIG_SAT = (1 << 4),
        TCS344x_AVALID = (1 << 6)
};

enum tcs344x_fifomap {
        TCS344x_CH0 = (1 << 1),
        TCS344x_CH1 = (1 << 2),
        TCS344x_CH2 = (1 << 3),
        TCS344x_CH3 = (1 << 4),
        TCS344x_CH4 = (1 << 5),
        TCS344x_CH5 = (1 << 6)
};

enum tcs344x_spec_measure_status {
        TCS344x_SPEC_STATUS = (1 << 0),
        TCS344x_SPEC_SYNCD = (1 << 1)
};

enum tcs344x_agc_again {
        AGAINL = (1 << 2),
        AGAINMAX = (1 << 4),
};

enum tcs344x_status6 {
       TCS344x_INIT_BUSY = (1 << 0),
       TCS344x_SAI_ACTIVE = (1 << 1),
       TCS344x_SP_TRIG = (1 << 2),
       TCS344x_FD_TRIG = (1 << 4),
       TCS344x_OVTEMP = (1 << 5),
       TCS344x_FIFO_OV = (1 << 7)
};

enum tcs344x_cfg6 {
        TCS344x_SMUX = (1 << 4),
        TCS344x_AGC_GAIN_MAX = (1 << 6)
};

enum tcs344x_cfg8 {
        TCS344x_AGC_ENABLE = (1 << 2)
};

enum tcs344x_cfg9 {
        TCS344x_SIEN_FD = (1 << 6)
};

enum tcs344x_cfg11 {
        TCS344x_AINT_DIRECT = (1 << 7)
};

enum tcs344x_cfg12 {
        TCS344x_MASK_TH_CHANNEL = 0x03
};

enum tcs344x_fd_cfg0 {
        TCS344x_FD_CFG0 = (1 << 7)
};

enum tcs344x_status5 {
        TCS344x_SINT_FD = (1 << 3)
};

enum tcs344x_cfg0_masks {
        TCS344x_LOW_POWER = (1 << 5),
        TCS344x_ALS_LONG = (1 << 2),
        TCS344x_RAM_BANK = 0x03,
        TCS344x_ANY_CFG0 = (TCS344x_LOW_POWER | TCS344x_ALS_LONG | TCS344x_RAM_BANK)
};

enum tcs344x_cfg3_masks {
        TCS344x_SAI = (1 << 4)
};

enum tcs344x_cfg4_masks {
        TCS344x_INTMAP = 0x70,
        TCS344x_INVERT = (1 << 3)
};


struct tsl2540_als_info {
        u16 als_ch0; /* photopic channel */
        u16 als_ch1; /* ir channel */
        u32 cpl;
        u32 saturation;
        u16 lux;
};

enum tcs344x_flickr_status {
        TCS344x_FLICKR_VALID = (1 << 5),
        TCS344x_FLICKR_SAT_DETECT = (1 << 4),
        TCS344x_FLICKR_120Hz_VALID = (1 << 3),
        TCS344x_FLICKR_100Hz_VALID = (1 << 2),
        TCS344x_FLICKR_120Hz = (1 << 1),
        TCS344x_FLICKR_100Hz = (1 << 0)
};


typedef enum tcs344x_modes_of_operation {
        TCS344x_MODE_IDLE,
        TCS344x_ALS_MODE,
} operation_mode;

typedef enum tcs344x_mux_state {
    TCS344x_MUX_CFG_A_STATE,
    TCS344x_MUX_CFG_B_STATE
}tcs344x_mux_state_t;

struct tcs344x_parameters {
        u8 config; /* register 0x70 - mode */
        u8 led_reg; /* 0x74 */
        u8 enable; /* reg 0x80 */
        u8 atime; /* als_time */
        u8 wtime;
        u8 status; /*reg 0x93 */
        u8 astatus; /* reg 0x94 */
        u8 status2; /* reg 0xA3 */
        u8 status3; /* reg 0xA4 */
        u8 status5;
        u8 status6;
        u8 cfg0; /* reg 0xA9 - ALS trigger long and bank selection */
        u8 cfg1; /* reg 0xC6 - ALS ATIME configuration */
        u8 cfg3; /* reg 0xAB - Sleep after interrupt */
        u8 cfg4; /* reg 0xAD - Interrupt pin map and interrupt invert */
        u8 cfg8; /* reg 0xB1 - spectral threshold - fd auto gain control and spectral again*/
        u8 cfg6; /* reg 0xAF - ALS saturation decrement */
        u8 cfg9; /* reg 0xB2 - flicker detection interrupt */
        u8 cfg10; /* reg 0xB3 - hysterisis - AGC - Low and High */
        u8 cfg11; /* reg 0xB4 - ALS interrupt */
        u8 cfg12; /* reg 0xB5 - spectral threshold channel */
        u8 cfg20; /* reg 0xD6 - smux auto config */
        u8 pcfg1;
        u8 persist; /* reg 0xBD */
        u8 gpio2;
        u8 ls_astep;
        u8 ms_astep;
        u8 auto_again;
        u8 again; /* CFG1 reg 0xAA - Auto gain or again */
        u8 again_max; /* reg 0xCF */
        u8 azconfig;
        u8 fd_time1;
        u8 fd_time2;
		u8 fd_fifo_map;
        u8 fd_cfg0;
        u8 fd_status;
        u8 intenab;
        u8 control;
        u8 fifo_map;
        u8 fifo_lvl;
        u8 valid;
        u8 init_state;
		u8 sai_enable;
		u8 ram_bank;
        u8 flicker_enable;
};

#define MATRIX_ROW_SIZE 3
#define MATRIX_COL_SIZE 13
#define NOMINAL_ATIME_DEFAULT   204800 /* 50ms Q20.12 */
#define NOMINAL_AGAIN_DEFAULT   4096

enum cal_ch_index {
    CH_IDX_F1 = 0,
    CH_IDX_F2,
    CH_IDX_Z,
    CH_IDX_F3,         
    CH_IDX_F4,         
    CH_IDX_Y,         
    CH_IDX_F5,         
    CH_IDX_X1,         
    CH_IDX_F6,      
    CH_IDX_F7,         
	CH_IDX_F8,
	CH_IDX_NIR,
	CH_IDX_VIS,
};

enum cal_tri_index {
    TRI_IDX_X = 0,
    TRI_IDX_Y,
    TRI_IDX_Z
};

struct matrix_data {
    int32_t data[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    uint32_t q_factor;
};

struct calibration_data {
    struct matrix_data coef;
    uint32_t nominal_atime;
    uint32_t nominal_again;
};

struct adc_data {
    uint16_t f1_raw;
    uint16_t f2_raw;
    uint16_t z_raw;
    uint16_t f3_raw;
    uint16_t f4_raw;
    uint16_t y_raw;
    uint16_t f5_raw;
    uint16_t x1_raw;
    uint16_t f6_raw;
    uint16_t f7_raw;
    uint16_t f8_raw;
    uint16_t nir_raw;
    uint16_t vis_raw;
	uint32_t again;
    uint32_t atime;
    uint32_t astep_l;
    uint32_t astep_h;
};

struct cie_tristimulus {
    uint64_t x;
    uint64_t y;
    uint64_t z;
};

struct als_xyz_data {
    struct cie_tristimulus tristimulus;
    uint64_t chromaticity_x;
    uint64_t chromaticity_y;
    uint32_t lux;
    uint32_t cct;
};

struct tcs344x_chip {
    struct mutex lock;
    struct i2c_client *client;
    struct tcs344x_parameters params;
    struct tcs344x_i2c_platform_data *pdata;
    wait_queue_head_t fifo_wait;
    u8 shadow[MAX_REGS];

    struct input_dev *als_idev;
    struct input_dev *cct_idev;
#ifdef CONFIG_QUALCOMM_AP
    struct sensors_classdev als_cdev;
#endif
    struct als_xyz_data xyz;
    int in_suspend;
    int wake_irq;
    int irq_pending;

    bool unpowered;
    bool enabled;
    bool is_als_valid;
    bool is_spectral_ready;
    bool in_asat;
    bool amscalcomplete;
    bool is_first_smux_done;
    u8 auxid;
    u8 freq;
	uint16_t chip_id;
	uint64_t chip_serial_number;
    uint8_t rev;
    u8 id;
    /* The output data CH0 thru CH5 u16 will be in an array */
    u32 saturation;
    struct task_struct *tcs344x_spectral;
    u8 device_index;
    bool is_data_read;
    operation_mode mode;
    bool als_input_open;
    bool valid_auxid;
    bool driver_remove;
    bool is_als_smux_configed;
    bool is_flicker_smux_configed;
    tcs344x_mux_state_t mux_state;
};

#define TCS344x_RAW_DATA_BYTE_COUNT 36

/* Must match definition in ../arch file */
struct tcs344x_i2c_platform_data {
    /* The following callback for power events received and handled by
       the driver.  Currently only for SUSPEND and RESUME */
    int (*platform_power)(struct device *dev, enum tcs344x_pwr_state state);
    int (*platform_init)(void);
    void (*platform_teardown)(struct device *dev);

    char const *als_name;
    struct tcs344x_parameters parameters;
    bool als_can_wake;
    u32 ams_irq_gpio; /* as per DTS */
    u16 out_data[TCS344x_RAW_DATA_BYTE_COUNT];
    u16 raw_data[TCS344x_RAW_DATA_BYTE_COUNT];
    int pos;
    bool dts_recived;

#if CONFIG_OF
    struct device_node *of_node;
#endif
};

#endif /* __TCS344x_H */
