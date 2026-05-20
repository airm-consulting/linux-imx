/*
 *****************************************************************************
 * Copyright 2017 by ams AG                                                  *
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
 * functionality within the AMS TCS344x family of devices.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>

#include <linux/init.h>
#include <linux/kfifo.h>

#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <linux/iio/ams/ams_tcs344x.h>
#include "ams_i2c.h"
#include "ams_tcs344x_als.h"

#define FLICKER_VALUE ABS_MISC
#ifdef CONFIG_QUALCOMM_AP
#include <linux/sensors.h>

#define FLICKER_ENABLE (0)

static u8 data[256];
/*
 * warning: if we change the kfifo size, then
 * we also need to change the kfifo overflow value
 */

static struct sensors_classdev als_sensors_cdev = {
	.name = "tcs344x-als",
	.vendor = "AMS",
	.version = 1,
	.handle = 0,
	.type = 1,
	.max_range = "1",
	.resolution = "1",
	.sensor_power = "1",
	.min_delay = 10000,
	.max_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 0,
	.sensors_enable = NULL,
};
#endif

enum tcs344x_channel_index {
	IDX_f1_raw = 0,
	IDX_f2_raw,
	IDX_z_raw,
	IDX_f3_raw,
	IDX_f4_raw,
	IDX_y_raw,
	IDX_f5_raw,
	IDX_x1_raw,
	IDX_f6_raw,
	IDX_f7_raw,
	IDX_f8_raw,
	IDX_nir_raw,
	IDX_vis_raw,
	IDX_AGAIN,
	IDX_ATIME,
	IDX_ASTEP_L,
	IDX_ASTEP_H,
};

/* SMUX Configuration */
#define SMUX_SIZE (30)

enum als_channel_config {
	ALS_SMUX_CFG_1,
	ALS_SMUX_CFG_2,
	ALS_SMUX_CFG_MAX = ALS_SMUX_CFG_2 + 1,
	ALS_SMUX_CFG_DFLT = ALS_SMUX_CFG_1
};


/* 
 * 2 rows, 30 columns
 */
static u8 smux_als_configuration_data[ALS_SMUX_CFG_MAX][SMUX_SIZE] = {
	{
		/*
		 * Values for register CHAIN_MUX
		 * Address of register 0xE7
		 */
		0x00, 0x04, 0x05, 0x02, 
		0x00, 0x05, 0x00, 0x01, 
		0x00, 0x30, 0x00, 0x00, 
		0x00, 0x20, 0x04, 0x00, 
		0x03, 0x00, 0x01, 0x00, 
		0x00, 0x00, 0x00, 0x00, 
		0x30, 0x00, 0x40, 0x10, 
		0x20, 0x00
	},
	{
		/*
		 * Values for register CHAINCMD
		 * Address of register 0xE4
		 */
		0x46, 0x46, 0x46, 0x46, 
		0x46, 0x46, 0x46, 0x46, 
		0x46, 0x46, 0x56, 0x56, 
		0x56, 0x56, 0x56, 0x56, 
		0x56, 0x56, 0x56, 0x56, 
		0x66, 0x66, 0x66, 0x66, 
		0x66, 0x66, 0x66, 0x66, 
		0x66, 0x66
	}
};


/* 
 * 2 rows, 30 columns
 */
static u8 smux_flicker_configuration_data[ALS_SMUX_CFG_MAX][SMUX_SIZE] = {
	{
		/*
		 * Values for register CHAIN_MUX
		 * Address of register 0xE7
		 */
		0x00, 0x00, 0x60, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x60, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x60, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00
	},
	{
		/*
		 * Values for register CHAINCMD
		 * Address of register 0xE4
		 */
		0x46, 0x46, 0x46, 0x46,
		0x46, 0x46, 0x46, 0x46,
		0x46, 0x46, 0x56, 0x56,
		0x56, 0x56, 0x56, 0x56,
		0x56, 0x56, 0x56, 0x56,
		0x66, 0x66, 0x66, 0x66,
		0x66, 0x66, 0x66, 0x66,
		0x66, 0x66
	}
};



/* TCS344x Identifiers */
static u8 const tcs344x_ids[] = {
	0, 2, 1, 8};    /* TCS344x auxids 0 - for present device, 2 - TCS344x and 8 - AS7351 */

static operation_mode operating_mode;
static DECLARE_KFIFO(ams_kfifo, u8, PAGE_SIZE);

static u8 const restorable_regs[] = {
	TCS344x_REGADDR_ENABLE,
	// TCS344x_REGADDR_AOFFSET0_H,
	// TCS344x_REGADDR_LED,
	TCS344x_REGADDR_ATIME,
	// TCS344x_REGADDR_CFG_11,
	TCS344x_REGADDR_ASTEP_L,
	TCS344x_REGADDR_ASTEP_H,
	TCS344x_REGADDR_CFG_1,
	TCS344x_REGADDR_CFG_3,
	TCS344x_REGADDR_CFG_8,
	TCS344x_REGADDR_WTIME,
	TCS344x_REGADDR_CFG_10,
	TCS344x_REGADDR_CFG_20,
	TCS344x_REGADDR_PERS,
	// TCS344x_REGADDR_GPIO2,
	TCS344x_REGADDR_AGC_GAIN_MAX,
	TCS344x_REGADDR_AZCONFIG,
	TCS344x_REGADDR_CFG_0,
	TCS344x_REGADDR_FD_CFG_0,
	TCS344x_REGADDR_FD_CFG_1,
	TCS344x_REGADDR_FD_CFG_3,
	TCS344x_REGADDR_FIFO_MAP,
	TCS344x_REGADDR_PCFG_1,
};

operation_mode get_spectral_mode(void)
{
	return operating_mode;
}

int set_spectral_mode(operation_mode mode)
{
	operating_mode = mode;
	return 0;
}

static void report_als_event(struct tcs344x_chip *chip, int type, int value)
{
	input_report_abs(chip->als_idev, type, value);
	input_sync(chip->als_idev);
}

int read_fifo_data(struct tcs344x_chip *chip, uint16_t *buf, int _size)
{
	int ret = 0;
	dev_dbg(&chip->client->dev, "%s %d \n", __func__, __LINE__);
	ret = kfifo_out(&ams_kfifo, (char *)buf, _size);
	kfifo_reset(&ams_kfifo);

	return(ret);
}

static void init_flicker(struct tcs344x_chip * chip)
{
	/*
	 * ENable Flicker Detection
	 */

	/*
	 * Set max AGC FD gain to 1024
	 */
	ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_AGC_GAIN_MAX, 0xF0, TCS344x_AGC_GAIN_MAX);

	/* Sets the number of consecutive flicker detect results that must be different before
	 * the flicker detect status will be changed. Flicker detection interrupts on SINT
	 * are affected by this setting. Flicker detect persistence is equal to 2^(FD_PERS -1)
	 */
	ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_CFG_10, 0x07, 0x00); //0x01);

	/* System Interrupt Flicker Detection. Enables system interrupt when flicker detection
	 *  status change has occurred.
	 */
	ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_CFG_9, TCS344x_SIEN_FD, TCS344x_SIEN_FD);

	return;
}

static void enable_flicker(struct tcs344x_chip *chip, u8 enable)
{
	if (enable) {
		ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_INTENAB,
				TCS344x_INTENAB_SIEN | TCS344x_INTENAB_FIEN, TCS344x_INTENAB_SIEN | TCS344x_INTENAB_FIEN );

		ams_i2c_modify(chip->client, chip->shadow,
				TCS344x_REGADDR_ENABLE, (TCS344x_FDEN | TCS344x_PON), (TCS344x_FDEN | TCS344x_PON));

	} else {
		ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, 0x00);
		ams_i2c_modify(chip->client, chip->shadow,
				TCS344x_REGADDR_INTENAB, TCS344x_INTENAB_SIEN, 0x00);
	}

	chip->enabled = enable;
}

static void enable_als(struct tcs344x_chip *chip, u8 enable)
{
	if (enable) {
		set_spectral_mode(TCS344x_ALS_MODE);

		ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_INTENAB,
				TCS344x_INTENAB_AIEN, TCS344x_INTENAB_AIEN );

		ams_i2c_modify(chip->client, chip->shadow,
				TCS344x_REGADDR_ENABLE, (TCS344x_AEN | TCS344x_PON), (TCS344x_AEN | TCS344x_PON));

	} else {
		ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, 0x00);
		ams_i2c_modify(chip->client, chip->shadow,
				TCS344x_REGADDR_INTENAB, TCS344x_INTENAB_AIEN, 0x00);

		set_spectral_mode(TCS344x_MODE_IDLE);
		chip->xyz.lux = 0;
		chip->xyz.cct = 0;

	}

	chip->enabled = enable;
}

/* Enable or disable the device - PON in REGADDR_ENABLE */
int tcs344x_enable_device(struct tcs344x_chip *chip, u8 state)
{
	if (state)
	{
		ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, TCS344x_PON);
	}
	else
	{
		ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, 0);
	}
	return 0;
}

#if 0
static void tcs344x_enable_sai(struct tcs344x_chip *chip)
{
	if (chip->pdata->parameters.sai_enable == 1)
	{
		ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_CFG_3, TCS344x_SAI, TCS344x_SAI);
	}
}

static void tcs344x_disable_sai(struct tcs344x_chip *chip)
{
	u8 status6;

	if (chip->pdata->parameters.sai_enable == 1)
	{
		ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_CFG_3, TCS344x_SAI, 0x00);

		/* Check for SAI */
		ams_i2c_read(chip->client, TCS344x_REGADDR_STATUS_6, &status6);
		if (status6 & TCS344x_SAI_ACTIVE)
		{
			/* Clear SAI condition */
			ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_CONTROL, TCS344x_CLEAR_SAI_ACT,
					TCS344x_CLEAR_SAI_ACT);
		}

	}
}
#endif

void smux_write_config_data(struct tcs344x_chip *chip, bool init, u8 smux_data[][SMUX_SIZE])
{
	u8 ret;
	uint8_t row_ctr = 0;
	uint8_t col_ctr = 0;

	if(chip->params.flicker_enable)   
	{
		/**
		 * Disable AEN and FDEN before writing the config values
		 * Keep PON = 1
		 */
		ret = ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_ENABLE, (TCS344x_AEN | TCS344x_FDEN), 0x00);
	}
	else
	{
		/**
		 * Disable AEN before writing the config values
		 * Keep PON = 1
		 */
		ret = ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_ENABLE, TCS344x_AEN, 0x00);
	}

	for(row_ctr = 0; row_ctr < SMUX_SIZE; row_ctr++)
	{
		for(col_ctr = 0; col_ctr < ALS_SMUX_CFG_MAX ; col_ctr++)
		{
			if(col_ctr == false)
			{
				//ams_i2c_write_direct(chip->client, TCS344x_REGADDR_CHAIN_SMUX, (u8)smux_als_configuration_data[col_ctr][row_ctr]);
				ams_i2c_write_direct(chip->client, TCS344x_REGADDR_CHAIN_SMUX, (u8)smux_data[col_ctr][row_ctr]);
			}
			else
			{
				//ams_i2c_write_direct(chip->client, TCS344x_REGADDR_CHAIN_CMD, (u8)smux_als_configuration_data[col_ctr][row_ctr]);
				ams_i2c_write_direct(chip->client, TCS344x_REGADDR_CHAIN_CMD, (u8)smux_data[col_ctr][row_ctr]);
			}
			mdelay(1);
		}
	}

	if(chip->params.flicker_enable)
	{
		/**
		 * Enable PON, AEN and FDEN before writing the config values
		 * Keep PON = 1
		 */
		if(chip->is_flicker_smux_configed == false)
		{
			ret = ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, (/*TCS344x_AEN |*/ TCS344x_FDEN | TCS344x_PON));
		}
		else
		{
			ret = ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, (TCS344x_AEN | /*TCS344x_FDEN |*/ TCS344x_PON));
		}
		ret = ams_i2c_write_direct(chip->client, TCS344x_REGADDR_CFG_20, 0x62);
	}
	else
	{
		ret = ams_i2c_write_direct(chip->client, TCS344x_REGADDR_CFG_20, 0x62);
		ret = ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, (TCS344x_AEN | TCS344x_PON));
	}


	return;
}

static void tcs344x_process_data(struct tcs344x_chip *chip)
{
	struct adc_data _adc_data;
	struct als_xyz_data *xyz = &chip->xyz;

	_adc_data.f1_raw    = chip->pdata->out_data[IDX_f1_raw];
	_adc_data.f2_raw 	= chip->pdata->out_data[IDX_f2_raw];
	_adc_data.z_raw     = chip->pdata->out_data[IDX_z_raw];
	_adc_data.f3_raw    = chip->pdata->out_data[IDX_f3_raw];
	_adc_data.f4_raw    = chip->pdata->out_data[IDX_f4_raw];
	_adc_data.y_raw     = chip->pdata->out_data[IDX_y_raw];
	_adc_data.f5_raw    = chip->pdata->out_data[IDX_f5_raw];
	_adc_data.x1_raw    = chip->pdata->out_data[IDX_x1_raw];
	_adc_data.f6_raw 	= chip->pdata->out_data[IDX_f6_raw];
	_adc_data.f7_raw 	= chip->pdata->out_data[IDX_f7_raw];
	_adc_data.f8_raw 	= chip->pdata->out_data[IDX_f8_raw];
	_adc_data.nir_raw   = chip->pdata->out_data[IDX_nir_raw];
	_adc_data.vis_raw 	= chip->pdata->out_data[IDX_vis_raw];
	_adc_data.again 	= chip->pdata->out_data[IDX_AGAIN];
	_adc_data.atime 	= chip->pdata->out_data[IDX_ATIME];
	_adc_data.astep_l 	= chip->pdata->out_data[IDX_ASTEP_L];
	_adc_data.astep_h 	= chip->pdata->out_data[IDX_ASTEP_H];

	tcs344x_calculate_lux_and_cct(chip, &_adc_data, NULL, xyz);
	dev_dbg(&chip->client->dev, "LUX = %d, CCT=%d\n", xyz->lux, xyz->cct);

	return;
}


/* Called from interrupt handler - put into kfifo */
static int tcs344x_read_data(struct tcs344x_chip *chip)
{
	/* struct device *dev = &chip->client->dev; */
	int ret;
	u8 data[40];
	u16 temp_value;
	u8 temp_value_1;
	int fifo_length;
	//u8 enable_reg;
	int i = 0;

	/* Read ASTATUS through ASTATUS and ALS Channel data, 37 */
	ret = ams_i2c_blk_read(chip->client, TCS344x_REGADDR_ASTATUS, &data[0], 31);
	ret = ams_i2c_blk_read(chip->client, TCS344x_REGADDR_CH15_DATA, &data[31], 6);

	if (ret != 0)
	{
		/*
		 * Organize the read ALS data in bytes to words
		 */
		temp_value = ((data[2] << 8) | data[1]);
		chip->pdata->out_data[IDX_z_raw] = temp_value;
		temp_value = ((data[4] << 8) | data[3]);
		chip->pdata->out_data[IDX_y_raw] = temp_value;
		temp_value = ((data[6] << 8) | data[5]);
		chip->pdata->out_data[IDX_x1_raw] = temp_value;
		temp_value = ((data[8] << 8) | data[7]);
		chip->pdata->out_data[IDX_nir_raw] = temp_value;
		temp_value = ((data[10] << 8) | data[9]);
		chip->pdata->out_data[IDX_vis_raw] = temp_value;
		temp_value = ((data[14] << 8) | data[13]);
		chip->pdata->out_data[IDX_f2_raw] = temp_value;
		temp_value = ((data[16] << 8) | data[15]);
		chip->pdata->out_data[IDX_f3_raw] = temp_value;
		temp_value = ((data[18] << 8) | data[17]);
		chip->pdata->out_data[IDX_f4_raw] = temp_value;
		temp_value = ((data[20] << 8) | data[19]);
		chip->pdata->out_data[IDX_f6_raw] = temp_value;
		temp_value = ((data[26] << 8) | data[25]);
		chip->pdata->out_data[IDX_f1_raw] = temp_value;
		temp_value = ((data[28] << 8) | data[27]);
		chip->pdata->out_data[IDX_f7_raw] = temp_value;
		temp_value = ((data[30] << 8) | data[29]);
		chip->pdata->out_data[IDX_f8_raw] = temp_value;
		temp_value = ((data[32] << 8) | data[31]);
		chip->pdata->out_data[IDX_f5_raw] = temp_value;

		dev_dbg(&chip->client->dev, "RAW data: %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
				chip->pdata->out_data[IDX_f1_raw], chip->pdata->out_data[IDX_f2_raw],
				chip->pdata->out_data[IDX_z_raw], chip->pdata->out_data[IDX_f3_raw],
				chip->pdata->out_data[IDX_f4_raw], chip->pdata->out_data[IDX_f5_raw],
				chip->pdata->out_data[IDX_y_raw], chip->pdata->out_data[IDX_x1_raw],
				chip->pdata->out_data[IDX_f6_raw], chip->pdata->out_data[IDX_f7_raw],
				chip->pdata->out_data[IDX_f8_raw], chip->pdata->out_data[IDX_nir_raw],
				chip->pdata->out_data[IDX_vis_raw]);

		ret = ams_i2c_read(chip->client, TCS344x_REGADDR_CFG_1, &temp_value_1);
		chip->pdata->out_data[IDX_AGAIN] = (u16)((temp_value_1 & 0x1F));

		ret = ams_i2c_read(chip->client, TCS344x_REGADDR_ATIME, &temp_value_1);
		chip->pdata->out_data[IDX_ATIME] = (u16)(temp_value_1);
		mdelay(1);
		ret = ams_i2c_read(chip->client, TCS344x_REGADDR_ASTEP_L, &temp_value_1);
		chip->pdata->out_data[IDX_ASTEP_L] = (u16)(temp_value_1);
		ret = ams_i2c_read(chip->client, TCS344x_REGADDR_ASTEP_H, &temp_value_1);
		chip->pdata->out_data[IDX_ASTEP_H] = (u16)(temp_value_1) ;

		for (i = 0; i < 17; i++) {
			chip->pdata->raw_data[i] = chip->pdata->out_data[i];
		}

		dev_dbg(&chip->client->dev, "Data ready - again: 0x%x, atime: 0x%x, astep_l: 0x%x, astep_h: 0x%x\n",
				chip->pdata->out_data[IDX_AGAIN],
				chip->pdata->out_data[IDX_ATIME],
				chip->pdata->out_data[IDX_ASTEP_L],
				chip->pdata->out_data[IDX_ASTEP_H]);

		chip->is_spectral_ready = true;

		if (chip->is_spectral_ready == true)
		{
			kfifo_in(&ams_kfifo, (u8 *)chip->pdata->out_data, TCS344x_RAW_DATA_BYTE_COUNT);
			fifo_length = kfifo_len(&ams_kfifo);
			dev_dbg(&chip->client->dev, "Data ready - kfifo_length = %d %s %d \n", fifo_length, __FILE__, __LINE__);

			tcs344x_process_data(chip);
			report_als_event(chip, ABS_MISC, chip->xyz.lux);
			memset(chip->pdata->out_data, 0x00, sizeof(chip->pdata->out_data));
		}
	}

#if 0
	/* Disable the sensor */
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, 0x00);
	/* Confirm that AEN is off before continuing*/
	do {
		ams_i2c_read(chip->client, TCS344x_REGADDR_ENABLE , &enable_reg);

	}while (enable_reg & TCS344x_AEN);

	tcs344x_disable_sai(chip);

	/* Swap the smux configuration */
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, TCS344x_PON);
	//smux_config(chip, false);

	/* Enable the sensor again */
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, 0x00);
	tcs344x_enable_sai(chip);
#endif

	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, (TCS344x_PON | TCS344x_AEN | TCS344x_FDEN));

	return ret;
}


static int tcs344x_read_flicker(struct tcs344x_chip *chip, uint8_t fd_status)
{
	int ret = 0;

	struct device *dev = &chip->client->dev;                                                                      

	//dev_info(dev, "fd_status = %x  %s %d", fd_status, __func__, __LINE__);                                  

	if(fd_status == 0)                                                                                         
	{                                                                                                             
		return ret;                                                                                               
	}                                                                                                             
	if(!(fd_status & TCS344x_FLICKR_VALID))                                                                   
	{                                                                                                             
		return ret;                                                                                               
	}                                                                                                             

	if(fd_status & TCS344x_FLICKR_SAT_DETECT)                                                                  
	{                                                                                                             
		return ret;                                                                                               
	}                                                                                                             
	else                                                                                                          
	{                                                                                                             
		if(fd_status & (1 << TCS344x_FLICKR_120Hz_VALID))                                                      
		{                                                                                                         
			dev_info(dev, "flicker freq is 120KHz valid %s %d\n", __func__, __LINE__);                            
		}                                                                                                         
		else if(fd_status & (1 << TCS344x_FLICKR_100Hz_VALID))                                                 
		{                                                                                                         
			dev_info(dev, "flicker freq is 100KHz valid %s %d\n", __func__, __LINE__);                            
		}                                                                                                         
	}                                                                                                             

	if(fd_status & 0x01)    //(1 << TCS344x_FLICKR_100Hz))                                                                
	{              
		chip->freq = 100;
		//dev_info(dev, "Detected flicker freq is 100KHz %s %d\n", __func__, __LINE__);                             
	}                                                                                                             

	if(fd_status & 0x02)    //(1 << TCS344x_FLICKR_120Hz))                                                                
	{                    
		chip->freq = 120;
		//dev_info(dev, "Detected flicker freq is 120KHz %s %d\n", __func__, __LINE__);                             
	}                                                                                                             

	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, (TCS344x_PON | TCS344x_AEN | TCS344x_FDEN));

	return ret;
}

static int tcs344x_irq_handler(struct tcs344x_chip *chip)
{
	u8 status, status2, status4, status5;
	u8 fd_status = 0;
	int ret;
	/* struct device *dev = &chip->client->dev; */

	AMS_MUTEX_LOCK(&chip->lock);
	ret = ams_i2c_read(chip->client, TCS344x_REGADDR_STATUS , &status);
	/* dev_info(dev, " irq handler status = %02x \n", status); */

	if (status == 0) {
		AMS_MUTEX_UNLOCK(&chip->lock);
		return 0; /* not our interrupt */
	}

	ams_i2c_read(chip->client, TCS344x_REGADDR_STATUS_2, &status2);
	ams_i2c_read(chip->client, TCS344x_REGADDR_STATUS_4, &status4);
	ams_i2c_read(chip->client, TCS344x_REGADDR_STATUS_5, &status5);
	ams_i2c_read(chip->client, TCS344x_REGADDR_FLICKR_STATUS, &fd_status);

	/* Check for SAI */
	if (status4 & TCS344x_SAI_ACTIVE)
	{
		/* Clear SAI condition */
		ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_CONTROL, TCS344x_CLEAR_SAI_ACT,
				TCS344x_CLEAR_SAI_ACT);
	}

	if (status & TCS344x_ASAT) {
		if (status2 & (TCS344x_ANA_SAT | TCS344x_DIG_SAT))
		{
			chip->in_asat = 1;

			ret = ams_i2c_read(chip->client, TCS344x_REGADDR_STATUS , &status);
			dev_warn(&chip->client->dev,
					"Saturation, ASAT is %d STATUS2 = %x \n", chip->in_asat, status2);

			chip->is_als_valid = 0;
		}
	} else {
		chip->in_asat = 0;
		chip->is_als_valid = 1;
	}

	/*
	 * Calibration
	 */
	if (status & TCS344x_CINT) {
		chip->amscalcomplete = true;
	}

	/* ALS and any system interrupt 
	 * The system interrupt will correspond to flicker
	 * as, flicker detection is configured.
	 */
#ifdef FLICKER_ENABLE 
	if ((status & (TCS344x_AINT | TCS344x_SINT)))
#else        
		if (status & TCS344x_AINT)
#endif        
		{
			if(status2 & TCS344x_AVALID)
			{
				ret = tcs344x_read_data(chip);
			}

			if(chip->params.flicker_enable)
			{
				ret = tcs344x_read_flicker(chip, fd_status);
			}

			/**
			 *  If flicker is enabled in the dts, then alternately 
			 *  write the smux for flicker and als.
			 *  Otherwise, write the als smux only once.
			 */
			if(chip->params.flicker_enable)
			{
				if(chip->is_flicker_smux_configed == false)
				{
					/* Turn only PON on and write smux config */
					ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, TCS344x_PON);
					smux_write_config_data(chip, true, smux_flicker_configuration_data);
					chip->is_flicker_smux_configed = true;
				}
				else
				{
					/* Turn only PON on and write smux config */
					ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, TCS344x_PON);
					smux_write_config_data(chip, true, smux_als_configuration_data);
					chip->is_flicker_smux_configed = false;
				}
			}
			else
			{
				/**
				 *  Write als smux configuration data only once
				 */
				/* Turn only PON on and write smux config */
				if(chip->is_flicker_smux_configed == false)
				{
					ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, TCS344x_PON);
					smux_write_config_data(chip, true, smux_als_configuration_data);
					chip->is_flicker_smux_configed = true;
				}
			}

			if (chip->is_spectral_ready == true)
			{
				wake_up_interruptible(&chip->fifo_wait);
			}
		}

	/* Clear the status */
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_STATUS, status);
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_STATUS_2, status2);

	if(chip->params.flicker_enable)
	{
		ams_i2c_write_direct(chip->client, TCS344x_REGADDR_FLICKR_STATUS, fd_status);
	}

	AMS_MUTEX_UNLOCK(&chip->lock);

	return 1; /* we handled the interrupt */
}

static irqreturn_t tcs344x_irq(int irq, void *handle)
{
	struct tcs344x_chip *chip = handle;
	struct device *dev = &chip->client->dev;
	int ret;

	if (chip->in_suspend) {
		dev_info(dev, "%s: in suspend\n", __func__);
		chip->irq_pending = 1;
		ret = 0;
		goto bypass;
	}
	ret = tcs344x_irq_handler(chip);

bypass:
	return ret ? IRQ_HANDLED : IRQ_NONE;
}

static int tcs344x_flush_regs(struct tcs344x_chip *chip)
{
	int i;
	int rc;
	u8 reg;

	/* disable sensor before writing configuration */
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, 0x01);

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow, reg,
				chip->shadow[reg]);

		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}

	return rc;
}

static int tcs344x_pltf_power_on(struct tcs344x_chip *chip)
{
	int rc = 0;

	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev, POWER_ON);
		mdelay(10);
	}
	chip->unpowered = rc != 0;
	dev_info(&chip->client->dev, "%s: unpowered=%d\n", __func__, chip->unpowered);
	return rc;
}

static int tcs344x_pltf_power_off(struct tcs344x_chip *chip)
{
	int rc = 0;

	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev, POWER_OFF);
		chip->unpowered = rc == 0;
	} else {
		chip->unpowered = false;
	}
	dev_info(&chip->client->dev, "%s: unpowered=%d\n", __func__,
			chip->unpowered);
	return rc;
}


static void tcs344x_set_defaults(struct tcs344x_chip *chip)
{
	u8 *sh = chip->shadow;
	struct device *dev = &chip->client->dev;

	/* Clear the register shadow area */
	memset(chip->shadow, 0x00, sizeof(chip->shadow));
	if (chip->pdata->dts_recived == 1) {
		dev_info(dev, "%s: use defaults from dts\n", __func__);
		/*Note - if it is a 0 in all bits in the data sheet - omit */
		chip->params.config         = chip->pdata->parameters.config;            /* No mode */
		chip->params.led_reg        = chip->pdata->parameters.led_reg;
		chip->params.atime          = chip->pdata->parameters.atime;
		chip->params.ms_astep       = chip->pdata->parameters.ms_astep;
		chip->params.ls_astep       = chip->pdata->parameters.ls_astep;
		chip->params.wtime          = chip->pdata->parameters.wtime;
		chip->params.auto_again     = chip->pdata->parameters.auto_again;
		chip->params.again          = chip->pdata->parameters.again;              /* default in data sheet */
		chip->params.again_max      = chip->pdata->parameters.again_max;
		chip->params.persist        = ALS_PERSIST(chip->pdata->parameters.persist);
		chip->params.azconfig       = chip->pdata->parameters.azconfig;
		chip->params.fd_time1       = chip->pdata->parameters.fd_time1;
		chip->params.fd_time2       = chip->pdata->parameters.fd_time2;
		chip->params.fd_fifo_map    = chip->pdata->parameters.fd_fifo_map;
		chip->params.sai_enable     = chip->pdata->parameters.sai_enable;
		chip->params.flicker_enable     = chip->pdata->parameters.flicker_enable;
	} else {
		dev_info(dev, "%s: use defaults\n", __func__);
		/*Note - if it is a 0 in all bits in the data sheet - omit */
		chip->params.config = 0x00;            /* No mode */
		chip->params.led_reg = 0x04;
		chip->params.atime = 0x11;
		chip->params.ms_astep = 0x03;
		chip->params.ls_astep = 0xe7;
		chip->params.wtime = 0;
		chip->params.auto_again = 1;
		chip->params.again = 8;                /* default in data sheet */
		chip->params.again_max = 0x99;
		chip->params.persist = ALS_PERSIST(0);
		chip->params.azconfig = 0x00;
		chip->params.fd_time1 = DEFAULT_FD_TIME_1;
		chip->params.fd_time2 = DEFAULT_FD_TIME_2;
		chip->params.fd_fifo_map = DEFAULT_FIFO_MAP;
		chip->params.sai_enable = 0;
		chip->params.flicker_enable = 0;
	}

	chip->params.enable = 0x01;            /* PON -also enable Spectral? - 0x03 ? */
	/*chip->params.astep = 0x0257; 599 recommended in data sheet */
	if (chip->params.auto_again == 0) {
		chip->params.cfg8 = 0xC8;       /* auto-gain AGC disable */
	} else {
		chip->params.cfg8 = 0xC4;       /* auto-gain AGC enable */
	}

	chip->params.fd_cfg0 = DISABLE_FIFO_WRITE_FD | NUM_FD_SAMPLES_256 | FD_COMPARE_LIMIT; //Goertzel mode | 1024 samples | compare_value          
	chip->params.cfg9 = 0x40;                                                                                                                     
	chip->params.cfg10 = 0xF0;                                                                                                                    
	//chip->params.cfg20 = 0x60;    //originally it was implemented from QCOM
	//chip->params.cfg20 = 0x02;                                                                                                                    
	chip->params.cfg20 = 0x62;                                                                                                                    
	chip->params.gpio2 = 0x02;                                                                                                                    
	chip->is_spectral_ready = false;                                                                                                              
	chip->params.ram_bank   = 0x00;     /* Selection of RAM bank 0/1 */                                                                           

	chip->params.pcfg1 = 0x08;

	if (chip->pdata->parameters.sai_enable == 1)
	{
		chip->params.cfg3 = 0x0c | TCS344x_SAI;
	}
	else
	{
		chip->params.cfg3 = 0x0c;
	}

	/* Copy the default values into the register shadow area */                                      
	sh[TCS344x_REGADDR_ENABLE]      = chip->params.enable;                                                
	sh[TCS344x_REGADDR_AOFFSET0_H]  = chip->params.config;                                            
	sh[TCS344x_REGADDR_LED]         = chip->params.led_reg;                                                  
	sh[TCS344x_REGADDR_ATIME]       = chip->params.atime;                                                  
	sh[TCS344x_REGADDR_ASTEP_L]     = chip->params.ls_astep;                                             
	sh[TCS344x_REGADDR_ASTEP_H]     = chip->params.ms_astep;                                             
	sh[TCS344x_REGADDR_CFG_0]       = chip->params.ram_bank;                                               
	sh[TCS344x_REGADDR_CFG_1]       = chip->params.again; 
	sh[TCS344x_REGADDR_CFG_3]       = chip->params.cfg3;                                                   
	sh[TCS344x_REGADDR_CFG_8]       = chip->params.cfg8;                                                   
	sh[TCS344x_REGADDR_CFG_10]      = chip->params.cfg10;                                                 
	sh[TCS344x_REGADDR_CFG_9]       = chip->params.cfg9;                                                   
	sh[TCS344x_REGADDR_CFG_20]      = chip->params.cfg20;                                                 
	sh[TCS344x_REGADDR_PERS]        = chip->params.persist;                                                 
	sh[TCS344x_REGADDR_GPIO2]       = chip->params.gpio2;                                                  
	sh[TCS344x_REGADDR_AGC_GAIN_MAX] = chip->params.again_max;                                       
	sh[TCS344x_REGADDR_AZCONFIG]    = chip->params.azconfig;                                            
	sh[TCS344x_REGADDR_FD_CFG_0]    = chip->params.fd_cfg0;                                             
	sh[TCS344x_REGADDR_FD_CFG_1]    = chip->params.fd_time1;                                            
	sh[TCS344x_REGADDR_FD_CFG_3]    = chip->params.fd_time2;
	sh[TCS344x_REGADDR_FIFO_MAP]    = chip->params.fd_fifo_map;
	sh[TCS344x_REGADDR_PCFG_1]      = chip->params.pcfg1;

	tcs344x_flush_regs(chip);

}

static int tcs344x_get_id(struct tcs344x_chip *chip, u8 *id, u8 *rev, u8 *auxid, uint8_t *chip_id)
{
#ifdef READ_CHIP_ID    
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_CFG_0, 0x10);
#endif        
	ams_i2c_read(chip->client, TCS344x_REGADDR_AUXID, auxid);
	ams_i2c_read(chip->client, TCS344x_REGADDR_REVID, rev);
	ams_i2c_read(chip->client, TCS344x_REGADDR_ID, id);
#ifdef READ_CHIP_ID
	ams_i2c_blk_read(chip->client, TCS344x_REGADDR_CHIP_ID , (u8 *)chip_id, 5);
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_CFG_0, 0x00);
#endif  

	return 0;
}

static bool tcs344x_present(struct tcs344x_chip *chip)
{
	u8 id;
	int ret = ams_i2c_read(chip->client, TCS344x_REGADDR_ID, &id);

	return (ret >= 0);
}

static void config_als(struct tcs344x_chip *chip)
{
	operation_mode mode = get_spectral_mode();

	chip->mode = mode;

	chip->is_spectral_ready = false;
	chip->is_data_read = false;

	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_FIFO_MAP, 0);

	/* AGC enable */
	ams_i2c_modify(chip->client, chip->shadow,
			TCS344x_REGADDR_CFG_6, TCS344x_AGC_GAIN_MAX, TCS344x_AGC_GAIN_MAX);
	if (chip->params.auto_again == 0) {
		ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_CFG_8, TCS344x_AGC_ENABLE, 0);
	} else {
		ams_i2c_modify(chip->client, chip->shadow, TCS344x_REGADDR_CFG_8, TCS344x_AGC_ENABLE, TCS344x_AGC_ENABLE);
	}

	/* Turn only PON on and write smux config */
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, TCS344x_PON);
	smux_write_config_data(chip, true, smux_als_configuration_data);
	chip->is_als_smux_configed = true;

	kfifo_reset(&ams_kfifo);
}

static int tcs344x_power_on(struct tcs344x_chip *chip)
{
	int rc;

	rc = tcs344x_pltf_power_on(chip);
	if (rc)
		return rc;
	dev_info(&chip->client->dev, "%s: chip was off, restoring regs\n",
			__func__);
	return tcs344x_flush_regs(chip);
}

static int tcs344x_als_idev_open(struct input_dev *idev)
{
	struct tcs344x_chip *chip = dev_get_drvdata(&idev->dev);
	int rc = 0;
	u8 status;

	dev_info(&idev->dev, "%s\n", __func__);

	if (chip->enabled == false)
	{
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->unpowered) {
			rc = tcs344x_power_on(chip);
			if (rc)
				goto chip_on_err;
		}

		ams_i2c_read(chip->client, TCS344x_REGADDR_ENABLE, &status);
		dev_info(&idev->dev, "als_idev_open Enable = %02x\n", status);
		set_spectral_mode(TCS344x_ALS_MODE);


		if(chip->params.flicker_enable)
		{
			init_flicker(chip);
			enable_flicker(chip, 1);
		}
		config_als(chip);
		//smux_write_config_data(chip, true);
		enable_als(chip, 1);

chip_on_err:
		AMS_MUTEX_UNLOCK(&chip->lock);
	}
	return 0;
}

static void tcs344x_als_idev_close(struct input_dev *idev)
{
	struct tcs344x_chip *chip = dev_get_drvdata(&idev->dev);
	u8 status;

	dev_info(&idev->dev, "%s\n", __func__);
	AMS_MUTEX_LOCK(&chip->lock);

	enable_als(chip, 0);

	ams_i2c_read(chip->client, TCS344x_REGADDR_ENABLE, &status);
	dev_info(&idev->dev, "als_idev_close Enable = %02x\n", status);
	tcs344x_pltf_power_off(chip);
	AMS_MUTEX_UNLOCK(&chip->lock);
}

#ifdef CONFIG_QUALCOMM_AP
static int tcs344x_als_set_enable(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	struct tcs344x_chip *chip = container_of(sensors_cdev, struct tcs344x_chip, als_cdev);

	if (enable) {
		chip->a_idev->open(chip->a_idev);
	} else {
		chip->a_idev->close(chip->a_idev);
	}
	return 0;
}
#endif

#if CONFIG_OF
int tcs344x_init_dt(struct tcs344x_i2c_platform_data *pdata)
{
	struct device_node *np = pdata->of_node;
	u32 val;
	const char* str;

	if (!pdata->of_node)
		return 0;

	if (!of_property_read_string(np, "als_name", &str))
		pdata->als_name = str;

	if (!of_property_read_u32(np, "als_config", &val))
		pdata->parameters.config = val;

	if (!of_property_read_u32(np, "als_led_reg", &val))
		pdata->parameters.led_reg = val;

	if (!of_property_read_u32(np, "als_atime", &val))
		pdata->parameters.atime = val;

	if (!of_property_read_u32(np, "als_ms_astep", &val))
		pdata->parameters.ms_astep = val;

	if (!of_property_read_u32(np, "als_ls_astep", &val))
		pdata->parameters.ls_astep = val;

	if (!of_property_read_u32(np, "als_wtime", &val))
		pdata->parameters.wtime = val;

	if (!of_property_read_u32(np, "als_auto_again", &val))
		pdata->parameters.auto_again = val;

	if (!of_property_read_u32(np, "als_again", &val))
		pdata->parameters.again = val;

	if (!of_property_read_u32(np, "als_again_max", &val))
		pdata->parameters.again_max = val;

	if (!of_property_read_u32(np, "als_persist", &val))
		pdata->parameters.persist = val;

	if (!of_property_read_u32(np, "als_azconfig", &val))
		pdata->parameters.azconfig = val;

	if (!of_property_read_u32(np, "fd_time1", &val))
		pdata->parameters.fd_time1 = val;

	if (!of_property_read_u32(np, "fd_time2", &val))
		pdata->parameters.fd_time2 = val;

	if (!of_property_read_u32(np, "sai_enable", &val))
		pdata->parameters.sai_enable = val;

	if (!of_property_read_u32(np, "flicker_enable", &val))
		pdata->parameters.flicker_enable = val;

	return 0;
}
#endif


u8 get_wait_time (struct tcs344x_chip *chip)
{
	u8 wtime;
	ams_i2c_read(chip->client, TCS344x_REGADDR_WTIME, &wtime);
	/* Rounding 2.78 to 3 */
	return (3 * (wtime + 1));
}

int set_idle_mode(struct tcs344x_chip *chip)
{
	/* Disable the interrupts */
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_INTENAB, 0);
	chip->params.intenab = 0;
	/* Set only PON */
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, TCS344x_PON);
	return 0;
}

int spectral_mode(struct tcs344x_chip *chip)
{
	/* u8 status; */
	struct device *dev = &chip->client->dev;
	operation_mode mode = get_spectral_mode();
	/* If input device is not already open - open to handle events */
	if (chip->als_input_open == false)
	{
		chip->als_idev->open(chip->als_idev);
		chip->als_input_open = true;
	}
	AMS_MUTEX_LOCK(&chip->lock);
	chip->mode = mode;
	chip->pdata->pos = 0;
	/* Setting the Flicker data not sent to FIFO */
	chip->params.fd_cfg0 = 0x21; /* Not sure about bit 7 for this register */
	/* ams_i2c_write_direct(chip->client, TCS344x_REGADDR_FD_CFG0, chip->params.fd_cfg0); */

	chip->params.intenab = TCS344x_INTENAB_AIEN;
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_INTENAB, TCS344x_INTENAB_AIEN);
	/* ams_i2c_read(chip->client, TCS344x_REGADDR_ENABLE, &status); */
	chip->is_spectral_ready = false;
	chip->is_data_read = false;
	dev_info(dev, "Start spectral one shot\n");

	/* Turn only PON on */
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, TCS344x_PON);
	//smux_config(chip, true);
	kfifo_reset(&ams_kfifo);
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, TCS344x_PON | TCS344x_AEN);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return 0;
}

int stop_spectral_measurement(struct tcs344x_chip *chip)
{
	struct device *dev = &chip->client->dev;
	set_spectral_mode(false);
	AMS_MUTEX_LOCK(&chip->lock);
	/* Turn off Spectral measure - leave chip on  */
	ams_i2c_write_direct(chip->client, TCS344x_REGADDR_ENABLE, TCS344x_PON);
	AMS_MUTEX_UNLOCK(&chip->lock);
	dev_info(dev, "Spectral measurement stopped");
	return 0;
}


/*
 * Sysfs ABI
 */

static ssize_t enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	int rc;
	//uint8_t enable;
	bool enable;

	//rc = kstrtol(buf, 0, (long *)(&(enable)));
	rc = kstrtobool(buf, &enable);
	if (rc != 0) {
		dev_err(&chip->client->dev, "kstrtolbool() error.\n");
		return -EINVAL;
	}

	AMS_MUTEX_LOCK(&chip->lock);
	enable_als(chip, enable);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	int count;
	u8 status;

	AMS_MUTEX_LOCK(&chip->lock);
	ams_i2c_read(chip->client, TCS344x_REGADDR_ENABLE, &status);
	chip->params.enable = status;

	count =  snprintf(buf, PAGE_SIZE, "%x\n", chip->enabled);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	int count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%x\n", chip->id);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t auxid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	int count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%d\n", chip->auxid);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t revid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	int count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%d\n", chip->rev);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t lux_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	int count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%d\n", chip->xyz.lux);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t cct_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	int count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%d\n", chip->xyz.cct);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t raw_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	int count;
	u16 astep;
	u16 atime;
	u16 again;

	AMS_MUTEX_LOCK(&chip->lock);
	astep = ((chip->pdata->raw_data[IDX_ASTEP_H] << 8) | chip->pdata->raw_data[IDX_ASTEP_L]);
	atime = chip->pdata->raw_data[IDX_ATIME];
	again = (1 << chip->pdata->raw_data[IDX_AGAIN]) / 2;
	count =  snprintf(buf, PAGE_SIZE, 
			"{"
			"\n\t\"407nm_count\": %d,"
			"\n\t\"424nm_count\": %d,"
			"\n\t\"450nm_count\": %d,"
			"\n\t\"473nm_count\": %d,"
			"\n\t\"516nm_count\": %d,"
			"\n\t\"546nm_count\": %d,"
			"\n\t\"560nm_count\": %d,"
			"\n\t\"596nm_count\": %d,"
			"\n\t\"636nm_count\": %d,"
			"\n\t\"687nm_count\": %d,"
			"\n\t\"748nm_count\": %d,"
			"\n\t\"855nm_count\": %d,"
			"\n\t\"visnm_count\": %d,"
			"\n\t\"astep\": %d,"
			"\n\t\"atime\": %d,"
			"\n\t\"again\": %d"
			"\n}",
			chip->pdata->raw_data[IDX_f1_raw],
			chip->pdata->raw_data[IDX_f2_raw],
			chip->pdata->raw_data[IDX_z_raw],
			chip->pdata->raw_data[IDX_f3_raw],
			chip->pdata->raw_data[IDX_f4_raw],
			chip->pdata->raw_data[IDX_f5_raw],
			chip->pdata->raw_data[IDX_y_raw],
			chip->pdata->raw_data[IDX_x1_raw],
			chip->pdata->raw_data[IDX_f6_raw],
			chip->pdata->raw_data[IDX_f7_raw],
			chip->pdata->raw_data[IDX_f8_raw],
			chip->pdata->raw_data[IDX_nir_raw],
			chip->pdata->raw_data[IDX_vis_raw],
			astep,
			atime,
			again);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t freq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	int count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%d\n", chip->freq);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}


static ssize_t chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	uint64_t count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%llx\n", chip->chip_serial_number);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

/* This is the read function for the binary sysfs attribute called "fifo" */
static ssize_t fifo_read(struct file *fp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t size)
{
	struct device *dev = kobj_to_dev(kobj);
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	int read = 0;
	int res;
	uint32_t _size = size;

	while ( kfifo_len(&ams_kfifo) < _size ) {
		AMS_MUTEX_UNLOCK(&chip->lock);
		res = wait_event_interruptible(chip->fifo_wait,
				((kfifo_len(&ams_kfifo) >= _size) ||
				 chip->driver_remove));
		if (res) return res;
		else if (chip->driver_remove) return 0;
		AMS_MUTEX_LOCK(&chip->lock);
	}

	read = read_fifo_data(chip, (uint16_t *)buf, _size);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return read;
}

/* static DEVICE_ATTR_RW(als_mode); */
static DEVICE_ATTR_RW(enable);
static DEVICE_ATTR_RO(id);
static DEVICE_ATTR_RO(auxid);
static DEVICE_ATTR_RO(revid);
static DEVICE_ATTR_RO(lux);
static DEVICE_ATTR_RO(cct);
static DEVICE_ATTR_RO(raw_data);
static DEVICE_ATTR_RO(chip_id);
static DEVICE_ATTR_RO(freq);

static BIN_ATTR_RO(fifo, PAGE_SIZE);

static struct attribute *tcs344x_attrs[] = {
	/* &dev_attr_als_mode.attr, */
	&dev_attr_enable.attr,
	&dev_attr_id.attr,
	&dev_attr_auxid.attr,
	&dev_attr_revid.attr,
	&dev_attr_lux.attr,
	&dev_attr_cct.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_freq.attr,
	NULL,
};

struct bin_attribute *tcs344x_bin_attrs[] = {
	&bin_attr_fifo,
	NULL,
};

struct attribute_group attrs_group = {
	.name = "als",
	.attrs = tcs344x_attrs,
	.bin_attrs = tcs344x_bin_attrs,
};

const struct attribute_group *tcs344x_sysfs_groups[] = {
	&attrs_group,
	NULL,
};

#if CONFIG_OF
static const struct of_device_id tcs344x_i2c_dt_ids[] = {
	{.compatible = "ams,tcs344x"},
	{}
};
MODULE_DEVICE_TABLE(of, tcs344x_i2c_dt_ids);
#endif

static int tcs344x_probe(struct i2c_client *client,
		const struct i2c_device_id *idp) {
	int ret, i;
	u8 id, rev, auxid;
	struct device *dev = &client->dev;
	static struct tcs344x_chip *chip;
	struct tcs344x_i2c_platform_data *pdata = dev->platform_data;
	static unsigned int irq_number;
	unsigned long default_irq_trigger = 0;
	u8 status;
	uint8_t chip_serial_number[5];
	uint64_t temp_chip_serial_number;

	pr_info("\nTCS344x: probe()\n");
	dev_info(dev, "%s: client->irq = %d\n", __func__, client->irq);

#if CONFIG_OF
	if (!pdata){
		pdata = kzalloc(sizeof(struct tcs344x_i2c_platform_data),
				GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		pdata->dts_recived = 0;
		if (of_match_device(tcs344x_i2c_dt_ids, &client->dev)) {
			pdata->of_node = client->dev.of_node;
			ret = tcs344x_init_dt(pdata);
			dev_info(dev, "%s: get default setting from dts\n", __func__);
			if (ret)
				return ret;
			pdata->dts_recived = 1;
		}
	}
#endif

	dev_info(dev, "%s: pdata->parameters.sai_enable = %d\n", __func__, pdata->parameters.sai_enable);

	/*
	 * Validate bus and device registration
	 */

	dev_info(dev, "%s: client->irq = %d\n", __func__, client->irq);
	dev_info(dev, "%s: als_name = %s\n", __func__, pdata->als_name );

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	if (!pdata) {
		dev_err(dev, "%s: platform data required\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}

	if (!(pdata->als_name) || client->irq < 0) {
		dev_err(dev, "%s: no reason to run.\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}

	if (pdata->platform_init) {
		ret = pdata->platform_init();
		if (ret)
			goto init_failed;
	}

	chip = kzalloc(sizeof(struct tcs344x_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	mutex_init(&chip->lock);
	chip->client = client;
	chip->driver_remove = false;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);

	ret = tcs344x_pltf_power_on(chip);
	if (ret) {
		dev_err(dev, "%s: pltf power on failed\n", __func__);
		goto malloc_failed;
	}

	// Make sure device is present and resonding on the I2C bus
	if (!tcs344x_present(chip)) {
		dev_err(dev, "%s: TCS344X Not Present\n", __func__);
		goto init_failed;
	}

	/*
	 * Initialize ALS
	 */
	if (!pdata->als_name) {
		goto bypass_als_idev;
	}
	chip->als_idev = input_allocate_device();
	if (!chip->als_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n", __func__,
				pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->als_idev->name = pdata->als_name;
	chip->als_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->als_idev->evbit);
	set_bit(ABS_MISC, chip->als_idev->absbit);
	set_bit(FIFO_DEPTH_EVENT, chip->als_idev->absbit);
	input_set_abs_params(chip->als_idev, ABS_MISC, 0, 65535, 0, 0);
	chip->als_idev->open = tcs344x_als_idev_open;
	chip->als_idev->close = tcs344x_als_idev_close;
	dev_set_drvdata(&chip->als_idev->dev, chip);
	chip->params.wtime = get_wait_time(chip);
	init_waitqueue_head(&chip->fifo_wait);
	/*
	 * Set chip defaults
	 */
	tcs344x_set_defaults(chip);

	/*
	 * Validate the appropriate ams device is available for this driver
	 */

	ret = tcs344x_get_id(chip, &id, &rev, &auxid, &chip_serial_number[0]);

	/*
	 * Get the serial nuber of the device, the 5 bytes from 0x0B - 0x0F
	 */
	chip->chip_serial_number = (((uint64_t)chip_serial_number[4]) << 32) | (chip_serial_number[3] << 24) | (chip_serial_number[2] << 16) \
										| (chip_serial_number[1] << 8) |  chip_serial_number[0];


	temp_chip_serial_number = (((uint64_t)chip_serial_number[4]) << 32) | (chip_serial_number[3] << 24) | (chip_serial_number[2] << 16) \
									  | (chip_serial_number[1] << 8) |  chip_serial_number[0];

	dev_info(dev, "%s: device id:%02x device aux id:%02x chip ID: %02llx device rev:%02x\n",
			__func__, id, auxid, temp_chip_serial_number, rev);

	for (i=0; i < ARRAY_SIZE(tcs344x_ids); i++)
	{
		if (auxid == tcs344x_ids[i])
		{
			chip->valid_auxid = true;
			break;
		}
	}

	if (chip->valid_auxid == true)
	{
		chip->id = id;
		chip->rev = rev;
		chip->auxid = auxid;
	}
	else
		goto id_failed;

	ret = input_register_device(chip->als_idev);
	if (ret) {
		input_free_device(chip->als_idev);
		dev_err(dev, "%s: cant register input '%s'\n", __func__,
				pdata->als_name);
		goto input_a_alloc_failed;
	}

	if (sysfs_create_groups(&chip->als_idev->dev.kobj, tcs344x_sysfs_groups)) {
		dev_err(&chip->als_idev->dev, "Error creating sysfs attribute group.\n");
		goto input_a_alloc_failed;
	}

	chip->als_input_open = false;
	/* If both Spectral enable and Wait Enable are set - use the thread */
	ams_i2c_read(chip->client, TCS344x_REGADDR_ENABLE, &status);
	if (ret) {
		goto input_a_sysfs_failed;
	}
#ifdef CONFIG_QUALCOMM_AP
	chip->als_cdev = als_sensors_cdev;
	chip->als_cdev.sensors_enable = tcs344x_als_set_enable;
	if (sensors_classdev_register(&chip->a_idev->dev, &chip->als_cdev)) {
		dev_err(dev, "sensors class register failed.\n");
	}
#endif

bypass_als_idev:
	/* Initialize IRQ & Handler */

	/* If this is a DTS build the following
	 * variable will be overwritten.
	 */
	irq_number = client->irq;
	default_irq_trigger = irqd_get_trigger_type(irq_get_irq_data(client->irq));
	ret = devm_request_threaded_irq(dev, client->irq,
			NULL, &tcs344x_irq,
			default_irq_trigger |
			IRQF_SHARED         |
			IRQF_ONESHOT,
			dev_name(dev), chip);
	if (ret) {
		dev_err(dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}

	INIT_KFIFO(ams_kfifo);

	tcs344x_pltf_power_off(chip);

	dev_info(dev, "Probe ok.\n");
	return 0;

	/*
	 * This must be unwound in the correct order, reverse
	 * from initialization above
	 */
irq_register_fail:
	if (chip->als_idev)
		sysfs_remove_groups(&chip->als_idev->dev.kobj, tcs344x_sysfs_groups);
input_a_sysfs_failed:
	if (chip->als_idev)
		input_unregister_device(chip->als_idev);

input_a_alloc_failed:
	/*
	 * Exit points for general device initialization failures
	 */
id_failed:
	i2c_set_clientdata(client, NULL);

malloc_failed:
	if (pdata->platform_power)
		pdata->platform_power(dev, POWER_OFF);
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	kfree(pdata);
	kfree(chip);
	dev_err(dev, "Probe failed.\n");
	return ret;
}

static int tcs344x_suspend(struct device *dev)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);

	pr_info("\nTCS344x: suspend()\n");
	dev_info(dev, "%s\n", __func__);
	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 1;

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		dev_info(dev, "powering off\n");
		tcs344x_pltf_power_off(chip);
	}
	AMS_MUTEX_UNLOCK(&chip->lock);

	return 0;
}

static int tcs344x_resume(struct device *dev)
{
	struct tcs344x_chip *chip = dev_get_drvdata(dev);
	bool als_on;

	return 0;
	pr_info("\nTCS344x: resume()\n");
	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 0;

	dev_info(dev, "%s: powerd %d, als: needed %d  enabled %d", __func__,
			!chip->unpowered, als_on, chip->enabled);

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}

	/* err_power: */
	AMS_MUTEX_UNLOCK(&chip->lock);

	return 0;
}

static void tcs344x_remove(struct i2c_client *client)
{
	struct tcs344x_chip *chip = i2c_get_clientdata(client);

	chip->driver_remove = true;
	dev_info(&client->dev, "%s\n", __func__);
	devm_free_irq(&client->dev, client->irq, chip);
	tcs344x_enable_device(chip, 0);
	tcs344x_pltf_power_off(chip);
	if (chip->als_idev) {
		sysfs_remove_groups(&chip->als_idev->dev.kobj, tcs344x_sysfs_groups);
		input_unregister_device(chip->als_idev);
	}

	if (chip->als_input_open == true)
	{
		chip->als_idev->close(chip->als_idev);
		chip->als_input_open = false;
	}

#ifdef CONFIG_QUALCOMM_AP
	sensors_classdev_unregister(&chip->als_cdev);
#endif
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
#if CONFIG_OF
	kfree(chip->pdata);
#endif
	kfree(chip);
	return;
}

#if 0
/*
 * This function is to set the I2C operating voltage of TCS344x
 */
static int tcs344x_set_i2C-op_voltage(struct tcs344x_chip *chip, enum voltage)
{
	return 0;
}
#endif

static struct i2c_device_id tcs344x_idtable[] = {{"tcs344x", 0}, {} };
MODULE_DEVICE_TABLE(i2c, tcs344x_idtable);

static const struct dev_pm_ops tcs344x_pm_ops = {.suspend = tcs344x_suspend,
	.resume = tcs344x_resume,};

static struct i2c_driver tcs344x_driver = {
	.driver = {.name = "tcs344x",
		.pm = &tcs344x_pm_ops,},
	.id_table = tcs344x_idtable,
	.probe = tcs344x_probe,
	.remove = tcs344x_remove,};

module_i2c_driver(tcs344x_driver);

MODULE_DESCRIPTION("AMS tcs344x Spectral ALS sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.8");
