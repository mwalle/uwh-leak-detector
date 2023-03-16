/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#include <stdbool.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "bmp581.h"
#include "twi.h"

#define BMP581_ADDR 0x46

#define REG_INT_CONFIG		0x14
#define  INT_MODE		_BV(0)
#define  INT_POL		_BV(1)
#define  INT_OD			_BV(2)
#define  INT_EN			_BV(3)
#define  INT_DRV(x)		((x) << 4)
#define REG_INT_SOURCE		0x15
#define  INT_SRC_DRDY		_BV(0)
#define  INT_SRC_FIFO_FULL	_BV(1)
#define  INT_SRC_FIFO_THS	_BV(2)
#define  INT_SRC_OOR_P_EN	_BV(3)

#define REG_TEMP_DATA_XLSB	0x1d
#define REG_TEMP_DATA_LSB	0x1e
#define REG_TEMP_DATA_MSB	0x1f
#define REG_PRESS_DATA_XLSB	0x20
#define REG_PRESS_DATA_LSB	0x21
#define REG_PRESS_DATA_MSB	0x22

#define REG_INT_STATUS		0x27
#define  INT_STATUS_DRDY	_BV(0)
#define REG_DSP_CONFIG		0x30

#define REG_DSP_IIR_CONFIG	0x31
#define  DSP_IIR_T_BYPASS	(0 << 0)
#define  DSP_IIR_T_COEFF_1	(1 << 0)
#define  DSP_IIR_T_COEFF_3	(2 << 0)
#define  DSP_IIR_T_COEFF_7	(3 << 0)
#define  DSP_IIR_T_COEFF_15	(4 << 0)
#define  DSP_IIR_T_COEFF_31	(5 << 0)
#define  DSP_IIR_T_COEFF_63	(6 << 0)
#define  DSP_IIR_T_COEFF_127	(7 << 0)
#define  DSP_IIR_P_BYPASS	(0 << 3)
#define  DSP_IIR_P_COEFF_1	(1 << 3)
#define  DSP_IIR_P_COEFF_3	(2 << 3)
#define  DSP_IIR_P_COEFF_7	(3 << 3)
#define  DSP_IIR_P_COEFF_15	(4 << 3)
#define  DSP_IIR_P_COEFF_31	(5 << 3)
#define  DSP_IIR_P_COEFF_63	(6 << 3)
#define  DSP_IIR_P_COEFF_127	(7 << 3)

#define REG_OOR_THR_P_LSB	0x32
#define REG_OOR_THR_P_MSB	0x33
#define REG_OOR_RANGE		0x34
#define REG_OOR_CONFIG		0x35
#define  OOR_THR_BIT16		_BV(0)
#define  OOR_CNT_LIM_1		(0 << 6)
#define  OOR_CNT_LIM_3		(1 << 6)
#define  OOR_CNT_LIM_7		(2 << 6)
#define  OOR_CNT_LIM_15		(3 << 6)
#define REG_OSR_CONFIG		0x36
#define  OSR_PRESS_EN		_BV(6)
#define  OSR_T_OVR_SAMPLE_1X	(0 << 0)
#define  OSR_T_OVR_SAMPLE_2X	(1 << 0)
#define  OSR_T_OVR_SAMPLE_4X	(2 << 0)
#define  OSR_T_OVR_SAMPLE_8X	(3 << 0)
#define  OSR_T_OVR_SAMPLE_16X	(4 << 0)
#define  OSR_T_OVR_SAMPLE_32X	(5 << 0)
#define  OSR_T_OVR_SAMPLE_64X	(6 << 0)
#define  OSR_T_OVR_SAMPLE_128X	(7 << 0)
#define  OSR_P_OVR_SAMPLE_1X	(0 << 3)
#define  OSR_P_OVR_SAMPLE_2X	(1 << 3)
#define  OSR_P_OVR_SAMPLE_4X	(2 << 3)
#define  OSR_P_OVR_SAMPLE_8X	(3 << 3)
#define  OSR_P_OVR_SAMPLE_16X	(4 << 3)
#define  OSR_P_OVR_SAMPLE_32X	(5 << 3)
#define  OSR_P_OVR_SAMPLE_64X	(6 << 3)
#define  OSR_P_OVR_SAMPLE_128X	(7 << 3)

#define REG_ODR_CONFIG		0x37
#define  ODR_STANDBY_MODE	(0 << 0)
#define  ODR_NORMAL_MODE	(1 << 0)
#define  ODR_FORCED_MODE	(2 << 0)
#define  ODR_NON_STOP_MODE	(3 << 0)
#define  ODR_SEL_240_HZ		(0 << 2)
#define  ODR_SEL_218_HZ		(1 << 2)
#define  ODR_SEL_200_HZ		(2 << 2)
#define  ODR_SEL_180_HZ		(3 << 2)
#define  ODR_SEL_160_HZ		(4 << 2)
#define  ODR_SEL_150_HZ		(5 << 2)
#define  ODR_SEL_140_HZ		(6 << 2)
#define  ODR_SEL_130_HZ		(7 << 2)
#define  ODR_SEL_120_HZ		(8 << 2)
#define  ODR_SEL_110_HZ		(9 << 2)
#define  ODR_SEL_100_HZ		(10 << 2)
#define  ODR_SEL_90_HZ		(11 << 2)
#define  ODR_SEL_80_HZ		(12 << 2)
#define  ODR_SEL_70_HZ		(13 << 2)
#define  ODR_SEL_60_HZ		(14 << 2)
#define  ODR_SEL_50_HZ		(15 << 2)
#define  ODR_SEL_45_HZ		(16 << 2)
#define  ODR_SEL_40_HZ		(17 << 2)
#define  ODR_SEL_35_HZ		(18 << 2)
#define  ODR_SEL_30_HZ		(19 << 2)
#define  ODR_SEL_25_HZ		(20 << 2)
#define  ODR_SEL_20_HZ		(21 << 2)
#define  ODR_SEL_15_HZ		(22 << 2)
#define  ODR_SEL_10_HZ		(23 << 2)
#define  ODR_SEL_5_HZ		(24 << 2)
#define  ODR_SEL_4_HZ		(25 << 2)
#define  ODR_SEL_3_HZ		(26 << 2)
#define  ODR_SEL_2_HZ		(27 << 2)
#define  ODR_SEL_1_HZ		(28 << 2)
#define  ODR_SEL_0_5_HZ		(29 << 2)
#define  ODR_SEL_0_25_HZ	(30 << 2)
#define  ODR_SEL_0_125_HZ	(31 << 2)
#define  ODR_DEEP_DIS		_BV(7)
#define REG_CMD 0x7e
#define  CMD_SOFT_RESET 0xb6

static void bmp581_write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf[2];

	buf[0] = reg;
	buf[1] = val;

	twi_transfer(BMP581_ADDR, buf, sizeof(buf), NULL, 0);
}

static void bmp581_soft_reset(void)
{
	bmp581_write_reg(REG_CMD, CMD_SOFT_RESET);
	_delay_ms(2);
}

void bmp581_init(void)
{
	bmp581_soft_reset();

	/* lowest resolution */
	bmp581_write_reg(REG_OSR_CONFIG,
			 OSR_T_OVR_SAMPLE_1X |
			 OSR_P_OVR_SAMPLE_1X |
			 OSR_PRESS_EN);

	bmp581_write_reg(REG_ODR_CONFIG, ODR_STANDBY_MODE);
}

void bmp581_configure_oor(uint16_t p, uint8_t window)
{
	bmp581_write_reg(REG_OOR_THR_P_LSB, (p << 2));
	bmp581_write_reg(REG_OOR_THR_P_MSB, (p >> 6));
	if (p & 0x4000)
		bmp581_write_reg(REG_OOR_CONFIG, OOR_CNT_LIM_3 | OOR_THR_BIT16);
	else
		bmp581_write_reg(REG_OOR_CONFIG, OOR_CNT_LIM_3);
}

void bmp581_enable_oor_mode(void)
{
	/* enable oor pressure interrupt */
	//bmp581_write_reg(REG_INT_SOURCE, INT_SRC_OOR_P_EN | INT_SRC_DRDY);
	bmp581_write_reg(REG_INT_SOURCE, INT_SRC_DRDY);

	/* open-drain, interrupt pulse, enable interrupt */
	bmp581_write_reg(REG_INT_CONFIG, INT_OD | INT_EN | INT_DRV(3));
	//bmp581_write_reg(REG_INT_CONFIG, INT_POL | INT_EN | INT_DRV(3));

	/* normal mode, one samples every two seconds */
	bmp581_write_reg(REG_ODR_CONFIG, ODR_NORMAL_MODE | ODR_SEL_2_HZ);
}

void bmp581_disable_oor_mode(void)
{
	/* open-drain, disable interrupt */
	bmp581_write_reg(REG_INT_CONFIG, INT_OD | INT_DRV(3));

	/* standby mode */
	bmp581_write_reg(REG_ODR_CONFIG, ODR_STANDBY_MODE);
}

uint16_t bmp581_read_pressure(void)
{
	uint8_t buf[3];

	buf[0] = REG_PRESS_DATA_XLSB;

	twi_transfer(BMP581_ADDR, buf, 1, buf, sizeof(buf));

	return buf[2] << 8 | buf[1];
}

uint8_t bmp581_read_temp(void)
{
	uint8_t buf[1];

	buf[0] = REG_TEMP_DATA_MSB;

	twi_transfer(BMP581_ADDR, buf, 1, buf, sizeof(buf));

	return buf[0];
}

void bmp581_clear_int_status(void)
{
	uint8_t buf;

	buf = REG_TEMP_DATA_MSB;

	twi_transfer(BMP581_ADDR, &buf, 1, &buf, 1);
}

uint16_t bmp581_one_shot(void)
{
	bmp581_write_reg(REG_ODR_CONFIG, ODR_FORCED_MODE);
	_delay_ms(5);
	return bmp581_read_pressure();
}
