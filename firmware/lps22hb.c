/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>

#include "sensor.h"
#include "twi.h"

#define LPS22HB_ADDR 0x5c
#define LPS22HB_CHIP_ID 0xb1

#define REG_INTERRUPT_CFG	0x0b
#define REG_THS_P_L		0x0f
#define REG_THS_P_H		0x0f
#define REG_WHO_AM_I		0x0f
#define REG_CTRL_REG1		0x10
#define  CTRL_REG1_SIM		(1 << 0)
#define  CTRL_REG1_BDU		(1 << 1)
#define  CTRL_REG1_LPFP_CFG	(1 << 2)
#define  CTRL_REG1_EN_LPFP	(1 << 3)
#define  CTRL_REG1_ODR_OFF	(0 << 4)
#define  CTRL_REG1_ODR_1HZ	(1 << 4)
#define  CTRL_REG1_ODR_10HZ	(2 << 4)
#define  CTRL_REG1_ODR_25HZ	(3 << 4)
#define  CTRL_REG1_ODR_50HZ	(4 << 4)
#define  CTRL_REG1_ODR_75HZ	(5 << 4)
#define REG_CTRL_REG2		0x11
#define  CTRL_REG2_ONE_SHOT	(1 << 0)
#define  CTRL_REG2_SWRESET	(1 << 2)
#define  CTRL_REG2_I2C_DIS	(1 << 3)
#define  CTRL_REG2_IF_ADD_INC	(1 << 4)
#define  CTRL_REG2_STOP_ON_FTH	(1 << 5)
#define  CTRL_REG2_FIFO_EN	(1 << 6)
#define  CTRL_REG2_BOOT		(1 << 7)
#define REG_CTRL_REG3		0x12
#define  CTRL_REG3_INT_S1	(1 << 0)
#define  CTRL_REG3_INT_S2	(1 << 1)
#define  CTRL_REG3_DRDY		(1 << 2)
#define  CTRL_REG3_F_OVR	(1 << 3)
#define  CTRL_REG3_F_FTH	(1 << 4)
#define  CTRL_REG3_F_FSS	(1 << 5)
#define  CTRL_REG3_PP_OD	(1 << 6)
#define  CTRL_REG3_INT_H_L	(1 << 7)

#define REG_RES_CONF		0x1a
#define  RES_CONF_LC_EN		(1 << 0)

#define REG_INT_SOURCE		0x25
#define  INT_SOURCE_PH		(1 << 0)
#define  INT_SOURCE_PL		(1 << 1)
#define  INT_SOURCE_IA		(1 << 2)
#define  INT_SOURCE_BOOT	(1 << 7)

#define REG_PRESS_OUT_XL	0x28
#define REG_PRESS_OUT_L		0x29
#define REG_PRESS_OUT_H		0x2a

#define REG_TEMP_OUT_L		0x2b
#define REG_TEMP_OUT_H		0x2c

static uint8_t lps22hb_read_reg(uint8_t reg)
{
	uint8_t buf;

	buf = reg;
	twi_transfer(LPS22HB_ADDR, &buf, 1, &buf, sizeof(buf));

	return buf;
}

static void lps22hb_write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf[2];

	buf[0] = reg;
	buf[1] = val;
	twi_transfer(LPS22HB_ADDR, buf, sizeof(buf), NULL, 0);
}

static void lps22hb_soft_reset(void)
{
	lps22hb_write_reg(REG_CTRL_REG2, CTRL_REG2_BOOT);
	while (lps22hb_read_reg(REG_CTRL_REG2) & CTRL_REG2_BOOT);

	lps22hb_write_reg(REG_CTRL_REG2, CTRL_REG2_SWRESET);
	while (lps22hb_read_reg(REG_CTRL_REG2) & CTRL_REG2_SWRESET);
}

static void lps22hb_init(void)
{
	lps22hb_soft_reset();

	/* open drain and active low INT_DRDY pin */
	lps22hb_write_reg(REG_CTRL_REG3, CTRL_REG3_PP_OD | CTRL_REG3_INT_H_L);
}

static bool lps22hb_is_present(void)
{
	return lps22hb_read_reg(REG_WHO_AM_I) == LPS22HB_CHIP_ID;
}

static void lps22hb_start_measurement(void)
{
	lps22hb_write_reg(REG_CTRL_REG2,
			  CTRL_REG2_IF_ADD_INC | CTRL_REG2_ONE_SHOT);
}

static uint16_t lps22hb_read_pressure(void)
{
	uint8_t buf[2];

	buf[0] = REG_PRESS_OUT_L;
	twi_transfer(LPS22HB_ADDR, buf, 1, buf, sizeof(buf));

	return (buf[1] << 4) | (buf[0] >> 4);
}

struct sensor_driver lps22hb_driver = {
	.is_present = lps22hb_is_present,
	.init = lps22hb_init,
	.start_measurement = lps22hb_start_measurement,
	.read_pressure = lps22hb_read_pressure,
};
