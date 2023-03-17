/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#include <stdbool.h>
#include <avr/pgmspace.h>

#include "sensor.h"

#define MBAR(x)			((uint32_t)(x) * 100 >> 2)

static const uint16_t demo_values[] PROGMEM = {
	MBAR(985), MBAR(850), MBAR(800), MBAR(805),
	MBAR(820), MBAR(820), MBAR(985), MBAR(1000),
};

static uint16_t demo_read_pressure(void)
{
	static uint8_t cnt = 0;
	uint16_t ret;

	ret = pgm_read_word(&demo_values[cnt >> 5]);
	if (cnt < 0xff)
		cnt++;

	return ret;
}

static bool demo_is_present(void)
{
	return true;
}

static void demo_init(void)
{
}

static uint16_t demo_one_shot(void)
{
	return demo_read_pressure();
}

static void demo_start_measurement(void)
{
}

struct sensor_driver demo_driver = {
	.is_present = demo_is_present,
	.init = demo_init,
	.one_shot = demo_one_shot,
	.start_measurement = demo_start_measurement,
	.read_pressure = demo_read_pressure,
};
