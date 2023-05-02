/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#include <stdbool.h>
#include <avr/pgmspace.h>

#include "sensor.h"

static const uint16_t demo_values[] PROGMEM = {
	985, 850, 800, 805, 820, 820, 985, 1000,
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

static void demo_start_measurement(void)
{
}

struct sensor_driver demo_driver = {
	.name = "demo",
	.is_present = demo_is_present,
	.init = demo_init,
	.start_measurement = demo_start_measurement,
	.read_pressure = demo_read_pressure,
};
