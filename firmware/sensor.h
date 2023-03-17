/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#ifndef __SENSOR_H
#define __SENSOR_H

struct sensor_driver {
	bool (*is_present)(void);
	void (*init)(void);
	uint16_t (*one_shot)(void);
	void (*start_measurement)(void);
	uint16_t (*read_pressure)(void);
};

extern struct sensor_driver bmp581_driver;
extern struct sensor_driver demo_driver;

#endif /* __SENSOR_H */
