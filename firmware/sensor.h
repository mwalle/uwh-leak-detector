/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#ifndef __SENSOR_H
#define __SENSOR_H

#include <util/delay.h>

struct sensor_driver {
	bool (*is_present)(void);
	void (*init)(void);
	void (*start_measurement)(void);
	uint16_t (*read_pressure)(void);
};

static inline bool sensor_is_present(struct sensor_driver *drv)
{
	return drv->is_present();
}

static inline void sensor_init(struct sensor_driver *drv)
{
	drv->init();
}

static inline void sensor_start_measurement(struct sensor_driver *drv)
{
	drv->start_measurement();
}

static inline uint16_t sensor_read_pressure(struct sensor_driver *drv)
{
	return drv->read_pressure();
}

static inline uint16_t sensor_one_shot(struct sensor_driver *drv)
{
	sensor_start_measurement(drv);
	_delay_ms(5);
	return sensor_read_pressure(drv);
}

extern struct sensor_driver bmp581_driver;
extern struct sensor_driver demo_driver;

#endif /* __SENSOR_H */
