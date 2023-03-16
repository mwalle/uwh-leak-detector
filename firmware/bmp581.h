/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#ifndef __BMP581_H
#define __BMP581_H

void bmp581_init(void);
uint16_t bmp581_read_pressure(void);
uint8_t bmp581_read_temp(void);
uint16_t bmp581_one_shot(void);
void bmp581_enable_oor_mode(void);
void bmp581_disable_oor_mode(void);
void bmp581_configure_oor(uint16_t p, uint8_t window);
void bmp581_clear_int_status(void);

#endif /* __BMP581_H */

