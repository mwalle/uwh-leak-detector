/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#ifndef __TICKS_H
#define __TICKS_H

uint16_t get_ticks(void);
extern volatile uint16_t __ticks;

#endif /* __TICKS_H */
