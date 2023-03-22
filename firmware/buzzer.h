/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#ifndef __BUZZER_H
#define __BUZZER_H

void __buzzer_tick(void);
void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);

#endif /* __BUZZER_H */
