/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "ticks.h"
#include "buzzer.h"

#define BUZZER_HZ 2000

void __buzzer_tick(void)
{
	if (__ticks & 0x2)
		DDRB |= _BV(PB1);
	else
		DDRB &= ~_BV(PB1);
}

void buzzer_init(void)
{
	/* PWM mode, output active low, prescaler to 8 */
	TCCR0A = _BV(WGM01) | _BV(WGM00) | _BV(COM0B1);
	TCCR0B = _BV(WGM02) | _BV(CS01);
	OCR0A = (F_CPU / 8 / BUZZER_HZ);
	OCR0B = (F_CPU / 8 / BUZZER_HZ / 2);

	/* enable pull-up resistor */
	PORTB |= _BV(PB1);
}

void buzzer_on(void)
{
	DDRB |= _BV(PB1);
	TCCR0B |= _BV(CS01);
}

void buzzer_off(void)
{
	TCCR0B &= ~_BV(CS01);
	DDRB &= ~_BV(PB1);
}
