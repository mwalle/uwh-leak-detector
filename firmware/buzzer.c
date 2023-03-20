/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "buzzer.h"

extern volatile uint16_t __ticks;

ISR(TIM0_COMPB_vect)
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
	OCR0A = (F_CPU / 8 / 2000);
	OCR0B = (F_CPU / 8 / 2000 / 2);

	/* enable pull-up resistor */
	PORTB |= _BV(PB1);
}

void buzzer_on(void)
{
	TIMSK &= ~_BV(OCIE0B);
	DDRB |= _BV(PB1);
	TCCR0B |= _BV(CS01);
}

void buzzer_toggle(void)
{
	TIMSK |= _BV(OCIE0B);
	TCCR0B |= _BV(CS01);
}

void buzzer_off(void)
{
	TCCR0B &= ~_BV(CS01);
	DDRB &= ~_BV(PB1);
}
