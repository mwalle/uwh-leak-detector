/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * lead-detector - simple pressure based leak detector
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "twi.h"
#include "swuart.h"
#include "bmp581.h"

#define DEBUG 1

static void leds_init(void)
{
	DDRB |= _BV(PB3) | _BV(PB4);
	PORTB |= ~(_BV(PB3) | _BV(PB4));
}

static void leds_pressure_ok(void)
{
	PORTB &= ~_BV(PB4);
	PORTB |= _BV(PB3);
}

static void leds_alarm(void)
{
	PORTB &= ~_BV(PB3);
	PORTB |= _BV(PB4);
}

static void leds_off(void)
{
	PORTB |= _BV(PB3) | _BV(PB4);
}

static void buzzer_on(void)
{
	/*
	 * The system clock is 1 MHz, use the 4096 prescaler and a counter
	 * value of 244.
	 */

	/* PWM mode, output active low, prescaler to 16 */
	TCCR1 = _BV(PWM1A) | _BV(COM1A1) | _BV(CS12) | _BV(CS10);
	OCR1A = (F_CPU / 16 / 2000 / 2);
	OCR1C = (F_CPU / 16 / 2000);
	DDRB |= _BV(PB1);
}

static void buzzer_off(void)
{
	TCCR1 = 0;
	DDRB &= ~_BV(PB1);
}

#define P_MBAR(x)	(x * 100 >> 2)
/* hyst of 10mbar for the alarm limit and 100mbar for turn on limit */
#define P_HYST_ALARM	P_MBAR(10)
#define P_HYST_ON	P_MBAR(100)

enum state {
	IDLE,
	NO_LEAK,
	LEAK,
};

static void print_pressure(char *buf, uint16_t p)
{
	uart_puts(itoa(p / (100 / 4), buf, 10));
	uart_puts_P(PSTR("mbar"));
}

static void print_startup(uint16_t p_turn_on)
{
	uint8_t t = bmp581_read_temp();
	char buf[8];

	uart_puts_P(PSTR("T="));
	uart_puts(itoa(t, buf, 10));
	uart_puts_P(PSTR("degC "));

	uart_puts_P(PSTR("p_turn_on="));
	print_pressure(buf, p_turn_on);
	uart_puts_P(PSTR("\n"));
}

static void print_status(enum state state, uint16_t p, uint16_t p_alarm)
{
	char buf[8];

	uart_puts_P(PSTR("S="));
	uart_puts(itoa(state, buf, 10));
	uart_puts_P(PSTR(" p="));
	print_pressure(buf, p);
	uart_puts_P(PSTR(" p_alarm="));
	print_pressure(buf, p_alarm);
	uart_puts_P(PSTR("\n"));
}

int main(void)
{
	uint16_t p_turn_on, p_alarm, p;
	enum state state;

	/* clear watchdog */
	MCUSR = 0;
	WDTCR |= _BV(WDCE) | _BV(WDE);
	WDTCR = 0;

	leds_init();
	uart_init();
	bmp581_init();

	sei();

	leds_pressure_ok();
	_delay_ms(1000);
	leds_alarm();
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	leds_off();

	state = IDLE;
	p_turn_on = bmp581_read_pressure() - P_HYST_ON;
	p_alarm = 0;

	if (DEBUG)
		print_startup(p_turn_on);

	while (true) {
		p = bmp581_read_pressure();
		if (DEBUG)
			print_status(state, p, p_alarm);

		switch (state) {
		case IDLE:
			if (p < p_turn_on) {
				p_alarm = p + P_HYST_ALARM;
				leds_pressure_ok();
				state = NO_LEAK;
			}
			break;
		case NO_LEAK:
			if (p > p_alarm) {
				leds_alarm();
				buzzer_on();
				state = LEAK;
			} else if (p + P_HYST_ALARM < p_alarm) {
				p_alarm = p + P_HYST_ALARM;
			}
			break;
		case LEAK:
			if (p > p_turn_on) {
				leds_off();
				buzzer_off();
				p_alarm = 0;
				state = IDLE;
			}
			break;
		}
		_delay_ms(500);
	};

	return 0;
}
