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
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "twi.h"
#include "swuart.h"
#include "bmp581.h"

#define DEBUG 1

/*
 * ticks will count the time since the last power-down (or start-up).
 * The watchdog timer period is 250ms, which gives us approximately
 * 4.5 hours (2^16 * 250ms) runtime until the counter overflows.
 **/
static volatile uint16_t __ticks;

enum state {
	/*
	 * Device was woken up by the pressure sensor and is now constantly
	 * sampling the pressure. On device reset the current pressure is saved
	 * as the "normal" value. There is a window around this normal value
	 * where it is considered that the sensor is not in use. If after some
	 * time, the pressure is still in this window, the state is
	 * transitioned back to the POWERED_DOWN state. If the safe pressure
	 * threshold is reached though, the state transition to PRESSURE_OK.
	 */
	IDLE,

	/*
	 * Device is powered-down, all clocks are turned off. Wake-up source is
	 * the watchdog interrupt. The watchdog interval is set to 2s to consume
	 * power. If the pressure is below the on-off threshold, the state will
	 * transition to to IDLE.
	 */
	POWERED_DOWN,

	/*
	 * Device constantly measures the pressure. Pressure is only allowed to
	 * go down (with a hysteresis). If the pressure goes up, a leak is
	 * detected and the state transitions to LEAK.
	 */
	PRESSURE_OK,

	/*
	 * Device detected a leak. If pressure reaches the "normal" value
	 * window again, the state transisitons to IDLE.
	 */
	LEAK,
};

/* lowest bit forces update */
enum led_state {
	LED_OFF		= 0x00,
	LED_IDLE	= 0x01,		/* flashing green */
	LED_PRESSURE_OK = 0x02,		/* solid green */
	LED_BAT_LOW	= 0x03,		/* alternating between green and red */
	LED_ERROR	= 0x04,		/* solid red */
	LED_LEAK	= 0x05,		/* flashing red */
};

#define LED_RED PB3
#define LED_GREEN PB4

struct context {
	uint16_t ticks;
	uint16_t p;
	uint16_t p_idle;
	uint16_t p_on_off;
	uint16_t p_alarm;
	uint16_t vbat;
	uint8_t state;
	uint8_t flags;
};

#define FLAGS_BAT_LOW_DETECTED (1 << 1)

static volatile uint8_t __led_state;
static volatile uint8_t __flags;
#define F_DEBUG (1 << 0)
#define F_DEMO (1 << 1)

static void buzzer_init(void)
{
	/* PWM mode, output active low, prescaler to 8 */
	TCCR0A = _BV(WGM01) | _BV(WGM00) | _BV(COM0B1);
	TCCR0B = _BV(WGM02) | _BV(CS01);
	OCR0A = (F_CPU / 8 / 2000);
	OCR0B = (F_CPU / 8 / 2000 / 2);
}

static void buzzer_on(void)
{
	TCCR0B |= _BV(CS01);
	DDRB |= _BV(PB1);
}

static void buzzer_off(void)
{
	TCCR0B &= ~_BV(CS01);
	DDRB &= ~_BV(PB1);
}

static void update_buzzer(void)
{
	return;

	if (__ticks & 0x2)
		buzzer_on();
	else
		buzzer_off();
}

/* Update the LEDs, called from ISR. */
static void update_led(void)
{
	static uint8_t old_led_state = 0;

	if (__flags & F_DEBUG)
		return;

	if (!(__led_state & 1) && old_led_state == __led_state)
		return;

	/* enable or disable output driver to consume power */
	switch (__led_state) {
	case LED_OFF:
		PORTB |= _BV(LED_GREEN);
		PORTB |= _BV(LED_RED);
		break;
	case LED_PRESSURE_OK:
		PORTB |= _BV(LED_RED);
		PORTB &= ~_BV(LED_GREEN);
		break;
	case LED_ERROR:
		PORTB |= _BV(LED_GREEN);
		PORTB &= ~_BV(LED_RED);
		break;
	case LED_IDLE:
		PORTB |= _BV(LED_RED);
		if (__ticks & 0x3)
			PORTB |= _BV(LED_GREEN);
		else
			PORTB &= ~_BV(LED_GREEN);
		break;
	case LED_LEAK:
		PORTB |= _BV(LED_GREEN);
		if (__ticks & 0x3)
			PORTB |= _BV(LED_RED);
		else
			PORTB &= ~_BV(LED_RED);
		break;
	case LED_BAT_LOW:
		if (__ticks & 0x2) {
			PORTB |= _BV(LED_RED);
			PORTB &= ~_BV(LED_GREEN);
		} else {
			PORTB |= _BV(LED_GREEN);
			PORTB &= ~_BV(LED_RED);
		}
	};

	old_led_state = __led_state;
}

static void set_led(uint8_t led_state)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		__led_state = led_state;
	}
}

ISR(WDT_vect)
{
	__ticks++;
	update_buzzer();
	update_led();
}

static uint16_t get_ticks(void)
{
	uint16_t _ticks;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		_ticks = __ticks;
	}
	return _ticks;
}

static void zero_ticks(void)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		__ticks = 0;
	}
}

#define IDLE_TIMEOUT (60 * 4)

static void wdt_power_down_mode()
{
	/* set watchdog period to 2s */
	WDTCR = _BV(WDIE) | _BV(WDP2) | _BV(WDP1) | _BV(WDP0);
}

static void wdt_normal_mode()
{
	/* enable interrupt and set period to 250ms */
	WDTCR = _BV(WDIE) | _BV(WDP2);
}

static void wdt_init(void)
{
	/*
	 * Watchdog enable survives an external reset, so disable the watchdog
	 * first. Just to be sure, in case there was a malfunction in our
	 * program.
	 */
	wdt_disable();

	wdt_normal_mode();
}

static void power_down(void)
{
	PORTB |= _BV(PB3);
	PORTB |= _BV(PB4);

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
}

static void idle_mode(void)
{
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_mode();
}

#define TO_MBAR(p)	(p / (100 / 4))
#define MBAR(x)		((x) * 100 >> 2)
/* hyst of 10mbar for the alarm limit and 100mbar for turn on limit */
#define P_HYST_ALARM	MBAR(10)
#define P_HYST_IDLE	MBAR(30)
#define P_HYST_ON_OFF	MBAR(100)

static void print_vbat(uint16_t vbat, char *buf)
{
	int millivolt = 110 * 256;

	millivolt /= vbat;
	uart_puts(itoa(millivolt, buf, 10));
	uart_puts_P(PSTR("0mV"));
}

static void print_status(struct context *ctx)
{
	char buf[8];

	uart_puts_P(PSTR("T="));
	uart_puts(itoa(get_ticks(), buf, 10));
	uart_puts_P(PSTR(" S="));
	uart_puts(itoa(ctx->state, buf, 10));
	uart_puts_P(PSTR(" F="));
	uart_puts(itoa(ctx->flags, buf, 16));
	uart_puts_P(PSTR(" p="));
	uart_puts(itoa(TO_MBAR(ctx->p), buf, 10));
	uart_puts_P(PSTR(" p_alarm="));
	uart_puts(itoa(TO_MBAR(ctx->p_alarm), buf, 10));
	uart_puts_P(PSTR(" "));
	uart_puts_P(PSTR("p_on_off="));
	uart_puts(itoa(TO_MBAR(ctx->p_on_off), buf, 10));
	uart_puts_P(PSTR(" "));
	uart_puts_P(PSTR("vbat="));
	print_vbat(ctx->vbat, buf);
	uart_puts_P(PSTR("\n"));
}

static void adc_init(void)
{
	/* Vref = Vcc, input = Vbg */
	ADMUX = _BV(MUX3) | _BV(MUX2) | _BV(ADLAR);
	_delay_ms(1);
	/* 128 kHz clock */
	ADCSRA = _BV(ADPS1) | _BV(ADPS0);
}

static uint8_t vbat_voltage(void)
{
	ADCSRA |= _BV(ADEN);
	_delay_ms(1);
	ADCSRA |= _BV(ADSC);
	loop_until_bit_is_clear(ADCSRA, ADSC);
	ADCSRA &= ~_BV(ADEN);

	return ADCH;
}

static void trigger_state_machine(struct context *ctx)
{
	uint8_t state = ctx->state;

	switch (ctx->state) {
	case POWERED_DOWN:
		zero_ticks();
		if (ctx->p < ctx->p_idle) {
			set_led(LED_IDLE);
			wdt_normal_mode();
			state = IDLE;
		}
		break;
	case IDLE:
		if (ctx->p < ctx->p_on_off) {
			ctx->p_alarm = ctx->p + P_HYST_ALARM;
			set_led(LED_PRESSURE_OK);
			state = PRESSURE_OK;
		} else if (ctx->ticks >= IDLE_TIMEOUT) {
			set_led(LED_OFF);
			wdt_power_down_mode();
			state = POWERED_DOWN;
		}
		break;
	case PRESSURE_OK:
		if (ctx->p > ctx->p_alarm) {
			set_led(LED_LEAK);
			buzzer_on();
			state = LEAK;
		} else if (ctx->p + P_HYST_ALARM < ctx->p_alarm) {
			ctx->p_alarm = ctx->p + P_HYST_ALARM;
		}
		break;
	case LEAK:
		if (ctx->p > ctx->p_on_off) {
			set_led(LED_IDLE);
			buzzer_off();
			ctx->p_alarm = 0;
			state = IDLE;
		}
		break;
	}

	ctx->state = state;
}

void led_init(void)
{
	PORTB |= _BV(PB3);
	DDRB |= _BV(PB3);
	PORTB |= _BV(PB4);
	DDRB |= _BV(PB4);
}

static void selftest(void)
{
	if (__flags & F_DEBUG)
		return;

	PORTB &= ~_BV(PB3);
	_delay_ms(500);
	PORTB |= _BV(PB3);
	PORTB &= ~_BV(PB4);
	_delay_ms(500);
	PORTB |= _BV(PB4);
	buzzer_on();
	_delay_ms(200);
	buzzer_off();
}

static volatile uint8_t nvflags __attribute__ ((section (".noinit")));

int main(void)
{
	struct context ctx = { 0 };
	uint8_t mcusr;

	mcusr = MCUSR;
	MCUSR = 0;
	wdt_init();

	if (mcusr & _BV(PORF))
		nvflags = 0;

	if (nvflags)
		__flags = F_DEBUG;
	nvflags = 1;

	led_init();
	adc_init();
	buzzer_init();
	uart_init();
	bmp581_init();

	/* disable unused blocks */
	ACSR |= _BV(ACD);

	selftest();

	nvflags = 0;
	__led_state = LED_IDLE;
	sei();

	ctx.p = bmp581_one_shot();
	ctx.p_idle = ctx.p - P_HYST_IDLE;
	ctx.p_on_off = ctx.p - P_HYST_ON_OFF;

	while (true) {
		ctx.p = bmp581_one_shot();
		ctx.vbat = vbat_voltage();
		ctx.ticks = get_ticks();

		trigger_state_machine(&ctx);

		if ((__flags & F_DEBUG) && !(ctx.ticks & 0x3))
			print_status(&ctx);

		if (ctx.state == POWERED_DOWN)
			power_down();
		else
retry:
			idle_mode();

		/* if ticks didn't advance, go back to sleep */
		if (get_ticks() == ctx.ticks)
			goto retry;
	};

	return 0;
}
