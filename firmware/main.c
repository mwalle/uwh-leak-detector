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

#include "config.h"
#include "twi.h"
#include "swuart.h"
#include "buzzer.h"
#include "sensor.h"

/* one tick is 250ms */
#define HZ 4

/* hyst of 10mbar for the alarm limit and 100mbar for turn on limit */
#define P_HYST_ALARM	10
#define P_HYST_IDLE	30
#define P_HYST_ON_OFF	100

#define IDLE_TIMEOUT	(60 * HZ)

#define MILLIVOLTS(v)		((uint32_t)1100 * 256 / (v))
#define TO_CENTIVOLTS(v)	(110 * 256 / (v))
#define VBAT_LOW	MILLIVOLTS(2700)

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
	 * detected and the state transitions to ALARM.
	 */
	PRESSURE_OK,

	/*
	 * Device detected a leak. If pressure reaches the "normal" value
	 * window again, the state transisitons to IDLE. After 10s, the state
	 * automatically transition to SILENT_ALARM.
	 */
	ALARM,

	/* Same as ALARM. */
	SILENT_ALARM,
};

struct context {
	uint16_t ticks;
	uint16_t p;
	uint16_t p_idle;
	uint16_t p_on_off;
	uint16_t p_alarm;
	uint8_t vbat;
	uint8_t state;
	uint8_t buzzer_off;
};

#define F_DEBUG (1 << 0)
#define F_DEMO (1 << 1)
#define F_BAT_LOW_DETECTED (1 << 2)

#define LED_BLINKING	(1 << 0)
#define LED_ALTERNATING	(1 << 1)
#define LED_GREEN	(1 << 2)
#define LED_RED		(1 << 3)

enum led_state {
	LED_OFF		= 0,
	LED_PRESSURE_OK = LED_GREEN,
	LED_IDLE	= LED_GREEN | LED_BLINKING,
	LED_ERROR	= LED_RED,
	LED_LEAK	= LED_RED | LED_BLINKING,
	/* alternating between green and red */
	LED_BAT_LOW	= LED_RED | LED_ALTERNATING,
};

/*
 * ticks will count the time since the last power-down (or start-up).
 * The watchdog timer period is 250ms, which gives us approximately
 * 4.5 hours (2^16 * 250ms) runtime until the counter overflows.
 **/
volatile uint16_t __ticks;

/*
 * rstcnt will increase on every external reset, that is on every reset button
 * press. After ~400ms the counter is reset to zero. rstcnt is also cleared on
 * a power-on reset. It is used to determine run mode of the leak detector.
 */
static volatile uint8_t rstcnt __attribute__ ((section (".noinit")));

static uint8_t led_state;
static uint8_t flags;

ISR(WDT_vect)
{
	__ticks++;
}

uint16_t get_ticks(void)
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

static void led_tick(void)
{
	uint8_t set = 0;

	if (flags & F_DEBUG)
		return;

	if (led_state)
		/* /8 prescaler */
		TCCR1 = _BV(CS12);
	else
		TCCR1 = 0;

	if (led_state & LED_RED)
		set = _BV(PB3);
	if (led_state & LED_GREEN)
		set = _BV(PB4);
	if (__ticks & 3) {
		if (led_state & LED_BLINKING)
			set = 0;
		if (led_state & LED_ALTERNATING)
			set = _BV(PB4);
	}

	DDRB &= ~(_BV(PB3) | _BV(PB4));
	DDRB |= set;
}

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

	wdt_power_down_mode();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
	wdt_normal_mode();
}

static void idle_mode(void)
{
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_mode();
}

static void print_status(struct context *ctx)
{
	char buf[8];

	if (!CONFIG_ENABLE_UART)
		return;

	if (!(flags & F_DEBUG))
		return;

	uart_puts_P(PSTR("t"));
	uart_puts(itoa(ctx->ticks, buf, 10));
	uart_puts_P(PSTR(" s"));
	uart_puts(itoa(ctx->state, buf, 10));
	uart_puts_P(PSTR(" f"));
	uart_puts(itoa(flags, buf, 10));
	uart_puts_P(PSTR(" p"));
	uart_puts(itoa(ctx->p, buf, 10));
	uart_puts_P(PSTR(" a"));
	uart_puts(itoa(ctx->p_alarm, buf, 10));
	uart_puts_P(PSTR(" o"));
	uart_puts(itoa(ctx->p_on_off, buf, 10));
	uart_puts_P(PSTR(" v"));
	/* The following assumes a valid range of the ctx->vbat */
	itoa(TO_CENTIVOLTS(ctx->vbat), buf, 10);
	uart_putc(buf[0]);
	uart_puts_P(PSTR("."));
	uart_puts(buf + 1);
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
			led_state = LED_IDLE;
			state = IDLE;
		}
		break;
	case IDLE:
		if (ctx->p < ctx->p_on_off) {
			ctx->p_alarm = ctx->p + P_HYST_ALARM;
			led_state = LED_PRESSURE_OK;
			state = PRESSURE_OK;
		} else if (ctx->ticks >= IDLE_TIMEOUT) {
			led_state = LED_OFF;
			state = POWERED_DOWN;
		} else if (flags & F_BAT_LOW_DETECTED) {
			led_state = LED_BAT_LOW;
		}
		break;
	case PRESSURE_OK:
		if (ctx->p > ctx->p_alarm) {
			led_state = LED_LEAK;
			buzzer_on();
			ctx->buzzer_off = 10 * HZ;
			state = ALARM;
		} else if (ctx->p + P_HYST_ALARM < ctx->p_alarm) {
			ctx->p_alarm = ctx->p + P_HYST_ALARM;
		}
		break;
	case ALARM:
		if (!(--ctx->buzzer_off)) {
			buzzer_off();
			state = SILENT_ALARM;
		}
		/* fallthrough */
	case SILENT_ALARM:
		if (ctx->p > ctx->p_on_off) {
			led_state = LED_IDLE;
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
	if (flags & F_DEBUG)
		return;

	/* 50% duty cycle */
	OCR1B = F_CPU / 8 / 2000 / 2;
	OCR1C = F_CPU / 8 / 2000;

	/* PWM mode, enable OC1B and OC1B# outputs, /8 prescaler */
	GTCCR = _BV(PWM1B) | _BV(COM1B0);
}

static void battery_check(struct context *ctx)
{
	ctx->vbat = vbat_voltage();
	/* vbat will increase with lower voltage */
	if (ctx->vbat > VBAT_LOW)
		flags |= F_BAT_LOW_DETECTED;
}

static void selftest(struct context *ctx)
{
	if (flags & F_DEBUG)
		return;

	TCCR1 = _BV(CS12);

	DDRB |= _BV(PB4);
	_delay_ms(500);
	DDRB &= ~_BV(PB4);
	DDRB |= _BV(PB3);
	_delay_ms(500);
	battery_check(ctx);
	DDRB &= ~_BV(PB3);
	buzzer_on();
	_delay_ms(200);
	buzzer_off();
}

static void error(void)
{
	cli();
	led_state = LED_ERROR;
	led_tick();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
}

int main(void)
{
	struct sensor_driver *drv = NULL;
	struct context ctx = { 0 };

	if (MCUSR & _BV(PORF))
		rstcnt = 0;
	MCUSR = 0;
	wdt_init();

	switch (++rstcnt) {
	case 3:
		flags = F_DEBUG;
		break;
	case 4:
		flags = F_DEMO;
		break;
	case 5:
		flags = F_DEMO | F_DEBUG;
		break;
	}

	/* give the user some time to reset again */
	_delay_ms(400);
	rstcnt = 0;

	led_init();
	adc_init();
	buzzer_init();
	if (CONFIG_ENABLE_UART && flags & F_DEBUG)
		uart_init();

	if (flags & F_DEMO)
		drv = &demo_driver;
	else if (CONFIG_ENABLE_BMP581 && sensor_is_present(&bmp581_driver))
		drv = &bmp581_driver;
	else if (CONFIG_ENABLE_LPS22HB && sensor_is_present(&lps22hb_driver))
		drv = &lps22hb_driver;

	if (!drv)
		error();

	sensor_init(drv);

	/* disable unused blocks */
	ACSR |= _BV(ACD);

	selftest(&ctx);

	/*
	 * Clear watchdog flag, so the watchdog ISR won't be called right away
	 * and the ticks will start at zero.
	 */
	WDTCR |= _BV(WDIF);

	led_state = LED_IDLE;
	sei();

	ctx.p = sensor_one_shot(drv);
	ctx.p_idle = ctx.p - P_HYST_IDLE;
	ctx.p_on_off = ctx.p - P_HYST_ON_OFF;

	while (true) {
		buzzer_tick();
		led_tick();

		ctx.ticks = get_ticks();
		ctx.p = sensor_read_pressure(drv);

		/* every 8 seconds */
		if ((ctx.ticks & 0x1f) == 0)
			battery_check(&ctx);

		trigger_state_machine(&ctx);

		/* every second */
		if ((ctx.ticks & 0x3) == 0)
			print_status(&ctx);

		/* Trigger the pressure measurment while we sleep */
		sensor_start_measurement(drv);

		/*
		 * This will either return on a watchdog interrupt, which means
		 * the device slept for 250ms or on any other interrupt, if the
		 * device is in idle mode. In the latter case, go back to idle
		 * mode until ticks has increased, to make sure, the main loop
		 * is only run once every tick.
		 */
		if (ctx.state == POWERED_DOWN)
			power_down();
		else
			while (get_ticks() == ctx.ticks)
				idle_mode();
	};

	return 0;
}
