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
#include "uart.h"
#include "sensor.h"

/* one tick is 250ms */
#define HZ 4

/* hyst of 10mbar for the alarm limit and 100mbar for turn on limit */
#define P_HYST_ALARM	10
#define P_HYST_IDLE	30
#define P_HYST_ON_OFF	100

#define IDLE_TIMEOUT	(60 * HZ)

#define BUZZER_HZ 2000

#define MILLIVOLTS(v)		((uint32_t)1100 * 256 / (v))
#define TO_CENTIVOLTS(v)	(110 * 256 / (v))
#define VBAT_LOW	MILLIVOLTS(2700)

#define LED_BLINKING	(1 << 0)
#define LED_ALTERNATING	(1 << 1)
#define LED_GREEN	(1 << 2)
#define LED_RED		(1 << 3)
#define BUZZER_ON	(1 << 4)

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
	IDLE = LED_GREEN | LED_BLINKING,
	IDLE_BAT_LOW_DETECTED = LED_RED | LED_ALTERNATING,

	/*
	 * Device is powered-down, all clocks are turned off. Wake-up source is
	 * the PIT interrupt. The PIT interval is set to 2s to consume power. If
	 * the pressure is below the on-off threshold, the state will  transition
	 * to to IDLE.
	 */
	POWERED_DOWN = 0,

	/*
	 * Device constantly measures the pressure. Pressure is only allowed to
	 * go down (with a hysteresis). If the pressure goes up, a leak is
	 * detected and the state transitions to ALARM.
	 */
	PRESSURE_OK = LED_GREEN,

	/*
	 * Device detected a leak. If pressure reaches the "normal" value
	 * window again, the state transisitons to IDLE. After 10s, the state
	 * automatically transition to SILENT_ALARM.
	 */
	ALARM = BUZZER_ON | LED_RED | LED_BLINKING,

	/* Same as ALARM. */
	SILENT_ALARM = LED_RED | LED_BLINKING,

	ERROR = LED_RED,
};

struct context {
	uint16_t ticks;
	uint16_t p;
	uint16_t p_idle;
	uint16_t p_on_off;
	uint16_t p_alarm;
	uint8_t vbat;
	uint8_t buzzer_off;
};

#define F_DEBUG (1 << 0)
#define F_DEMO (1 << 1)
#define F_BAT_LOW_DETECTED (1 << 2)
#define F_POWER_OFF (1 << 3)

/*
 * ticks will count the time since the last power-down (or start-up).
 * The PIT timer period is 250ms, which gives us approximately
 * 4.5 hours (2^16 * 250ms) runtime until the counter overflows.
 **/
static volatile uint16_t __ticks;

/*
 * rstcnt will increase on every external reset, that is on every reset button
 * press. After ~400ms the counter is reset to zero. rstcnt is also cleared on
 * a power-on reset. It is used to determine run mode of the leak detector.
 */
static volatile uint8_t rstcnt __attribute__ ((section (".noinit")));

static uint8_t state;
static uint8_t flags;

ISR(RTC_PIT_vect)
{
	RTC_PITINTFLAGS = RTC_PI_bm;
	__ticks++;
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

static void led_tick(void)
{
	uint8_t set = 0;

	if (flags & F_DEBUG)
		return;

	if (state & LED_RED)
		set = LED_RED;
	if (state & LED_GREEN)
		set = LED_GREEN;
	if (__ticks & 3) {
		if (state & LED_BLINKING)
			set = 0;
		if (state & LED_ALTERNATING)
			set = LED_GREEN;
	}

	if (set & LED_RED)
		CCL_LUT0CTRLA |= CCL_ENABLE_bm;
	else
		CCL_LUT0CTRLA &= ~CCL_ENABLE_bm;

	if (set & LED_GREEN)
		CCL_LUT1CTRLA |= CCL_ENABLE_bm;
	else
		CCL_LUT1CTRLA &= ~CCL_ENABLE_bm;
}

static void buzzer_tick(void)
{
	if (state & BUZZER_ON && __ticks & 0x2)
		PORTA_DIRSET = PIN3_bm;
	else
		PORTA_DIRCLR = PIN3_bm;
}

static void pwm_init(void)
{
	/* 50% duty cycle */
	TCA0_SINGLE_PER = F_CPU / 8 / BUZZER_HZ;
	TCA0_SINGLE_CMP0 = F_CPU / 8 / BUZZER_HZ / 2;

	/* PWM mode, /8 prescaler */
	TCA0_SINGLE_CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;
	TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm;
}

static void pit_power_down_mode()
{
	/* enable interrupt and set period to 2s */
	RTC_PITCTRLA = RTC_PERIOD_CYC2048_gc | RTC_PITEN_bm;
	loop_until_bit_is_clear(RTC_PITSTATUS, RTC_CTRLBUSY_bp);
}

static void pit_normal_mode()
{
	/* enable interrupt and set period to 250ms */
	RTC_PITCTRLA = RTC_PERIOD_CYC256_gc | RTC_PITEN_bm;
	loop_until_bit_is_clear(RTC_PITSTATUS, RTC_CTRLBUSY_bp);
}

static void pit_init(void)
{
	/* use 1.024 kHz RTC clock */
	RTC_CLKSEL = RTC_CLKSEL_INT1K_gc;

	/* enable PIT interrupt */
	RTC_PITINTCTRL = RTC_PI_bm;

	pit_normal_mode();
}

static void power_off(void)
{
	PORTA_DIRSET = PIN3_bm;
	_delay_ms(200);
	PORTA_DIRCLR = PIN3_bm;
	_delay_ms(100);
	PORTA_DIRSET = PIN3_bm;
	_delay_ms(200);
	PORTA_DIRCLR = PIN3_bm;

	PORTA_DIRCLR = 0xff;
	RTC_PITCTRLA = 0;
	cli();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
}

static void power_down(void)
{
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
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
	uart_puts(itoa(state, buf, 10));
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
	/* internal vref = 1.1V */
	VREF_CTRLA = VREF_ADC0REFSEL_1V1_gc;

	/* Vref = Vdd, 156.25 kHz clock */
	ADC0_CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV32_gc;

	/* input = internal reference */
	ADC0_MUXPOS = ADC_MUXPOS_INTREF_gc;

	ADC0_CTRLA = ADC_RESSEL_8BIT_gc;

	_delay_ms(1);
}

static uint8_t vbat_voltage(void)
{
	ADC0_CTRLA |= ADC_ENABLE_bm;
	_delay_ms(1);
	ADC0_COMMAND = ADC_STCONV_bm;
	loop_until_bit_is_clear(ADC0_COMMAND, ADC_STCONV_bp);
	ADC0_CTRLA &= ~ADC_ENABLE_bm;

	return ADC0_RESL;
}

static void gpio_init(void)
{
	PORTA_PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA_PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA_PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA_PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
	PORTA_PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
	PORTA_PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
}

static void trigger_state_machine(struct context *ctx)
{
	switch (state) {
	case POWERED_DOWN:
		zero_ticks();
		if (ctx->p < ctx->p_idle) {
			pit_normal_mode();
			state = IDLE;
		}
		break;
	case IDLE:
		if (flags & F_BAT_LOW_DETECTED)
			state = IDLE_BAT_LOW_DETECTED;
		/* fallthrough */
	case IDLE_BAT_LOW_DETECTED:
		if (ctx->p < ctx->p_on_off) {
			ctx->p_alarm = ctx->p + P_HYST_ALARM;
			state = PRESSURE_OK;
		} else if (ctx->ticks >= IDLE_TIMEOUT) {
			pit_power_down_mode();
			state = POWERED_DOWN;
		}
		break;
	case PRESSURE_OK:
		if (ctx->p > ctx->p_alarm) {
			ctx->buzzer_off = 10 * HZ;
			state = ALARM;
		} else if (ctx->p + P_HYST_ALARM < ctx->p_alarm) {
			ctx->p_alarm = ctx->p + P_HYST_ALARM;
		}
		break;
	case ALARM:
		if (!(--ctx->buzzer_off))
			state = SILENT_ALARM;
		/* fallthrough */
	case SILENT_ALARM:
		if (ctx->p > ctx->p_on_off) {
			ctx->p_alarm = 0;
			state = IDLE;
		}
		break;
	}
}

void led_init(void)
{
	if (flags & F_DEBUG)
		return;

	/*
	 * Route pins through CCL, because the LEDs aren't connected to the WOn
	 * of the counter.
	 */
	CCL_LUT0CTRLB = CCL_INSEL0_TCA0_gc;
	CCL_TRUTH0 = 0x01;
	CCL_LUT0CTRLA = CCL_OUTEN_bm;

	CCL_LUT1CTRLB = CCL_INSEL0_TCA0_gc;
	CCL_TRUTH1 = 0x01;
	CCL_LUT1CTRLA = CCL_OUTEN_bm;
	CCL_CTRLA = CCL_ENABLE_bm;
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

	/* leds */
	CCL_LUT1CTRLA |= CCL_ENABLE_bm;
	_delay_ms(500);
	CCL_LUT1CTRLA &= ~CCL_ENABLE_bm;
	CCL_LUT0CTRLA |= CCL_ENABLE_bm;
	_delay_ms(500);
	battery_check(ctx);
	CCL_LUT0CTRLA &= ~CCL_ENABLE_bm;

	/* buzzer */
	PORTA_DIRSET = PIN3_bm;
	_delay_ms(200);
	PORTA_DIRCLR = PIN3_bm;
}

static void error(void)
{
	cli();
	state = ERROR;
	led_tick();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
}

int main(void)
{
	struct sensor_driver *drv = NULL;
	struct context ctx = { 0 };

	/* 5 MHz system clock */
	_PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm);

	if (RSTCTRL_RSTFR & RSTCTRL_PORF_bm)
		rstcnt = 0;
	RSTCTRL_RSTFR = 0xff;

	pit_init();

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
	case 8:
		flags = F_POWER_OFF;
		break;
	}

	/* give the user some time to reset again */
	_delay_ms(400);
	rstcnt = 0;

	gpio_init();
	pwm_init();
	led_init();
	adc_init();
	if (CONFIG_ENABLE_UART && flags & F_DEBUG)
		uart_init();

	twi_init();

	if (CONFIG_ENABLE_DEMO && flags & F_DEMO)
		drv = &demo_driver;
	else if (CONFIG_ENABLE_BMP581 && sensor_is_present(&bmp581_driver))
		drv = &bmp581_driver;
	else if (CONFIG_ENABLE_LPS22HB && sensor_is_present(&lps22hb_driver))
		drv = &lps22hb_driver;

	if (!drv)
		error();

	sensor_init(drv);

	if (flags & F_POWER_OFF)
		power_off();

	selftest(&ctx);

	/*
	 * Clear PIT flag, so the ISR won't be called right away and the ticks will
	 * start at zero.
	 */
	RTC_PITINTFLAGS = RTC_PI_bm;

	state = IDLE;
	sei();

	ctx.p = sensor_one_shot(drv);
	ctx.p_idle = ctx.p - P_HYST_IDLE;
	ctx.p_on_off = ctx.p - P_HYST_ON_OFF;

	while (true) {
		ctx.ticks = get_ticks();
		ctx.p = sensor_read_pressure(drv);

		/* every 8 seconds */
		if ((ctx.ticks & 0x1f) == 0)
			battery_check(&ctx);

		trigger_state_machine(&ctx);

		buzzer_tick();
		led_tick();

		/* every second */
		if ((ctx.ticks & 0x3) == 0)
		    print_status(&ctx);

		/* Trigger the pressure measurment while we sleep */
		sensor_start_measurement(drv);

		/*
		 * This will either return on a PIT interrupt, which means
		 * the device slept for 250ms or on any other interrupt, if the
		 * device is in idle mode. In the latter case, go back to idle
		 * mode until ticks has increased, to make sure, the main loop
		 * is only run once every tick.
		 */
		if (state == POWERED_DOWN)
			power_down();
		else
			while (get_ticks() == ctx.ticks)
				idle_mode();
	};

	return 0;
}
