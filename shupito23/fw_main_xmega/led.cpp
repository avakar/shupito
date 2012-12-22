#include "led.hpp"
#include "pins.hpp"
#include "clock.hpp"

static uint8_t refcount;
static avrlib::stopwatch<clock_t> sw;
static clock_t::time_type blink_length;

void led_init()
{
	pin_led::make_low();
	refcount = 0;
	sw.init_stopped(clock);
}

void led_process()
{
	if (sw.get() > blink_length)
	{
		led_off();
		sw.cancel();
	}
}

void led_on()
{
	++refcount;
	pin_led::set_high();
}

void led_off()
{
	if (--refcount == 0)
		pin_led::set_low();
}

static void led_blink(clock_t::time_type blink_len)
{
	led_on();
	if (sw.running())
		--refcount;
	sw.restart();
	blink_length = blink_len;
}

void led_blink_short()
{
	led_blink(clock_t::us<10000>::value);
}

void led_blink_long()
{
	led_blink(clock_t::us<25000>::value);
}
