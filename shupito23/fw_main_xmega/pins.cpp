#include "pins.hpp"
#include "clock.hpp"
#include "../../fw_common/avrlib/stopwatch.hpp"

static enum { st_none, st_lo, st_vccio, st_hiv } rst_next_state;
static avrlib::timeout<clock_t> rst_timeout;

static void clear()
{
	PORTC_OUTCLR = (1<<4)|(1<<0);
	PORTB_OUTCLR = (1<<3);
}

void pin_rst::init()
{
	clear();
	PORTC_DIRSET = (1<<4)|(1<<0);
	PORTB_DIRSET = (1<<3);
	rst_next_state = st_none;
	rst_timeout.init_stopped(clock, clock_t::us<10>::value);
}

void pin_rst::process()
{
	if (rst_timeout)
	{
		switch (rst_next_state)
		{
		case st_lo:
			PORTC_OUTSET = (1<<0);
			break;
		case st_vccio:
			PORTC_OUTSET = (1<<4);
			break;
		case st_hiv:
			PORTB_OUTSET = (1<<3);
			break;
		default:
			break;
		}

		rst_next_state = st_none;
		rst_timeout.cancel();
	}
}

bool pin_rst::ready()
{
	return rst_next_state == st_none;
}

void pin_rst::make_input()
{
	clear();
}

void pin_rst::make_high()
{
	clear();
	rst_next_state = st_vccio;
	rst_timeout.restart();
}

void pin_rst::make_low()
{
	clear();
	rst_next_state = st_lo;
	rst_timeout.restart();
}

void pin_rst::set_high()
{
	clear();
	rst_next_state = st_vccio;
	rst_timeout.restart();
}

void pin_rst::set_low()
{
	clear();
	rst_next_state = st_lo;
	rst_timeout.restart();
}

void pin_rst::apply_hiv()
{
	clear();
	rst_next_state = st_hiv;
	rst_timeout.restart();
}
