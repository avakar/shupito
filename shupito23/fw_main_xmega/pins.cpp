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

bitbang_t::bitbang_t()
	: m_mask(0)
{
}

void bitbang_t::set_mask(uint8_t mask)
{
	this->set_impl(m_mask & ~mask, 0);
	m_mask = mask;
}

template <typename Pin>
static void bitbang(bool val, bool oe)
{
	if (!oe)
	{
		if (val)
			Pin::make_input();
	}
	else
	{
		if (val)
			Pin::make_high();
		else
			Pin::make_low();
	}
}

bool bitbang_t::set(uint8_t const * data, uint8_t size)
{
	if (size > 2 || (size % 2) != 0 || (data[1] & 0xf0) != 0)
		return false;

	this->set_impl(data[0] & m_mask, data[1] & m_mask);
	return true;
}

void bitbang_t::set_impl(uint8_t vals, uint8_t oes)
{
	bitbang<pin_aux_rst>(vals & 0x1, oes & 0x1);
	bitbang<pin_pdi>(vals & 0x2, oes & 0x2);
	bitbang<pin_xck>(vals & 0x4, oes & 0x4);
	bitbang<pin_txd>(vals & 0x8, oes & 0x8);
}
