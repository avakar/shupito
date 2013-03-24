#include "hiv.hpp"
#include <avr/io.h>
#include "clock.hpp"
#include "../../fw_common/avrlib/stopwatch.hpp"
#include "../../fw_common/avrlib/assert.hpp"

static bool g_allowed;
static bool g_enabled;
static uint8_t g_cur_voltage;
static uint16_t g_per;
static uint32_t g_setpoint_recip;
static avrlib::timeout<clock_t> g_timeout;

static uint16_t const max_per = 0xfff;

void hiv_init()
{
	PORTD_OUTCLR = (1<<3);
	PORTD_DIRSET = (1<<3);

	g_per = max_per;
	TCD0_PER = max_per;
	TCD0_CCD = 8;
	TCD0_CTRLB = TC_WGMODE_SINGLESLOPE_gc;
	TCD0_CTRLA = TC_CLKSEL_DIV1_gc;

	PORTA_PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
	ADCA_CH1_CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA_CH1_MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;

	g_timeout.init(clock, clock_t::us<1000>::value);
	g_setpoint_recip = 0x1b7c6;
	g_enabled = false;
	g_allowed = true;
}

extern "C" uint16_t hiv_compute_per(uint16_t old_per, uint8_t cur_voltage, uint32_t setpoint_recip);

void hiv_process()
{
	if (g_timeout)
	{
		ADCA_CH1_CTRL |= ADC_CH_START_bm;
		g_timeout.cancel();
	}

	if (ADCA_CH1_INTFLAGS & ADC_CH_CHIF_bm)
	{
		uint8_t cur_voltage = ADCA_CH1RES >> 3;
		g_cur_voltage = cur_voltage;

		if (g_enabled)
		{
			uint16_t tmp = hiv_compute_per(g_per, cur_voltage, g_setpoint_recip);
			if (tmp > max_per)
			{
				g_per = max_per;
			}
			else if (tmp < 40)
			{
				g_per = 40;
			}
			else
			{
				g_per = tmp;
			}

			TCD0_PERBUF = g_per;
		}

		ADCA_CH1_INTFLAGS = ADC_CH_CHIF_bm;
		g_timeout.restart();
	}
}

void hiv_enable()
{
	AVRLIB_ASSERT(g_allowed);
	TCD0_CTRLB |= TC0_CCDEN_bm;
	g_enabled = true;
}

void hiv_disable()
{
	AVRLIB_ASSERT(g_allowed);
	TCD0_CTRLB &= ~TC0_CCDEN_bm;
	TCD0_PERBUF = max_per;
	g_enabled = false;
}

void hiv_allow()
{
	AVRLIB_ASSERT(!g_allowed);
	AVRLIB_ASSERT(!g_enabled);
	g_allowed = true;
}

void hiv_disallow()
{
	AVRLIB_ASSERT(!g_enabled);
	AVRLIB_ASSERT(g_allowed);
	g_allowed = false;
}

void hiv_setpoint_recip(uint32_t v)
{
	g_setpoint_recip = v;
}

uint8_t hiv_get_voltage()
{
	return g_cur_voltage;
}

uint16_t hiv_get_period()
{
	return g_per;
}
