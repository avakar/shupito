#include "btn.hpp"
#include "clock.hpp"
#include "pins.hpp"
#include "../../fw_common/avrlib/stopwatch.hpp"

static bool filtered_state;
static avrlib::timeout<clock_t> timeout;

void btn_init()
{
	pin_button::pullup();
	filtered_state = true;
	timeout.init_stopped(clock, clock_t::us<50000>::value);
}

void btn_process()
{
	if (filtered_state == pin_button::read())
	{
		timeout.cancel();
	}
	else
	{
		timeout.start();
		if (timeout)
			filtered_state = !filtered_state;
	}
}

bool btn_pressed()
{
	return !filtered_state;
}
