#ifndef SHUPITO_SHUPITO23_CLOCK_HPP
#define SHUPITO_SHUPITO23_CLOCK_HPP

#include <stdint.h>
#include <avr/io.h>

struct timer_xd0
{
	template <uint32_t v>
	struct us { static const uint32_t value = (v + 7) >> 3; };

	typedef uint16_t time_type;
	static const uint8_t value_bits = 16;

	static void init()
	{
		TCD0.CTRLA = TC_CLKSEL_DIV256_gc;
	}

	static time_type value() { return TCD0.CNT; }
};

typedef timer_xd0 clock_t;
extern clock_t clock;

#endif // SHUPITO_SHUPITO23_CLOCK_HPP
