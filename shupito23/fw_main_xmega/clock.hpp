#ifndef SHUPITO_SHUPITO23_CLOCK_HPP
#define SHUPITO_SHUPITO23_CLOCK_HPP

#include <stdint.h>
#include <avr/io.h>
#include "../../fw_common/avrlib/stopwatch.hpp"

struct clock_t
{
	template <uint32_t v>
	struct us { static const uint32_t value = (v + 7) >> 3; };

	typedef uint16_t time_type;
	static const uint8_t value_bits = 16;

	static void init()
	{
		TCE0_CTRLA = TC_CLKSEL_DIV256_gc;
	}

	static time_type value() { return TCE0_CNT; }
};

extern clock_t clock;

#endif // SHUPITO_SHUPITO23_CLOCK_HPP
