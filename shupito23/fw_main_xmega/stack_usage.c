#include "stack_usage.h"

extern uint8_t _end;
extern uint8_t __stack;

uint16_t get_stack_usage()
{
	uint8_t const * p = &_end;
	while (*p == 0xc5 && p <= &__stack)
		++p;

	return &__stack - p + 1;
}

uint16_t get_stack_size()
{
	return &__stack - &_end + 1;
}
