#ifndef SHUPITO_FIRMWARE_HANDLER_BASE_HPP
#define SHUPITO_FIRMWARE_HANDLER_BASE_HPP

#include "avrlib/command_parser.hpp"

struct handler_base
{
	typedef uint8_t error_t;

	virtual error_t select() { return 0; }
	virtual void unselect() {}
	virtual void handle_command(avrlib::command_parser & cp) {}
};

#endif
