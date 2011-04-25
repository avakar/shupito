#ifndef SHUPITO_FIRMWARE_HANDLER_BASE_HPP
#define SHUPITO_FIRMWARE_HANDLER_BASE_HPP

#include "avrlib/command_parser.hpp"

struct handler_base
{
	virtual bool select() { return true; }
	virtual void unselect() {}
	virtual void handle_command(avrlib::command_parser & cp) {}
};

#endif
