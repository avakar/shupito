#ifndef SHUPITO_FIRMWARE_HANDLER_BASE_HPP
#define SHUPITO_FIRMWARE_HANDLER_BASE_HPP

#include "avrlib/command_parser.hpp"

template <typename Com>
struct handler_base
{
	typedef uint8_t error_t;
	typedef Com com_t;

	virtual error_t select() { return 0; }
	virtual void unselect() {}
	virtual bool handle_command(avrlib::command_parser & cp, com_t & com) { return false; }
};

#endif
