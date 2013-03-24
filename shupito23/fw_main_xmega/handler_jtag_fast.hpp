#ifndef SHUPITO_FIRMWARE_HANDLER_JTAG_FAST_HPP
#define SHUPITO_FIRMWARE_HANDLER_JTAG_FAST_HPP

#include "../../fw_common/handler_base.hpp"

struct handler_jtag_fast
	: handler_base
{
	bool handle_command(uint8_t cmd, uint8_t const * cp, uint8_t size, com_t & com);
	handler_base::error_t select();
	void unselect();

private:
	bool m_timer_running;
};

#endif
