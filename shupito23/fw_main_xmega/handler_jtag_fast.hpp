#ifndef SHUPITO_FIRMWARE_HANDLER_JTAG_FAST_HPP
#define SHUPITO_FIRMWARE_HANDLER_JTAG_FAST_HPP

#include "../../fw_common/handler_base.hpp"
#include "pins.hpp"

struct handler_jtag_fast
	: handler_base
{
	typedef pin_aux_rst pin_tms;
	typedef pin_pdi pin_tck;
	typedef pin_rxd pin_tdo;
	typedef pin_txd pin_tdi;

	bool handle_command(uint8_t cmd, uint8_t const * cp, uint8_t size, com_t & com);
	handler_base::error_t select();
	void unselect();
	void process_selected();
};

#endif
