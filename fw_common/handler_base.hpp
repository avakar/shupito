#ifndef SHUPITO_FIRMWARE_HANDLER_BASE_HPP
#define SHUPITO_FIRMWARE_HANDLER_BASE_HPP

#include "avrlib/command_parser.hpp"

class yb_writer
{
public:
	explicit yb_writer(uint8_t max_packet_size)
		: m_max_packet_size(max_packet_size)
	{
	}

	virtual uint8_t * alloc(uint8_t cmd, uint8_t size) { return 0; }
	virtual uint8_t * alloc_sync(uint8_t cmd, uint8_t size) { return 0; }
	virtual void commit() {}
	virtual bool send(uint8_t cmd, uint8_t const * data, uint8_t size) { return false; }
	virtual void send_sync(uint8_t cmd, uint8_t const * data, uint8_t size) {}

	uint8_t max_packet_size() const
	{
		return m_max_packet_size;
	}

private:
	uint8_t m_max_packet_size;
};

struct handler_base
{
	typedef uint8_t error_t;
	typedef yb_writer com_t;

	virtual error_t select() { return 0; }
	virtual void unselect() {}
	virtual bool handle_command(uint8_t cmd, uint8_t const * cp, uint8_t size, com_t & com) { AVRLIB_ASSERT(0); return false; }
	virtual void process_selected() {}
};

#endif
