#ifndef SHUPITO_SHUPITO23_USART_HPP
#define SHUPITO_SHUPITO23_USART_HPP

#include <stdint.h>
#include "../../fw_common/avrlib/buffer.hpp"

class usart_t
{
public:
	typedef uint8_t error_t;

	void start(uint32_t baudrate);
	void clear();

	void send(uint8_t v);
	uint8_t recv();

	uint8_t rx_size() const;
	bool tx_full() const;

private:
	avrlib::buffer<uint8_t, 64> m_buf;
};

#endif // SHUPITO_SHUPITO23_USART_HPP
