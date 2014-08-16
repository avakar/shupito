#ifndef SHUPITO_SHUPITO23_USART_HPP
#define SHUPITO_SHUPITO23_USART_HPP

#include <stdint.h>

class usart_t
{
public:
	typedef uint8_t error_t;

	void start(uint32_t baudrate);
	void clear();

	uint8_t send(uint8_t const * v, uint8_t size);
	uint8_t recv(uint8_t const *& data);
	void recv_commit();
};

#endif // SHUPITO_SHUPITO23_USART_HPP
