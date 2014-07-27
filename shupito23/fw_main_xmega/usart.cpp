#include "usart.hpp"
#include "app.hpp"

void usart_t::start(uint32_t baudrate)
{
	g_app.disallow_tunnel();
}

void usart_t::clear()
{
	g_app.allow_tunnel();
}

void usart_t::send(uint8_t v)
{
	m_buf.push(v);
}

uint8_t usart_t::recv()
{
	uint8_t res = m_buf.top();
	m_buf.pop();
	return res;
}

uint8_t usart_t::rx_size() const
{
	return m_buf.size();
}

bool usart_t::tx_full() const
{
	return m_buf.full();
}
