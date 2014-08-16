#include "usart.hpp"
#include "app.hpp"
#include "tunnel.hpp"

void usart_t::start(uint32_t baudrate)
{
	g_app.disallow_tunnel();

	uint16_t baudctrl;
	bool dblspeed;
	g_app.get_baudctrl(baudrate, baudctrl, dblspeed);
	app_tunnel_start(baudctrl, USART_CHSIZE_8BIT_gc, dblspeed);
}

void usart_t::clear()
{
	app_tunnel_stop();

	uint8_t const * data;
	while (app_tunnel_recv(data))
		app_tunnel_recv_commit();

	g_app.allow_tunnel();
}

uint8_t usart_t::send(uint8_t const * v, uint8_t size)
{
	return app_tunnel_send(v, size);
}

uint8_t usart_t::recv(uint8_t const *& data)
{
	return app_tunnel_recv(data);
}

void usart_t::recv_commit()
{
	app_tunnel_recv_commit();
}
