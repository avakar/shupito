#include "spi.hpp"
#include "pins.hpp"
#include "app.hpp"
#include "led.hpp"

void spi_t::clear()
{
	USARTC1.CTRLB = 0;
	USARTC1.CTRLC = 0;

	pin_txd::make_input();
	pin_xck::make_input();
	g_app.allow_tunnel();
}

spi_t::error_t spi_t::start_master(uint16_t bsel, bool sample_on_trailing)
{
	g_app.disallow_tunnel();
	pin_txd::make_low();
	pin_xck::make_low();

	if (bsel)
		--bsel;
	USARTC1.BAUDCTRLA = bsel;
	USARTC1.BAUDCTRLB = bsel >> 8;
	USARTC1.CTRLC = USART_CMODE_MSPI_gc | (sample_on_trailing? (1<<1): 0);
	USARTC1.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	return 0;
}

uint8_t spi_t::send(uint8_t v)
{
	USARTC1.DATA = v;
	led_on();
	while ((USARTC1.STATUS & USART_RXCIF_bm) == 0)
	{
	}
	led_off();
	return USARTC1.DATA;
}

void spi_t::enable_tx()
{
	pin_txd::make_low();
}

void spi_t::disable_tx()
{
	pin_txd::make_input();
}

bool spi_t::read_raw()
{
	return pin_rxd::read();
}
