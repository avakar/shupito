#ifndef SHUPITO_SHUPITO23_SPI_HPP
#define SHUPITO_SHUPITO23_SPI_HPP

#include <stdint.h>

class spi_t
{
public:
	typedef uint8_t error_t;

	void clear();
	error_t start_master(uint16_t speed_khz, uint8_t mode, bool lsb_first);
	uint8_t send(uint8_t v);
	void enable_tx();
	void disable_tx();
	bool read_raw();
};

#endif // SHUPITO_SHUPITO23_SPI_HPP
