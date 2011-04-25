#ifndef SHUPITO_SPI_HPP
#define SHUPITO_SPI_HPP

#include <avr/io.h>

class spi_t
{
public:
	typedef uint8_t error_t;

	void clear()
	{
		DDRB &= ~((1<<4)|(1<<5)|(1<<7));
		SPCR = 0;
	}

	error_t start_master(uint16_t speed_khz)
	{
		/*static const struct {
			uint16_t speed;
			uint8_t prescaler;
			uint8_t prescaler_sr;
		} prescalers[] = 
		{
			{ 24000, 0, (1<<SPI2X) },
			{ 12000, 0, 0 },
			{  6000, (1<<SPR0), (1<<SPI2X) },
			{  3000, (1<<SPR0), 0 },
			{  1500, (1<<SPR1), (1<<SPI2X) },
			{   750, (1<<SPR1), 0 },
			{   375, (1<<SPR1)|(1<<SPR0), (1<<SPI2X) },
			{   188, (1<<SPR1)|(1<<SPR0), 0 },
		};
		
		bool ok = false;
		for (uint8_t i = 0; !ok && i < sizeof prescalers / sizeof prescalers[0]; ++i)
		{
			if (prescalers[i].speed <= remote_clock_khz)
			{
				SPSR = prescalers[i].prescaler_sr;
				SPCR = (1<<SPE)|(1<<MSTR)|prescalers[i].prescaler;
				ok = true;
			}
		}*/

		PORTB &= ~(1<<4);
		DDRB |= (1<<4);
		SPSR = (1<<SPIF)|(1<<SPI2X);
		SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);
		DDRB |= (1<<5)|(1<<7);
		return 0;
	}

	uint8_t send(uint8_t v)
	{
		SPDR = v;
		while ((SPSR & (1<<SPIF)) == 0)
		{
		}
		return SPDR;
	}
};

#endif
