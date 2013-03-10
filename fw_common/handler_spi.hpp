#ifndef SHUPITO_FW_COMMON_HANDLER_SPI_HPP
#define SHUPITO_FW_COMMON_HANDLER_SPI_HPP

#include "handler_base.hpp"

template <typename Spi, typename ResetPin>
class handler_spi
	: public handler_base
{
public:
	typedef Spi spi_t;

	handler_spi(spi_t & spi)
		: spi(spi)
	{
	}

	void unselect()
	{
		spi.clear();
		ResetPin::make_input();
	}

	bool handle_command(avrlib::command_parser & cp, com_t & com)
	{
		switch (cp.command())
		{
		case 1: // PROGEN 2'bsel
			{
				uint8_t err = 0;
				if (cp.size() != 2)
					err = 1;
				
				if (!err)
					err = spi.start_master(cp[0] | (cp[1] << 8), false);

				if (!err)
					ResetPin::make_high();

				com.send_sync(1, &err, 1);
			}
			break;
		case 2: // Leave programming mode
			spi.clear();
			ResetPin::make_input();

			{
				uint8_t err = 0;
				com.send_sync(2, &err, 1);
			}
			break;

		case 3: // COMM 1'flags *'data
			{
				if (cp.size() == 0)
				{
					uint8_t err = 1;
					com.send_sync(3, &err, 1);
				}
				else
				{
					uint8_t * wbuf = com.alloc_sync(3, cp.size());
					*wbuf++ = 0;

					ResetPin::set_value((cp[0] & 1) != 0);
					for (uint8_t i = 1; i < cp.size(); ++i)
						*wbuf++ = spi.send(cp[i]);
					ResetPin::set_value((cp[0] & 2) != 0);

					com.commit();
				}
			}
			break;

		default:
			return false;
		}

		return true;
	}

private:
	spi_t & spi;
};

#endif // SHUPITO_FW_COMMON_HANDLER_SPI_HPP
