#ifndef SHUPITO_FW_COMMON_HANDLER_UART_HPP
#define SHUPITO_FW_COMMON_HANDLER_UART_HPP

#include "handler_base.hpp"
#include "avrlib/serialize.hpp"

template <typename Usart>
class handler_uart
	: public handler_base
{
public:
	typedef Usart usart_t;

	handler_uart(usart_t & usart)
		: usart(usart), m_enabled(false), m_send_pos(0)
	{
	}

	void unselect()
	{
		usart.clear();
		m_enabled = false;
	}

	bool handle_command(uint8_t cmd, uint8_t const * cp, uint8_t size, com_t & com)
	{
		switch (cmd)
		{
		case 1: // PROGEN 32'baudrate
			{
				uint8_t err = 0;
				if (size != 4)
					err = 1;

				if (!err)
					usart.start(avrlib::deserialize<uint32_t>(cp));

				com.send_sync(1, &err, 1);
			}
			break;
		case 2: // Leave programming mode
			usart.clear();

			{
				uint8_t err = 0;
				com.send_sync(2, &err, 1);
			}
			break;

		case 3: // COMM *'data
			{
				AVRLIB_ASSERT(m_send_pos <= size);

				while (m_send_pos < size)
				{
					if (usart.tx_full())
						return false;
					usart.send(cp[m_send_pos++]);
				}

				m_send_pos = 0;

				{
					uint8_t err = 0;
					com.send_sync(3, &err, 1);
				}
			}
			break;

		default:
			return false;
		}

		return true;
	}

	void process_selected(com_t & com)
	{
		uint8_t rx_size = usart.rx_size();
		uint8_t com_avail = com.avail();
		if (rx_size > com_avail)
			rx_size = com_avail;

		if (rx_size)
		{
			led_blink_short();

			uint8_t * p = com.alloc(4, rx_size);
			AVRLIB_ASSERT(p);

			for (; rx_size; --rx_size)
				*p++ = usart.recv();
			com.commit();
		}
	}

private:
	usart_t & usart;
	bool m_enabled;
	uint8_t m_send_pos;
};

#endif // SHUPITO_FW_COMMON_HANDLER_UART_HPP
