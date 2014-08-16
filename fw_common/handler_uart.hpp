#ifndef SHUPITO_FW_COMMON_HANDLER_UART_HPP
#define SHUPITO_FW_COMMON_HANDLER_UART_HPP

#include "handler_base.hpp"
#include "avrlib/serialize.hpp"

template <typename Usart, typename Bitbang>
class handler_uart
	: public handler_base
{
public:
	typedef Usart usart_t;
	typedef Bitbang bitbang_t;

	handler_uart(usart_t & usart, bitbang_t & bitbang)
		: usart(usart), bitbang(bitbang), m_enabled(false), m_send_pos(0)
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

				m_send_pos += usart.send(cp + m_send_pos, size - m_send_pos);
				if (m_send_pos != size)
					return false;

				m_send_pos = 0;

				uint8_t err = 0;
				com.send_sync(3, &err, 1);
			}
			break;

		case 5: // BITBANG 8'out 8'data
			{
				uint8_t err = (bitbang.set(cp, size)? 0: 1);
				com.send_sync(1, &err, 1);
			}
			break;

		default:
			return false;
		}

		return true;
	}

	void process_selected(com_t & com)
	{
		uint8_t const * recv_data;
		uint8_t recv_size = usart.recv(recv_data);
		if (!recv_size || com.avail() < recv_size)
			return;

		led_blink_short();
		com.send(4, recv_data, recv_size);
		usart.recv_commit();
	}

private:
	usart_t & usart;
	bitbang_t & bitbang;
	bool m_enabled;
	uint8_t m_send_pos;
};

#endif // SHUPITO_FW_COMMON_HANDLER_UART_HPP
