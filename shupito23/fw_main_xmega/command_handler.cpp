#include "app.hpp"
#include "utils.hpp"

bool app::handle_packet(uint8_t cmd, uint8_t const * cp, uint8_t size, yb_writer & w)
{
	switch (cmd)
	{
	case 0x0:
		if (size && cp[0] == 1)
		{
			uint8_t * wbuf = w.alloc(0, 1);
			if (!wbuf)
				return false;

			uint8_t err = 1;
			if (size == 3 && cp[1] == 0)
			{
				switch (cp[2])
				{
				case 0:
					err = this->select_handler(&m_handler_avricsp);
					break;
				case 1:
					err = this->select_handler(&m_handler_pdi);
					break;
				}
			}

			*wbuf = err;
			w.commit();
		}
		else if (size && cp[0] == 2)
		{
			uint8_t * wbuf = w.alloc(0, 1);
			if (!wbuf)
				return false;

			if (size == 3 && cp[1] == 0)
				this->select_handler(0);

			*wbuf++ = 0;
			w.commit();
		}

		break;
	case 0xa:
		if (size == 2 && cp[0] == 0 && cp[1] == 0)
		{
			static uint8_t const vccio_list[] = {
				0, 0,
				0, // flags
				5, 'V', 'C', 'C', 'I', 'O'
			};

			if (!w.send(0xa, vccio_list, sizeof vccio_list))
				return false;

			m_send_vcc_driver_list_scheduled = true;
			m_send_vccio_state_scheduled = true;
		}
		else if (size == 3 && cp[0] == 1 && cp[1] == 2)
		{
			this->set_vccio_drive(cp[2]);
		}

		break;

	case 0xb: // fw update
		if (size == 1 && cp[0] == 0)
			initiate_software_reset();
		break;

	case 0xd: // led
		if (size == 1)
		{
			switch (cp[0])
			{
			case 1:
				led_blink_short();
				break;
			case 2:
				led_blink_long();
				break;
			}
		}
		break;

	default:
		if (m_handler)
			return m_handler->handle_command(cmd, cp, size, w);
	}

	return true;
}

