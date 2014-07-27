#ifndef SHUPITO_FIRMWARE_HANDLER_JTAG_HPP
#define SHUPITO_FIRMWARE_HANDLER_JTAG_HPP

#include "handler_base.hpp"

template <typename PinRst, typename PinClk, typename PinRx, typename PinTx, typename Process>
struct handler_jtagg
	: handler_base
{
	typedef PinRst pin_tms;
	typedef PinClk pin_tck;
	typedef PinRx  pin_tdo;
	typedef PinTx  pin_tdi;
	typedef typename Process::led led;

	handler_jtagg(Process process = Process())
		: m_process(process)
	{
	}

	bool handle_command(uint8_t cmd, uint8_t const * cp, uint8_t size, com_t & com)
	{
		switch (cmd)
		{
		case 1: // STATE 8'length length'state_path
			if (size >= 1)
			{
				led l(true);
				uint8_t length = cp[0];
				if (size >= (length + 15) / 8)
				{
					tick();
					for (uint8_t i = 1; length != 0; ++i)
					{
						uint8_t chunk = length;
						if (chunk > 8)
							chunk = 8;
						length -= chunk;

						uint8_t val = cp[i];
						for (; chunk; --chunk)
						{
							tock();
							pin_tms::set_value(val & 1);
							val >>= 1;
							tick();
						}
					}
					tock();
					com.send_sync(1, 0, 0);
				}
			}
			return true;
		case 2: // SHIFT 8'length length'data
			if (size > 1 && cp[0] <= 8*14)
			{
				led l(true);
				uint8_t length = cp[0];

				uint8_t * wbuf = com.alloc(2, size);
				if (!wbuf)
					return false;

				*wbuf++ = length;

				// PAUSE ->1 EXIT2 ->0 SHIFT
				tick();
				tock();
				pin_tms::set_high();
				pin_tdi::make_output();
				tick();
				tock();
				pin_tms::set_low();
				tick();

				for (uint8_t i = 1; length != 0; ++i)
				{
					uint8_t chunk = length;
					if (chunk > 8)
						chunk = 8;
					length -= chunk;

					uint8_t val = cp[i];
					uint8_t tdo = 0;
					for (; chunk; --chunk)
					{
						tock();
						pin_tdi::set_value(val & 1);
						if (!length && chunk == 1)
							pin_tms::set_high();
						val >>= 1;
						tick();
						tdo = (tdo >> 1);
						if (pin_tdo::read())
							tdo |= 0x80;
					}

					*wbuf++ = tdo;
				}

				// (SHIFT ->1) EXIT1 ->0 PAUSE
				tock();
				pin_tdi::make_input();
				pin_tms::set_low();
				tick();
				tock();

				com.commit();
			}
			return true;
		case 3: // FREQUENCY 32'wait_time
			{
				uint8_t err = 1;
				com.send_sync(3, &err, 1);
			}
			return true;
		case 4: // CLOCK 32'ticks
			if (size >= 4)
			{
				led l(true);
				uint32_t clocks = cp[0]
					| (uint32_t(cp[1]) << 8)
					| (uint32_t(cp[2]) << 16)
					| (uint32_t(cp[3]) << 24);
				while (clocks != 0)
				{
					uint16_t chunk = clocks > 2000? 2000: clocks;
					clocks -= chunk;
					for (; chunk; --chunk)
					{
						tick();
						m_process();
						tock();
					}

					uint8_t last = clocks == 0;
					com.send_sync(4, &last, 1);
				}
			}
			return true;
		}

		return false;
	}

	handler_base::error_t select()
	{
		m_process.disallow_tunnel();
		pin_tms::make_high();
		pin_tck::make_low();
		return 0;
	}

	void unselect()
	{
		pin_tck::make_input();
		pin_tms::make_input();
		m_process.allow_tunnel();
	}

	void process_selected(com_t &)
	{
		tick();
		tock();
	}

private:
	static void nop()
	{
		asm __volatile__ ("nop");
	}

	void clock_wait()
	{
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
	}

	void tick()
	{
		clock_wait();
		pin_tck::set_high();
	}

	void tock()
	{
		clock_wait();
		pin_tck::set_low();
	}

	Process m_process;
};

#endif
