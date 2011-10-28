#ifndef SHUPITO_FIRMWARE_HANDLER_JTAG_HPP
#define SHUPITO_FIRMWARE_HANDLER_JTAG_HPP

#include "handler_base.hpp"

template <typename Com, typename PinRst, typename PinClk, typename PinRx, typename PinTx>
struct handler_jtagg
	: handler_base<Com>
{
	typedef PinRst pin_tms;
	typedef PinClk pin_tck;
	typedef PinRx  pin_tdo;
	typedef PinTx  pin_tdi;

	bool handle_command(avrlib::command_parser & cp, Com & com)
	{
		switch (cp.command())
		{
		case 1: // STATE 8'length length'state_path
			if (cp.size() >= 1)
			{
				uint8_t length = cp[0];
				if (cp.size() >= (length + 15) / 8)
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
					com.write(0x80);
					com.write(0x10);
				}
			}
			return true;
		case 2: // SHIFT 8'length length'data
			if (cp.size() > 1 && cp[0] < 8*14)
			{
				uint8_t length = cp[0];

				com.write(0x80);
				com.write(0x20 | cp.size());
				com.write(length);

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
						tdo = (tdo >> 1);
						if (pin_tdo::read())
							tdo |= 0x80;
						tock();
						pin_tdi::set_value(val & 1);
						if (!length && chunk == 0)
							pin_tms::set_high();
						val >>= 1;
						tick();
					}

					com.write(tdo);
				}

				// SHIFT ->1 EXIT1 ->0 PAUSE
				tock();
				pin_tms::set_high();
				tick();
				tock();
				pin_tdi::make_input();
				pin_tms::set_low();
				tick();
				tock();
			}
			return true;
		case 3: // FREQUENCY 32'wait_time
			return true;
		}

		return false;
	}

	typename handler_base<Com>::error_t select()
	{
		pin_tms::make_high();
		pin_tck::make_low();
		return 0;
	}

	void unselect()
	{
		pin_tck::make_input();
		pin_tms::make_input();
	}

	void process_selected()
	{
		/*tick();
		tock();*/
	}

private:
	static void nop()
	{
		asm __volatile__ ("nop");
	}

	static void clock_wait()
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

	template <typename Pin>
	static void write(bool out, bool value)
	{
		if (!out)
			Pin::make_input();
		else
		{
			if (out)
				Pin::make_high();
			else
				Pin::make_low();
		}
	}
};

#endif
