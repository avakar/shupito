#include "handler_jtag_fast.hpp"
#include "led.hpp"
#include "app.hpp"

static uint8_t g_jtag_out_buffer[255 + 3];
static uint8_t g_jtag_in_buffer[255 + 3];

bool handler_jtag_fast::handle_command(uint8_t cmd, uint8_t const * cp, uint8_t size, com_t & com)
{
	uint8_t err = 1;
	switch (cmd)
	{
	case 1: // STATE 8'length length'state_path
		if (size >= 1)
		{
			uint8_t length = cp[0];
			if (size == (length + 15) / 8)
			{
				uint8_t templ = PORTC_OUT & ~pin_tms::value_pin::bm;

				uint8_t const * p = cp + 1;
				uint8_t i = 0;
				while (length)
				{
					uint8_t chunk = length;
					if (chunk > 8)
						chunk = 8;
					length -= chunk;

					uint8_t v = *p++;
					for (; chunk; --chunk)
					{
						g_jtag_out_buffer[i++] = (v & 1)? templ | pin_tms::value_pin::bm: templ;
						v >>= 1;
					}
				}

				led_holder l(true);

				DMA_CH2_TRFCNT = i;
				DMA_CH2_CTRLA = DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
				DMA_CH2_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
				while (DMA_CH2_CTRLA & DMA_CH_ENABLE_bm)
				{
					g_process();
				}

				err = 0;
			}
		}

		com.send_sync(1, &err, 1);
		return true;

	case 2: // SHIFT 3'length 5'flags length'data
		if (size > 1)
		{
			bool verify = (cp[0] & 0x10) == 0;

			uint8_t * wbuf = com.alloc(2, verify? size: 1);
			if (!wbuf)
				return false;

			uint16_t length = (size - 1) * 8;
			uint8_t mod = cp[0] & 0x07;
			if (mod)
				length = length - 8 + mod;

			uint8_t const * out_data = cp + 1;

			*wbuf++ = 0;
			while (length)
			{
				uint8_t block_length = length > 248? 248: length;
				length -= block_length;

				uint8_t templ = PORTC_OUT & ~(pin_tms::value_pin::bm | pin_tdi::value_pin::bm);
				uint8_t * buf = g_jtag_out_buffer;

				// PAUSE ->1 EXIT2 ->0 SHIFT
				*buf++ = templ | pin_tms::value_pin::bm | pin_tdi::value_pin::bm;
				*buf++ = templ;

				uint8_t remaining_length = block_length;
				while (remaining_length)
				{
					uint8_t chunk = remaining_length >= 8? 8: remaining_length;
					remaining_length -= chunk;

					uint8_t v = *out_data++;
					for (; chunk; --chunk)
					{
						*buf++ = (v & 1)? templ | pin_tdi::value_pin::bm: templ;
						v >>= 1;
					}
				}

				// (SHIFT ->1) EXIT1 ->0 PAUSE
				buf[-1] |= pin_tms::value_pin::bm;
				*buf++ = templ | pin_tdi::value_pin::bm;

				uint16_t buf_len = buf - g_jtag_out_buffer;

				led_holder l(true);

				pin_tdi::make_high();
				DMA_CH2_TRFCNT = buf_len;
				DMA_CH2_CTRLA = DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;

				if (verify)
				{
					DMA_CH3_TRFCNT = buf_len;
					DMA_CH3_CTRLA = DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;

					cli();
					DMA_CH2_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
					DMA_CH3_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
					sei();

					while ((DMA_CH2_CTRLA & DMA_CH_ENABLE_bm) != 0
						|| (DMA_CH3_CTRLA & DMA_CH_ENABLE_bm) != 0)
					{
						g_process();
					}
				}
				else
				{
					DMA_CH2_CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
					while ((DMA_CH2_CTRLA & DMA_CH_ENABLE_bm) != 0)
						g_process();
				}

				pin_tdi::make_input();

				if (verify)
				{
					uint8_t offset = 0;
					for (; offset < 2; ++offset)
					{
						if ((g_jtag_in_buffer[offset] & pin_tdi::value_pin::bm) == 0)
							break;
					}

					if (offset == 3)
						wbuf[-1] = 3;

					uint8_t * in_data = g_jtag_in_buffer + offset + 1;
					remaining_length = block_length;
					while (remaining_length)
					{
						uint8_t chunk = remaining_length >= 8? 8: remaining_length;
						remaining_length -= chunk;

						uint8_t v = 0;
						for (; chunk; --chunk)
						{
							v >>= 1;
							v |= (*in_data++ & pin_tdo::bm)? 0x80: 0;
						}

						*wbuf++ = v;
					}
				}
			}

			com.commit();
		}
		else
		{
			com.send_sync(1, &err, 1);
		}
		return true;
	case 3: // FREQUENCY 32'wait_time
		{
			uint16_t per = cp[0] | (cp[1] << 8);
			if (cp[2] || cp[3] || per == 0xffff)
				per = 0xfffe;
			else
				per = (per + 1) & 0xfffe;

			if (m_timer_running)
			{
				TCC0_CTRLFSET = TC0_LUPD_bm;
				TCC0_PERBUF = per - 1;
				TCC0_CCDBUF = per / 2;
				TCC0_CTRLFCLR = TC0_LUPD_bm;
			}
			else
			{
				TCC0_CNT = 0;
				TCC0_PER = per - 1;
				TCC0_CCD = per / 2;
				TCC0_CTRLB = TC0_CCDEN_bm | TC_WGMODE_SINGLESLOPE_gc;
				TCC0_CTRLA = TC_CLKSEL_DIV1_gc;
				m_timer_running = true;
			}

			TCC0_INTFLAGS = TC0_OVFIF_bm;
			while ((TCC0_INTFLAGS & TC0_OVFIF_bm) == 0)
				g_process();

			com.send_sync(3, (uint8_t const *)&per, 2);
		}
		return true;
	case 4: // CLOCK 32'ticks
		if (size >= 4)
		{
			led_holder l(true);

			uint16_t clocks_hi = cp[2] | (cp[3] << 8);

			TCC1_CNT = cp[0] | (cp[1] << 8);
			TCC1_CTRLFSET = TC1_DIR_bm;
			TCC1_INTFLAGS = TC1_OVFIF_bm;
			TCC1_CTRLA = TC_CLKSEL_EVCH1_gc;

			avrlib::timeout<clock_t> update_timeout(clock, clock_t::us<10000>::value);
			for (;;)
			{
				g_process();

				uint16_t captured_cnt = TCC1_CNT;
				if (TCC1_INTFLAGS & TC1_OVFIF_bm)
				{
					TCC1_INTFLAGS = TC1_OVFIF_bm;
					if (clocks_hi-- == 0)
						break;
					captured_cnt = TCC1_CNT;
				}

				if (update_timeout)
				{
					update_timeout.force();
					if (uint8_t * wbuf = com.alloc(4, 4))
					{
						*wbuf++ = captured_cnt;
						*wbuf++ = captured_cnt >> 8;
						*wbuf++ = clocks_hi;
						*wbuf = clocks_hi >> 8;
						com.commit();
						update_timeout.restart();
					}
				}
			}

			TCC1_CTRLA = 0;

			err = 0;
			com.send_sync(4, &err, 1);
		}
		return true;

	case 5: // TRST 2'reset_mode
		{
			switch (cp[0] & 0x03)
			{
			case 0: // ON
				pin_rst::make_high();
				break;
			case 1: // OFF
				pin_rst::make_low();
				break;
			default: // Z or ABSENT
				pin_rst::make_input();
			}

			while (!pin_rst::ready())
				g_process();

			uint8_t err = 0;
			com.send_sync(5, &err, 1);
		}
		return true;
	}

	return false;
}

handler_base::error_t handler_jtag_fast::select()
{
	m_timer_running = false;

	hiv_disallow();
	g_app.disallow_tunnel();
	pin_tms::make_high();
	pin_tck::make_low();

	EVSYS_CH0MUX = EVSYS_CHMUX_TCC0_CCD_gc;
	EVSYS_CH0CTRL = 0;
	EVSYS_CH1MUX = EVSYS_CHMUX_TCC0_OVF_gc;
	EVSYS_CH1CTRL = 0;

	DMA_CH2_TRIGSRC = DMA_CH_TRIGSRC_EVSYS_CH0_gc;
	DMA_CH2_ADDRCTRL = DMA_CH_SRCRELOAD_TRANSACTION_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_FIXED_gc;

	uint16_t srcaddr = (uint16_t)g_jtag_out_buffer;
	DMA_CH2_SRCADDR0 = srcaddr;
	DMA_CH2_SRCADDR1 = srcaddr >> 8;
	DMA_CH2_SRCADDR2 = 0;

	uint16_t destaddr = (uint16_t)&PORTC_OUT;
	DMA_CH2_DESTADDR0 = destaddr;
	DMA_CH2_DESTADDR1 = destaddr >> 8;
	DMA_CH2_DESTADDR2 = 0;

	DMA_CH3_TRIGSRC = DMA_CH_TRIGSRC_EVSYS_CH1_gc;
	DMA_CH3_ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc | DMA_CH_SRCDIR_FIXED_gc | DMA_CH_DESTRELOAD_TRANSACTION_gc | DMA_CH_DESTDIR_INC_gc;

	srcaddr = (uint16_t)&PORTC_IN;
	DMA_CH3_SRCADDR0 = srcaddr;
	DMA_CH3_SRCADDR1 = srcaddr >> 8;
	DMA_CH3_SRCADDR2 = 0;

	destaddr = (uint16_t)g_jtag_in_buffer;
	DMA_CH3_DESTADDR0 = destaddr;
	DMA_CH3_DESTADDR1 = destaddr >> 8;
	DMA_CH3_DESTADDR2 = 0;

	return 0;
}

void handler_jtag_fast::unselect()
{
	pin_rst::make_input();

	TCC0_CCDBUF = 0;
	TCC0_INTFLAGS = TC0_OVFIF_bm;
	while ((TCC0_INTFLAGS & TC0_OVFIF_bm) == 0)
		g_process();
	TCC0_CTRLA = 0;

	pin_tms::make_input();
	pin_tck::make_input();
	g_app.allow_tunnel();

	while (!pin_rst::ready())
		g_process();
	hiv_allow();
}
