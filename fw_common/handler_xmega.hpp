#ifndef SHUPITO_FIRMWARE_HANDLER_XMEGA_HPP
#define SHUPITO_FIRMWARE_HANDLER_XMEGA_HPP

#include "handler_base.hpp"
#include "pdi_instr.hpp"

template <typename Pdi, typename Clock, typename Process>
class handler_xmega
	: public handler_base
{
public:
	typedef Pdi pdi_t;
	typedef Clock clock_t;

	handler_xmega(pdi_t & pdi, clock_t & clock, Process process = Process())
		: pdi(pdi), clock(clock), m_fuse_address(0), process(process)
	{
	}

	void unselect()
	{
		pdi.clear();
	}

	bool handle_command(uint8_t cmd, uint8_t const * cp, uint8_t size, com_t & w)
	{
		switch (cmd)
		{
		case 1: // PROGEN 2'bsel
			{
				// Enable programming mode
				pdi.init(cp[0] | (cp[1] << 8));
				while (!pdi.tx_ready())
					process();

				uint8_t error = 0;

				pdi_stcs(pdi, 0x01, 0x59); // Ensure RESET

				uint8_t reset = 0;
				pdi_ldcs(pdi, 1, &reset);
				error = pdi_wait_read(pdi, clock, process);
				if (!error && (reset & 1) == 0)
					error = 5;

				if (!error)
				{
					pdi_key(pdi, 0x1289AB45CDD888FFull);
					error = this->wait_for_nvm();
				}

				if (error != 0)
					pdi.clear();

				w.send_sync(1, &error, 1);
			}
			break;
		case 2: // Leave programming mode
			// Clear the RESET register first; otherwise the chip will be held in reset
			// by the PDI controller until an external reset is issued.
			if (pdi.enabled())
			{
				pdi_stcs(pdi, 0x01, 0x00);
				pdi.clear();
			}

			{
				uint8_t error = 0;
				w.send_sync(2, &error, 1);
			}
			break;
		case 3:
			{
				// Read signature
				uint8_t buf[5];
				pdi_lds(pdi, (uint32_t)0x01000090, 4, buf);

				uint8_t error = pdi_wait_read(pdi, clock, process);
				if (error)
					pdi.clear();
				buf[4] = error;
				w.send_sync(3, buf, sizeof buf);
			}
			break;
		case 4: // READ 1'memid 4'addr 2'size
			if (size == 7)
			{
				uint8_t error = 0;

				uint8_t memid = cp[0];
				if (memid == 1 || memid == 2)
				{
					uint32_t addr = cp[1] | ((uint16_t)cp[2] << 8) | ((uint32_t)cp[3] << 16);

					addr += memid == 1? 0x800000: 0x8C0000;

					pdi_sts(pdi, (uint32_t)0x010001CA, memid == 1? (uint8_t)0x43: (uint8_t)0x06);
					pdi_st_ptr(pdi, addr);

					uint16_t len = cp[5] | (cp[6] << 8);

					while (error == 0)
					{
						uint8_t max_packet_size = w.max_packet_size();
						uint8_t chunk = len > max_packet_size? max_packet_size: (uint8_t)len;
						if (error != 0)
							chunk = 0;

						uint8_t * wbuf = w.alloc_sync(4, chunk);
						pdi_rep_ld(pdi, chunk, wbuf);
						error = pdi_wait_read(pdi, clock, process);
						w.commit();

						len -= chunk;
						if (chunk < max_packet_size)
							break;
					}
				}
				else if (memid == 3)
				{
					uint8_t len = cp[5];
					if (len > 8)
						len = 8;

					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x07/*read fuse*/);

					uint8_t * wbuf = w.alloc_sync(4, len);
					error = pdi_ptrcopy(pdi, wbuf, 0x008F0020 | (cp[1] & 0x07), len, clock, process);
					w.commit();
				}
				else
				{
					error = 1;
				}

				if (error)
				{
					pdi.clear();
					w.send_sync(2, &error, 1);
				}
			}
			break;
		case 5:
			// ERASE 1'memid
			{
				uint8_t error = 0;
				if (size == 0)
				{
					// CMD = Chip erase
					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x40);
					pdi_sts(pdi, (uint32_t)0x010001CB, (uint8_t)0x01);

					error = pdi_wait_nvm_busy(pdi, clock, Clock::template us<50000>::value, process);
					if (error)
						pdi.clear();
				}
				w.send_sync(5, &error, 1);
			}
			break;
		case 6:
			// Prepare memory page for a load and write.
			// WPREP 1'memid 4'addr
			if (size >= 5)
			{
				uint8_t memid = cp[0];

				uint8_t error = 0;
				if (memid == 1 || memid == 2)
				{
					uint32_t addr = cp[1] | ((uint16_t)cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
					addr += memid == 1? 0x800000: 0x8C0000;

					// Erase page buffer
					pdi_sts(pdi, (uint32_t)0x010001CA, memid == 1? (uint8_t)0x26: (uint8_t)0x36);
					pdi_sts(pdi, (uint32_t)0x010001CB, (uint8_t)0x01);

					error = pdi_wait_nvm_busy(pdi, clock, Clock::template us<10000>::value, process);
					if (error)
					{
						pdi.clear();
					}
					else
					{
						// Load page buffer
						pdi_sts(pdi, (uint32_t)0x010001CA, memid == 1? (uint8_t)0x23: (uint8_t)0x33);
						pdi_st_ptr(pdi, addr);
					}
				}
				else if (memid == 3)
				{
					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x4C/*write fuse*/);
					m_fuse_address = cp[1];
				}
				else
				{
					error = 1;
				}

				w.send_sync(6, &error, 1);
			}
			break;
		case 7:
			// Prepare memory page for a load and write.
			// WFILL 1'memid (1'data)*
			if (size >= 1)
			{
				uint8_t error = 0;

				uint8_t memid = cp[0];
				if (memid == 1 || memid == 2)
				{
					for (uint8_t i = 1; i < size; ++i)
					{
						while (!pdi.tx_empty())
							process();
						pdi_st(pdi, cp[i]);
					}
				}
				else if (memid == 3)
				{
					for (uint8_t i = 1; !error && i < size; ++i)
					{
						while (!pdi.tx_empty())
							process();
						pdi_sts(pdi, uint32_t(0x08F0020 | (m_fuse_address & 0x07)), cp[i]);
						error = pdi_wait_nvm_busy(pdi, clock, Clock::template us<100000>::value, process);
						if (error)
							pdi.clear();
						++m_fuse_address;
					}
				}

				w.send_sync(7, &error, 1);
			}
			break;
		case 8:
			// WRITE 1'memid 4'addr
			if (size == 5)
			{
				uint8_t error = 0;

				uint8_t memid = cp[0];
				uint32_t addr = cp[1] | ((uint16_t)cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
				if (memid == 1 || memid == 2)
				{
					addr += memid == 1? 0x800000: 0x8C0000;

					// CMD = Erase & Write Page
					pdi_sts(pdi, (uint32_t)0x010001CA, memid == 1? (uint8_t)0x2F: (uint8_t)0x35);
					pdi_sts(pdi, addr, (uint8_t)0);

					error = pdi_wait_nvm_busy(pdi, clock, Clock::template us<50000>::value, process);
					if (error)
						pdi.clear();
				}
				else if (memid == 3)
				{
					// Cycle reset to reload the new fuse values
					pdi_stcs(pdi, 1, 0x00);
					pdi_stcs(pdi, 1, 0x59);
					error = this->wait_for_nvm();
				}
				else
				{
					error = 1;
				}

				w.send_sync(8, &error, 1);
			}
			break;

		default:
			return false;
		}

		return true;
	}

private:
	uint8_t wait_for_nvm()
	{
		uint8_t error = 0;
		uint8_t pdi_status = 0;
		typename clock_t::time_type t = clock.value();

		// Note that XMEGAs can be configured to stay in reset for up to 64ms.
		// Therefore, we may have to wait for at least that long.
		while (!error && (pdi_status & 0x02) == 0 && clock.value() - t < Clock::template us<128000>::value)
		{
			pdi_ldcs(pdi, 0, &pdi_status);
			error = pdi_wait_read(pdi, clock, process);
		}

		if (!error && (pdi_status & 0x02) == 0)
			error = 3;

		return error;
	}

	pdi_t & pdi;
	clock_t & clock;

	uint8_t m_fuse_address;

	Process process;
};

#endif
