#ifndef SHUPITO_FIRMWARE_HANDLER_XMEGA_HPP
#define SHUPITO_FIRMWARE_HANDLER_XMEGA_HPP

#include "handler_base.hpp"
#include "pdi_instr.hpp"

template <typename Pdi, typename Com, typename Clock, typename Process>
class handler_xmega
	: public handler_base<Com>
{
public:
	typedef Pdi pdi_t;
	typedef Clock clock_t;
	typedef Com com_t;

	handler_xmega(pdi_t & pdi, clock_t & clock, Process process = Process())
		: pdi(pdi), clock(clock), m_fuse_address(0), process(process)
	{
	}

	void unselect()
	{
		pdi.clear();
	}

	bool handle_command(avrlib::command_parser & cp, com_t & com)
	{
		switch (cp.command())
		{
		case 1:
			{
				// Enable programming mode
				pdi.init();
				while (!pdi.tx_ready())
					process();

				typename clock_t::time_type t = clock.value();

				uint8_t error = 0;

				pdi_stcs(pdi, 0x01, 0x59); // Ensure RESET
				pdi_ldcs(pdi, 1);

				uint8_t reset = 0;
				error = pdi_read(pdi, reset, clock, process);
				if (!error && (reset & 1) == 0)
					error = 5;

				if (!error)
					pdi_key(pdi, 0x1289AB45CDD888FFull);

				uint8_t pdi_status = 0;
				while (!error && (pdi_status & 0x02) == 0 && clock.value() - t < Clock::template us<100000>::value)
				{
					pdi_ldcs(pdi, 0);
					error = pdi_read(pdi, pdi_status, clock, process);
				}

				if (!error && (pdi_status & 0x02) == 0)
					error = 3;

				if (error != 0)
					pdi.clear();

				com.write(0x80);
				com.write(0x11);
				com.write(error);
			}
			break;
		case 2: // Leave programming mode

			// Clear the RESET register first; otherwise the chip will be held in reset
			// by the PDI controller until an external reset is issued.
			pdi_stcs(pdi, 0x01, 0x00);

			pdi.clear();

			com.write(0x80);
			com.write(0x21);
			com.write(0);
			break;
		case 3:
			{
				// Read signature
				pdi_sts(pdi, (uint8_t)0x010001CA, (uint8_t)0x43);
				pdi_lds(pdi, (uint32_t)0x01000090, 4);

				com.write(0x80);
				com.write(0x34);

				uint8_t error = 0;
				
				for (uint8_t i = 0; i != 4; ++i)
				{
					uint8_t v = 0;
					if (!error)
						error = pdi_read(pdi, v, clock, process);
					com.write(v);
				}

				if (error)
				{
					pdi.clear();
					com.write(0x80);
					com.write(0x21);
					com.write(error);
				}
			}
			break;
		case 4: // READ 1'memid 4'addr 2'size
			if (cp.size() == 7)
			{
				uint8_t error = 0;

				uint8_t memid = cp[0];
				if (memid == 1 || memid == 2)
				{
					uint32_t addr = cp[1] | (cp[2] << 8) | ((uint32_t)cp[3] << 16);

					addr += memid == 1? 0x800000: 0x8C0000;

					pdi_sts(pdi, (uint32_t)0x010001CA, memid == 1? (uint8_t)0x43: (uint8_t)0x06);
					pdi_st_ptr(pdi, addr);

					uint16_t len = cp[5] | (cp[6] << 8);

					while (error == 0)
					{
						uint8_t chunk = len > 15? 15: (uint8_t)len;
						if (error != 0)
							chunk = 0;

						com.write(0x80);
						com.write(0x40 | chunk);
						for (uint8_t i = chunk; i != 0; --i, ++addr)
						{
							pdi_ld(pdi);
							uint8_t v = 0;
							if (error == 0)
								error = pdi_read(pdi, v, clock, process);
							com.write(v);
							process();
						}

						len -= chunk;
						if (chunk < 15)
							break;
					}
				}
				else if (memid == 3)
				{
					uint8_t len = cp[5];
					if (len > 8)
						len = 8;

					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x07/*read fuse*/);

					com.write(0x80);
					com.write(0x40 | len);
					error = pdi_ptrcopy(pdi, com, 0x008F0020 | (cp[1] & 0x07), len, clock, process);
				}
				else
				{
					error = 1;
				}

				if (error)
				{
					pdi.clear();
					com.write(0x80);
					com.write(0x21);
					com.write(error);
				}
			}
			break;
		case 5:
			// ERASE 1'memid
			{
				uint8_t error = 0;
				if (cp.size() == 0)
				{
					// CMD = Chip erase
					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x40);
					pdi_sts(pdi, (uint32_t)0x010001CB, (uint8_t)0x01);

					error = pdi_wait_nvm_busy(pdi, clock, Clock::template us<50000>::value, process);
					if (error)
						pdi.clear();
				}
				com.write(0x80);
				com.write(0x51);
				com.write(error);
			}
			break;
		case 6:
			// Prepare memory page for a load and write.
			// WPREP 1'memid 4'addr
			if (cp.size() >= 5)
			{
				uint8_t memid = cp[0];

				uint8_t error = 0;
				if (memid == 1 || memid == 2)
				{
					uint32_t addr = cp[1] | (cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
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

				com.write(0x80);
				com.write(0x61);
				com.write(error);
			}
			break;
		case 7:
			// Prepare memory page for a load and write.
			// WFILL 1'memid (1'data)*
			if (cp.size() >= 1)
			{
				uint8_t error = 0;

				uint8_t memid = cp[0];
				if (memid == 1 || memid == 2)
				{
					for (uint8_t i = 1; i < cp.size(); ++i)
					{
						while (!pdi.tx_empty())
							process();
						pdi_st(pdi, cp[i]);
					}
				}
				else if (memid == 3)
				{
					for (uint8_t i = 1; !error && i < cp.size(); ++i)
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

				com.write(0x80);
				com.write(0x71);
				com.write(error);
			}
			break;
		case 8:
			// WRITE 1'memid 4'addr
			if (cp.size() == 5)
			{
				uint8_t error = 0;

				uint8_t memid = cp[0];
				uint32_t addr = cp[1] | (cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
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
				}
				else
				{
					error = 1;
				}

				com.write(0x80);
				com.write(0x81);
				com.write(error);
			}
			break;

		default:
			return false;
		}

		return true;
	}

private:
	pdi_t & pdi;
	clock_t & clock;

	uint8_t m_fuse_address;

	Process process;
};

#endif
