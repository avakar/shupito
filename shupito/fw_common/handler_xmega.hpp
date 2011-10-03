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

	handler_xmega(pdi_t & pdi, clock_t & clock, Process process)
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
				uint8_t pdi_status = 0;

				// Enable programming mode
				pdi.init();
				while (!pdi.tx_ready())
					process();

				pdi_stcs(pdi, 0x01, 0x59); // Ensure RESET
				pdi_key(pdi, 0x1289AB45CDD888FFull);

				typename clock_t::time_type t = clock.value();

				bool success = true;
				while (success && (pdi_status & 0x02) == 0 && clock.value() - t < 1000000)
				{
					pdi_ldcs(pdi, 0);
					success = pdi_read(pdi, pdi_status, clock, process);
				}

				uint8_t err = !success? 1: (pdi_status & 0x02)  == 0? 3: 0;
				if (err != 0)
					pdi.clear();

				com.write(0x80);
				com.write(0x11);
				com.write(err);
			}
			break;
		case 2: // Leave programming mode

			// Clear the RESET register first
			// TODO: is this necessary?
			//pdi_stcs(pdi, 0x01, 0x00);

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

				bool success = true;
				
				for (uint8_t i = 0; i != 4; ++i)
				{
					uint8_t v = 0;
					if (success)
						success = pdi_read(pdi, v, clock, process);
					com.write(v);
				}

				if (!success)
				{
					pdi.clear();
					com.write(0x80);
					com.write(0x21);
					com.write(1);
				}
			}
			break;
		case 4: // READ 1'memid 4'addr 1'size
			if (cp.size() == 6)
			{
				bool success = true;

				uint8_t memid = cp[0];
				if (memid == 1)
				{
					uint32_t addr = cp[1] | (cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
					addr %= 0x0C0000;
					addr += 0x800000;

					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x43);

					uint8_t len = cp[5];

					com.write(0x80);
					com.write(0xf4);
					com.write(len);

					success = pdi_ptrcopy(pdi, com, addr, len, clock, process);
					com.write(0);
				}
				else if (memid == 3)
				{
					uint8_t len = cp[5];
					if (len > 8)
						len = 8;

					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x07/*read fuse*/);

					com.write(0x80);
					com.write(0x40 | len);
					success = pdi_ptrcopy(pdi, com, 0x008F0020 | (cp[1] & 0x07), len, clock, process);
				}
				else
				{
					success = false;
				}

				if (!success)
				{
					pdi.clear();
					com.write(0x80);
					com.write(0x21);
					com.write(1);
				}
			}
			break;
		case 5:
			// ERASE 1'memid
			if (cp.size() == 0)
			{
				// CMD = Chip erase
				pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x40);
				pdi_sts(pdi, (uint32_t)0x010001CB, (uint8_t)0x01);

				bool success = pdi_wait_nvm_busy(pdi, clock, 50000, process);
				com.write(0x80);
				com.write(0x51);
				com.write(!success);
			}
			break;
		case 6:
			// Prepare memory page for a load and write.
			// WPREP 1'memid 4'addr
			if (cp.size() >= 5)
			{
				uint8_t memid = cp[0];

				bool success = true;
				if (memid == 1)
				{
					uint32_t addr = cp[1] | (cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
					addr %= 0x0C0000;
					addr += 0x800000;

					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x26);
					pdi_sts(pdi, (uint32_t)0x010001CB, (uint8_t)0x01);

					success = pdi_wait_nvm_busy(pdi, clock, 10000, process);

					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x23);
					pdi_st_ptr(pdi, addr);
				}
				else if (memid == 3)
				{
					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x4C/*write fuse*/);
					m_fuse_address = cp[1];
				}
				else
				{
					success = false;
				}

				com.write(0x80);
				com.write(0x61);
				com.write(!success);
			}
			break;
		case 7:
			// Prepare memory page for a load and write.
			// WFILL 1'memid (1'data)*
			if (cp.size() >= 1)
			{
				bool success = true;

				uint8_t memid = cp[0];
				if (memid == 1)
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
					for (uint8_t i = 1; success && i < cp.size(); ++i)
					{
						while (!pdi.tx_empty())
							process();
						pdi_sts(pdi, uint32_t(0x08F0020 | (m_fuse_address & 0x07)), cp[i]);
						success = pdi_wait_nvm_busy(pdi, clock, 100000, process);
						++m_fuse_address;
					}
				}

				com.write(0x80);
				com.write(0x71);
				com.write(!success);
			}
			break;
		case 8:
			// WRITE 1'memid 4'addr
			if (cp.size() == 5)
			{
				bool success = true;

				uint8_t memid = cp[0];
				uint32_t addr = cp[1] | (cp[2] << 8) | ((uint32_t)cp[3] << 16) | ((uint32_t)cp[4] << 24);
				if (memid == 1)
				{
					addr %= 0x0C0000;
					addr += 0x800000;

					// CMD = Erase & Write Flash Page
					pdi_sts(pdi, (uint32_t)0x010001CA, (uint8_t)0x2F);
					pdi_sts(pdi, addr, (uint8_t)0);

					success = pdi_wait_nvm_busy(pdi, clock, 10000, process);
				}
				else if (memid == 3)
				{
					// Cycle reset to reload the new fuse values
					pdi_stcs(pdi, 1, 0x00);
					pdi_stcs(pdi, 1, 0x59);
				}
				else
				{
					success = false;
				}

				com.write(0x80);
				com.write(0x81);
				com.write(!success);
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
