#ifndef SHUPITO_PDI_INSTR_HPP
#define SHUPITO_PDI_INSTR_HPP

#include <stdint.h>

template <typename Pdi, typename T>
void pdi_lds(Pdi & pdi, T addr, uint8_t rx_count)
{
	pdi.write(0x00 | ((sizeof addr - 1) << 2) | (rx_count - 1));
	for (uint8_t i = 1; i != sizeof addr; ++i)
	{
		pdi.write((uint8_t)addr);
		addr >>= 8;
	}
	pdi.write((uint8_t)addr, rx_count);
}

template <typename R, typename Pdi, typename T>
void pdi_sts(Pdi & pdi, T addr, R data)
{
	pdi.write(0x40 | ((sizeof addr - 1) << 2) | (sizeof data - 1));
	for (uint8_t i = 0; i != sizeof addr; ++i)
	{
		pdi.write((uint8_t)addr);
		addr >>= 8;
	}
	for (uint8_t i = 0; i != sizeof data; ++i)
	{
		pdi.write((uint8_t)data);
		data >>= 8;
	}
}

template <typename Pdi>
void pdi_ldcs(Pdi & pdi, uint8_t addr)
{
	pdi.write(0x80 | addr, 1);
}

template <typename Pdi>
void pdi_stcs(Pdi & pdi, uint8_t addr, uint8_t data)
{
	pdi.write(0xC0 | addr);
	pdi.write(data);
}

template <typename Pdi>
void pdi_key(Pdi & pdi, uint64_t key)
{
	pdi.write(0xe0);
	for (uint8_t i = 0; i != 8; ++i)
	{
		pdi.write(key);
		key >>= 8;
	}
}

template <typename Pdi, typename T>
void pdi_st_ptr(Pdi & pdi, T addr)
{
	pdi.write(0x68 | (sizeof addr - 1));
	for (uint8_t i = 0; i != sizeof addr; ++i)
	{
		pdi.write(addr);
		addr >>= 8;
	}	
}

template <typename Pdi>
void pdi_ld_ptr(Pdi & pdi, uint8_t len)
{
	pdi.write(0x28 | (len - 1), len);
}

template <typename Pdi>
void pdi_ld(Pdi & pdi)
{
	pdi.write(0x24, 1);
}

template <typename Pdi, typename T>
void pdi_st(Pdi & pdi, T t)
{
	pdi.write(0x64 | (sizeof t - 1));
	for (uint8_t i = 0; i != sizeof t; ++i)
	{
		pdi.write(t);
		t >>= 8;
	}	
}

template <typename Pdi, typename T>
void pdi_repeat(Pdi & pdi, T count)
{
	pdi.write(0xa0 | (sizeof count - 1));
	for (uint8_t i = 0; i != sizeof count; ++i)
	{
		pdi.write(count);
		count >>= 8;
	}	
}

/*
template <typename Pdi, typename T, typename U>
void pdi_ld_range(Pdi & pdi, T addr, U count)
{
	pdi_repeat(pdi, count - 1);
	pdi_st_ptr(pdi, addr);
	pdi.write(0x24, count);
}
*/

template <typename R, typename Pdi, typename Clock, typename Process>
uint8_t pdi_read(Pdi & pdi, R & r, Clock & clock, Process const & process)
{
	r = 0;

	typename Clock::time_type t = clock.value();

	for (uint8_t i = 0; i != sizeof(R); ++i)
	{
		while (!pdi.rx_ready())
		{
			if (clock.value() - t > 4000)
			{
				return 1;
			}
			process();
		}

		r <<= 8;
		r |= pdi.read();
	}

	return 0;
}

template <typename Pdi, typename Clock, typename Process>
uint8_t pdi_wait_nvm_busy(Pdi & pdi, Clock & clock, typename Clock::time_type timeout, Process const & process)
{
	typename Clock::time_type t = clock.value();

	for (;;)
	{
		pdi_lds(pdi, (uint32_t)0x010001CF, 1);

		uint8_t status;
		uint8_t error = pdi_read(pdi, status, clock, process);
		if (error)
			return error;
		if ((status & 0x80) == 0)
			return 0;
		if (clock.value() - t > timeout)
			return 2;
	}
}


template <typename Pdi, typename Com, typename Clock, typename Process>
uint8_t pdi_ptrcopy(Pdi & pdi, Com & com, uint32_t addr, uint8_t len, Clock & clock, Process const & process)
{
	uint8_t error = 0;
	while (len > 0)
	{
		uint8_t chunk = len > 14? 14: len;

		if (!error)
		{
			pdi_st_ptr(pdi, addr);
			pdi_repeat(pdi, (uint8_t)(chunk - 1));
			pdi.write(0x24, chunk);
		}

		for (uint8_t i = 0; i != chunk; ++i)
		{
			uint8_t v = 0;
			if (!error)
				error = pdi_read(pdi, v, clock, process);
			com.write(v);
		}

		addr += chunk;
		len -= chunk;
	}

	return error;
}

#endif
