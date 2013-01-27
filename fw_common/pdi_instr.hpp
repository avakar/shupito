#ifndef SHUPITO_PDI_INSTR_HPP
#define SHUPITO_PDI_INSTR_HPP

#include <stdint.h>

template <typename Pdi, typename T>
void pdi_lds(Pdi & pdi, T addr, uint8_t rx_count, uint8_t * buf)
{
	pdi.write(0x00 | ((sizeof addr - 1) << 2) | (rx_count - 1));
	
	uint8_t const * addr_ptr = reinterpret_cast<uint8_t const *>(&addr);
	for (uint8_t i = 1; i != sizeof addr; ++i)
		pdi.write(*addr_ptr++);
	pdi.write(*addr_ptr, rx_count, buf);
}

template <typename R, typename Pdi, typename T>
void pdi_sts(Pdi & pdi, T addr, R data)
{
	pdi.write(0x40 | ((sizeof addr - 1) << 2) | (sizeof data - 1));

	uint8_t const * addr_ptr = reinterpret_cast<uint8_t const *>(&addr);
	for (uint8_t i = 0; i != sizeof addr; ++i)
		pdi.write(*addr_ptr++);

	uint8_t const * data_ptr = reinterpret_cast<uint8_t const *>(&data);
	for (uint8_t i = 0; i != sizeof data; ++i)
		pdi.write(*data_ptr++);
}

template <typename Pdi>
void pdi_ldcs(Pdi & pdi, uint8_t addr, uint8_t * buf)
{
	pdi.write(0x80 | addr, 1, buf);
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
	
	uint8_t const * ptr = reinterpret_cast<uint8_t const *>(&key);
	for (uint8_t i = 0; i != 8; ++i)
		pdi.write(*ptr++);
}

template <typename Pdi, typename T>
void pdi_st_ptr(Pdi & pdi, T addr)
{
	pdi.write(0x68 | (sizeof addr - 1));

	uint8_t const * addr_ptr = reinterpret_cast<uint8_t const *>(&addr);
	for (uint8_t i = 0; i != sizeof addr; ++i)
		pdi.write(*addr_ptr++);
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

template <typename Pdi>
void pdi_rep_ld(Pdi & pdi, uint8_t count, uint8_t * buf)
{
	if (count)
	{
		// REPEAT count-1; LD byte, *ptr++
		pdi.write(0xa0);
		pdi.write(count-1);
		pdi.write(0x24, count, buf);
	}
}

template <typename Pdi, typename T>
void pdi_st(Pdi & pdi, T t)
{
	pdi.write(0x64 | (sizeof t - 1));
	
	uint8_t const * ptr = reinterpret_cast<uint8_t const *>(&t);
	for (uint8_t i = 0; i != sizeof t; ++i)
		pdi.write(*ptr++);
}

template <typename Pdi, typename T>
void pdi_repeat(Pdi & pdi, T count)
{
	pdi.write(0xa0 | (sizeof count - 1));

	uint8_t const * count_ptr = reinterpret_cast<uint8_t const *>(&count_ptr);
	for (uint8_t i = 0; i != sizeof count; ++i)
		pdi.write(*count_ptr++);
}

template <typename Pdi, typename Clock, typename Process>
uint8_t pdi_wait_read(Pdi & pdi, Clock & clock, Process const & process)
{
	typename Clock::time_type t = clock.value();

	uint8_t r = pdi.read_count();
	while (r)
	{
		uint8_t new_r = pdi.read_count();
		if (new_r != r)
		{
			t = clock.value();
			r = new_r;
			continue;
		}

		if (clock.value() - t > Clock::template us<20000>::value)
			return 1;
		process();
	}

	return 0;
}

template <typename Pdi, typename Clock, typename Process>
uint8_t pdi_wait_nvm_busy(Pdi & pdi, Clock & clock, typename Clock::time_type timeout, Process const & process)
{
	typename Clock::time_type t = clock.value();

	for (;;)
	{
		uint8_t status;
		pdi_lds(pdi, (uint32_t)0x010001CF, 1, &status);

		uint8_t error = pdi_wait_read(pdi, clock, process);
		if (error)
			return error;
		if ((status & 0x80) == 0)
			return 0;
		if (clock.value() - t > timeout)
			return 2;
	}
}


template <typename Pdi, typename Clock, typename Process>
uint8_t pdi_ptrcopy(Pdi & pdi, uint8_t * wbuf, uint32_t addr, uint8_t len, Clock & clock, Process const & process)
{
	uint8_t error = 0;
	while (!error && len > 0)
	{
		uint8_t chunk = len > 14? 14: len;

		pdi_st_ptr(pdi, addr);
		pdi_repeat(pdi, (uint8_t)(chunk - 1));
		pdi.write(0x24, chunk, wbuf);
		error = pdi_wait_read(pdi, clock, process);

		addr += chunk;
		len -= chunk;
	}

	return error;
}

#endif
