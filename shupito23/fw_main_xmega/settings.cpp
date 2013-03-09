#include "settings.hpp"
#include "led.hpp"
#include "dbg.h"
#include "../../fw_common/avrlib/assert.hpp"
#include <avr/io.h>
#include <string.h>

uint8_t g_namedesc[62] = { 16, 3, 'S', 0, 'h', 0, 'u', 0, 'p', 0, 'i', 0, 't', 0, 'o', 0 };
static uint16_t g_settings_seq = 0xffff;

struct settings_t
{
	// the size should not include the CRC
	uint8_t size;
	uint8_t version;
	uint16_t sequence;
	uint8_t namedesc_and_crc[64];
};

static void calc_crc(uint8_t res[2], uint8_t const * p, uint8_t size)
{
	CRC_CTRL = CRC_RESET_RESET0_gc | CRC_SOURCE_DISABLE_gc;
	CRC_CTRL = CRC_RESET_NO_gc | CRC_SOURCE_IO_gc;
	for (; size; --size)
		CRC_DATAIN = *p++;
	CRC_STATUS = CRC_BUSY_bm;
	res[0] = CRC_CHECKSUM0;
	res[1] = CRC_CHECKSUM1;
	CRC_CTRL = CRC_RESET_NO_gc | CRC_SOURCE_DISABLE_gc;
}

bool load_settings()
{
	NVM_CTRLB = NVM_EEMAPEN_bm;

	settings_t const * ptr = (settings_t const *)MAPPED_EEPROM_START;
	uint8_t size = ptr->size;
	if (size < 4 || size == 0xff)
		return false;

	uint8_t crc[2];
	calc_crc(crc, (uint8_t const *)ptr, size);

	uint8_t const * raw_ptr = (uint8_t const *)MAPPED_EEPROM_START;
	if (crc[0] != raw_ptr[size] || crc[1] != raw_ptr[size+1])
		return false;

	uint8_t namedesc_size = ptr->namedesc_and_crc[0];
	if (ptr->version != 1 || namedesc_size > size - 4 || namedesc_size > sizeof g_namedesc || ptr->namedesc_and_crc[1] != 3)
		return false;

	g_settings_seq = ptr->sequence;
	memcpy(g_namedesc, ptr->namedesc_and_crc, namedesc_size);
	return true;
}

static void store_eeprom(uint8_t const * p, uint8_t size)
{
	uint8_t addr = 0;
	while (size)
	{
		uint8_t chunk = size;
		if (chunk > EEPROM_PAGE_SIZE)
			chunk = EEPROM_PAGE_SIZE;

		memcpy((uint8_t *)(MAPPED_EEPROM_START + addr), p, chunk);
		NVM_ADDR0 = addr;
		NVM_ADDR1 = addr >> 8;
		NVM_CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
		CCP = CCP_IOREG_gc;
		NVM_CTRLA = NVM_CMDEX_bm;
		while (NVM_STATUS & NVM_NVMBUSY_bm)
		{
		}

		size -= chunk;
		p += chunk;
		addr += chunk;
	}
}

void update_settings()
{
	AVRLIB_ASSERT(g_namedesc[0] <= 62);
	uint8_t desc_len = g_namedesc[0];

	settings_t buf;
	buf.size = desc_len + 4;
	buf.version = 1;
	buf.sequence = ++g_settings_seq;
	memcpy(buf.namedesc_and_crc, g_namedesc, desc_len);
	calc_crc(buf.namedesc_and_crc + desc_len, (uint8_t const *)&buf, desc_len + 4);

	store_eeprom((uint8_t const *)&buf, desc_len + 6);
	led_blink_short();
}
