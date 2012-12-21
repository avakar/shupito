#ifndef SHUPITO_VOLTAGE_HPP
#define SHUPITO_VOLTAGE_HPP

#include <stdint.h>

template <uint8_t SampleCount>
class voltage_filter
{
public:
	static uint8_t const sample_count = SampleCount;

	void init()
	{
		for (uint8_t i = 0; i != sample_count; ++i)
			m_measurements[i] = 0;
		m_average = 0;
		m_current_ptr = 0;
	}

	void push(int16_t data)
	{
		m_average -= m_measurements[m_current_ptr];
		m_measurements[m_current_ptr] = data;
		if (m_current_ptr == sample_count - 1)
			m_current_ptr = 0;
		else
			++m_current_ptr;
		m_average += data;
	}

	int16_t current() const
	{
		return m_average / sample_count;
	}

private:
	int16_t m_measurements[sample_count];
	int32_t m_average;
	uint8_t m_current_ptr;
};

#endif // SHUPITO_VOLTAGE_HPP
