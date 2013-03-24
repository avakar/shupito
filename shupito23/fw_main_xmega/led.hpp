#ifndef SHUPITO_LED_HPP
#define SHUPITO_LED_HPP

void led_init();
void led_process();

void led_on();
void led_off();

void led_blink_short();
void led_blink_long();

class led_holder
{
public:
	led_holder(bool enable = false)
		: m_on(false)
	{
		if (enable)
			this->on();
	}

	~led_holder()
	{
		this->off();
	}

	void on()
	{
		if (!m_on)
		{
			led_on();
			m_on = true;
		}
	}

	void off()
	{
		if (m_on)
		{
			led_off();
			m_on = false;
		}
	}

private:
	bool m_on;
};

#endif // SHUPITO_LED_HPP
