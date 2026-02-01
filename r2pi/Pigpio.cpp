#include "Pigpio.h"

#include <pigpiod_if2.h>

#include <stdio.h>

Pigpio::Pigpio()
{
}

Pigpio::~Pigpio()
{
	Stop();
}

bool Pigpio::Start()
{
	m_Id = pigpio_start(nullptr, nullptr);
	bool started = m_Id >= 0;

	if (!started)
		fprintf(stderr, "Pigpio: initialization error. Is pigpiod daemon running?\n");

	return started;
}

void Pigpio::Stop()
{
	if (m_Id >= 0)
	{
		pigpio_stop(m_Id);
		m_Id = -1;
	}
}

unsigned int ConvertPullUpDown(Pigpio::PullUpDown value)
{
	switch (value)
	{
	case Pigpio::PullUpDown::OFF:  return PI_PUD_OFF;
	case Pigpio::PullUpDown::DOWN: return PI_PUD_DOWN;
	case Pigpio::PullUpDown::UP:   return PI_PUD_UP;
	default:                       return PI_PUD_OFF;
	}
}

unsigned int ConvertMode(Pigpio::Mode value)
{
	switch (value)
	{
	case Pigpio::Mode::INPUT:  return PI_INPUT;
	case Pigpio::Mode::OUTPUT: return PI_OUTPUT;
	default:                   return PI_INPUT;
	}
}

bool Pigpio::SetPullUpDown(unsigned int gpio, PullUpDown pullUpDown)
{
	if (m_Id < 0)
		return false;

	unsigned int pud = ConvertPullUpDown(pullUpDown);
	int result = set_pull_up_down(m_Id, gpio, pud);
	return result == 0;
}

bool Pigpio::SetMode(unsigned int gpio, Mode mode)
{
	if (m_Id < 0)
		return false;

	unsigned int _mode = ConvertMode(mode);
	int result = set_mode(m_Id, gpio, _mode);
	return result == 0;
}

bool Pigpio::Write(unsigned int gpio, bool value)
{
	if (m_Id < 0)
		return false;

	unsigned int _value = value ? 1 : 0;
	int result = gpio_write(m_Id, gpio, _value);
	return result == 0;
}

bool Pigpio::Read(unsigned int gpio)
{
	if (m_Id < 0)
		return false;

	return static_cast<bool>(gpio_read(m_Id, gpio));
}

bool Pigpio::SetPwmFrequency(unsigned int gpio, unsigned int freq)
{
	if (m_Id < 0)
		return false;

	int retVal = set_PWM_frequency(m_Id, gpio, freq);

	return retVal != PI_BAD_USER_GPIO && retVal != PI_NOT_PERMITTED;
}

bool Pigpio::SetPwmRange(unsigned int gpio, unsigned int range)
{
	if (m_Id < 0)
		return false;

	int retVal = set_PWM_range(m_Id, gpio, range);

	return retVal != PI_BAD_USER_GPIO && retVal != PI_BAD_DUTYRANGE && retVal != PI_NOT_PERMITTED;
}

bool Pigpio::SetPwmDutycycle(unsigned int gpio, unsigned int dutycycle)
{
	if (m_Id < 0)
		return false;

	int retVal = set_PWM_dutycycle(m_Id, gpio, dutycycle);
	return retVal != PI_BAD_USER_GPIO && retVal != PI_BAD_DUTYCYCLE && retVal != PI_NOT_PERMITTED;
}

bool Pigpio::GetPwmDutycycle(unsigned int gpio, unsigned int& outDutycycle)
{
	if (m_Id < 0)
		return false;

	int retVal = get_PWM_dutycycle(m_Id, gpio);

	if (retVal == PI_BAD_USER_GPIO || retVal == PI_NOT_PWM_GPIO)
		return false;

	outDutycycle = static_cast<unsigned int>(retVal);
	return true;
}