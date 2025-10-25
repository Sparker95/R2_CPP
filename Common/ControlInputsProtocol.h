#pragma once

#include <stdint.h>

namespace ControlInputsProtocol
{
	constexpr unsigned int CONTROL_INPUTS_PORT = 4568;

	struct Message
	{
        constexpr static uint8_t MAGIC_NUMBER_0 = 0x79;
        constexpr static uint8_t MAGIC_NUMBER_1 = 0x5A;
        constexpr static uint8_t MAGIC_NUMBER_2 = 0xA5;

        // Data
        uint8_t m_iMagicNumber0 = MAGIC_NUMBER_0;
        uint8_t m_iMagicNumber1 = MAGIC_NUMBER_1;
        uint8_t m_iMagicNumber2 = MAGIC_NUMBER_2;
        bool m_bBrake = 0;
		float m_fSteer = 0;
        float m_fMotorSpeed= 0;

        bool IsValid() const
        {
            return m_iMagicNumber0 == MAGIC_NUMBER_0 &&
                m_iMagicNumber1 == MAGIC_NUMBER_1 &&
                m_iMagicNumber2 == MAGIC_NUMBER_2;
        }
	};
}
