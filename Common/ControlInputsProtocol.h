#pragma once

#include <stdint.h>

namespace ControlInputsProtocol
{
	constexpr unsigned int CONTROL_INPUTS_PORT = 4568;

	// !!! Remember to increase it every time version changes! !!!
	constexpr uint8_t PROTOCOL_VERSION = 1;

	struct Message
	{
        constexpr static uint8_t MAGIC_NUMBER_0 = 0x80;
        constexpr static uint8_t MAGIC_NUMBER_1 = 0x5A;
        constexpr static uint8_t MAGIC_NUMBER_2 = 0xA5;

        // Values for m_CmdFlags
        constexpr static uint8_t CMD_FLAG_BRAKE        = 1 << 0;
        constexpr static uint8_t CMD_FLAG_MOVE         = 1 << 1;
        constexpr static uint8_t CMD_FLAG_MOVE_DIR_FWD = 1 << 2;

        enum class MoveSpeed : uint8_t
        {
            SLOW   = 0,
            CRUISE = 1,
            FAST   = 2
        };

        // Data
        uint8_t m_iMagicNumber0 = MAGIC_NUMBER_0;
        uint8_t m_iMagicNumber1 = MAGIC_NUMBER_1;
        uint8_t m_iMagicNumber2 = MAGIC_NUMBER_2;
        uint8_t m_CmdFlags = 0;
        float m_fSteerNorm = 0.0f;
        MoveSpeed m_MoveSpeed = {};

        bool IsValid() const
        {
            return m_iMagicNumber0 == MAGIC_NUMBER_0 &&
                m_iMagicNumber1 == MAGIC_NUMBER_1 &&
                m_iMagicNumber2 == MAGIC_NUMBER_2;
        }
	};
}
