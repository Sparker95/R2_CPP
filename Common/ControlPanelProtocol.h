#pragma once

#include <stdint.h>
#include <functional>

namespace ControlPanelProtocol
{
	constexpr unsigned int CONTROL_PANEL_PORT = 4567;
	constexpr unsigned int MAX_PAYLOAD_SIZE = 8 * 1024;
	constexpr unsigned char HEADER_MAGIC_NR_0 = 0x5A;
	constexpr unsigned char HEADER_MAGIC_NR_1 = 0xA5;

	enum class MessageType : uint8_t
	{
		MSG_INVALID = 0,

		// Universal
		MSG_PRINT = 1, // Receiver will print characters in UI

		// Client to Server
		MSG_RQ_SENSOR_SCAN_DATA, // Requests sensor scan data
		MSG_ICP_TEST, // Requests ICP test. Data is byte: 0 - store scan 0, 1 - store scan 1 and run ICP on them.

		// Server to Client
        MSG_TEST0,
		MSG_SENSOR_SCAN_DATA // Sends whole scan data
	};

	struct MessageHeader
	{
	public:
		constexpr static unsigned int SIZE_PACKED = 6;

		// Data:

		uint8_t m_MagicNumber0 = HEADER_MAGIC_NR_0;
		uint8_t m_MsgType = static_cast<uint8_t>(MessageType::MSG_INVALID);
		uint16_t m_PayloadSize = 0;
		uint8_t m_MagicNumber1 = HEADER_MAGIC_NR_1;
		uint8_t m_Checksum = 0;

		// Methods:

        // Sets msg type and PL size, calculates checksum. After that header can be sent.
        void Init(MessageType msgType, unsigned int payloadSize);

        uint8_t CalculateChecksum() const;
		bool IsValid() const; // Verifies checksum, message type, size of payload
	};

	class MessageDecoder
	{
	public:
		void ReadStream(const char* pStream, unsigned int nBytes);

		MessageDecoder();
		~MessageDecoder();

		template<typename F>
		void BindOnReceivedMessage(F f) { m_OnReceivedMessage = f; }

		template<typename F>
		void BindOnError(F f) { m_OnError = f; }

		void Reset();

	private:
		enum class State
		{
			WAITING_MSG_HEADER,
			READING_MSG_HEADER,
			READING_MSG_PAYLOAD
		};

		enum class ErrorCode : uint8_t
		{
			ERROR_UNEXPECTED_CHAR, // Wrong character while waiting for header. Extra data: the character
			ERROR_INVALID_HEADER   // Extra data: message type
		};

		char* m_MsgPayloadBufferMisaligned = nullptr;
		char* m_MsgPayloadBuffer = nullptr;
		char m_MsgHeaderBuffer[sizeof(MessageHeader)]; // Mind alignment of this!
		State m_State = State::WAITING_MSG_HEADER;
		unsigned int m_iReadCounter = 0;

		// Callbacks
		std::function<void(uint8_t, char*, unsigned int)> m_OnReceivedMessage; // Msg type, payload ptr, payload size
		std::function<void(uint8_t, unsigned int)> m_OnError; // ErrorCode, extra data deppends on error code

	private:
		// Methods

		void OnByte(char b);
	};
}

