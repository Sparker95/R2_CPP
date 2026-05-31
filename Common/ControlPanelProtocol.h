#pragma once

#include <stdint.h>
#include <functional>

#include "Chassis.h"

namespace ControlPanelProtocol
{
	constexpr unsigned int CONTROL_PANEL_PORT = 4567;
	constexpr unsigned int MAX_PAYLOAD_SIZE = 2 * 1024 * 1024;
	constexpr unsigned char HEADER_MAGIC_NR_0 = 0x5A;
	constexpr unsigned char HEADER_MAGIC_NR_1 = 0xA5;

	// !!! Remember to increase protocol version any time anything changes in protocol !!!
	constexpr uint8_t PROTOCOL_VERSION = 1;

	enum class MessageType : uint8_t
	{
		MSG_INVALID = 0,

		// Protocol info
		MSG_REQUEST_PROTOCOL_INFO,     // Request protocol information
		MSG_PROTOCOL_INFO,             // Protocol information data (ControlPanelProtocol version, ControlInputsProtocol version)

		// Universal
		MSG_PRINT, // Receiver will print characters in UI

		//---------------------------------------------------
		// Client to Server

		// Toggles for options
        MSG_ENABLE_STREAM_SENSOR_DATA, // Enables sensor data streaming. Data is byte: 0 - disable, 1 - enable
		MSG_ENABLE_LOCALIZER,          // Enables localizer updates
		MSG_ENABLE_SENSOR_MOTOR,       // Enables sensor motor
		MSG_SWITCH_LOCALIZER_MODE,     // Switches mode of localizer
		MSG_ENABLE_AUTO_CONTROLS_MODE, // Enables AutoControlsMode in RobotCore.

		// Requests for data, single commands without payload
		MSG_REQUEST_MAP,               // Requests Map to be sent back
		MSG_REQUEST_LOC,               // Requests localization data to be sent back
		MSG_RESET_LOCALIZER,           // Resets localizer state
		MSG_REQUEST_OPTIONS,           // Requests current options to be sent back
		MSG_REQUEST_PATH,              // Requests (previously built) path

		// Commands
		MSG_MOVE,                      // Requests movement to position

		//---------------------------------------------------
		// Server to Client

		MSG_SENSOR_DATA, // Scan and odometer data from one cycle
		MSG_MAP,         // Whole Map
		MSG_LOC,         // Localization data
		MSG_OPTIONS,     // Options
		MSG_PATH,        // Path
		MSG_DIAG_DATA    // Diagnostics data
	};

	struct MessageHeader
	{
	public:
		constexpr static unsigned int SIZE_PACKED = 8; // !!! Remember to adjust this whenever header changes !!!

		// Data:
		uint8_t m_MagicNumber0 = HEADER_MAGIC_NR_0;
		uint8_t m_MagicNumber1 = HEADER_MAGIC_NR_1;
		uint8_t m_MsgType = static_cast<uint8_t>(MessageType::MSG_INVALID);
		uint8_t m_Checksum = 0;
		uint32_t m_PayloadSize = 0;

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

	// Serialization and deserialization helpers
	class BufferWriter
	{
	private:
		char* m_pBufferStart;
		char* m_pBufferEnd;
		char* m_pWritePtr;
	public:
		BufferWriter(char* pData, unsigned int bufferSize) :
			m_pBufferStart(pData), m_pBufferEnd(pData + bufferSize), m_pWritePtr(pData)
		{}
		BufferWriter() = delete;

		template<class T>
		void Write(const T& value)
		{
			if (m_pBufferEnd - m_pWritePtr < static_cast<long int>(sizeof(T)))
				return;
			*reinterpret_cast<T*>(m_pWritePtr) = value;
			m_pWritePtr += sizeof(T);
		}

		unsigned int GetNumBytesWritten() const
		{
			return static_cast<unsigned int>(m_pWritePtr - m_pBufferStart);
		}
	};

	class BufferReader
	{
	private:
		const char* m_pBufferStart;
		const char* m_pBufferEnd;
		const char* m_pReadPtr;
	public:
		BufferReader(const char* pData, unsigned int bufferSize) :
			m_pBufferStart(pData), m_pBufferEnd(pData + bufferSize), m_pReadPtr(pData)
		{}
		BufferReader() = delete;

		template<class T>
		const T& Read()
		{
			if (m_pBufferEnd - m_pReadPtr < static_cast<long int>(sizeof(T)))
			{
				// Force crash
				T* null = nullptr;
				return *null;
			}

			const T* returnPtr = reinterpret_cast<const T*>(m_pReadPtr);
			m_pReadPtr += sizeof(T);
			return *returnPtr;
		}

		unsigned int GetNumBytesRead() const { return static_cast<unsigned int>(m_pReadPtr - m_pBufferStart); }
	};

	// Details of serialization protocol
	enum class ChassisMoveState : uint8_t
	{
		STOP     = 0,
		FORWARD  = 1,
		BACKWARD = 2
	};

	// Data of various message types
	// Although some messages are (de)serialized explicitly

	// MSG_SENSOR_DATA
	struct SensorDataHeader
	{
		uint32_t m_Timestamp_ms;
		uint16_t m_Odometer;
		Chassis::MoveState m_MoveState;
		uint8_t _padding;
		float m_Steer;
		uint32_t m_nSamples;
	};

	// MSG_REQUEST_MAP
	struct RequestMapHeader
	{
		constexpr static uint8_t MAP_TYPE_MOST_LIKELY = 0;
		constexpr static uint8_t MAP_TYPE_REAL_TIME = 1;

		uint8_t m_MapType = 0;
	};

	// MSG_MAP
	struct MapDataHeader
	{
		float m_Res_w = 0;
		uint m_xSize = 0;
		uint m_ySize = 0;
		uint m_xCenter = 0;
		uint m_yCenter = 0;
	};

	struct OptionsHeader
	{
		constexpr static uint8_t OPT_STREAM_SENSOR_DATA = 1 << 0;
		constexpr static uint8_t OPT_SENSOR_MOTOR       = 1 << 1;
		constexpr static uint8_t OPT_UPDATE_LOCALIZER   = 1 << 2;
		constexpr static uint8_t OPT_AUTO_CONTROLS      = 1 << 3;

		uint8_t m_Options = 0;
		uint8_t m_LocalizerMode = 0; // Localizer::Mode
	};

	// MSG_MOVE
	struct MoveHeader
	{
		float m_X;
		float m_Y;
		float m_Radius;
	};

	// MSG_DIAG_DATA
	struct DiagDataHeader
	{
		// Localizer
		Pose m_MeanPose;

		// PathFollower
		uint32_t m_PathPointId = -1; // -1 means it's disabled
		uint32_t m_PathEndPointId = -1;
		Vector2 m_PathPointPos;
		Vector2 m_PathEndPointPos;
		Vector2 m_PathLookaheadPos;
	};
}

