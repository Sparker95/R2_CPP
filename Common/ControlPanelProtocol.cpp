#include "ControlPanelProtocol.h"


namespace ControlPanelProtocol
{
	//########################################################################################
	// Message header

void MessageHeader::Init(MessageType msgType, unsigned int payloadSize)
{
    m_MagicNumber0 = HEADER_MAGIC_NR_0;
    m_MagicNumber1 = HEADER_MAGIC_NR_1;
    m_MsgType = static_cast<uint8_t>(msgType);
    m_PayloadSize = static_cast<uint32_t>(payloadSize);
    m_Checksum = CalculateChecksum();
}

uint8_t MessageHeader::CalculateChecksum() const
	{
		uint8_t sum = 0;
		sum += m_MagicNumber0;
		sum += m_MsgType;
		sum += static_cast<uint8_t>((m_PayloadSize >> 24) & 0xFF);
		sum += static_cast<uint8_t>((m_PayloadSize >> 16) & 0xFF);
		sum += static_cast<uint8_t>((m_PayloadSize >> 8) & 0xFF);
		sum += static_cast<uint8_t>(m_PayloadSize & 0xFF);
		sum += m_MagicNumber1;
		return sum;
	}

	bool MessageHeader::IsValid() const
	{
		return m_Checksum == CalculateChecksum() &&
			m_MsgType != static_cast<uint8_t>(MessageType::MSG_INVALID) &&
			m_PayloadSize < MAX_PAYLOAD_SIZE;
	}

	//########################################################################################
	// Message decoder

	MessageDecoder::MessageDecoder()
	{
		m_MsgPayloadBufferMisaligned = new char[MAX_PAYLOAD_SIZE + 16];
		m_MsgPayloadBuffer = m_MsgPayloadBufferMisaligned;
		// Ensure memory is 8-aligned
		while (reinterpret_cast<size_t>(m_MsgPayloadBuffer) & 7)
			m_MsgPayloadBuffer++;
	}

	MessageDecoder::~MessageDecoder()
	{
		delete[] m_MsgPayloadBufferMisaligned;
	}

	void MessageDecoder::Reset()
	{
		m_State = State::WAITING_MSG_HEADER;
		m_iReadCounter = 0;
	}

	void MessageDecoder::ReadStream(const char* pStream, unsigned int nBytes)
	{
		for (unsigned int i = 0; i < nBytes; i++)
			OnByte(pStream[i]);
	}

	void MessageDecoder::OnByte(char b)
	{
		switch (m_State)
		{
			case State::WAITING_MSG_HEADER:
			{
				if (b == HEADER_MAGIC_NR_0)
				{
					m_MsgHeaderBuffer[0] = b;
					m_iReadCounter = 1;
					m_State = State::READING_MSG_HEADER;
				}
				else
				{
					if (m_OnError != nullptr)
						m_OnError(static_cast<uint8_t>(ErrorCode::ERROR_UNEXPECTED_CHAR), static_cast<unsigned int>(b));
				}
				break;
			}
			case State::READING_MSG_HEADER:
			{
				m_MsgHeaderBuffer[m_iReadCounter] = b;
				m_iReadCounter++;
				if (m_iReadCounter == MessageHeader::SIZE_PACKED)
				{
					MessageHeader* pHeader = reinterpret_cast<MessageHeader*>(m_MsgHeaderBuffer);
					if (pHeader->IsValid())
					{
						// Is there any payload?
						if (pHeader->m_PayloadSize)
							m_State = State::READING_MSG_PAYLOAD;
						else
						{
							m_State = State::WAITING_MSG_HEADER;

							// Invoke msg received without payload
							if (m_OnReceivedMessage != nullptr)
								m_OnReceivedMessage(pHeader->m_MsgType, m_MsgPayloadBuffer, 0);
						}
						m_iReadCounter = 0;
					}
					else
					{
						if (m_OnError != nullptr)
							m_OnError(static_cast<uint8_t>(ErrorCode::ERROR_INVALID_HEADER), static_cast<unsigned int>(pHeader->m_MsgType));
					}
				}
				break;
			}
			case State::READING_MSG_PAYLOAD:
			{
				m_MsgPayloadBuffer[m_iReadCounter] = b;
				m_iReadCounter++;
				MessageHeader* pHeader = reinterpret_cast<MessageHeader*>(m_MsgHeaderBuffer);
				if (m_iReadCounter == static_cast<unsigned int>(pHeader->m_PayloadSize))
				{
					m_iReadCounter = 0;
					m_State = State::WAITING_MSG_HEADER;

					// Invoke msg received
					if (m_OnReceivedMessage != nullptr)
						m_OnReceivedMessage(pHeader->m_MsgType, m_MsgPayloadBuffer, pHeader->m_PayloadSize);
				}
				break;
			}
		}
	}
}
