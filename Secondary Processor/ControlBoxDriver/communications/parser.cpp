#include "parser.hpp"

const uint8_t START_BYTE = 0x06;
const uint8_t MINIMUM_SIZE = 6;

FastTransferParser::FastTransferParser ()
{
	index = 0;
}

void FastTransferParser::push (uint8_t byte)
{
	if (is_empty() && byte != START_BYTE)
		return;
	buffer[index++] = byte;
}


bool FastTransferParser::valid_messages () const
{

	// Verify that the size is at least the minimum
	if (size() < MINIMUM_SIZE)
		return false;

	// Verify that the receiver address matches argument
	if (buffer[2] != 0x01)
		return false;

	// Verify that the overall size makes sense according to the packet size
	if (buffer[4] + MINIMUM_SIZE != size())
		return false;

	return verify_crc();
}

bool FastTransferParser::is_full () const
{
	return size() >= BUFFER_SIZE;
}

bool FastTransferParser::is_empty () const
{
	return size() == 0;
}


uint8_t FastTransferParser::size () const
{
	return index;
}

void FastTransferParser::clear ()
{
	index = 0;
}

Message FastTransferParser::get_message (uint8_t i) const
{
	uint8_t index = Message::DATA_INDEX + i * Message::LENGTH;
	Message msg;
	msg.address = buffer[index];
	msg.first = buffer[index + 1];
	msg.second = buffer[index + 2];
	return msg;
}

uint8_t FastTransferParser::num_messages () const
{
	if (!valid_messages())
		return 0;
	return buffer[Message::COUNT_INDEX] / Message::LENGTH;
}

bool FastTransferParser::verify_crc () const
{
	const uint8_t CRC_POLYNOMIAL = 0x8C;
	uint8_t crc = 0;
	for (int i = 5; i < size() - 1; ++i)
	{
		uint8_t data = buffer[i];
		for (int j = 0; j < 8; ++j)
		{
			uint8_t sum = (crc ^ data) & 0x01;
			crc >>= 1;
			if (sum)
				crc ^= CRC_POLYNOMIAL;
			data >>= 1;
		}
	}
	return crc == buffer[size() - 1];
}