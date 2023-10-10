#ifndef __CRC32_HPP
#define __CRC32_HPP
static const uint32_t IEEE8023_CRC32_POLYNOMIAL = 0x04C11DB7UL;

template <uint32_t poly>
class crc32_generator
{
private:
	uint32_t generate(uint32_t index)
	{
		uint32_t crc = (index << 24) & 0xFFFFFFFF;
		for (uint8_t i = 0; i < 8; ++i)
		{
			bool carry = crc & 0x80000000;
			crc = (crc << 1) ^ (carry ? poly : 0);
		}
		return crc;
	}

	uint32_t table[256];

public:
	crc32_generator()
	{
		for (uint32_t i = 0; i < 256; ++i)
		{
			table[i] = generate(i);
		}
	}

	const uint32_t calculate_crc32(const uint8_t *data, const size_t length, const uint32_t init, const uint32_t xorout)
	{

		uint32_t crc = init;

		// calculate crc32 checksum for each byte
		for (auto i = 0; i < length; ++i)
		{
			crc = (crc << 8) ^ table[((crc >> 24) ^ data[i]) & 0xFF];
		}

		return crc ^ xorout;
	}

	const uint32_t *getTable() const
	{
		return table;
	}
};

#endif /* __CRC32_HPP */