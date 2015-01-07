#include <stdint.h>
#include <stdio.h>
#include <assert.h>

static int32_t signExtend24Bit(uint32_t u)
{
	//If sign bit is set, fill the top bits with 1s to sign-extend
	return (u & 0x800000) ? (int32_t) (u | 0xFF000000) : u;
}

static int32_t signExtend6Bit(uint8_t byte)
{
	//If sign bit is set, fill the top bits with 1s to sign-extend
	return (byte & 0x20) ? (int32_t) (int8_t) (byte | 0xC0) : byte;
}

static int32_t signExtend4Bit(uint8_t nibble)
{
	//If sign bit is set, fill the top bits with 1s to sign-extend
	return (nibble & 0x08) ? (int32_t) (int8_t) (nibble | 0xF0) : nibble;
}

static int32_t signExtend2Bit(uint8_t byte)
{
	//If sign bit is set, fill the top bits with 1s to sign-extend
	return (byte & 0x02) ? (int32_t) (int8_t) (byte | 0xFC) : byte;
}

int main(void)
{
	for (int32_t i = -8388608; i < 8388608; i++) {
		uint32_t u24 = i & 0x00FFFFFF;

		assert(i == signExtend24Bit(u24));
	}

	for (int32_t i = -32; i < 32; i++) {
		uint32_t u6 = i & 0x3F;

		assert(i == signExtend6Bit(u6));
	}

	for (int32_t i = -8; i < 8; i++) {
		uint32_t u4 = i & 0x0F;

		assert(i == signExtend4Bit(u4));
	}

	for (int32_t i = -2; i < 2; i++) {
		uint32_t u2 = i & 0x00FFFFFF;

		assert(i == signExtend2Bit(u2));
	}

	printf("Done");

	return 0;
}
