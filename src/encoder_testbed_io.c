#include <stdio.h>
#include <stdarg.h>
#include <limits.h>

#include "tools.h"

#include "encoder_testbed_io.h"

uint32_t blackboxWrittenBytes;

void blackboxWrite(uint8_t ch)
{
    putc(ch, stdout);

    blackboxWrittenBytes++;
}

// Print the null-terminated string 's' to the serial port and return the number of bytes written
int blackboxPrint(const char *s)
{
    const char *pos = s;

    while (*pos) {
        blackboxWrite(*pos);
        pos++;
    }

    return pos - s;
}

//printf() to the blackbox serial port with no blocking shenanigans (so it's caller's responsibility to not write too fast!)
int blackboxPrintf(const char *format, ...)
{
    char buffer[512];

    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);

    int result = blackboxPrint(buffer);

    va_end(args);

    return result;
}

/**
 * Write an unsigned integer to the blackbox serial port using variable byte encoding.
 */
void blackboxWriteUnsignedVB(uint32_t value)
{
    //While this isn't the final byte (we can only write 7 bits at a time)
    while (value > 127) {
        blackboxWrite((uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
        value >>= 7;
    }
    blackboxWrite(value);
}

/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
void blackboxWriteSignedVB(int32_t value)
{
    //ZigZag encode to make the value always positive
    blackboxWriteUnsignedVB(zigzagEncode(value));
}

void blackboxWriteS16(int16_t value)
{
    blackboxWrite(value & 0xFF);
    blackboxWrite((value >> 8) & 0xFF);
}

static uint8_t blackboxBitBuffer = 0;
static uint8_t blackboxBitBufferCount = 0;

#define BLACKBOX_BIT_BUFFER_CAPACITY (sizeof(blackboxBitBuffer) * CHAR_BIT)

void blackboxWriteBits(uint32_t bits, unsigned int bitCount) {
    if (bitCount == 0)
        return; // Nothing to write! (return now to avoid shifting left by 32 on the next line, which is undefined)

    // Align the bits to be written to the top of that variable:
    bits <<= sizeof(bits) * CHAR_BIT - bitCount;

    do {
        uint8_t availableCapacity = BLACKBOX_BIT_BUFFER_CAPACITY - blackboxBitBufferCount;
        uint8_t numBitsToWrite = bitCount <= availableCapacity ? bitCount : availableCapacity;

        // Align the bits to be written to the correct part of the buffer and insert them
        blackboxBitBuffer |= bits >> ((sizeof(bits) - sizeof(blackboxBitBuffer)) * CHAR_BIT + blackboxBitBufferCount);
        blackboxBitBufferCount += numBitsToWrite;

        // Did we fill the buffer? If so write the whole thing out
        if (blackboxBitBufferCount == BLACKBOX_BIT_BUFFER_CAPACITY) {
            for (int i = sizeof(blackboxBitBuffer) - 1; i >= 0; i--) {
                blackboxWrite(blackboxBitBuffer >> (CHAR_BIT * i));
            }
            blackboxBitBuffer = 0;
            blackboxBitBufferCount = 0;
        }

        bitCount -= numBitsToWrite;
        bits <<= numBitsToWrite;
    } while (bitCount > 0);
}

void blackboxFlushBits() {
    if (sizeof(blackboxBitBuffer) > 1) {
        // Round up the bits to get the number of occupied bytes
        int numBytes = (blackboxBitBufferCount + CHAR_BIT - 1) / CHAR_BIT;

        for (int i = 0; i < numBytes; i++) {
            // Write the top byte
            blackboxWrite(blackboxBitBuffer >> ((sizeof(blackboxBitBuffer) - 1) * CHAR_BIT));

            // And shift the remaining bits up to fill that space
            blackboxBitBuffer <<= CHAR_BIT;
        }

        blackboxBitBufferCount = 0;
    } else {
        if (blackboxBitBufferCount > 0) {
            blackboxWrite(blackboxBitBuffer);
            blackboxBitBuffer = 0;
            blackboxBitBufferCount = 0;
        }
    }
}

/**
 * How many bits would be required to fit the given integer? `i` must not be zero.
 */
static int numBitsToStoreInteger(uint32_t i)
{
    return sizeof(i) * CHAR_BIT - __builtin_clz(i);
}

void blackboxWriteU32EliasDelta(uint32_t value)
{
    unsigned int valueLen, lengthOfValueLen;

    /* We can't encode value=0, so we need to add 1 to the value before encoding
     *
     * That would make it impossible to encode MAXINT, so instead use MAXINT-1 as an escape code which can mean
     * either MAXINT-1 or MAXINT
     */
    if (value == 0xFFFFFFFF) {
        // Write the escape code of MAXINT - 1
        blackboxWriteU32EliasDelta(0xFFFFFFFF - 1);
        // Add a one bit after the escape code to mean "MAXINT"
        blackboxWriteBits(1, 1);
        return;
    }

    value += 1;

    valueLen = numBitsToStoreInteger(value);
    lengthOfValueLen = numBitsToStoreInteger(valueLen);

    // Use unary to encode the number of bits we'll need to write the length of the `value`
    blackboxWriteBits(0, lengthOfValueLen - 1);
    // Now write the length of the `value`
    blackboxWriteBits(valueLen, lengthOfValueLen);
    // Having now encoded the position of the top bit of `value`, write its remaining bits
    blackboxWriteBits(value, valueLen - 1);

    // Did this end up being an escape code? We must have been trying to write MAXINT - 1
    if (value == 0xFFFFFFFF) {
        // Add a zero bit after the escape code to mean "MAXINT - 1"
        blackboxWriteBits(0, 1);
    }
}

void blackboxWriteS32EliasDelta(int32_t value)
{
    blackboxWriteU32EliasDelta(zigzagEncode(value));
}

void blackboxWriteU32EliasGamma(uint32_t value)
{
    unsigned int lengthOfValue;

    /*
     * We can't encode value=0, so we need to add 1 to the value before encoding
     *
     * That would make it impossible to encode MAXINT, so instead use MAXINT-1 as an escape code which can mean
     * either MAXINT-1 or MAXINT
     */
    if (value == 0xFFFFFFFF) {
        // Write the escape code of MAXINT - 1
        blackboxWriteU32EliasGamma(0xFFFFFFFF - 1);
        // Add a one bit after the escape code to mean "MAXINT"
        blackboxWriteBits(1, 1);
        return;
    }

    value += 1;

    lengthOfValue = numBitsToStoreInteger(value);

    // Use unary to encode the number of bits we'll need to write `value`
    blackboxWriteBits(0, lengthOfValue);

    // Now the bits of value
    blackboxWriteBits(value, lengthOfValue);

    // Did this end up being an escape code? We must have been trying to write MAXINT - 1
    if (value == 0xFFFFFFFF) {
        // Add a zero bit after the escape code to mean "MAXINT - 1"
        blackboxWriteBits(0, 1);
    }
}

void blackboxWriteS32EliasGamma(int32_t value)
{
    blackboxWriteU32EliasGamma(zigzagEncode(value));
}

/**
 * Attempt to guarantee that the given amount of bytes can be written to the Blackbox device without blocking or
 * data loss.
 *
 * Returns:
 *  BLACKBOX_RESERVE_SUCCESS - Upon success
 *  BLACKBOX_RESERVE_TEMPORARY_FAILURE - The buffer is currently too full to service the request, try again later
 *  BLACKBOX_RESERVE_PERMANENT_FAILURE - The buffer is too small to ever service this request
 */
blackboxBufferReserveStatus_e blackboxDeviceReserveBufferSpace(uint32_t bytes)
{
    (void) bytes;

    return BLACKBOX_RESERVE_SUCCESS;
}
