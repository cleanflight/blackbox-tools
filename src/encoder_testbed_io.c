#include <stdio.h>
#include <stdarg.h>

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

/**
 * Write a 2 bit tag followed by 3 signed fields of 2, 4, 6 or 32 bits
 */
void blackboxWriteTag2_3S32(int32_t *values) {
    static const int NUM_FIELDS = 3;

    //Need to be enums rather than const ints if we want to switch on them (due to being C)
    enum {
        BITS_2  = 0,
        BITS_4  = 1,
        BITS_6  = 2,
        BITS_32 = 3
    };

    enum {
        BYTES_1  = 0,
        BYTES_2  = 1,
        BYTES_3  = 2,
        BYTES_4  = 3
    };

    int x;
    int selector = BITS_2, selector2;

    /*
     * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
     * below:
     *
     * Selector possibilities
     *
     * 2 bits per field  ss11 2233,
     * 4 bits per field  ss00 1111 2222 3333
     * 6 bits per field  ss11 1111 0022 2222 0033 3333
     * 32 bits per field sstt tttt followed by fields of various byte counts
     */
    for (x = 0; x < NUM_FIELDS; x++) {
        //Require more than 6 bits?
        if (values[x] >= 32 || values[x] < -32) {
            selector = BITS_32;
            break;
        }

        //Require more than 4 bits?
        if (values[x] >= 8 || values[x] < -8) {
             if (selector < BITS_6) {
                 selector = BITS_6;
             }
        } else if (values[x] >= 2 || values[x] < -2) { //Require more than 2 bits?
            if (selector < BITS_4) {
                selector = BITS_4;
            }
        }
    }

    switch (selector) {
        case BITS_2:
            blackboxWrite((selector << 6) | ((values[0] & 0x03) << 4) | ((values[1] & 0x03) << 2) | (values[2] & 0x03));
        break;
        case BITS_4:
            blackboxWrite((selector << 6) | (values[0] & 0x0F));
            blackboxWrite((values[1] << 4) | (values[2] & 0x0F));
        break;
        case BITS_6:
            blackboxWrite((selector << 6) | (values[0] & 0x3F));
            blackboxWrite((uint8_t)values[1]);
            blackboxWrite((uint8_t)values[2]);
        break;
        case BITS_32:
            /*
             * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
             *
             * Selector2 field possibilities
             * 0 - 8 bits
             * 1 - 16 bits
             * 2 - 24 bits
             * 3 - 32 bits
             */
            selector2 = 0;

            //Encode in reverse order so the first field is in the low bits:
            for (x = NUM_FIELDS - 1; x >= 0; x--) {
                selector2 <<= 2;

                if (values[x] < 128 && values[x] >= -128) {
                    selector2 |= BYTES_1;
                } else if (values[x] < 32768 && values[x] >= -32768) {
                    selector2 |= BYTES_2;
                } else if (values[x] < 8388608 && values[x] >= -8388608) {
                    selector2 |= BYTES_3;
                } else {
                    selector2 |= BYTES_4;
                }
            }

            //Write the selectors
            blackboxWrite((selector << 6) | selector2);

            //And now the values according to the selectors we picked for them
            for (x = 0; x < NUM_FIELDS; x++, selector2 >>= 2) {
                switch (selector2 & 0x03) {
                    case BYTES_1:
                        blackboxWrite(values[x]);
                    break;
                    case BYTES_2:
                        blackboxWrite(values[x]);
                        blackboxWrite(values[x] >> 8);
                    break;
                    case BYTES_3:
                        blackboxWrite(values[x]);
                        blackboxWrite(values[x] >> 8);
                        blackboxWrite(values[x] >> 16);
                    break;
                    case BYTES_4:
                        blackboxWrite(values[x]);
                        blackboxWrite(values[x] >> 8);
                        blackboxWrite(values[x] >> 16);
                        blackboxWrite(values[x] >> 24);
                    break;
                }
            }
        break;
    }
}

/**
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
void blackboxWriteTag8_4S16(int32_t *values) {

    //Need to be enums rather than const ints if we want to switch on them (due to being C)
    enum {
        FIELD_ZERO  = 0,
        FIELD_4BIT  = 1,
        FIELD_8BIT  = 2,
        FIELD_16BIT = 3
    };

    uint8_t selector, buffer;
    int nibbleIndex;
    int x;

    selector = 0;
    //Encode in reverse order so the first field is in the low bits:
    for (x = 3; x >= 0; x--) {
        selector <<= 2;

        if (values[x] == 0) {
            selector |= FIELD_ZERO;
        } else if (values[x] < 8 && values[x] >= -8) {
            selector |= FIELD_4BIT;
        } else if (values[x] < 128 && values[x] >= -128) {
            selector |= FIELD_8BIT;
        } else {
            selector |= FIELD_16BIT;
        }
    }

    blackboxWrite(selector);

    nibbleIndex = 0;
    buffer = 0;
    for (x = 0; x < 4; x++, selector >>= 2) {
        switch (selector & 0x03) {
            case FIELD_ZERO:
                //No-op
            break;
            case FIELD_4BIT:
                if (nibbleIndex == 0) {
                    //We fill high-bits first
                    buffer = values[x] << 4;
                    nibbleIndex = 1;
                } else {
                    blackboxWrite(buffer | (values[x] & 0x0F));
                    nibbleIndex = 0;
                }
            break;
            case FIELD_8BIT:
                if (nibbleIndex == 0) {
                    blackboxWrite(values[x]);
                } else {
                    //Write the high bits of the value first (mask to avoid sign extension)
                    blackboxWrite(buffer | ((values[x] >> 4) & 0x0F));
                    //Now put the leftover low bits into the top of the next buffer entry
                    buffer = values[x] << 4;
                }
            break;
            case FIELD_16BIT:
                if (nibbleIndex == 0) {
                    //Write high byte first
                    blackboxWrite(values[x] >> 8);
                    blackboxWrite(values[x]);
                } else {
                    //First write the highest 4 bits
                    blackboxWrite(buffer | ((values[x] >> 12) & 0x0F));
                    // Then the middle 8
                    blackboxWrite(values[x] >> 4);
                    //Only the smallest 4 bits are still left to write
                    buffer = values[x] << 4;
                }
            break;
        }
    }
    //Anything left over to write?
    if (nibbleIndex == 1) {
        blackboxWrite(buffer);
    }
}

/**
 * Write `valueCount` fields from `values` to the Blackbox using signed variable byte encoding. A 1-byte header is
 * written first which specifies which fields are non-zero (so this encoding is compact when most fields are zero).
 *
 * valueCount must be 8 or less.
 */
void blackboxWriteTag8_8SVB(int32_t *values, int valueCount)
{
    uint8_t header;
    int i;

    if (valueCount > 0) {
        //If we're only writing one field then we can skip the header
        if (valueCount == 1) {
            blackboxWriteSignedVB(values[0]);
        } else {
            //First write a one-byte header that marks which fields are non-zero
            header = 0;

            // First field should be in low bits of header
            for (i = valueCount - 1; i >= 0; i--) {
                header <<= 1;

                if (values[i] != 0) {
                    header |= 0x01;
                }
            }

            blackboxWrite(header);

            for (i = 0; i < valueCount; i++) {
                if (values[i] != 0) {
                    blackboxWriteSignedVB(values[i]);
                }
            }
        }
    }
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
