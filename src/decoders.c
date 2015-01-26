#include "decoders.h"
#include "tools.h"

void streamReadTag2_3S32(mmapStream_t *stream, int32_t *values)
{
    uint8_t leadByte;
    uint8_t byte1, byte2, byte3, byte4;
    int i;

    leadByte = streamReadByte(stream);

    // Check the selector in the top two bits to determine the field layout
    switch (leadByte >> 6) {
        case 0:
            // 2-bit fields
            values[0] = signExtend2Bit((leadByte >> 4) & 0x03);
            values[1] = signExtend2Bit((leadByte >> 2) & 0x03);
            values[2] = signExtend2Bit(leadByte & 0x03);
        break;
        case 1:
            // 4-bit fields
            values[0] = signExtend4Bit(leadByte & 0x0F);

            leadByte = streamReadByte(stream);

            values[1] = signExtend4Bit(leadByte >> 4);
            values[2] = signExtend4Bit(leadByte & 0x0F);
        break;
        case 2:
            // 6-bit fields
            values[0] = signExtend6Bit(leadByte & 0x3F);

            leadByte = streamReadByte(stream);
            values[1] = signExtend6Bit(leadByte & 0x3F);

            leadByte = streamReadByte(stream);
            values[2] = signExtend6Bit(leadByte & 0x3F);
        break;
        case 3:
            // Fields are 8, 16 or 24 bits, read selector to figure out which field is which size

            for (i = 0; i < 3; i++) {
                switch (leadByte & 0x03) {
                    case 0: // 8-bit
                        byte1 = streamReadByte(stream);

                        // Sign extend to 32 bits
                        values[i] = (int32_t) (int8_t) (byte1);
                    break;
                    case 1: // 16-bit
                        byte1 = streamReadByte(stream);
                        byte2 = streamReadByte(stream);

                        // Sign extend to 32 bits
                        values[i] = (int32_t) (int16_t) (byte1 | (byte2 << 8));
                    break;
                    case 2: // 24-bit
                        byte1 = streamReadByte(stream);
                        byte2 = streamReadByte(stream);
                        byte3 = streamReadByte(stream);

                        values[i] = signExtend24Bit(byte1 | (byte2 << 8) | (byte3 << 16));
                    break;
                    case 3: // 32-bit
                        byte1 = streamReadByte(stream);
                        byte2 = streamReadByte(stream);
                        byte3 = streamReadByte(stream);
                        byte4 = streamReadByte(stream);

                        values[i] = (int32_t) (byte1 | (byte2 << 8) | (byte3 << 16) | (byte4 << 24));
                    break;
                }

                leadByte >>= 2;
            }
        break;
    }
}

void streamReadTag8_4S16_v1(mmapStream_t *stream, int32_t *values)
{
    uint8_t selector, combinedChar;
    uint8_t char1, char2;
    int i;

    enum {
        FIELD_ZERO  = 0,
        FIELD_4BIT  = 1,
        FIELD_8BIT  = 2,
        FIELD_16BIT = 3
    };

    selector = streamReadByte(stream);

    //Read the 4 values from the stream
    for (i = 0; i < 4; i++) {
        switch (selector & 0x03) {
            case FIELD_ZERO:
                values[i] = 0;
            break;
            case FIELD_4BIT: // Two 4-bit fields
                combinedChar = (uint8_t) streamReadByte(stream);

                values[i] = signExtend4Bit(combinedChar & 0x0F);

                i++;
                selector >>= 2;

                values[i] = signExtend4Bit(combinedChar >> 4);
            break;
            case FIELD_8BIT: // 8-bit field
                //Sign extend...
                values[i] = (int32_t) (int8_t) streamReadByte(stream);
            break;
            case FIELD_16BIT: // 16-bit field
                char1 = streamReadByte(stream);
                char2 = streamReadByte(stream);

                //Sign extend...
                values[i] = (int16_t) (char1 | (char2 << 8));
            break;
        }

        selector >>= 2;
    }
}

void streamReadTag8_4S16_v2(mmapStream_t *stream, int32_t *values)
{
    uint8_t selector;
    uint8_t char1, char2;
    uint8_t buffer;
    int nibbleIndex;

    int i;

    enum {
        FIELD_ZERO  = 0,
        FIELD_4BIT  = 1,
        FIELD_8BIT  = 2,
        FIELD_16BIT = 3
    };

    selector = streamReadByte(stream);

    //Read the 4 values from the stream
    nibbleIndex = 0;
    for (i = 0; i < 4; i++) {
        switch (selector & 0x03) {
            case FIELD_ZERO:
                values[i] = 0;
            break;
            case FIELD_4BIT:
                if (nibbleIndex == 0) {
                    buffer = (uint8_t) streamReadByte(stream);
                    values[i] = signExtend4Bit(buffer >> 4);
                    nibbleIndex = 1;
                } else {
                    values[i] = signExtend4Bit(buffer & 0x0F);
                    nibbleIndex = 0;
                }
            break;
            case FIELD_8BIT:
                if (nibbleIndex == 0) {
                    //Sign extend...
                    values[i] = (int32_t) (int8_t) streamReadByte(stream);
                } else {
                    char1 = buffer << 4;
                    buffer = (uint8_t) streamReadByte(stream);

                    char1 |= buffer >> 4;
                    values[i] = (int32_t) (int8_t) char1;
                }
            break;
            case FIELD_16BIT:
                if (nibbleIndex == 0) {
                    char1 = (uint8_t) streamReadByte(stream);
                    char2 = (uint8_t) streamReadByte(stream);

                    //Sign extend...
                    values[i] = (int16_t) (uint16_t) ((char1 << 8) | char2);
                } else {
                    /*
                     * We're in the low 4 bits of the current buffer, then one byte, then the high 4 bits of the next
                     * buffer.
                     */
                    char1 = (uint8_t) streamReadByte(stream);
                    char2 = (uint8_t) streamReadByte(stream);

                    values[i] = (int16_t) (uint16_t) ((buffer << 12) | (char1 << 4) | (char2 >> 4));

                    buffer = char2;
                }
            break;
        }

        selector >>= 2;
    }
}

void streamReadTag8_8SVB(mmapStream_t *stream, int32_t *values, int valueCount)
{
    uint8_t header;

    if (valueCount == 1) {
        values[0] = streamReadSignedVB(stream);
    } else {
        header = (uint8_t) streamReadByte(stream);

        for (int i = 0; i < 8; i++, header >>= 1)
            values[i] = (header & 0x01) ? streamReadSignedVB(stream) : 0;
    }
}

float streamReadRawFloat(mmapStream_t *stream)
{
    union floatConvert_t {
        float f;
        uint8_t bytes[4];
    } floatConvert;

    for (int i = 0; i < 4; i++) {
        floatConvert.bytes[i] = streamReadByte(stream);
    }

    return floatConvert.f;
}
