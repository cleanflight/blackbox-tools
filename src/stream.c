#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <assert.h>

#include "platform.h"
#include "tools.h"

#include "stream.h"

uint32_t streamReadUnsignedVB(mmapStream_t *stream)
{
    int i, c, shift = 0;
    uint32_t result = 0;

    // 5 bytes is enough to encode 32-bit unsigned quantities
    for (i = 0; i < 5; i++) {
        c = streamReadByte(stream);

        if (c == EOF) {
            return 0;
        }

        result = result | ((c & ~0x80) << shift);

        //Final byte?
        if (c < 128) {
            return result;
        }

        shift += 7;
    }

    // This VB-encoded int is too long!
    return 0;
}

int32_t streamReadSignedVB(mmapStream_t *stream)
{
    uint32_t i = streamReadUnsignedVB(stream);

    // Apply ZigZag decoding to recover the signed value
    return zigzagDecode(i);
}

int streamPeekChar(mmapStream_t *stream)
{
    if (stream->pos < stream->end) {
        return *stream->pos;
    }

    stream->eof = true;

    return EOF;
}

/**
 * Read an unsigned byte from the stream, or EOF if the end of stream was reached.
 */
int streamReadByte(mmapStream_t *stream)
{
    if (stream->pos < stream->end) {
        int result = (uint8_t) *stream->pos;
        stream->pos++;
        return result;
    }

    stream->eof = true;

    return EOF;
}

/**
 * Read a char from the stream, or EOF if the end of stream was reached.
 */
int streamReadChar(mmapStream_t *stream)
{
    if (stream->pos < stream->end) {
        int result = *stream->pos;
        stream->pos++;
        return result;
    }

    stream->eof = true;

    return EOF;
}

void streamUnreadChar(mmapStream_t *stream, int c)
{
    (void) c;

    stream->pos--;
}

void streamRead(mmapStream_t *stream, void *buf, int len)
{
    char *buffer = (char*) buf;

    if (len > stream->end - stream->pos) {
        len = stream->end - stream->pos;
        stream->eof = true;
    }

    for (int i = 0; i < len; i++, stream->pos++, buffer++) {
        *buffer = *stream->pos;
    }
}

/**
 * Read `numBits` (at most 32) at the current bit index and advance the bit pointer. The first bit in the stream becomes
 * the highest bit set in the result, and the last bit in the stream will be the least significant bit in the result.
 *
 * It is an error to later attempt to read a *byte* from the stream if the bit pointer is not byte-aligned (call streamByteAlign).
 *
 * If EOF is encountered before all the requested bits were read, the `pos` is set to the end of the stream, EOF is
 * returned, the EOF flag is set, and the bit pointer is properly aligned.
 */
uint32_t streamReadBits(mmapStream_t *stream, int numBits)
{
    // Round up the bit count to get the byte count
    int numBytes = (numBits + CHAR_BIT - 1) / CHAR_BIT;

    assert(numBits <= 32);

    if (stream->pos + numBytes <= stream->end) {
        uint32_t result = 0;

        while (numBits > 0) {
            result |= ((((uint8_t)*stream->pos) >> stream->bitPos) & 0x01) << (numBits - 1);

            if (stream->bitPos == 0) {
                stream->pos++;
                stream->bitPos = CHAR_BIT - 1;
            } else {
                stream->bitPos--;
            }
            numBits--;
        }

        return result;
    } else {
        stream->pos = stream->end;
        stream->eof = true;
        stream->bitPos = CHAR_BIT - 1;
        return EOF;
    }
}

/**
 * Read the bit at the current bit index and advance the bit pointer. Returns 1 if the bit was set and 0 if the bit
 * was not set.
 *
 * It is an error to later attempt to read a *byte* from the stream if the bit pointer is not byte-aligned (call streamByteAlign).
 *
 * If the file was already at EOF, EOF is returned and the EOF flag is set, and the bit pointer is byte-aligned.
 */
int streamReadBit(mmapStream_t *stream)
{
    return streamReadBits(stream, 1);
}

/**
 * If the bit pointer is partway through the current byte, it is advanced to point to the beginning of the next byte.
 *
 * EOF is never set by this routine as the routine never needs to attempt to read beyond the end of the stream.
 */
void streamByteAlign(mmapStream_t *stream)
{
    if (stream->bitPos != CHAR_BIT - 1) {
        stream->bitPos = CHAR_BIT - 1;
        stream->pos++;
    }
}

mmapStream_t* streamCreate(int fd)
{
    mmapStream_t *result = malloc(sizeof(*result));

    if (!mmap_file(&result->mapping, fd)) {
        free(result);
        return 0;
    }

    result->data = result->mapping.data;
    result->size = result->mapping.size;

    result->start = result->data;
    result->pos = result->start;
    result->bitPos = CHAR_BIT - 1;
    result->end = result->start + result->size;
    result->eof = false;

    return result;
}

void streamDestroy(mmapStream_t *stream)
{
    munmap_file(&stream->mapping);
    free(stream);
}
