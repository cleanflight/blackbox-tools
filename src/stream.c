#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

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
    return (i >> 1) ^ -(int32_t) (i & 1);
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
    result->end = result->start + result->size;
    result->eof = false;

    return result;
}

void streamDestroy(mmapStream_t *stream)
{
    munmap_file(&stream->mapping);
}
