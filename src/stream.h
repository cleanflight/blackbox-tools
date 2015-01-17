#ifndef STREAM_H_
#define STREAM_H_

#include "platform.h"

typedef struct mmapStream_t {
    fileMapping_t mapping;

    //The start of the entire data block
    const char *data;

    //The length of the entire data block
    size_t size;

    //The section of the file which is currently being examined by the stream:
    const char *start, *end, *pos;

    //Set to true if we attempt to read from the log when it is already exhausted
    bool eof;
} mmapStream_t;

mmapStream_t* streamCreate(int fd);
void streamDestroy(mmapStream_t *stream);

int streamPeekChar(mmapStream_t *stream);
int streamReadChar(mmapStream_t *stream);
int streamReadByte(mmapStream_t *stream);
void streamUnreadChar(mmapStream_t *stream, int c);

void streamRead(mmapStream_t *stream, void *buf, int len);

uint32_t streamReadUnsignedVB(mmapStream_t *stream);
int32_t streamReadSignedVB(mmapStream_t *stream);

#endif
