#ifndef GPXWRITER_H_
#define GPXWRITER_H_

#include <stdint.h>
#include <stdio.h>

#include "parser.h"

typedef enum GpxWriterState {
    GPXWRITER_STATE_EMPTY = 0,
    GPXWRITER_STATE_WRITING_TRACK,
} GpxWriterState;

typedef struct gpxWriter_t {
    GpxWriterState state;
    FILE *file;
    char *filename;
    uint64_t startTime;
} gpxWriter_t;

void gpxWriterAddPoint(gpxWriter_t *gpx, flightLog_t *log, int64_t time, int32_t lat, int32_t lon, int16_t altitude);
gpxWriter_t* gpxWriterCreate(const char *filename);
void gpxWriterDestroy(gpxWriter_t* gpx);

#endif
