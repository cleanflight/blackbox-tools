#ifndef DATAPOINTS_H_
#define DATAPOINTS_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct datapoints_t {
    int fieldCount, frameCount;
    int frameCapacity;
    char **fieldNames;

    int64_t *frames;
    int64_t *frameTime;
    uint8_t *frameGap;
} datapoints_t;

datapoints_t *datapointsCreate(int fieldCount, char **fieldNames, int frameCapacity);
void datapointsDestroy(datapoints_t *points);

bool datapointsGetFrameAtIndex(datapoints_t *points, int frameIndex, int64_t *frameTime, int64_t *frame);

bool datapointsGetFieldAtIndex(datapoints_t *points, int frameIndex, int fieldIndex, int64_t *frameValue);
bool datapointsSetFieldAtIndex(datapoints_t *points, int frameIndex, int fieldIndex, int64_t frameValue);

bool datapointsGetGapStartsAtIndex(datapoints_t *points, int frameIndex);
bool datapointsGetTimeAtIndex(datapoints_t *points, int frameIndex, int64_t *frameTime);
int datapointsFindFrameAtTime(datapoints_t *points, int64_t time);

bool datapointsAddFrame(datapoints_t *points, int64_t frameTime, const int64_t *frame);
void datapointsAddGap(datapoints_t *points);

void datapointsSmoothField(datapoints_t *points, int fieldIndex, int windowSize);

#endif
