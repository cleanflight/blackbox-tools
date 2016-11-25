#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "datapoints.h"
#include "parser.h"

datapoints_t *datapointsCreate(int fieldCount, char **fieldNames, int frameCapacity)
{
    datapoints_t *result = (datapoints_t*) malloc(sizeof(datapoints_t));

    result->fieldCount = fieldCount;
    result->fieldNames = fieldNames;

    result->frameCount = 0;
    result->frameCapacity = frameCapacity;

    result->frames = malloc(sizeof(*result->frames) * fieldCount * frameCapacity);
    result->frameTime = calloc(1, sizeof(*result->frameTime) * frameCapacity);
    result->frameGap = calloc(1, sizeof(*result->frameGap) * frameCapacity);

    return result;
}

void datapointsDestroy(datapoints_t *points)
{
    free(points->frames);
    free(points->frameTime);
    free(points->frameGap);
    free(points);
}

/**
 * Smooth the values for the field with the given index by replacing each value with an
 * average over the a window of width (windowRadius*2+1) centered at the point.
 */
void datapointsSmoothField(datapoints_t *points, int fieldIndex, int windowRadius)
{
    int windowSize = windowRadius * 2 + 1;
    // How many of the frames in the history actually have a valid value in them (so we can average only those)
    int valuesInHistory = 0;

    int64_t accumulator;

    if (fieldIndex < 0 || fieldIndex >= points->fieldCount) {
        fprintf(stderr, "Attempt to smooth field that doesn't exist %d\n", fieldIndex);
        exit(-1);
    }

    // Field values so that we know what they were originally before we overwrote them
    int64_t *history = (int64_t*) malloc(sizeof(*history) * windowSize);
    int historyHead = 0; //Points to the next location to insert into
    int historyTail = 0; //Points to the last value in the window

    int windowCenterIndex;
    int partitionLeft, partitionRight;
    int windowLeftIndex, windowRightIndex;

    for (windowCenterIndex = 0; windowCenterIndex < points->frameCount; ) {
        partitionLeft = windowCenterIndex;
        //We'll refine this guess later if we find discontinuities:
        partitionRight = points->frameCount;

        /*
         * We start the right edge of the window at the beginning of the partition so that the main loop can begin by
         * accumulating windowRadius points in the history. Those are the values we'll need before we can work out
         * the moving average of the first value of the partition.
         */
        windowCenterIndex = windowCenterIndex - windowRadius;

        windowLeftIndex = windowCenterIndex - windowRadius;
        windowRightIndex = windowCenterIndex + windowRadius;

        accumulator = 0;
        valuesInHistory = 0;
        historyHead = 0;
        historyTail = 0;

        //The main loop, where we march our [leftIndex...rightIndex] history window along until we exhaust this partition
        for (; windowCenterIndex < partitionRight; windowCenterIndex++, windowLeftIndex++, windowRightIndex++) {

            // Oldest value falls out of the window
            if (windowLeftIndex - 1 >= partitionLeft) {
                accumulator -= history[historyTail];
                historyTail = (historyTail + 1) % windowSize;

                valuesInHistory--;
            }

            //New value is added to the window
            if (windowRightIndex < partitionRight) {
                int64_t fieldValue = points->frames[points->fieldCount * windowRightIndex + fieldIndex];

                accumulator += fieldValue;

                history[historyHead] = fieldValue;
                historyHead = (historyHead + 1) % windowSize;

                valuesInHistory++;

                //If there is a discontinuity after this point, adjust the right edge of the partition so we stop looking further
                if (points->frameGap[windowRightIndex])
                    partitionRight = windowRightIndex + 1;
            }

            // Store the average of the history window into the frame in the center of the window
            if (windowCenterIndex >= partitionLeft) {
                points->frames[points->fieldCount * windowCenterIndex + fieldIndex] = accumulator / valuesInHistory;
            }
        }
    }

    free(history);
}

/**
 * Find the index of the latest frame whose time is equal to or later than 'time'.
 *
 * Returns -1 if the time is before any frame in the datapoints.
 */
int datapointsFindFrameAtTime(datapoints_t *points, int64_t time)
{
    int i, lastGoodFrame = -1;

    //TODO make me a binary search
    for (i = 0; i < points->frameCount; i++) {
        if (time < points->frameTime[i]) {
            return lastGoodFrame;
        }
        lastGoodFrame = i;
    }

    return lastGoodFrame;
}

bool datapointsGetFrameAtIndex(datapoints_t *points, int frameIndex, int64_t *frameTime, int64_t *frame)
{
    if (frameIndex < 0 || frameIndex >= points->frameCount)
        return false;

    memcpy(frame, points->frames + frameIndex * points->fieldCount, points->fieldCount * sizeof(*points->frames));
    *frameTime = points->frameTime[frameIndex];

    return true;
}

bool datapointsGetFieldAtIndex(datapoints_t *points, int frameIndex, int fieldIndex, int64_t *frameValue)
{
    if (frameIndex < 0 || frameIndex >= points->frameCount)
        return false;

    *frameValue = points->frames[frameIndex * points->fieldCount + fieldIndex];

    return true;
}

bool datapointsSetFieldAtIndex(datapoints_t *points, int frameIndex, int fieldIndex, int64_t frameValue)
{
    if (frameIndex < 0 || frameIndex >= points->frameCount)
        return false;

    points->frames[frameIndex * points->fieldCount + fieldIndex] = frameValue;

    return true;
}

bool datapointsGetTimeAtIndex(datapoints_t *points, int frameIndex, int64_t *frameTime)
{
    if (frameIndex < 0 || frameIndex >= points->frameCount)
        return false;

    *frameTime = points->frameTime[frameIndex];

    return true;
}

bool datapointsGetGapStartsAtIndex(datapoints_t *points, int frameIndex)
{
    return frameIndex >= 0 && frameIndex < points->frameCount && points->frameGap[frameIndex];
}

/**
 * Set the data for the frame with the given index. The second field of the frame is expected to be a timestamp
 * (if you want to be able to find frames at given times).
 */
bool datapointsAddFrame(datapoints_t *points, int64_t frameTime, const int64_t *frame)
{
    if (points->frameCount >= points->frameCapacity)
        return false;

    points->frameTime[points->frameCount] = frameTime;
    memcpy(points->frames + points->frameCount * points->fieldCount, frame, points->fieldCount * sizeof(*points->frames));

    points->frameCount++;

    return true;
}

/**
 * Mark that a gap in the log begins after the last frame added.
 */
void datapointsAddGap(datapoints_t *points)
{
    if (points->frameCount > 0)
        points->frameGap[points->frameCount - 1] = 1;
}
