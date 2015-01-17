#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//For msvcrt to define M_PI:
#define _USE_MATH_DEFINES
#include <math.h>

#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "platform.h"
#include "parser.h"
#include "tools.h"
#include "stream.h"
#include "decoders.h"

#define LOG_START_MARKER "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"

typedef enum ParserState
{
    PARSER_STATE_HEADER = 0,
    PARSER_STATE_DATA
} ParserState;

typedef struct flightLogFrameDefs_t {
    int predictor[FLIGHT_LOG_MAX_FIELDS];
    int encoding[FLIGHT_LOG_MAX_FIELDS];
} flightLogFrameDefs_t;

typedef struct flightLogPrivate_t
{
    // We own this memory to store the field names for these frame types (as a single string)
    char *mainFieldNamesLine, *gpsHomeFieldNamesLine, *gpsFieldNamesLine;

    //Information about fields which we need to decode them properly
    flightLogFrameDefs_t frameDefs[256];

    int dataVersion;

    // Indexes of named fields that we need to use to apply predictions against
    int motor0Index, home0Index, home1Index;

    // Blackbox state:
    int32_t blackboxHistoryRing[3][FLIGHT_LOG_MAX_FIELDS];

    /* Points into blackboxHistoryRing to give us a circular buffer.
     *
     * 0 - space to decode new frames into, 1 - previous frame, 2 - previous previous frame
     *
     * Previous frame pointers are NULL when no valid history exists of that age.
     */
    int32_t* mainHistory[3];
    bool mainStreamIsValid;

    int32_t gpsHomeHistory[2][FLIGHT_LOG_MAX_FIELDS]; // 0 - space to decode new frames into, 1 - previous frame
    bool gpsHomeIsValid;

    //Because these events don't depend on previous events, we don't keep copies of the old state, just the current one:
    flightLogEvent_t lastEvent;
    int32_t lastGPS[FLIGHT_LOG_MAX_FIELDS];

    // Event handlers:
    FlightLogMetadataReady onMetadataReady;
    FlightLogFrameReady onFrameReady;
    FlightLogEventReady onEvent;

    mmapStream_t *stream;
} flightLogPrivate_t;

typedef void (*FlightLogFrameParse)(flightLog_t *log, mmapStream_t *stream, bool raw);
typedef void (*FlightLogFrameComplete)(flightLog_t *log, mmapStream_t *stream, char frameType, const char *frameStart, const char *frameEnd, bool raw);

typedef struct flightLogFrameType_t {
    uint8_t marker;
    FlightLogFrameParse parse;
    FlightLogFrameComplete complete;
} flightLogFrameType_t;

static void parseIntraframe(flightLog_t *log, mmapStream_t *stream, bool raw);
static void parseInterframe(flightLog_t *log, mmapStream_t *stream, bool raw);
static void parseGPSFrame(flightLog_t *log, mmapStream_t *stream, bool raw);
static void parseGPSHomeFrame(flightLog_t *log, mmapStream_t *stream, bool raw);
static void parseEventFrame(flightLog_t *log, mmapStream_t *stream, bool raw);

static void completeIntraframe(flightLog_t *log, mmapStream_t *stream, char frameType, const char *frameStart, const char *frameEnd, bool raw);
static void completeInterframe(flightLog_t *log, mmapStream_t *stream, char frameType, const char *frameStart, const char *frameEnd, bool raw);
static void completeEventFrame(flightLog_t *log, mmapStream_t *stream, char frameType, const char *frameStart, const char *frameEnd, bool raw);
static void completeGPSFrame(flightLog_t *log, mmapStream_t *stream, char frameType, const char *frameStart, const char *frameEnd, bool raw);
static void completeGPSHomeFrame(flightLog_t *log, mmapStream_t *stream, char frameType, const char *frameStart, const char *frameEnd, bool raw);

static const flightLogFrameType_t frameTypes[] = {
    {.marker = 'I', .parse = parseIntraframe,   .complete = completeIntraframe},
    {.marker = 'P', .parse = parseInterframe,   .complete = completeInterframe},
    {.marker = 'G', .parse = parseGPSFrame,     .complete = completeGPSFrame},
    {.marker = 'H', .parse = parseGPSHomeFrame, .complete = completeGPSHomeFrame},
    {.marker = 'E', .parse = parseEventFrame,   .complete = completeEventFrame}
};

/**
 * Parse a comma-separated list of field names into the fieldNames array. `fieldNamesCombined` is set to point to
 * the memory allocated to hold the field names (call `free` later). `fieldCount` is set to the number of field names
 * parsed.
 */
static void parseFieldNames(const char *line, char **fieldNamesCombined, char **fieldNames, int *fieldCount)
{
    char *start, *end;
    bool done = false;

    //Make a copy of the line so we can manage its lifetime (and write to it to null terminate the fields)
    *fieldNamesCombined = strdup(line);
    *fieldCount = 0;

    start = *fieldNamesCombined;

    while (!done && *start) {
        end = start;

        do {
            end++;
        } while (*end != ',' && *end != 0);

        fieldNames[(*fieldCount)++] = start;

        if (*end == 0)
            done = true;

        *end = 0;

        start = end + 1;
    }
}

static void parseCommaSeparatedIntegers(char *line, int *target, int maxCount)
{
    char *start, *end;
    bool done = false;

    start = line;

    while (!done && *start && maxCount > 0) {
        end = start + 1;

        while (*end != ',' && *end != 0)
            end++;

        if (*end == 0)
            done = true;

        *end = 0;

        *target = atoi(start);
        target++;
        maxCount--;

        start = end + 1;
    }
}

static void parseHeaderLine(flightLog_t *log, mmapStream_t *stream)
{
    char *fieldName, *fieldValue;
    const char *lineStart, *lineEnd, *separatorPos;
    int i, c;
    char valueBuffer[1024];
    union {
        float f;
        uint32_t u;
    } floatConvert;

    if (streamPeekChar(stream) != ' ')
        return;

    //Skip the space
    stream->pos++;

    lineStart = stream->pos;
    separatorPos = 0;

    for (i = 0; i < 1024; i++) {
        c = streamReadChar(stream);

        if (c == ':' && !separatorPos)
            separatorPos = stream->pos - 1;

        if (c == '\n')
            break;

        if (c == EOF || c == '\0')
            // Line ended before we saw a newline or it has binary stuff in there that shouldn't be there
            return;
    }

    if (!separatorPos)
        return;

    lineEnd = stream->pos;

    //Make a duplicate copy of the line so we can null-terminate the two parts
    memcpy(valueBuffer, lineStart, lineEnd - lineStart);

    fieldName = valueBuffer;
    valueBuffer[separatorPos - lineStart] = '\0';

    fieldValue = valueBuffer + (separatorPos - lineStart) + 1;
    valueBuffer[lineEnd - lineStart - 1] = '\0';

    if (strcmp(fieldName, "Field I name") == 0) {
        parseFieldNames(fieldValue, &log->private->mainFieldNamesLine, log->mainFieldNames, &log->mainFieldCount);

        for (i = 0; i < log->mainFieldCount; i++) {
            if (strcmp(log->mainFieldNames[i], "motor[0]") == 0) {
                log->private->motor0Index = i;
                break;
            }
        }
    } else if (strcmp(fieldName, "Field G name") == 0) {
        parseFieldNames(fieldValue, &log->private->gpsFieldNamesLine, log->gpsFieldNames, &log->gpsFieldCount);
    } else if (strcmp(fieldName, "Field H name") == 0) {
        parseFieldNames(fieldValue, &log->private->gpsHomeFieldNamesLine, log->gpsHomeFieldNames, &log->gpsHomeFieldCount);

        for (i = 0; i < log->gpsHomeFieldCount; i++) {
            if (strcmp(log->gpsHomeFieldNames[i], "GPS_home[0]") == 0) {
                log->private->home0Index = i;
            } else if (strcmp(log->gpsHomeFieldNames[i], "GPS_home[1]") == 0) {
                log->private->home1Index = i;
            }
        }
    } else if (strlen(fieldName) == strlen("Field X predictor") && startsWith(fieldName, "Field ") && endsWith(fieldName, " predictor")) {
        parseCommaSeparatedIntegers(fieldValue, log->private->frameDefs[(uint8_t)fieldName[strlen("Field ")]].predictor, FLIGHT_LOG_MAX_FIELDS);
    } else if (strlen(fieldName) == strlen("Field X encoding") && startsWith(fieldName, "Field ") && endsWith(fieldName, " encoding")) {
        parseCommaSeparatedIntegers(fieldValue, log->private->frameDefs[(uint8_t)fieldName[strlen("Field ")]].encoding, FLIGHT_LOG_MAX_FIELDS);
    } else if (strcmp(fieldName, "Field I signed") == 0) {
        parseCommaSeparatedIntegers(fieldValue, log->mainFieldSigned, FLIGHT_LOG_MAX_FIELDS);
    } else if (strcmp(fieldName, "I interval") == 0) {
        log->frameIntervalI = atoi(fieldValue);
        if (log->frameIntervalI < 1)
            log->frameIntervalI = 1;
    } else if (strcmp(fieldName, "P interval") == 0) {
        char *slashPos = strchr(fieldValue, '/');

        if (slashPos) {
            log->frameIntervalPNum = atoi(fieldValue);
            log->frameIntervalPDenom = atoi(slashPos + 1);
        }
    } else if (strcmp(fieldName, "Data version") == 0) {
        log->private->dataVersion = atoi(fieldValue);
    } else if (strcmp(fieldName, "Firmware type") == 0) {
        if (strcmp(fieldValue, "Cleanflight") == 0)
            log->firmwareType = FIRMWARE_TYPE_CLEANFLIGHT;
        else
            log->firmwareType = FIRMWARE_TYPE_BASEFLIGHT;
    } else if (strcmp(fieldName, "minthrottle") == 0) {
        log->minthrottle = atoi(fieldValue);
    } else if (strcmp(fieldName, "maxthrottle") == 0) {
        log->maxthrottle = atoi(fieldValue);
    } else if (strcmp(fieldName, "rcRate") == 0) {
        log->rcRate = atoi(fieldValue);
    } else if (strcmp(fieldName, "vbatscale") == 0) {
        log->vbatscale = atoi(fieldValue);
    } else if (strcmp(fieldName, "vbatref") == 0) {
        log->vbatref = atoi(fieldValue);
    } else if (strcmp(fieldName, "vbatcellvoltage") == 0) {
        int vbatcellvoltage[3];
        parseCommaSeparatedIntegers(fieldValue, vbatcellvoltage, 3);

        log->vbatmincellvoltage = vbatcellvoltage[0];
        log->vbatwarningcellvoltage = vbatcellvoltage[1];
        log->vbatmaxcellvoltage = vbatcellvoltage[2];
    } else if (strcmp(fieldName, "gyro.scale") == 0) {
        floatConvert.u = strtoul(fieldValue, 0, 16);

        log->gyroScale = floatConvert.f;

        /* Baseflight uses a gyroScale that'll give radians per microsecond as output, whereas Cleanflight produces degrees
         * per second and leaves the conversion to radians per us to the IMU. Let's just convert Cleanflight's scale to
         * match Baseflight so we can use Baseflight's IMU for both: */

        if (log->firmwareType == FIRMWARE_TYPE_CLEANFLIGHT) {
            log->gyroScale = (float) (log->gyroScale * (M_PI / 180.0) * 0.000001);
        }
    } else if (strcmp(fieldName, "acc_1G") == 0) {
        log->acc_1G = atoi(fieldValue);
    }
}

/**
 * Should a frame with the given index exist in this log (based on the user's selection of sampling rates)?
 */
static int shouldHaveFrame(flightLog_t *log, int32_t frameIndex)
{
    return (frameIndex % log->frameIntervalI + log->frameIntervalPNum - 1) % log->frameIntervalPDenom < log->frameIntervalPNum;
}

/**
 * Take the raw value for a a field, apply the prediction that is configured for it, and return it.
 */
static int32_t applyPrediction(flightLog_t *log, int fieldIndex, int predictor, uint32_t value, int32_t *current, int32_t *previous, int32_t *previous2)
{
    flightLogPrivate_t *private = log->private;

    // First see if we have a prediction that doesn't require a previous frame as reference:
    switch (predictor) {
        case FLIGHT_LOG_FIELD_PREDICTOR_0:
            // No correction to apply
        break;
        case FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE:
            value += log->minthrottle;
        break;
        case FLIGHT_LOG_FIELD_PREDICTOR_1500:
            value += 1500;
        break;
        case FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0:
            if (private->motor0Index < 0) {
                fprintf(stderr, "Attempted to base prediction on motor0 without that field being defined\n");
                exit(-1);
            }
            value += (uint32_t) current[private->motor0Index];
        break;
        case FLIGHT_LOG_FIELD_PREDICTOR_VBATREF:
            value += log->vbatref;
        break;
        case FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS:
            if (!previous)
                break;

            value += (uint32_t) previous[fieldIndex];
        break;
        case FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE:
            if (!previous)
                break;

            value += 2 * (uint32_t) previous[fieldIndex] - (uint32_t) previous2[fieldIndex];
        break;
        case FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2:
            if (!previous)
                break;

            if (log->mainFieldSigned[fieldIndex])
                value += (uint32_t) ((int32_t) ((uint32_t) previous[fieldIndex] + (uint32_t) previous2[fieldIndex]) / 2);
            else
                value += ((uint32_t) previous[fieldIndex] + (uint32_t) previous2[fieldIndex]) / 2;
        break;
        case FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD:
            if (private->home0Index < 0) {
                fprintf(stderr, "Attempted to base prediction on GPS home position without GPS home frame definition\n");
                exit(-1);
            }

            value += private->gpsHomeHistory[1][private->home0Index];
        break;
        case FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD_1:
            if (private->home1Index < 1) {
                fprintf(stderr, "Attempted to base prediction on GPS home position without GPS home frame definition\n");
                exit(-1);
            }

            value += private->gpsHomeHistory[1][private->home1Index];
        break;
        case FLIGHT_LOG_FIELD_PREDICTOR_LAST_MAIN_FRAME_TIME:
            if (private->mainHistory[1])
                value += private->mainHistory[1][FLIGHT_LOG_FIELD_INDEX_TIME];
        break;
        default:
            fprintf(stderr, "Unsupported field predictor %d\n", predictor);
            exit(-1);
    }

    return (int32_t) value;
}

/**
 * Attempt to parse the frame of the given `frameType` into the supplied `frame` buffer using the encoding/predictor
 * definitions from log->private->frameDefs[`frameType`].
 *
 * raw - Set to true to disable predictions (and so store raw values)
 * skippedFrames - Set to the number of field iterations that were skipped over by rate settings since the last frame.
 */
static void parseFrame(flightLog_t *log, mmapStream_t *stream, uint8_t frameType, int32_t *frame, int32_t *previous, int32_t *previous2, int fieldCount, int skippedFrames, bool raw)
{
    flightLogPrivate_t *private = log->private;
    int *predictor = private->frameDefs[frameType].predictor;
    int *encoding = private->frameDefs[frameType].encoding;
    int i, j, groupCount;

    i = 0;
    while (i < fieldCount) {
        uint32_t value;
        uint32_t values[8];

        if (log->private->frameDefs[frameType].predictor[i] == FLIGHT_LOG_FIELD_PREDICTOR_INC) {
            frame[i] = skippedFrames + 1;

            if (previous)
                frame[i] += previous[i];

            i++;
        } else {
            switch (private->frameDefs[frameType].encoding[i]) {
                case FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB:
                    value = (uint32_t) streamReadSignedVB(stream);
                break;
                case FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB:
                    value = streamReadUnsignedVB(stream);
                break;
                case FLIGHT_LOG_FIELD_ENCODING_NEG_14BIT:
                    value = (uint32_t) -signExtend14Bit(streamReadUnsignedVB(stream));
                break;
                case FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16:
                    if (log->private->dataVersion < 2)
                        streamReadTag8_4S16_v1(stream, (int32_t*)values);
                    else
                        streamReadTag8_4S16_v2(stream, (int32_t*)values);

                    //Apply the predictors for the fields:
                    for (j = 0; j < 4; j++, i++)
                        frame[i] = applyPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : predictor[i], values[j], frame, previous, previous2);

                    continue;
                break;
                case FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32:
                    streamReadTag2_3S32(stream, (int32_t*)values);

                    //Apply the predictors for the fields:
                    for (j = 0; j < 3; j++, i++)
                        frame[i] = applyPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : predictor[i], values[j], frame, previous, previous2);

                    continue;
                break;
                case FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB:
                    //How many fields are in this encoded group? Check the subsequent field encodings:
                    for (j = i + 1; j < i + 8 && j < log->mainFieldCount; j++)
                        if (encoding[j] != FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB)
                            break;

                    groupCount = j - i;

                    streamReadTag8_8SVB(stream, (int32_t*) values, groupCount);

                    for (j = 0; j < groupCount; j++, i++)
                        frame[i] = applyPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : predictor[i], values[j], frame, previous, previous2);

                    continue;
                break;
                case FLIGHT_LOG_FIELD_ENCODING_NULL:
                    //Nothing to read
                    value = 0;
                break;
                default:
                    fprintf(stderr, "Unsupported field encoding %d\n", encoding[i]);
                    exit(-1);
            }

            frame[i] = applyPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : predictor[i], value, frame, previous, previous2);
            i++;
        }
    }
}

/**
 * Attempt to parse the Intraframe at the current log position into the history buffer at mainHistory[0].
 */
static void parseIntraframe(flightLog_t *log, mmapStream_t *stream, bool raw)
{
    flightLogPrivate_t *private = log->private;
    int skippedFrames = 0;
    int *current, *previous;

    current = private->mainHistory[0];
    previous = private->mainHistory[1];

    if (previous) {
        for (uint32_t frameIndex = previous[FLIGHT_LOG_FIELD_INDEX_ITERATION] + 1; !shouldHaveFrame(log, frameIndex); frameIndex++) {
            skippedFrames++;
        }
        log->stats.intentionallyAbsentIterations += skippedFrames;
    }

    parseFrame(log, stream, 'I', current, previous, NULL, log->mainFieldCount, skippedFrames, raw);
}

/**
 * Attempt to parse the interframe at the current log position into the history buffer at mainHistory[0].
 */
static void parseInterframe(flightLog_t *log, mmapStream_t *stream, bool raw)
{
    int32_t *current = log->private->mainHistory[0];
    int32_t *previous = log->private->mainHistory[1];
    int32_t *previous2 = log->private->mainHistory[2];

    uint32_t frameIndex;
    uint32_t skippedFrames = 0;

    if (previous) {
        //Work out how many frames we skipped to get to this one, based on the log sampling rate
        for (frameIndex = previous[FLIGHT_LOG_FIELD_INDEX_ITERATION] + 1; !shouldHaveFrame(log, frameIndex); frameIndex++)
            skippedFrames++;
    }

    log->stats.intentionallyAbsentIterations += skippedFrames;

    parseFrame(log, stream, 'P', current, previous, previous2, log->mainFieldCount, skippedFrames, raw);
}

static void parseGPSFrame(flightLog_t *log, mmapStream_t *stream, bool raw)
{
    parseFrame(log, stream, 'G', log->private->lastGPS, NULL, NULL, log->gpsFieldCount, 0, raw);
}

static void parseGPSHomeFrame(flightLog_t *log, mmapStream_t *stream, bool raw)
{
    parseFrame(log, stream, 'H', log->private->gpsHomeHistory[0], NULL, NULL, log->gpsHomeFieldCount, 0, raw);
}

/**
 * Attempt to parse an event frame at the current location into the log->private->lastEvent struct.
 * Return false if the event couldn't be parsed (e.g. unknown event ID), or true if it might have been
 * parsed successfully.
 */
static void parseEventFrame(flightLog_t *log, mmapStream_t *stream, bool raw)
{
    static const char END_OF_LOG_MESSAGE[] = "End of log\0";
    enum { END_OF_LOG_MESSAGE_LEN = 11};

    char endMessage[END_OF_LOG_MESSAGE_LEN];
    (void) raw;

    uint8_t eventType = streamReadByte(stream);

    flightLogEventData_t *data = &log->private->lastEvent.data;
    log->private->lastEvent.event = eventType;

    switch (eventType) {
        case FLIGHT_LOG_EVENT_SYNC_BEEP:
            data->syncBeep.time = streamReadUnsignedVB(stream);
        break;
        case FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_START:
            data->autotuneCycleStart.phase = streamReadByte(stream);
            data->autotuneCycleStart.cycle = streamReadByte(stream);
            data->autotuneCycleStart.p = streamReadByte(stream);
            data->autotuneCycleStart.i = streamReadByte(stream);
            data->autotuneCycleStart.d = streamReadByte(stream);
        break;
        case FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_RESULT:
            data->autotuneCycleResult.overshot = streamReadByte(stream);
            data->autotuneCycleResult.p = streamReadByte(stream);
            data->autotuneCycleResult.i = streamReadByte(stream);
            data->autotuneCycleResult.d = streamReadByte(stream);
        break;
        case FLIGHT_LOG_EVENT_LOG_END:
            streamRead(stream, endMessage, END_OF_LOG_MESSAGE_LEN);

            if (strncmp(endMessage, END_OF_LOG_MESSAGE, END_OF_LOG_MESSAGE_LEN) == 0) {
                //Adjust the end of stream so we stop reading, this log is done
                stream->end = stream->pos;
            } else {
                /*
                 * This isn't the real end of log message, it's probably just some bytes that happened to look like
                 * an event header.
                 */
                log->private->lastEvent.event = -1;
            }
        break;
        default:
            log->private->lastEvent.event = -1;
    }
}

static void updateFieldStatistics(flightLog_t *log, int32_t *fields)
{
    int i;

    if (log->stats.frame['I'].validCount + log->stats.frame['P'].validCount <= 1) {
        //If this is the first frame, there are no minimums or maximums in the stats to compare with
        for (i = 0; i < log->mainFieldCount; i++) {
            if (log->mainFieldSigned[i]) {
                log->stats.field[i].max = fields[i];
                log->stats.field[i].min = fields[i];
            } else {
                log->stats.field[i].max = (uint32_t) fields[i];
                log->stats.field[i].min = (uint32_t) fields[i];
            }
        }
    } else {
        for (i = 0; i < log->mainFieldCount; i++) {
            if (log->mainFieldSigned[i]) {
                log->stats.field[i].max = fields[i] > log->stats.field[i].max ? fields[i] : log->stats.field[i].max;
                log->stats.field[i].min = fields[i] < log->stats.field[i].min ? fields[i] : log->stats.field[i].min;
            } else {
                log->stats.field[i].max = (uint32_t) fields[i] > log->stats.field[i].max ? (uint32_t) fields[i] : log->stats.field[i].max;
                log->stats.field[i].min = (uint32_t) fields[i] < log->stats.field[i].min ? (uint32_t) fields[i] : log->stats.field[i].min;
            }
        }
    }
}

unsigned int flightLogVbatToMillivolts(flightLog_t *log, uint16_t vbat)
{
    // ADC is 12 bit (i.e. max 0xFFF), voltage reference is 3.3V, vbatscale is premultiplied by 100
    return (vbat * 330 * log->vbatscale) / 0xFFF;
}

int flightLogEstimateNumCells(flightLog_t *log)
{
    int i;
    int refVoltage;

    refVoltage = flightLogVbatToMillivolts(log, log->vbatref) / 100;

    for (i = 1; i < 8; i++) {
        if (refVoltage < i * log->vbatmaxcellvoltage)
            break;
    }

    return i;
}

flightLog_t * flightLogCreate(int fd)
{
    const char *logSearchStart;
    int logIndex;

    flightLog_t *log;
    flightLogPrivate_t *private;

    log = (flightLog_t *) malloc(sizeof(*log));
    private = (flightLogPrivate_t *) malloc(sizeof(*private));

    memset(log, 0, sizeof(*log));
    memset(private, 0, sizeof(*private));

    private->stream = streamCreate(fd);

    if (!private->stream) {
        free(log);
        free(private);

        return 0;
    }

    if (private->stream->size == 0) {
        fprintf(stderr, "Error: This log is zero-bytes long!\n");

        streamDestroy(private->stream);

        free(log);
        free(private);

        return 0;
    }

    //First check how many logs are in this one file (each time the FC is rearmed, a new log is appended)
    logSearchStart = private->stream->data;

    for (logIndex = 0; logIndex < FLIGHT_LOG_MAX_LOGS_IN_FILE && logSearchStart < private->stream->data + private->stream->size; logIndex++) {
        log->logBegin[logIndex] = memmem(logSearchStart, (private->stream->data + private->stream->size) - logSearchStart, LOG_START_MARKER, strlen(LOG_START_MARKER));

        if (!log->logBegin[logIndex])
            break; //No more logs found in the file

        //Search for the next log after this header ends
        logSearchStart = log->logBegin[logIndex] + strlen(LOG_START_MARKER);
    }

    log->logCount = logIndex;

    /*
     * Stick the end of the file as the beginning of the "one past end" log, so we can easily compute each log size.
     *
     * We have room for this because the logBegin array has an extra element on the end for it.
     */
    log->logBegin[log->logCount] = private->stream->data + private->stream->size;

    log->private = private;

    return log;
}

static const flightLogFrameType_t* getFrameType(uint8_t c)
{
    for (int i = 0; i < (int) ARRAY_LENGTH(frameTypes); i++)
        if (frameTypes[i].marker == c)
            return &frameTypes[i];

    return 0;
}

static void flightLoginvalidateStream(flightLog_t *log)
{
    log->private->mainStreamIsValid = false;
    log->private->mainHistory[1] = 0;
    log->private->mainHistory[2] = 0;
}

static void completeIntraframe(flightLog_t *log, mmapStream_t *stream, char frameType, const char *frameStart, const char *frameEnd, bool raw)
{
    flightLogPrivate_t *private = log->private;

    (void) frameType;

    // Only accept this frame as valid if time and iteration count are moving forward:
    if (raw || ((uint32_t)private->mainHistory[0][FLIGHT_LOG_FIELD_INDEX_ITERATION] >= log->stats.field[FLIGHT_LOG_FIELD_INDEX_ITERATION].max
        && (uint32_t)private->mainHistory[0][FLIGHT_LOG_FIELD_INDEX_TIME] >= log->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].max)) {

        log->private->mainStreamIsValid = true;

        updateFieldStatistics(log, log->private->mainHistory[0]);
    } else {
        flightLoginvalidateStream(log);
    }

    if (log->private->onFrameReady)
        log->private->onFrameReady(log, private->mainStreamIsValid, private->mainHistory[0], frameType, log->mainFieldCount, frameStart - stream->data, frameEnd - frameStart);

    if (log->private->mainStreamIsValid) {
        // Rotate history buffers

        // Both the previous and previous-previous states become the I-frame, because we can't look further into the past than the I-frame
        private->mainHistory[1] = private->mainHistory[0];
        private->mainHistory[2] = private->mainHistory[0];

        // And advance the current frame into an empty space ready to be filled
        private->mainHistory[0] += FLIGHT_LOG_MAX_FIELDS;
        if (private->mainHistory[0] >= &private->blackboxHistoryRing[3][0])
            private->mainHistory[0] = &private->blackboxHistoryRing[0][0];
    }
}

static void completeInterframe(flightLog_t *log, mmapStream_t *stream, char frameType, const char *frameStart, const char *frameEnd, bool raw)
{
    flightLogPrivate_t *private = log->private;

    (void) frameType;
    (void) raw;

    if (log->private->mainStreamIsValid)
        updateFieldStatistics(log, log->private->mainHistory[0]);
    else
        log->stats.frame['P'].desyncCount++;

    //Receiving a P frame can't resynchronise the stream so it doesn't set mainStreamIsValid to true

    if (log->private->onFrameReady)
        log->private->onFrameReady(log, private->mainStreamIsValid, private->mainHistory[0], frameType, log->mainFieldCount, frameStart - stream->data, frameEnd - frameStart);

    if (log->private->mainStreamIsValid) {
        // Rotate history buffers

        private->mainHistory[2] = private->mainHistory[1];
        private->mainHistory[1] = private->mainHistory[0];

        // And advance the current frame into an empty space ready to be filled
        private->mainHistory[0] += FLIGHT_LOG_MAX_FIELDS;
        if (private->mainHistory[0] >= &private->blackboxHistoryRing[3][0])
            private->mainHistory[0] = &private->blackboxHistoryRing[0][0];
    }
}

static void completeEventFrame(flightLog_t *log, mmapStream_t *stream, char frameType, const char *frameStart, const char *frameEnd, bool raw)
{
    (void) stream;
    (void) frameType;
    (void) frameStart;
    (void) frameEnd;
    (void) raw;

    //Don't bother reporting invalid event types since they're likely just garbage data that happened to look like an event
    if (log->private->lastEvent.event != (FlightLogEvent) -1 && log->private->onEvent)
        log->private->onEvent(log, &log->private->lastEvent);
}

static void completeGPSHomeFrame(flightLog_t *log, mmapStream_t *stream, char frameType, const char *frameStart, const char *frameEnd, bool raw)
{
    (void) frameType;
    (void) frameStart;
    (void) frameEnd;
    (void) raw;

    //Copy the decoded frame into the "last state" entry of gpsHomeHistory to publish it:
    memcpy(&log->private->gpsHomeHistory[1], &log->private->gpsHomeHistory[0], sizeof(*log->private->gpsHomeHistory));
    log->private->gpsHomeIsValid = true;

    if (log->private->onFrameReady) {
        log->private->onFrameReady(log, true, log->private->gpsHomeHistory[1], frameType, log->gpsHomeFieldCount, frameStart - stream->data, frameEnd - frameStart);
    }
}

static void completeGPSFrame(flightLog_t *log, mmapStream_t *stream, char frameType, const char *frameStart, const char *frameEnd, bool raw)
{
    (void) frameType;
    (void) frameStart;
    (void) frameEnd;
    (void) raw;

    if (log->private->onFrameReady) {
        log->private->onFrameReady(log, log->private->gpsHomeIsValid, log->private->lastGPS, frameType, log->gpsFieldCount, frameStart - stream->data, frameEnd - frameStart);
    }
}

bool flightLogParse(flightLog_t *log, int logIndex, FlightLogMetadataReady onMetadataReady, FlightLogFrameReady onFrameReady, FlightLogEventReady onEvent, bool raw)
{
    ParserState parserState = PARSER_STATE_HEADER;
    bool looksLikeFrameCompleted = false;

    bool prematureEof = false;
    const char *frameStart = 0;
    const flightLogFrameType_t *frameType = 0, *lastFrameType = 0;

    flightLogPrivate_t *private = log->private;

    if (logIndex < 0 || logIndex >= log->logCount)
        return false;

    //Reset any parsed information from previous parses
    memset(&log->stats, 0, sizeof(log->stats));

    free(private->mainFieldNamesLine);
    free(private->gpsFieldNamesLine);
    free(private->gpsHomeFieldNamesLine);
    private->mainFieldNamesLine = NULL;
    private->gpsFieldNamesLine = NULL;
    private->gpsHomeFieldNamesLine = NULL;

    log->mainFieldCount = 0;
    log->gpsFieldCount = 0;
    log->gpsHomeFieldCount = 0;

    private->gpsHomeIsValid = false;
    flightLoginvalidateStream(log);

    private->mainHistory[0] = private->blackboxHistoryRing[0];
    private->mainHistory[1] = NULL;
    private->mainHistory[2] = NULL;

    //Default to MW's defaults
    log->minthrottle = 1150;
    log->maxthrottle = 1850;

    log->vbatref = 4095;
    log->vbatscale = 110;
    log->vbatmincellvoltage = 33;
    log->vbatmaxcellvoltage = 43;
    log->vbatwarningcellvoltage = 35;

    log->frameIntervalI = 32;
    log->frameIntervalPNum = 1;
    log->frameIntervalPDenom = 1;

    private->motor0Index = -1;
    private->home0Index = -1;
    private->home1Index = -1;
    private->lastEvent.event = -1;

    private->onMetadataReady = onMetadataReady;
    private->onFrameReady = onFrameReady;
    private->onEvent = onEvent;

    //Set parsing ranges up for the log the caller selected
    private->stream->start = log->logBegin[logIndex];
    private->stream->pos = private->stream->start;
    private->stream->end = log->logBegin[logIndex + 1];
    private->stream->eof = false;

    while (1) {
        int command = streamReadByte(private->stream);

        switch (parserState) {
            case PARSER_STATE_HEADER:
                switch (command) {
                    case 'H':
                        parseHeaderLine(log, private->stream);
                    break;
                    case EOF:
                        fprintf(stderr, "Data file contained no events\n");
                        return false;
                    default:
                        frameType = getFrameType(command);

                        if (frameType) {
                            streamUnreadChar(private->stream, command);

                            if (log->mainFieldCount == 0) {
                                fprintf(stderr, "Data file is missing field name definitions\n");
                                return false;
                            }

                            /* Home coord predictors appear in pairs (lat/lon), but the predictor ID is the same for both. It's easier to
                             * apply the right predictor during parsing if we rewrite the predictor ID for the second half of the pair here:
                             */
                            for (int i = 1; i < log->gpsFieldCount; i++) {
                                if (private->frameDefs['G'].predictor[i - 1] == FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD &&
                                        private->frameDefs['G'].predictor[i] == FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD) {
                                    private->frameDefs['G'].predictor[i] = FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD_1;
                                }
                            }

                            parserState = PARSER_STATE_DATA;
                            lastFrameType = NULL;
                            frameStart = private->stream->pos;

                            if (onMetadataReady)
                                onMetadataReady(log);
                        } // else skip garbage which apparently precedes the first data frame
                    break;
                }
            break;
            case PARSER_STATE_DATA:
                if (lastFrameType) {
                    unsigned int lastFrameSize = private->stream->pos - frameStart;

                    // Is this the beginning of a new frame?
                    frameType = command == EOF ? 0 : getFrameType((uint8_t) command);
                    looksLikeFrameCompleted = frameType || (!prematureEof && command == EOF);

                    // If we see what looks like the beginning of a new frame, assume that the previous frame was valid:
                    if (lastFrameSize <= FLIGHT_LOG_MAX_FRAME_LENGTH && looksLikeFrameCompleted) {
                        //Update statistics for this frame type
                        log->stats.frame[lastFrameType->marker].bytes += lastFrameSize;
                        log->stats.frame[lastFrameType->marker].sizeCount[lastFrameSize]++;
                        log->stats.frame[lastFrameType->marker].validCount++;

                        if (lastFrameType->complete)
                            lastFrameType->complete(log, log->private->stream, lastFrameType->marker, frameStart, private->stream->pos, raw);

                    } else {
                        //The previous frame was corrupt

                        //We need to resynchronise before we can deliver another main frame:
                        private->mainStreamIsValid = false;
                        log->stats.frame[lastFrameType->marker].corruptCount++;
                        log->stats.totalCorruptFrames++;

                        //Let the caller know there was a corrupt frame (don't give them a pointer to the frame data because it is totally worthless)
                        if (onFrameReady)
                            onFrameReady(log, false, 0, lastFrameType->marker, 0, frameStart - private->stream->data, lastFrameSize);

                        /*
                         * Start the search for a frame beginning after the first byte of the previous corrupt frame.
                         * This way we can find the start of the next frame after the corrupt frame if the corrupt frame
                         * was truncated.
                         */
                        private->stream->pos = frameStart;
                        lastFrameType = NULL;
                        prematureEof = false;
                        private->stream->eof = false;
                        continue;
                    }
                }

                if (command == EOF)
                    goto done;

                frameType = getFrameType((uint8_t) command);
                frameStart = private->stream->pos;

                if (frameType) {
                    frameType->parse(log, private->stream, raw);
                } else {
                    private->mainStreamIsValid = false;
                }

                //We shouldn't read an EOF during reading a frame (that'd imply the frame was truncated)
                if (private->stream->eof)
                    prematureEof = true;

                lastFrameType = frameType;
            break;
        }
    }

    done:
    log->stats.totalBytes = private->stream->end - private->stream->start;

    return true;
}

void flightLogDestroy(flightLog_t *log)
{
    streamDestroy(log->private->stream);

    free(log->private->mainFieldNamesLine);
    free(log->private->gpsFieldNamesLine);
    free(log->private->gpsHomeFieldNamesLine);
    free(log->private);
    free(log);

    //TODO clean up mainFieldNames
}
