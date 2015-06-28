#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>

#include <errno.h>
#include <fcntl.h>

//For msvcrt to define M_PI:
#define _USE_MATH_DEFINES
#include <math.h>

#ifdef WIN32
    #include <io.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WIN32
    #include "getopt.h"
#else
    #include <getopt.h>
#endif

#include "parser.h"
#include "platform.h"
#include "tools.h"
#include "gpxwriter.h"
#include "imu.h"
#include "battery.h"
#include "units.h"
#include "stats.h"

typedef struct decodeOptions_t {
    int help, raw, limits, debug, toStdout;
    int logNumber;
    int simulateIMU, imuIgnoreMag;
    int simulateCurrentMeter;
    int mergeGPS;
    const char *outputPrefix;

    bool overrideSimCurrentMeterOffset, overrideSimCurrentMeterScale;
    int16_t simCurrentMeterOffset, simCurrentMeterScale;

    Unit unitGPSSpeed, unitFrameTime, unitVbat, unitAmperage, unitHeight, unitAcceleration, unitRotation, unitFlags;
} decodeOptions_t;

decodeOptions_t options = {
    .help = 0, .raw = 0, .limits = 0, .debug = 0, .toStdout = 0,
    .logNumber = -1,
    .simulateIMU = false, .imuIgnoreMag = 0,
    .simulateCurrentMeter = false,
    .mergeGPS = 0,

    .overrideSimCurrentMeterOffset = false,
    .overrideSimCurrentMeterScale = false,

    .simCurrentMeterOffset = 0, .simCurrentMeterScale = 0,

    .outputPrefix = NULL,

    .unitGPSSpeed = UNIT_METERS_PER_SECOND,
    .unitFrameTime = UNIT_MICROSECONDS,
    .unitVbat = UNIT_VOLTS,
    .unitAmperage = UNIT_AMPS,
    .unitHeight = UNIT_CENTIMETERS,
    .unitAcceleration = UNIT_RAW,
    .unitRotation = UNIT_RAW,
    .unitFlags = UNIT_FLAGS,
};

//We'll use field names to identify GPS field units so the values can be formatted for display
typedef enum {
    GPS_FIELD_TYPE_INTEGER,
    GPS_FIELD_TYPE_DEGREES_TIMES_10, // for headings
    GPS_FIELD_TYPE_COORDINATE_DEGREES_TIMES_10000000,
    GPS_FIELD_TYPE_METERS_PER_SECOND_TIMES_100,
    GPS_FIELD_TYPE_METERS
} GPSFieldType;

static GPSFieldType gpsFieldTypes[FLIGHT_LOG_MAX_FIELDS];

static uint32_t lastFrameTime, lastFrameIteration;

static FILE *csvFile = 0, *eventFile = 0, *gpsCsvFile = 0;
static char *eventFilename = 0, *gpsCsvFilename = 0;
static gpxWriter_t *gpx = 0;

// Computed states:
static currentMeterState_t currentMeterMeasured;
static currentMeterState_t currentMeterVirtual;
static attitude_t attitude;

static Unit mainFieldUnit[FLIGHT_LOG_MAX_FIELDS];
static Unit gpsGFieldUnit[FLIGHT_LOG_MAX_FIELDS];
static Unit slowFieldUnit[FLIGHT_LOG_MAX_FIELDS];

static int32_t bufferedSlowFrame[FLIGHT_LOG_MAX_FIELDS];
static int32_t bufferedMainFrame[FLIGHT_LOG_MAX_FIELDS];
static bool haveBufferedMainFrame;

static uint32_t bufferedFrameTime, bufferedFrameIteration;

static int32_t bufferedGPSFrame[FLIGHT_LOG_MAX_FIELDS];

static seriesStats_t looptimeStats;

#define ADJUSTMENT_FUNCTION_COUNT 21
static char *INFLIGHT_ADJUSTMENT_FUNCTIONS[ADJUSTMENT_FUNCTION_COUNT] = {
        "NONE",
        "RC_RATE",
        "RC_EXPO",
        "THROTTLE_EXPO",
        "PITCH_ROLL_RATE",
        "YAW_RATE",
        "PITCH_ROLL_P",
        "PITCH_ROLL_I",
        "PITCH_ROLL_D",
        "YAW_P",
        "YAW_I",
        "YAW_D",
        "RATE_PROFILE",
        "PITCH_RATE",
        "ROLL_RATE",
        "PITCH_P",
        "PITCH_I",
        "PITCH_D",
        "ROLL_P",
        "ROLL_I",
        "ROLL_D"};

static void fprintfMilliampsInUnit(FILE *file, int32_t milliamps, Unit unit)
{
    switch (unit) {
        case UNIT_AMPS:
            fprintf(file, "%.3f", milliamps / 1000.0);
        break;
        case UNIT_MILLIAMPS:
            fprintf(file, "%d", milliamps);
        break;
        default:
            fprintf(stderr, "Bad amperage unit %d\n", (int) unit);
            exit(-1);
        break;
    }
}

static void fprintfMicrosecondsInUnit(FILE *file, uint32_t microseconds, Unit unit)
{
    switch (unit) {
        case UNIT_MICROSECONDS:
            fprintf(file, "%u", microseconds);
        break;
        case UNIT_MILLISECONDS:
            fprintf(file, "%.3f", microseconds / 1000.0);
        break;
        case UNIT_SECONDS:
            fprintf(file, "%.6f", microseconds / 1000000.0);
        break;
        default:
            fprintf(stderr, "Bad time unit %d\n", (int) unit);
            exit(-1);
        break;
    }
}

static bool fprintfMainFieldInUnit(flightLog_t *log, FILE *file, int fieldIndex, int32_t fieldValue, Unit unit)
{
    /* Convert the fieldValue to the given unit based on the original unit of the field (that we decide on by looking
     * for a well-known field that corresponds to the given fieldIndex.)
     */
    switch (unit) {
        case UNIT_VOLTS:
            if (fieldIndex == log->mainFieldIndexes.vbatLatest) {
                fprintf(file, "%.3f", flightLogVbatADCToMillivolts(log, (uint16_t)fieldValue) / 1000.0);
                return true;
            }
        break;
        case UNIT_MILLIVOLTS:
            if (fieldIndex == log->mainFieldIndexes.vbatLatest) {
                fprintf(file, "%u", flightLogVbatADCToMillivolts(log, (uint16_t)fieldValue));
                return true;
            }
        break;
        case UNIT_AMPS:
        case UNIT_MILLIAMPS:
            if (fieldIndex == log->mainFieldIndexes.amperageLatest) {
                fprintfMilliampsInUnit(file, flightLogAmperageADCToMilliamps(log, (uint16_t)fieldValue), unit);
                return true;
            }
        break;
        case UNIT_CENTIMETERS:
            if (fieldIndex == log->mainFieldIndexes.BaroAlt) {
                fprintf(file, "%d", fieldValue);
                return true;
            }
        break;
        case UNIT_METERS:
            if (fieldIndex == log->mainFieldIndexes.BaroAlt) {
                fprintf(file, "%.2f", (double) fieldValue / 100);
                return true;
            }
        break;
        case UNIT_FEET:
            if (fieldIndex == log->mainFieldIndexes.BaroAlt) {
                fprintf(file, "%.2f", (double) fieldValue / 100 * FEET_PER_METER);
                return true;
            }
        break;
        case UNIT_DEGREES_PER_SECOND:
            if (fieldIndex >= log->mainFieldIndexes.gyroADC[0] && fieldIndex <= log->mainFieldIndexes.gyroADC[2]) {
                fprintf(file, "%.2f", flightlogGyroToRadiansPerSecond(log, fieldValue) * (180 / M_PI));
                return true;
            }
        break;
        case UNIT_RADIANS_PER_SECOND:
            if (fieldIndex >= log->mainFieldIndexes.gyroADC[0] && fieldIndex <= log->mainFieldIndexes.gyroADC[2]) {
                fprintf(file, "%.2f", flightlogGyroToRadiansPerSecond(log, fieldValue));
                return true;
            }
        break;
        case UNIT_METERS_PER_SECOND_SQUARED:
            if (fieldIndex >= log->mainFieldIndexes.accSmooth[0] && fieldIndex <= log->mainFieldIndexes.accSmooth[2]) {
                fprintf(file, "%.2f", flightlogAccelerationRawToGs(log, fieldValue) * ACCELERATION_DUE_TO_GRAVITY);
                return true;
            }
        break;
        case UNIT_GS:
            if (fieldIndex >= log->mainFieldIndexes.accSmooth[0] && fieldIndex <= log->mainFieldIndexes.accSmooth[2]) {
                fprintf(file, "%.2f", flightlogAccelerationRawToGs(log, fieldValue));
                return true;
            }
        break;
        case UNIT_MICROSECONDS:
        case UNIT_MILLISECONDS:
        case UNIT_SECONDS:
            if (fieldIndex == log->mainFieldIndexes.time) {
                fprintfMicrosecondsInUnit(file, (uint32_t) fieldValue, unit);
                return true;
            }
        break;
        case UNIT_RAW:
            if (log->frameDefs['I'].fieldSigned[fieldIndex] || options.raw) {
                fprintf(file, "%3d", (int32_t) fieldValue);
            } else {
                fprintf(file, "%3u", (uint32_t) fieldValue);
            }
            return true;
        break;
        default:
        break;
    }

    // Unit could not be handled
    return false;
}

void onEvent(flightLog_t *log, flightLogEvent_t *event)
{
    (void) log;

    // Open the event log if it wasn't open already
    if (!eventFile) {
        if (eventFilename) {
            eventFile = fopen(eventFilename, "wb");

            if (!eventFile) {
                fprintf(stderr, "Failed to create event log file %s\n", eventFilename);
                return;
            }
        } else {
            //Nowhere to log
            return;
        }
    }

    switch (event->event) {
        case FLIGHT_LOG_EVENT_SYNC_BEEP:
            fprintf(eventFile, "{\"name\":\"Sync beep\", \"time\":%u}\n", event->data.syncBeep.time);
        break;
        case FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_START:
            fprintf(eventFile, "{\"name\":\"Autotune cycle start\", \"time\":%u, \"data\":{\"phase\":%d,\"cycle\":%d,\"p\":%u,\"i\":%u,\"d\":%u,\"rising\":%d}}\n", lastFrameTime,
                event->data.autotuneCycleStart.phase, event->data.autotuneCycleStart.cycle & 0x7F /* Top bit used for "rising: */,
                event->data.autotuneCycleStart.p, event->data.autotuneCycleStart.i, event->data.autotuneCycleStart.d,
                event->data.autotuneCycleStart.cycle >> 7);
        break;
        case FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_RESULT:
            fprintf(eventFile, "{\"name\":\"Autotune cycle result\", \"time\":%u, \"data\":{\"overshot\":%s,\"timedout\":%s,\"p\":%u,\"i\":%u,\"d\":%u}}\n", lastFrameTime,
                event->data.autotuneCycleResult.flags & FLIGHT_LOG_EVENT_AUTOTUNE_FLAG_OVERSHOT ? "true" : "false",
                event->data.autotuneCycleResult.flags & FLIGHT_LOG_EVENT_AUTOTUNE_FLAG_TIMEDOUT ? "true" : "false",
                event->data.autotuneCycleResult.p, event->data.autotuneCycleResult.i, event->data.autotuneCycleResult.d);
        break;
        case FLIGHT_LOG_EVENT_AUTOTUNE_TARGETS:
            fprintf(eventFile, "{\"name\":\"Autotune cycle targets\", \"time\":%u, \"data\":{\"currentAngle\":%.1f,\"targetAngle\":%d,\"targetAngleAtPeak\":%d,\"firstPeakAngle\":%.1f,\"secondPeakAngle\":%.1f}}\n", lastFrameTime,
                event->data.autotuneTargets.currentAngle / 10.0,
                event->data.autotuneTargets.targetAngle, event->data.autotuneTargets.targetAngleAtPeak,
                event->data.autotuneTargets.firstPeakAngle / 10.0, event->data.autotuneTargets.secondPeakAngle / 10.0);
        break;
        case FLIGHT_LOG_EVENT_GTUNE_CYCLE_RESULT:
            fprintf(eventFile, "{\"name\":\"Gtune result\", \"time\":%u, \"data\":{\"axis\":%d,\"gyroAVG\":%d,\"newP\":%d}}\n", lastFrameTime,
                event->data.gtuneCycleResult.axis,
                event->data.gtuneCycleResult.gyroAVG,
                event->data.gtuneCycleResult.newP);
        break;
        case FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT:
            fprintf(eventFile, "{\"name\":\"Inflight adjustment\", \"time\":%u, \"data\":{\"adjustmentFunction\":\"%s\",\"value\":", lastFrameTime,
                    INFLIGHT_ADJUSTMENT_FUNCTIONS[event->data.inflightAdjustment.adjustmentFunction & 127]);
            if (event->data.inflightAdjustment.adjustmentFunction > 127) {
                fprintf(eventFile, "%g", event->data.inflightAdjustment.newFloatValue);
            } else {
                fprintf(eventFile, "%d", event->data.inflightAdjustment.newValue);
            }
            fprintf(eventFile, "}}\n");
        break;
        case FLIGHT_LOG_EVENT_LOGGING_RESUME:
            fprintf(eventFile, "{\"name\":\"Logging resume\", \"time\":%u, \"data\":{\"logIteration\":%d}}\n", event->data.loggingResume.currentTime,
                    event->data.loggingResume.logIteration);
        break;
        case FLIGHT_LOG_EVENT_LOG_END:
            fprintf(eventFile, "{\"name\":\"Log clean end\", \"time\":%u}\n", lastFrameTime);
        break;
        default:
            fprintf(eventFile, "{\"name\":\"Unknown event\", \"time\":%u, \"data\":{\"eventID\":%d}}\n", lastFrameTime, event->event);
        break;
    }
}

/**
 * Print out a comma separated list of field names for the given frame (and field units if not raw),
 * minus the "time" field if `skipTime` is set.
 */
void outputFieldNamesHeader(FILE *file, flightLogFrameDef_t *frame, Unit *fieldUnit, bool skipTime)
{
    bool needComma = false;

    for (int i = 0; i < frame->fieldCount; i++) {
        if (skipTime && strcmp(frame->fieldName[i], "time") == 0)
            continue;

        if (needComma) {
            fprintf(file, ", ");
        } else {
            needComma = true;
        }

        fprintf(file, "%s", frame->fieldName[i]);

        if (fieldUnit && fieldUnit[i] != UNIT_RAW) {
            fprintf(file, " (%s)", UNIT_NAME[fieldUnit[i]]);
        }
    }
}

/**
 * Attempt to create a file to log GPS data in CSV format. On success, gpsCsvFile is non-NULL.
 */
void createGPSCSVFile(flightLog_t *log)
{
    if (!gpsCsvFile && gpsCsvFilename) {
        gpsCsvFile = fopen(gpsCsvFilename, "wb");

        if (gpsCsvFile) {
            // Since the GPS frame itself may or may not include a timestamp field, skip it and print our own:
            fprintf(gpsCsvFile, "time (%s), ", UNIT_NAME[options.unitFrameTime]);

            outputFieldNamesHeader(gpsCsvFile, &log->frameDefs['G'], gpsGFieldUnit, true);

            fprintf(gpsCsvFile, "\n");
        }
    }
}

static void updateSimulations(flightLog_t *log, int32_t *frame, uint32_t currentTime)
{
    int16_t gyroADC[3];
    int16_t accSmooth[3];
    int16_t magADC[3];

    bool hasMag = log->mainFieldIndexes.magADC[0] > -1;
    bool hasThrottle = log->mainFieldIndexes.rcCommand[3] != -1;
    bool hasAmperageADC = log->mainFieldIndexes.amperageLatest != -1;

    int i;

    if (options.simulateIMU) {
        for (i = 0; i < 3; i++) {
            gyroADC[i] = (int16_t) frame[log->mainFieldIndexes.gyroADC[i]];
            accSmooth[i] = (int16_t) frame[log->mainFieldIndexes.accSmooth[i]];
        }

        if (hasMag && !options.imuIgnoreMag) {
            for (i = 0; i < 3; i++) {
                magADC[i] = (int16_t) frame[log->mainFieldIndexes.magADC[i]];
            }
        }

        updateEstimatedAttitude(gyroADC, accSmooth, hasMag && !options.imuIgnoreMag ? magADC : NULL,
            currentTime, log->sysConfig.acc_1G, log->sysConfig.gyroScale, &attitude);
    }

    if (hasAmperageADC) {
        currentMeterUpdateMeasured(
            &currentMeterMeasured,
            flightLogAmperageADCToMilliamps(log, frame[log->mainFieldIndexes.amperageLatest]),
            currentTime
        );
    }

    if (options.simulateCurrentMeter && hasThrottle) {
        int16_t throttle = frame[log->mainFieldIndexes.rcCommand[3]];

        currentMeterUpdateVirtual(
            &currentMeterVirtual,
            options.overrideSimCurrentMeterOffset ? options.simCurrentMeterOffset : log->sysConfig.currentMeterOffset,
            options.overrideSimCurrentMeterScale ? options.simCurrentMeterScale : log->sysConfig.currentMeterScale,
            throttle,
            currentTime
        );
    }
}

/**
 * Print the GPS fields from the given GPS frame as comma-separated values (the GPS frame time is not printed).
 */
void outputGPSFields(flightLog_t *log, FILE *file, int32_t *frame)
{
    int i;
    int32_t degrees;
    uint32_t fracDegrees;
    bool needComma = false;

    for (i = 0; i < log->frameDefs['G'].fieldCount; i++) {
        //We've already printed the time:
        if (i == log->gpsFieldIndexes.time)
            continue;

        if (needComma)
            fprintf(file, ", ");
        else
            needComma = true;

        switch (gpsFieldTypes[i]) {
            case GPS_FIELD_TYPE_COORDINATE_DEGREES_TIMES_10000000:
                degrees = frame[i] / 10000000;
                fracDegrees = abs(frame[i]) % 10000000;

                fprintf(file, "%d.%07u", degrees, fracDegrees);
            break;
            case GPS_FIELD_TYPE_DEGREES_TIMES_10:
                fprintf(file, "%d.%01u", frame[i] / 10, abs(frame[i]) % 10);
            break;
            case GPS_FIELD_TYPE_METERS_PER_SECOND_TIMES_100:
                if (options.unitGPSSpeed == UNIT_RAW) {
                    fprintf(file, "%d", frame[i]);
                } else if (options.unitGPSSpeed == UNIT_METERS_PER_SECOND) {
                    fprintf(file, "%d.%02u", frame[i] / 100, abs(frame[i]) % 100);
                } else {
                    fprintf(file, "%.2f", convertMetersPerSecondToUnit(frame[i] / 100.0, options.unitGPSSpeed));
                }
            break;
            case GPS_FIELD_TYPE_METERS:
                fprintf(file, "%d", frame[i]);
            break;
            case GPS_FIELD_TYPE_INTEGER:
            default:
                fprintf(file, "%d", frame[i]);
        }
    }
}

void outputGPSFrame(flightLog_t *log, int32_t *frame)
{
    uint32_t gpsFrameTime;

    // If we're not logging every loop iteration, we include a timestamp field in the GPS frame:
    if (log->gpsFieldIndexes.time != -1) {
        gpsFrameTime = frame[log->gpsFieldIndexes.time];
    } else {
        // Otherwise this GPS frame was recorded at the same time as the main stream frame we read before the GPS frame:
        gpsFrameTime = lastFrameTime;
    }

    // We need at least lat/lon/altitude from the log to write a useful GPX track
    if (log->gpsFieldIndexes.GPS_coord[0] != -1 && log->gpsFieldIndexes.GPS_coord[0] != -1 && log->gpsFieldIndexes.GPS_altitude != -1) {
        gpxWriterAddPoint(gpx, lastFrameTime, frame[log->gpsFieldIndexes.GPS_coord[0]], frame[log->gpsFieldIndexes.GPS_coord[1]], frame[log->gpsFieldIndexes.GPS_altitude]);
    }

    createGPSCSVFile(log);

    if (gpsCsvFile) {
        fprintfMicrosecondsInUnit(gpsCsvFile, gpsFrameTime, options.unitFrameTime);
        fprintf(gpsCsvFile, ", ");

        outputGPSFields(log, gpsCsvFile, frame);

        fprintf(gpsCsvFile, "\n");
    }
}

void outputSlowFrameFields(flightLog_t *log, int32_t *frame)
{
    enum {
        BUFFER_LEN = 1024
    };
    char buffer[BUFFER_LEN];

    bool needComma = false;

    for (int i = 0; i < log->frameDefs['S'].fieldCount; i++) {
        if (needComma) {
            fprintf(csvFile, ", ");
        } else {
            needComma = true;
        }

        if ((i == log->slowFieldIndexes.flightModeFlags || i == log->slowFieldIndexes.stateFlags)
                && options.unitFlags == UNIT_FLAGS) {

            if (i == log->slowFieldIndexes.flightModeFlags) {
                flightlogFlightModeToString(frame[i], buffer, BUFFER_LEN);
            } else {
                flightlogFlightStateToString(frame[i], buffer, BUFFER_LEN);
            }

            fprintf(csvFile, "%s", buffer);
        } else if (i == log->slowFieldIndexes.failsafePhase && options.unitFlags == UNIT_FLAGS) {
            flightlogFailsafePhaseToString(frame[i], buffer, BUFFER_LEN);

            fprintf(csvFile, "%s", buffer);
        } else {
            //Print raw
            fprintf(csvFile, "%u", frame[i]);
        }
    }
}

/**
 * Print out the fields from the main log stream in comma separated format.
 *
 * Provide (uint32_t) -1 for the frameTime in order to mark the frame time as unknown.
 */
void outputMainFrameFields(flightLog_t *log, uint32_t frameTime, int32_t *frame)
{
    int i;
    bool needComma = false;

    for (i = 0; i < log->frameDefs['I'].fieldCount; i++) {
        if (needComma) {
            fprintf(csvFile, ", ");
        } else {
            needComma = true;
        }

        if (i == FLIGHT_LOG_FIELD_INDEX_TIME) {
            // Use the time the caller provided instead of the time in the frame
            if (frameTime == (uint32_t) -1) {
                fprintf(csvFile, "X");
            } else if (!fprintfMainFieldInUnit(log, csvFile, i, frameTime, mainFieldUnit[i])) {
                fprintf(stderr, "Bad unit for field %d\n", i);
                exit(-1);
            }
        } else if (!fprintfMainFieldInUnit(log, csvFile, i, frame[i], mainFieldUnit[i])) {
            fprintf(stderr, "Bad unit for field %d\n", i);
            exit(-1);
        }
    }

    if (options.simulateIMU) {
        fprintf(csvFile, ", %.2f, %.2f, %.2f", attitude.roll * 180 / M_PI, attitude.pitch * 180 / M_PI, attitude.heading * 180 / M_PI);
    }

    if (log->mainFieldIndexes.amperageLatest != -1) {
        // Integrate the ADC's current measurements to get cumulative energy usage
        fprintf(csvFile, ", %d", (int) round(currentMeterMeasured.energyMilliampHours));
    }

    if (options.simulateCurrentMeter) {
        fprintf(csvFile, ", ");

        fprintfMilliampsInUnit(csvFile, currentMeterVirtual.currentMilliamps, options.unitAmperage);

        fprintf(csvFile, ", %d", (int) round(currentMeterVirtual.energyMilliampHours));
    }

    // Do we have a slow frame to print out too?
    if (log->frameDefs['S'].fieldCount > 0) {
        fprintf(csvFile, ", ");

        outputSlowFrameFields(log, bufferedSlowFrame);
    }
}

void outputMergeFrame(flightLog_t *log)
{
    outputMainFrameFields(log, bufferedFrameTime, bufferedMainFrame);
    fprintf(csvFile, ", ");
    outputGPSFields(log, csvFile, bufferedGPSFrame);
    fprintf(csvFile, "\n");

    haveBufferedMainFrame = false;
}

void updateFrameStatistics(flightLog_t *log, int32_t *frame)
{
    (void) log;

    if (lastFrameIteration != (uint32_t) -1 && (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_ITERATION] > lastFrameIteration) {
        uint32_t looptime = (frame[FLIGHT_LOG_FIELD_INDEX_TIME] - lastFrameTime) / (frame[FLIGHT_LOG_FIELD_INDEX_ITERATION] - lastFrameIteration);

        seriesStats_append(&looptimeStats, looptime);
    }
}

/**
 * This is called when outputting the log in GPS merge mode. When we parse a main frame, we don't know if a GPS frame
 * exists at the same frame time yet, so we we buffer up the main frame data to print later until we know for sure.
 *
 * We also keep a copy of the GPS frame data so we can print it out multiple times if multiple main frames arrive
 * between GPS updates.
 */
void onFrameReadyMerge(flightLog_t *log, bool frameValid, int32_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize)
{
    uint32_t gpsFrameTime;

    (void) frameOffset;
    (void) frameSize;

    switch (frameType) {
        case 'G':
            if (frameValid) {
                if (log->gpsFieldIndexes.time == -1 || (uint32_t) frame[log->gpsFieldIndexes.time] == lastFrameTime) {
                    //This GPS frame was logged in the same iteration as the main frame that preceded it
                    gpsFrameTime = lastFrameTime;
                } else {
                    gpsFrameTime = frame[log->gpsFieldIndexes.time];

                    /*
                     * This GPS frame happened some time after the main frame that preceded it, so print out that main
                     * frame with its older timestamp first if we didn't print it already.
                     */
                    if (haveBufferedMainFrame) {
                        outputMergeFrame(log);
                    }
                }

                /*
                 * Copy this GPS data for later since we may need to duplicate it if there is another main frame before
                 * we get another GPS update.
                 */
                memcpy(bufferedGPSFrame, frame, sizeof(*bufferedGPSFrame) * fieldCount);
                bufferedFrameTime = gpsFrameTime;

                outputMergeFrame(log);

                // We need at least lat/lon/altitude from the log to write a useful GPX track
                if (log->gpsFieldIndexes.GPS_coord[0] != -1 && log->gpsFieldIndexes.GPS_coord[0] != -1 && log->gpsFieldIndexes.GPS_altitude != -1) {
                    gpxWriterAddPoint(gpx, gpsFrameTime, frame[log->gpsFieldIndexes.GPS_coord[0]], frame[log->gpsFieldIndexes.GPS_coord[1]], frame[log->gpsFieldIndexes.GPS_altitude]);
                }
            }
        break;
        case 'S':
            if (frameValid) {
                if (haveBufferedMainFrame) {
                    outputMergeFrame(log);
                }

                memcpy(bufferedSlowFrame, frame, sizeof(bufferedSlowFrame));
            }
        break;
        case 'P':
        case 'I':
            if (frameValid || (frame && options.raw)) {
                if (haveBufferedMainFrame) {
                    outputMergeFrame(log);
                }

                if (frameValid) {
                    updateFrameStatistics(log, frame);

                    lastFrameIteration = (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_ITERATION];
                    lastFrameTime = (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_TIME];

                    updateSimulations(log, frame, lastFrameTime);

                    /*
                     * Store this frame to print out later since we don't know if a GPS frame follows it yet.
                     */
                    memcpy(bufferedMainFrame, frame, sizeof(*bufferedMainFrame) * fieldCount);

                    haveBufferedMainFrame = true;

                    bufferedFrameIteration = lastFrameIteration;
                    bufferedFrameTime = lastFrameTime;
                } else {
                    haveBufferedMainFrame = false;

                    bufferedFrameIteration = -1;
                    bufferedFrameTime = -1;
                }
            }
        break;
    }
}

void onFrameReady(flightLog_t *log, bool frameValid, int32_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize)
{
    if (options.mergeGPS && log->frameDefs['G'].fieldCount > 0) {
        //Use the alternate frame processing routine which merges main stream data and GPS data together
        onFrameReadyMerge(log, frameValid, frame, frameType, fieldCount, frameOffset, frameSize);
        return;
    }

    switch (frameType) {
        case 'G':
            if (frameValid) {
                outputGPSFrame(log, frame);
            }
        break;
        case 'S':
            if (frameValid) {
                memcpy(bufferedSlowFrame, frame, sizeof(bufferedSlowFrame));

                if (options.debug) {
                    fprintf(csvFile, "S frame: ");
                    outputSlowFrameFields(log, bufferedSlowFrame);
                    fprintf(csvFile, "\n");
                }
            }
        break;
        case 'P':
        case 'I':
            if (frameValid || (frame && options.raw)) {
                if (frameValid) {
                    updateFrameStatistics(log, frame);

                    updateSimulations(log, frame, lastFrameTime);

                    lastFrameIteration = (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_ITERATION];
                    lastFrameTime = (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_TIME];
                }

                outputMainFrameFields(log, frameValid ? (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_TIME] : (uint32_t) -1, frame);

                if (options.debug) {
                    fprintf(csvFile, ", %c, offset %d, size %d\n", (char) frameType, frameOffset, frameSize);
                } else
                    fprintf(csvFile, "\n");
            } else if (options.debug) {
                // Print to stdout so that these messages line up with our other output on stdout (stderr isn't synchronised to it)
                if (frame) {
                    /*
                     * We'll assume that the frame's iteration count is still fairly sensible (if an earlier frame was corrupt,
                     * the frame index will be smaller than it should be)
                     */
                    fprintf(csvFile, "%c Frame unusuable due to prior corruption, offset %d, size %d\n", (char) frameType, frameOffset, frameSize);
                } else {
                    fprintf(csvFile, "Failed to decode %c frame, offset %d, size %d\n", (char) frameType, frameOffset, frameSize);
                }
            }
        break;
    }
}

void resetGPSFieldIdents()
{
    for (int i = 0; i < FLIGHT_LOG_MAX_FIELDS; i++) {
        gpsFieldTypes[i] = GPS_FIELD_TYPE_INTEGER;
    }
}

/**
 * Sets the units/display format we should use for each GPS field into the global `gpsFieldTypes`.
 */
void identifyGPSFields(flightLog_t *log)
{
    int i;

    for (i = 0; i < log->frameDefs['G'].fieldCount; i++) {
        const char *fieldName = log->frameDefs['G'].fieldName[i];

        if (strcmp(fieldName, "GPS_coord[0]") == 0) {
            gpsFieldTypes[i] = GPS_FIELD_TYPE_COORDINATE_DEGREES_TIMES_10000000;
        } else if (strcmp(fieldName, "GPS_coord[1]") == 0) {
            gpsFieldTypes[i] = GPS_FIELD_TYPE_COORDINATE_DEGREES_TIMES_10000000;
        } else if (strcmp(fieldName, "GPS_altitude") == 0) {
            gpsFieldTypes[i] = GPS_FIELD_TYPE_METERS;
        } else if (strcmp(fieldName, "GPS_speed") == 0) {
            gpsFieldTypes[i] = GPS_FIELD_TYPE_METERS_PER_SECOND_TIMES_100;
        } else if (strcmp(fieldName, "GPS_ground_course") == 0) {
            gpsFieldTypes[i] = GPS_FIELD_TYPE_DEGREES_TIMES_10;
        } else {
            gpsFieldTypes[i] = GPS_FIELD_TYPE_INTEGER;
        }
    }
}

/**
 * After reading in what fields are present, this routine is called in order to apply the user's
 * commandline choices for field units to the global "mainFieldUnit" and "gpsGFieldUnit" arrays.
 */
void applyFieldUnits(flightLog_t *log)
{
    if (options.raw) {
        for (int i = 0; i < FLIGHT_LOG_MAX_FIELDS; i++) {
            mainFieldUnit[i] = UNIT_RAW;
            gpsGFieldUnit[i] = UNIT_RAW;
            slowFieldUnit[i] = UNIT_RAW;
        }
    } else {
        memset(mainFieldUnit, 0, sizeof(mainFieldUnit));
        memset(gpsGFieldUnit, 0, sizeof(gpsGFieldUnit));
        memset(slowFieldUnit, 0, sizeof(slowFieldUnit));
    
        if (log->mainFieldIndexes.vbatLatest > -1) {
            mainFieldUnit[log->mainFieldIndexes.vbatLatest] = options.unitVbat;
        }

        if (log->mainFieldIndexes.amperageLatest > -1) {
            mainFieldUnit[log->mainFieldIndexes.amperageLatest] = options.unitAmperage;
        }

        if (log->mainFieldIndexes.BaroAlt > -1) {
            mainFieldUnit[log->mainFieldIndexes.BaroAlt] = options.unitHeight;
        }

        if (log->mainFieldIndexes.time > -1) {
            mainFieldUnit[log->mainFieldIndexes.time] = options.unitFrameTime;
        }

        if (log->gpsFieldIndexes.GPS_speed > -1) {
            gpsGFieldUnit[log->gpsFieldIndexes.GPS_speed] = options.unitGPSSpeed;
        }

        for (int i = 0; i < 3; i++) {
            if (log->mainFieldIndexes.accSmooth[i] > -1) {
                mainFieldUnit[log->mainFieldIndexes.accSmooth[i]] = options.unitAcceleration;
            }

            if (log->mainFieldIndexes.gyroADC[i] > -1) {
                mainFieldUnit[log->mainFieldIndexes.gyroADC[i]] = options.unitRotation;
            }
        }

        // Slow frame fields:
        if (log->slowFieldIndexes.flightModeFlags > -1) {
            slowFieldUnit[log->slowFieldIndexes.flightModeFlags] = options.unitFlags;
        }
        if (log->slowFieldIndexes.stateFlags > -1) {
            slowFieldUnit[log->slowFieldIndexes.stateFlags] = options.unitFlags;
        }
        if (log->slowFieldIndexes.failsafePhase > -1) {
            slowFieldUnit[log->slowFieldIndexes.failsafePhase] = options.unitFlags;
        }
    }
}

void writeMainCSVHeader(flightLog_t *log)
{
    int i;

    for (i = 0; i < log->frameDefs['I'].fieldCount; i++) {
        if (i > 0)
            fprintf(csvFile, ", ");

        fprintf(csvFile, "%s", log->frameDefs['I'].fieldName[i]);

        if (mainFieldUnit[i] != UNIT_RAW) {
            fprintf(csvFile, " (%s)", UNIT_NAME[mainFieldUnit[i]]);
        }
    }

    if (options.simulateIMU) {
        fprintf(csvFile, ", roll, pitch, heading");
    }

    if (log->mainFieldIndexes.amperageLatest != -1) {
        fprintf(csvFile, ", energyCumulative (mAh)");
    }

    if (options.simulateCurrentMeter) {
        fprintf(csvFile, ", currentVirtual (%s), energyCumulativeVirtual (mAh)", UNIT_NAME[options.unitAmperage]);
    }

    if (log->frameDefs['S'].fieldCount > 0) {
        fprintf(csvFile, ", ");

        outputFieldNamesHeader(csvFile, &log->frameDefs['S'], slowFieldUnit, false);
    }

    if (options.mergeGPS && log->frameDefs['G'].fieldCount > 0) {
        fprintf(csvFile, ", ");

        outputFieldNamesHeader(csvFile, &log->frameDefs['G'], gpsGFieldUnit, true);
    }

    fprintf(csvFile, "\n");
}

void onMetadataReady(flightLog_t *log)
{
    if (log->frameDefs['I'].fieldCount == 0) {
        fprintf(stderr, "No fields found in log, is it missing its header?\n");
        return;
    } else if (options.simulateIMU && (log->mainFieldIndexes.accSmooth[0] == -1 || log->mainFieldIndexes.gyroADC[0] == -1)){
        fprintf(stderr, "Can't simulate the IMU because accelerometer or gyroscope data is missing\n");
        options.simulateIMU = false;
    }

    identifyGPSFields(log);
    applyFieldUnits(log);

    writeMainCSVHeader(log);
}

void printStats(flightLog_t *log, int logIndex, bool raw, bool limits)
{
    flightLogStatistics_t *stats = &log->stats;
    uint32_t intervalMS = (uint32_t) ((stats->field[FLIGHT_LOG_FIELD_INDEX_TIME].max - stats->field[FLIGHT_LOG_FIELD_INDEX_TIME].min) / 1000);

    uint32_t goodBytes = stats->frame['I'].bytes + stats->frame['P'].bytes;
    uint32_t goodFrames = stats->frame['I'].validCount + stats->frame['P'].validCount;
    uint32_t totalFrames = (uint32_t) (stats->field[FLIGHT_LOG_FIELD_INDEX_ITERATION].max - stats->field[FLIGHT_LOG_FIELD_INDEX_ITERATION].min + 1);
    int32_t missingFrames = totalFrames - goodFrames - stats->intentionallyAbsentIterations;

    uint32_t runningTimeMS, runningTimeSecs, runningTimeMins;
    uint32_t startTimeMS, startTimeSecs, startTimeMins;
    uint32_t endTimeMS, endTimeSecs, endTimeMins;

    uint8_t frameTypes[] = {'I', 'P', 'H', 'G', 'E', 'S'};

    int i;

    if (missingFrames < 0)
        missingFrames = 0;

    runningTimeMS = intervalMS;
    runningTimeSecs = runningTimeMS / 1000;
    runningTimeMS %= 1000;
    runningTimeMins = runningTimeSecs / 60;
    runningTimeSecs %= 60;

    startTimeMS = (uint32_t) (stats->field[FLIGHT_LOG_FIELD_INDEX_TIME].min / 1000);
    startTimeSecs = startTimeMS / 1000;
    startTimeMS %= 1000;
    startTimeMins = startTimeSecs / 60;
    startTimeSecs %= 60;

    endTimeMS = (uint32_t) (stats->field[FLIGHT_LOG_FIELD_INDEX_TIME].max / 1000);
    endTimeSecs = endTimeMS / 1000;
    endTimeMS %= 1000;
    endTimeMins = endTimeSecs / 60;
    endTimeSecs %= 60;

    fprintf(stderr, "\nLog %d of %d", logIndex + 1, log->logCount);

    if (intervalMS > 0 && !raw) {
        fprintf(stderr, ", start %02d:%02d.%03d, end %02d:%02d.%03d, duration %02d:%02d.%03d\n\n",
            startTimeMins, startTimeSecs, startTimeMS,
            endTimeMins, endTimeSecs, endTimeMS,
            runningTimeMins, runningTimeSecs, runningTimeMS
        );
    }

    fprintf(stderr, "Statistics\n");

    if (seriesStats_getCount(&looptimeStats) > 0) {
        fprintf(stderr, "Looptime %14d avg %14.1f std dev (%.1f%%)\n", (int) seriesStats_getMean(&looptimeStats),
            seriesStats_getStandardDeviation(&looptimeStats), seriesStats_getStandardDeviation(&looptimeStats) / seriesStats_getMean(&looptimeStats) * 100);
    }

    for (i = 0; i < (int) sizeof(frameTypes); i++) {
        uint8_t frameType = frameTypes[i];

        if (stats->frame[frameType].validCount ) {
            fprintf(stderr, "%c frames %7d %6.1f bytes avg %8d bytes total\n", (char) frameType, stats->frame[frameType].validCount,
                (float) stats->frame[frameType].bytes / stats->frame[frameType].validCount, stats->frame[frameType].bytes);
        }
    }

    if (goodFrames) {
        fprintf(stderr, "Frames %9d %6.1f bytes avg %8d bytes total\n", goodFrames, (float) goodBytes / goodFrames, goodBytes);
    } else {
        fprintf(stderr, "Frames %8d\n", 0);
    }

    if (intervalMS > 0 && !raw) {
        fprintf(stderr, "Data rate %4uHz %6u bytes/s %10u baud\n",
            (unsigned int) (((int64_t) goodFrames * 1000) / intervalMS),
            (unsigned int) (((int64_t) stats->totalBytes * 1000) / intervalMS),
            (unsigned int) ((((int64_t) stats->totalBytes * 1000 * (8 + 1 + 1)) / intervalMS + 100 - 1) / 100 * 100)); /* Round baud rate up to nearest 100 */
    } else {
        fprintf(stderr, "Data rate: Unknown, no timing information available.\n");
    }

    if (totalFrames && (stats->totalCorruptFrames || missingFrames || stats->intentionallyAbsentIterations)) {
        fprintf(stderr, "\n");

        if (stats->totalCorruptFrames || stats->frame['P'].desyncCount || stats->frame['I'].desyncCount) {
            fprintf(stderr, "%d frames failed to decode, rendering %d loop iterations unreadable. ", stats->totalCorruptFrames, stats->frame['P'].desyncCount + stats->frame['P'].corruptCount + stats->frame['I'].desyncCount + stats->frame['I'].corruptCount);
            if (!missingFrames)
                fprintf(stderr, "\n");
        }
        if (missingFrames) {
            fprintf(stderr, "%d iterations are missing in total (%ums, %.2f%%)\n",
                missingFrames,
                (unsigned int) (((int64_t) missingFrames * intervalMS) / totalFrames),
                (double) missingFrames / totalFrames * 100);
        }
        if (stats->intentionallyAbsentIterations) {
            fprintf(stderr, "%d loop iterations weren't logged because of your blackbox_rate settings (%ums, %.2f%%)\n",
                stats->intentionallyAbsentIterations,
                (unsigned int) (((int64_t)stats->intentionallyAbsentIterations * intervalMS) / totalFrames),
                (double) stats->intentionallyAbsentIterations / totalFrames * 100);
        }
    }

    if (limits) {
        fprintf(stderr, "\n\n    Field name          Min          Max        Range\n");
        fprintf(stderr,     "-----------------------------------------------------\n");

        for (i = 0; i < log->frameDefs['I'].fieldCount; i++) {
            fprintf(stderr, "%14s %12" PRId64 " %12" PRId64 " %12" PRId64 "\n",
                log->frameDefs['I'].fieldName[i],
                stats->field[i].min,
                stats->field[i].max,
                stats->field[i].max - stats->field[i].min
            );
        }
    }

    fprintf(stderr, "\n");
}

void resetParseState() {
    if (options.simulateIMU) {
        imuInit();
    }

    if (options.mergeGPS) {
        haveBufferedMainFrame = false;
        bufferedFrameTime = bufferedFrameIteration = (uint32_t) -1;
        memset(bufferedGPSFrame, 0, sizeof(bufferedGPSFrame));
        memset(bufferedMainFrame, 0, sizeof(bufferedMainFrame));
    }

    memset(bufferedSlowFrame, 0, sizeof(bufferedSlowFrame));

    lastFrameIteration = (uint32_t) -1;
    lastFrameTime = (uint32_t) -1;

    seriesStats_init(&looptimeStats);
}

int decodeFlightLog(flightLog_t *log, const char *filename, int logIndex)
{
    // Organise output files/streams
    gpx = NULL;

    gpsCsvFile = NULL;
    gpsCsvFilename = NULL;

    eventFile = NULL;
    eventFilename = NULL;

    if (options.toStdout) {
        csvFile = stdout;
    } else {
        char *csvFilename = 0, *gpxFilename = 0;
        int filenameLen;

        const char *outputPrefix = 0;
        int outputPrefixLen;

        if (options.outputPrefix) {
            outputPrefix = options.outputPrefix;
            outputPrefixLen = strlen(options.outputPrefix);
        } else {
            const char *fileExtensionPeriod = strrchr(filename, '.');
            const char *logNameEnd;

            if (fileExtensionPeriod) {
                logNameEnd = fileExtensionPeriod;
            } else {
                logNameEnd = filename + strlen(filename);
            }

            outputPrefix = filename;
            outputPrefixLen = logNameEnd - outputPrefix;
        }

        filenameLen = outputPrefixLen + strlen(".00.csv") + 1;
        csvFilename = malloc(filenameLen * sizeof(char));

        snprintf(csvFilename, filenameLen, "%.*s.%02d.csv", outputPrefixLen, outputPrefix, logIndex + 1);

        filenameLen = outputPrefixLen + strlen(".00.gps.gpx") + 1;
        gpxFilename = malloc(filenameLen * sizeof(char));

        snprintf(gpxFilename, filenameLen, "%.*s.%02d.gps.gpx", outputPrefixLen, outputPrefix, logIndex + 1);

        filenameLen = outputPrefixLen + strlen(".00.gps.csv") + 1;
        gpsCsvFilename = malloc(filenameLen * sizeof(char));

        snprintf(gpsCsvFilename, filenameLen, "%.*s.%02d.gps.csv", outputPrefixLen, outputPrefix, logIndex + 1);

        filenameLen = outputPrefixLen + strlen(".00.event") + 1;
        eventFilename = malloc(filenameLen * sizeof(char));

        snprintf(eventFilename, filenameLen, "%.*s.%02d.event", outputPrefixLen, outputPrefix, logIndex + 1);

        csvFile = fopen(csvFilename, "wb");

        if (!csvFile) {
            fprintf(stderr, "Failed to create output file %s\n", csvFilename);

            free(csvFilename);
            return -1;
        }

        fprintf(stderr, "Decoding log '%s' to '%s'...\n", filename, csvFilename);
        free(csvFilename);

        gpx = gpxWriterCreate(gpxFilename);
        free(gpxFilename);
    }

    resetParseState();

    int success = flightLogParse(log, logIndex, onMetadataReady, onFrameReady, onEvent, options.raw);

    if (options.mergeGPS && haveBufferedMainFrame) {
        // Print out last log entry that wasn't already printed
        outputMergeFrame(log);
    }

    if (success)
        printStats(log, logIndex, options.raw, options.limits);

    if (!options.toStdout)
        fclose(csvFile);

    free(eventFilename);
    if (eventFile)
        fclose(eventFile);

    free(gpsCsvFilename);
    if (gpsCsvFile)
        fclose(gpsCsvFile);

    gpxWriterDestroy(gpx);

    return success ? 0 : -1;
}

int validateLogIndex(flightLog_t *log)
{
    //Did the user pick a log to render?
    if (options.logNumber > 0) {
        if (options.logNumber > log->logCount) {
            fprintf(stderr, "Couldn't load log #%d from this file, because there are only %d logs in total.\n", options.logNumber, log->logCount);
            return -1;
        }

        return options.logNumber - 1;
    } else if (log->logCount == 1) {
        // If there's only one log, just parse that
        return 0;
    } else {
        fprintf(stderr, "This file contains multiple flight logs, please choose one with the --index argument:\n\n");

        fprintf(stderr, "Index  Start offset  Size (bytes)\n");
        for (int i = 0; i < log->logCount; i++) {
            fprintf(stderr, "%5d %13d %13d\n", i + 1, (int) (log->logBegin[i] - log->logBegin[0]), (int) (log->logBegin[i + 1] - log->logBegin[i]));
        }

        return -1;
    }
}

void printUsage(const char *argv0)
{
    fprintf(stderr,
        "Blackbox flight log decoder by Nicholas Sherlock ("
#ifdef BLACKBOX_VERSION
            "v" STR(BLACKBOX_VERSION) ", "
#endif
            __DATE__ " " __TIME__ ")\n\n"
        "Usage:\n"
        "     %s [options] <input logs>\n\n"
        "Options:\n"
        "   --help                   This page\n"
        "   --index <num>            Choose the log from the file that should be decoded (or omit to decode all)\n"
        "   --limits                 Print the limits and range of each field\n"
        "   --stdout                 Write log to stdout instead of to a file\n"
        "   --unit-amperage <unit>   Current meter unit (raw|mA|A), default is A (amps)\n"
        "   --unit-flags <unit>      State flags unit (raw|flags), default is flags\n"
        "   --unit-frame-time <unit> Frame timestamp unit (us|s), default is us (microseconds)\n"
        "   --unit-height <unit>     Height unit (m|cm|ft), default is cm (centimeters)\n"
        "   --unit-rotation <unit>   Rate of rotation unit (raw|deg/s|rad/s), default is raw\n"
        "   --unit-acceleration <u>  Acceleration unit (raw|g|m/s2), default is raw\n"
        "   --unit-gps-speed <unit>  GPS speed unit (mps|kph|mph), default is mps (meters per second)\n"
        "   --unit-vbat <unit>       Vbat unit (raw|mV|V), default is V (volts)\n"
        "   --merge-gps              Merge GPS data into the main CSV log file instead of writing it separately\n"
        "   --simulate-current-meter Simulate a virtual current meter using throttle data\n"
        "   --sim-current-meter-scale   Override the FC's settings for the current meter simulation\n"
        "   --sim-current-meter-offset  Override the FC's settings for the current meter simulation\n"
        "   --simulate-imu           Compute tilt/roll/heading fields from gyro/accel/mag data\n"
        "   --imu-ignore-mag         Ignore magnetometer data when computing heading\n"
        "   --declination <val>      Set magnetic declination in degrees.minutes format (e.g. -12.58 for New York)\n"
        "   --declination-dec <val>  Set magnetic declination in decimal degrees (e.g. -12.97 for New York)\n"
        "   --debug                  Show extra debugging information\n"
        "   --raw                    Don't apply predictions to fields (show raw field deltas)\n"
        "\n", argv0
    );
}

double parseDegreesMinutes(const char *s)
{
    int combined = (int) round(atof(s) * 100);

    int degrees = combined / 100;
    int minutes = combined % 100;

    return degrees + (double) minutes / 60;
}

void parseCommandlineOptions(int argc, char **argv)
{
    int c;

    enum {
        SETTING_PREFIX = 1,
        SETTING_INDEX,
        SETTING_CURRENT_METER_OFFSET,
        SETTING_CURRENT_METER_SCALE,
        SETTING_DECLINATION,
        SETTING_DECLINATION_DECIMAL,
        SETTING_UNIT_GPS_SPEED,
        SETTING_UNIT_VBAT,
        SETTING_UNIT_AMPERAGE,
        SETTING_UNIT_HEIGHT,
        SETTING_UNIT_ROTATION,
        SETTING_UNIT_ACCELERATION,
        SETTING_UNIT_FRAME_TIME,
        SETTING_UNIT_FLAGS,
    };

    while (1)
    {
        static struct option long_options[] = {
            {"help", no_argument, &options.help, 1},
            {"raw", no_argument, &options.raw, 1},
            {"debug", no_argument, &options.debug, 1},
            {"limits", no_argument, &options.limits, 1},
            {"stdout", no_argument, &options.toStdout, 1},
            {"merge-gps", no_argument, &options.mergeGPS, 1},
            {"simulate-imu", no_argument, &options.simulateIMU, 1},
            {"simulate-current-meter", no_argument, &options.simulateCurrentMeter, 1},
            {"imu-ignore-mag", no_argument, &options.imuIgnoreMag, 1},
            {"sim-current-meter-scale", required_argument, 0, SETTING_CURRENT_METER_SCALE},
            {"sim-current-meter-offset", required_argument, 0, SETTING_CURRENT_METER_OFFSET},
            {"declination", required_argument, 0, SETTING_DECLINATION},
            {"declination-dec", required_argument, 0, SETTING_DECLINATION_DECIMAL},
            {"prefix", required_argument, 0, SETTING_PREFIX},
            {"index", required_argument, 0, SETTING_INDEX},
            {"unit-gps-speed", required_argument, 0, SETTING_UNIT_GPS_SPEED},
            {"unit-vbat", required_argument, 0, SETTING_UNIT_VBAT},
            {"unit-amperage", required_argument, 0, SETTING_UNIT_AMPERAGE},
            {"unit-height", required_argument, 0, SETTING_UNIT_HEIGHT},
            {"unit-rotation", required_argument, 0, SETTING_UNIT_ROTATION},
            {"unit-acceleration", required_argument, 0, SETTING_UNIT_ACCELERATION},
            {"unit-frame-time", required_argument, 0, SETTING_UNIT_FRAME_TIME},
            {"unit-flags", required_argument, 0, SETTING_UNIT_FLAGS},
            {0, 0, 0, 0}
        };

        int option_index = 0;

        opterr = 0;

        c = getopt_long (argc, argv, "", long_options, &option_index);

        if (c == -1)
            break;

        switch (c) {
            case SETTING_INDEX:
                options.logNumber = atoi(optarg);
            break;
            case SETTING_PREFIX:
                options.outputPrefix = optarg;
            break;
            case SETTING_UNIT_GPS_SPEED:
                if (!unitFromName(optarg, &options.unitGPSSpeed)) {
                    fprintf(stderr, "Bad GPS speed unit\n");
                    exit(-1);
                }
            break;
            case SETTING_UNIT_FRAME_TIME:
                if (!unitFromName(optarg, &options.unitFrameTime)) {
                    fprintf(stderr, "Bad frame time unit\n");
                    exit(-1);
                }
            break;
            case SETTING_UNIT_VBAT:
                if (!unitFromName(optarg, &options.unitVbat)) {
                    fprintf(stderr, "Bad VBAT unit\n");
                    exit(-1);
                }
            break;
            case SETTING_UNIT_AMPERAGE:
                if (!unitFromName(optarg, &options.unitAmperage)) {
                    fprintf(stderr, "Bad Amperage unit\n");
                    exit(-1);
                }
            break;
            case SETTING_UNIT_HEIGHT:
                if (!unitFromName(optarg, &options.unitHeight)) {
                    fprintf(stderr, "Bad height unit\n");
                    exit(-1);
                }
            break;
            case SETTING_UNIT_ACCELERATION:
                if (!unitFromName(optarg, &options.unitAcceleration)) {
                    fprintf(stderr, "Bad acceleration unit\n");
                    exit(-1);
                }
            break;
            case SETTING_UNIT_ROTATION:
                if (!unitFromName(optarg, &options.unitRotation)) {
                    fprintf(stderr, "Bad rotation unit\n");
                    exit(-1);
                }
            break;
            case SETTING_UNIT_FLAGS:
                if (!unitFromName(optarg, &options.unitFlags)) {
                    fprintf(stderr, "Bad flags unit\n");
                    exit(-1);
                }
            break;
            case SETTING_DECLINATION:
                imuSetMagneticDeclination(parseDegreesMinutes(optarg));
            break;
            case SETTING_DECLINATION_DECIMAL:
                imuSetMagneticDeclination(atof(optarg));
            break;
            case SETTING_CURRENT_METER_SCALE:
                options.overrideSimCurrentMeterScale = true;
                options.simCurrentMeterScale = atoi(optarg);
            break;
            case SETTING_CURRENT_METER_OFFSET:
                options.overrideSimCurrentMeterOffset = true;
                options.simCurrentMeterOffset = atoi(optarg);
            break;
            case '\0':
                //Longopt which has set a flag
            break;
            case ':':
                fprintf(stderr, "%s: option '%s' requires an argument\n", argv[0], argv[optind-1]);
                exit(-1);
            break;
            default:
                if (optopt == 0)
                    fprintf(stderr, "%s: option '%s' is invalid\n", argv[0], argv[optind-1]);
                else
                    fprintf(stderr, "%s: option '-%c' is invalid\n", argv[0], optopt);

                exit(-1);
            break;
        }
    }
}

int main(int argc, char **argv)
{
    flightLog_t *log;
    int fd;
    int logIndex;

    platform_init();

    parseCommandlineOptions(argc, argv);

    if (options.help || argc == 1) {
        printUsage(argv[0]);
        return -1;
    }

    if (options.toStdout && argc - optind > 1) {
        fprintf(stderr, "You can only decode one log at a time if you're printing to stdout\n");
        return -1;
    }

    for (int i = optind; i < argc; i++) {
        const char *filename = argv[i];

        fd = open(filename, O_RDONLY);
        if (fd < 0) {
            fprintf(stderr, "Failed to open log file '%s': %s\n\n", filename, strerror(errno));
            continue;
        }

        log = flightLogCreate(fd);

        if (!log) {
            fprintf(stderr, "Failed to read log file '%s'\n\n", filename);
            continue;
        }

        if (log->logCount == 0) {
            fprintf(stderr, "Couldn't find the header of a flight log in the file '%s', is this the right kind of file?\n\n", filename);
            continue;
        }

        if (options.logNumber > 0 || options.toStdout) {
            logIndex = validateLogIndex(log);

            if (logIndex == -1)
                return -1;

            decodeFlightLog(log, filename, logIndex);
        } else {
            //Decode all the logs
            for (logIndex = 0; logIndex < log->logCount; logIndex++)
                decodeFlightLog(log, filename, logIndex);
        }

        flightLogDestroy(log);
    }

    return 0;
}
