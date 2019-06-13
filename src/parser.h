#ifndef PARSER_H_
#define PARSER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "blackbox_fielddefs.h"

#define FLIGHT_LOG_MAX_LOGS_IN_FILE 128
#define FLIGHT_LOG_MAX_FIELDS 128
#define FLIGHT_LOG_MAX_FRAME_LENGTH 256

#define FLIGHT_LOG_FIELD_INDEX_ITERATION 0
#define FLIGHT_LOG_FIELD_INDEX_TIME 1

#define FLIGHT_LOG_MAX_MOTORS 8
#define FLIGHT_LOG_MAX_SERVOS 8

typedef enum FirmwareType {
    FIRMWARE_TYPE_UNKNOWN = 0,
    FIRMWARE_TYPE_BASEFLIGHT,
    FIRMWARE_TYPE_CLEANFLIGHT,
} FirmwareType;

typedef enum FirmwareRevison {
    FIRMWARE_REVISON_UNKNOWN = 0,
    FIRMWARE_REVISON_BETAFLIGHT,
    FIRMWARE_REVISON_INAV,
} FirmwareRevison;

typedef enum VbatType {
    ORIGINAL = 0,
    TRANSITIONAL,
    INAV_V2
} VbatType;

typedef struct flightLogFrameStatistics_t {
    uint32_t bytes;
    // Frames decoded to the right length and had reasonable data in them:
    uint32_t validCount;

    // Frames decoded to the right length but the data looked bad so they were rejected, or stream was desynced from previous lost frames:
    uint32_t desyncCount;

    // Frames didn't decode to the right length at all
    uint32_t corruptCount;

    uint32_t sizeCount[FLIGHT_LOG_MAX_FRAME_LENGTH + 1];
} flightLogFrameStatistics_t;

typedef struct flightLogFieldStatistics_t {
    int64_t min, max;
} flightLogFieldStatistics_t;

typedef struct flightLogStatistics_t {
    uint32_t totalBytes;

    // Number of frames that failed to decode:
    uint32_t totalCorruptFrames;

    //If our sampling rate is less than 1, we won't log every loop iteration, and that is accounted for here:
    uint32_t intentionallyAbsentIterations;

    bool haveFieldStats;
    flightLogFieldStatistics_t field[FLIGHT_LOG_MAX_FIELDS];
    flightLogFrameStatistics_t frame[256];
} flightLogStatistics_t;

struct flightLogPrivate_t;

/*
 * We provide a list of indexes of well-known fields to save callers the trouble of comparing field name strings
 * to hunt down the fields they're interested in. Absent fields will have index -1.
 */
typedef struct gpsGFieldIndexes_t {
    int time;
    int GPS_numSat;
    int GPS_coord[2];
    int GPS_altitude;
    int GPS_speed;
    int GPS_ground_course;
} gpsGFieldIndexes_t;

typedef struct gpsHFieldIndexes_t {
    int GPS_home[2];
} gpsHFieldIndexes_t;

typedef struct slowFieldIndexes_t {
    int flightModeFlags;
    int stateFlags;
    int failsafePhase;
} slowFieldIndexes_t;

typedef struct mainFieldIndexes_t {
    int loopIteration;
    int time;

    int pid[3][3]; //First dimension is [P, I, D], second dimension is axis

    int rcCommand[4];

    int vbatLatest, amperageLatest;
    int magADC[3];
    int BaroAlt;
    int sonarRaw;
    int rssi;

    int gyroADC[3];
    int accSmooth[3];

    int motor[FLIGHT_LOG_MAX_MOTORS];
    int servo[FLIGHT_LOG_MAX_SERVOS];
} mainFieldIndexes_t;

/**
 * Information about the system configuration of the craft being logged (aids in interpretation
 * of the log data).
 */
typedef struct flightLogSysConfig_t {
    int minthrottle, maxthrottle;
    int motorOutputLow, motorOutputHigh; // Betaflight
    unsigned int rcRate, yawRate;

    // Calibration constants from the hardware sensors:
    uint16_t acc_1G;
    float gyroScale;

    uint8_t vbatscale;
    uint8_t vbatmaxcellvoltage;
    uint8_t vbatmincellvoltage;
    uint8_t vbatwarningcellvoltage;

    int16_t currentMeterOffset, currentMeterScale;

    uint16_t vbatref;

    FirmwareType firmwareType;
    FirmwareRevison firmwareRevison;

    VbatType vbatType;

    struct tm logStartTime;
} flightLogSysConfig_t;

typedef struct flightLogFrameDef_t {
    char *namesLine; // The parser owns this memory to store the field names for this frame type (as a single string)

    int fieldCount;

    char *fieldName[FLIGHT_LOG_MAX_FIELDS];
    
    int fieldSigned[FLIGHT_LOG_MAX_FIELDS];
    int fieldWidth[FLIGHT_LOG_MAX_FIELDS];
    int predictor[FLIGHT_LOG_MAX_FIELDS];
    int encoding[FLIGHT_LOG_MAX_FIELDS];
} flightLogFrameDef_t;

typedef struct flightLog_t {
    flightLogStatistics_t stats;

    //Information about fields which we need to decode them properly
    flightLogFrameDef_t frameDefs[256];

    flightLogSysConfig_t sysConfig;

    //Information about log sections:
    const char *logBegin[FLIGHT_LOG_MAX_LOGS_IN_FILE + 1];
    int logCount;

    unsigned int frameIntervalI;
    unsigned int frameIntervalPNum, frameIntervalPDenom;

    mainFieldIndexes_t mainFieldIndexes;
    gpsGFieldIndexes_t gpsFieldIndexes;
    gpsHFieldIndexes_t gpsHomeFieldIndexes;
    slowFieldIndexes_t slowFieldIndexes;

    struct flightLogPrivate_t *private;
} flightLog_t;

typedef void (*FlightLogMetadataReady)(flightLog_t *log);
typedef void (*FlightLogFrameReady)(flightLog_t *log, bool frameValid, int64_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize);
typedef void (*FlightLogEventReady)(flightLog_t *log, flightLogEvent_t *event);

flightLog_t* flightLogCreate(int fd);

int flightLogEstimateNumCells(flightLog_t *log);

unsigned int flightLogVbatADCToMillivolts(flightLog_t *log, uint16_t vbatADC);
int flightLogAmperageADCToMilliamps(flightLog_t *log, uint16_t amperageADC);
double flightlogGyroToRadiansPerSecond(flightLog_t *log, int32_t gyroRaw);
double flightlogAccelerationRawToGs(flightLog_t *log, int32_t accRaw);
void flightlogFlightModeToString(flightLog_t *log, uint64_t flightMode, char *dest, int destLen);
void flightlogFlightStateToString(flightLog_t *log, uint64_t flightState, char *dest, int destLen);
void flightlogFailsafePhaseToString(uint8_t failsafePhase, char *dest, int destLen);

bool flightLogParse(flightLog_t *log, int logIndex, FlightLogMetadataReady onMetadataReady, FlightLogFrameReady onFrameReady, FlightLogEventReady onEvent, bool raw);
void flightLogDestroy(flightLog_t *log);

#endif
