/**
 * This tool reads in a flight log and re-encodes it using a private copy of the encoder. This allows experiments
 * to be run on improving the encoder's efficiency, and allows any changes to the encoder to be verified (by comparing
 * decoded logs against the ones produced original encoder).
 */

#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <limits.h>

#ifdef WIN32
    #include "getopt.h"
#else
    #include <getopt.h>
#endif

#include "parser.h"
#include "encoder_testbed_io.h"
#include "tools.h"

#define MAG
#define BARO
#define SONAR
#define XYZ_AXIS_COUNT 3
#define MAX_SUPPORTED_MOTORS 8
#define MAX_SUPPORTED_SERVOS 8
#define USE_SERVOS
#define MIXER_TRI 3
#define MIXER_QUAD 4

#define BLACKBOX_I_INTERVAL 32

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

#define STATIC_ASSERT(condition, name ) \
    typedef char assert_failed_ ## name [(condition) ? 1 : -1 ]

// Some macros to make writing FLIGHT_LOG_FIELD_* constants shorter:
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define CONCAT_HELPER(x,y) x ## y
#define CONCAT(x,y) CONCAT_HELPER(x, y)

#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
#define CONDITION(x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED

static const char blackboxHeader[] =
    "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
    "H Data version:2\n"
    "H I interval:" STR(BLACKBOX_I_INTERVAL) "\n";

static const char* const blackboxFieldHeaderNames[] = {
    "name",
    "signed",
    "predictor",
    "encoding",
    "predictor",
    "encoding"
};

/* All field definition structs should look like this (but with longer arrs): */
typedef struct blackboxFieldDefinition_t {
    const char *name;
    // If the field name has a number to be included in square brackets [1] afterwards, set it here, or -1 for no brackets:
    int8_t fieldNameIndex;

    // Each member of this array will be the value to print for this field for the given header index
    uint8_t arr[1];
} blackboxFieldDefinition_t;

#define BLACKBOX_DELTA_FIELD_HEADER_COUNT       ARRAY_LENGTH(blackboxFieldHeaderNames)
#define BLACKBOX_SIMPLE_FIELD_HEADER_COUNT      (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)
#define BLACKBOX_CONDITIONAL_FIELD_HEADER_COUNT (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)

typedef struct blackboxSimpleFieldDefinition_t {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
} blackboxSimpleFieldDefinition_t;

typedef struct blackboxConditionalFieldDefinition_t {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxConditionalFieldDefinition_t;

typedef struct blackboxDeltaFieldDefinition_t {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t Ipredict;
    uint8_t Iencode;
    uint8_t Ppredict;
    uint8_t Pencode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxDeltaFieldDefinition_t;

/**
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
static const blackboxDeltaFieldDefinition_t blackboxMainFields[] = {
    /* loopIteration doesn't appear in P frames since it always increments */
    {"loopIteration",-1, UNSIGNED, .Ipredict = PREDICT(0),     .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(INC),           .Pencode = FLIGHT_LOG_FIELD_ENCODING_NULL, CONDITION(ALWAYS)},
    /* Time advances pretty steadily so the P-frame prediction is a straight line */
    {"time",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(STRAIGHT_LINE), .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"axisP",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"axisP",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"axisP",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    /* I terms get special packed encoding in P frames: */
    {"axisI",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"axisI",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"axisI",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"axisD",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(NONZERO_PID_D_0)},
    {"axisD",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(NONZERO_PID_D_1)},
    {"axisD",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(NONZERO_PID_D_2)},
    /* rcCommands are encoded together as a group in P-frames: */
    {"rcCommand",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"rcCommand",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"rcCommand",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    /* Throttle is always in the range [minthrottle..maxthrottle]: */
    {"rcCommand",   3, UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},

    {"vbatLatest",    -1, UNSIGNED, .Ipredict = PREDICT(VBATREF),  .Iencode = ENCODING(NEG_14BIT),   .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(ELIAS_DELTA_S32), FLIGHT_LOG_FIELD_CONDITION_VBAT},
    {"amperageLatest",-1, UNSIGNED, .Ipredict = PREDICT(0),        .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(ELIAS_DELTA_S32), FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC},

#ifdef MAG
    {"magADC",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), FLIGHT_LOG_FIELD_CONDITION_MAG},
    {"magADC",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), FLIGHT_LOG_FIELD_CONDITION_MAG},
    {"magADC",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), FLIGHT_LOG_FIELD_CONDITION_MAG},
#endif
#ifdef BARO
    {"BaroAlt",    -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), FLIGHT_LOG_FIELD_CONDITION_BARO},
#endif
#ifdef SONAR
    {"sonarRaw",   -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), FLIGHT_LOG_FIELD_CONDITION_SONAR},
#endif
    {"rssi",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), FLIGHT_LOG_FIELD_CONDITION_RSSI},

    /* Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact */
    {"gyroADC",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"gyroADC",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"gyroADC",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"accSmooth",  0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"accSmooth",  1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    {"accSmooth",  2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(ALWAYS)},
    /* Motors only rarely drops under minthrottle (when stick falls below mincommand), so predict minthrottle for it and use *unsigned* encoding (which is large for negative numbers but more compact for positive ones): */
    {"motor",      0, UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(AT_LEAST_MOTORS_1)},
    /* Subsequent motors base their I-frame values on the first one, P-frame values on the average of last two frames: */
    {"motor",      1, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(AT_LEAST_MOTORS_2)},
    {"motor",      2, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(AT_LEAST_MOTORS_3)},
    {"motor",      3, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(AT_LEAST_MOTORS_4)},
    {"motor",      4, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(AT_LEAST_MOTORS_5)},
    {"motor",      5, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(AT_LEAST_MOTORS_6)},
    {"motor",      6, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(AT_LEAST_MOTORS_7)},
    {"motor",      7, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(AT_LEAST_MOTORS_8)},

    /* Tricopter tail servo */
    {"servo",      5, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(ELIAS_DELTA_S32), CONDITION(TRICOPTER)}
};

#ifdef GPS
// GPS position/vel frame
static const blackboxConditionalFieldDefinition_t blackboxGpsGFields[] = {
    {"time",              -1, UNSIGNED, PREDICT(LAST_MAIN_FRAME_TIME), ENCODING(UNSIGNED_VB), CONDITION(NOT_LOGGING_EVERY_FRAME)},
    {"GPS_numSat",        -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_coord",          0, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_coord",          1, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_altitude",      -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_speed",         -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_ground_course", -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)}
};

// GPS home frame
static const blackboxSimpleFieldDefinition_t blackboxGpsHFields[] = {
    {"GPS_home",           0, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)},
    {"GPS_home",           1, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)}
};
#endif

// Rarely-updated fields
static const blackboxSimpleFieldDefinition_t blackboxSlowFields[] = {
    {"flightModeFlags",   -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB)},
    {"stateFlags",        -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB)},
    {"failsafePhase",     -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB)}
};

typedef struct blackboxMainState_t {
    uint32_t time;

    int32_t axisPID_P[XYZ_AXIS_COUNT], axisPID_I[XYZ_AXIS_COUNT], axisPID_D[XYZ_AXIS_COUNT];

    int16_t rcCommand[4];
    int16_t gyroADC[XYZ_AXIS_COUNT];
    int16_t accSmooth[XYZ_AXIS_COUNT];
    int16_t motor[MAX_SUPPORTED_MOTORS];
    int16_t servo[MAX_SUPPORTED_SERVOS];

    uint16_t vbatLatest;
    uint16_t amperageLatest;

#ifdef BARO
    int32_t BaroAlt;
#endif
#ifdef MAG
    int16_t magADC[XYZ_AXIS_COUNT];
#endif
#ifdef SONAR
    int32_t sonarRaw;
#endif
    uint16_t rssi;
} blackboxMainState_t;

typedef struct blackboxGpsState_t {
    int32_t GPS_home[2], GPS_coord[2];
    uint8_t GPS_numSat;
} blackboxGpsState_t;

// This data is updated really infrequently:
typedef struct blackboxSlowState_t {
    uint16_t flightModeFlags;
    uint8_t stateFlags;
    uint8_t failsafePhase;
}
#ifndef _MSC_VER
    __attribute__((__packed__)) // We pack this struct so that padding doesn't interfere with memcmp()
#endif
    blackboxSlowState_t;

static int motorCount = 0;

static struct {
    uint32_t headerIndex;

    /* Since these fields are used during different blackbox states (never simultaneously) we can
     * overlap them to save on RAM
     */
    union {
        int fieldIndex;
        uint32_t startTime;
    } u;
} xmitState;

// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
static uint32_t blackboxConditionCache;

STATIC_ASSERT((sizeof(blackboxConditionCache) * 8) >= FLIGHT_LOG_FIELD_CONDITION_NEVER, too_many_flight_log_conditions);

static uint32_t blackboxIteration;

/*
 * We store voltages in I-frames relative to this, which was the voltage when the blackbox was activated.
 * This helps out since the voltage is only expected to fall from that point and we can reduce our diffs
 * to encode:
 */
static uint16_t vbatReference;

static blackboxGpsState_t gpsHistory;
static blackboxSlowState_t slowHistory;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxMainState_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackboxMainState_t* blackboxHistory[3];

static flightLog_t *flightLog;

// Program options
static int optionDebug;
static char *optionFilename = 0;

static flightLogStatistics_t encodedStats;

static bool testBlackboxConditionUncached(FlightLogFieldCondition condition)
{
    switch (condition) {
        case FLIGHT_LOG_FIELD_CONDITION_ALWAYS:
            return true;

        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8:
            return motorCount >= (int) condition - FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1 + 1;
        
        case FLIGHT_LOG_FIELD_CONDITION_TRICOPTER:
            return motorCount == 3;

        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2:
            return flightLog->mainFieldIndexes.pid[2][condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0] != -1;

        case FLIGHT_LOG_FIELD_CONDITION_MAG:
#ifdef MAG
            return flightLog->mainFieldIndexes.magADC[0] != -1;
#else
            return false;
#endif

        case FLIGHT_LOG_FIELD_CONDITION_BARO:
#ifdef BARO
            return flightLog->mainFieldIndexes.BaroAlt != -1;
#else
            return false;
#endif

        case FLIGHT_LOG_FIELD_CONDITION_VBAT:
            return flightLog->mainFieldIndexes.vbatLatest != -1;

        case FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC:
            return flightLog->mainFieldIndexes.amperageLatest != -1;

        case FLIGHT_LOG_FIELD_CONDITION_SONAR:
#ifdef SONAR
            return flightLog->mainFieldIndexes.sonarRaw != -1;
#else
            return false;
#endif

        case FLIGHT_LOG_FIELD_CONDITION_RSSI:
            return flightLog->mainFieldIndexes.rssi != -1;

        case FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
            return flightLog->frameIntervalPNum < flightLog->frameIntervalPDenom;

        case FLIGHT_LOG_FIELD_CONDITION_NEVER:
            return false;
        default:
            return false;
    }
}

static void blackboxBuildConditionCache()
{
    FlightLogFieldCondition cond;

    blackboxConditionCache = 0;

    for (cond = FLIGHT_LOG_FIELD_CONDITION_FIRST; cond <= FLIGHT_LOG_FIELD_CONDITION_LAST; cond++) {
        if (testBlackboxConditionUncached(cond)) {
            blackboxConditionCache |= 1 << cond;
        }
    }
}

static bool testBlackboxCondition(FlightLogFieldCondition condition)
{
    return (blackboxConditionCache & (1 << condition)) != 0;
}

static void writeIntraframe(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    int x;

    blackboxWrite('I');

    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB(blackboxCurrent->time);

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->axisPID_P[x]);
    }

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->axisPID_I[x]);
    }

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x)) {
            blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x]);
        }
    }

    for (x = 0; x < 3; x++) {
        blackboxWriteSignedVB(blackboxCurrent->rcCommand[x]);
    }

    blackboxWriteUnsignedVB(blackboxCurrent->rcCommand[3] - flightLog->sysConfig.minthrottle); //Throttle lies in range [minthrottle..maxthrottle]

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
        /*
         * Our voltage is expected to decrease over the course of the flight, so store our difference from
         * the reference:
         *
         * Write 14 bits even if the number is negative (which would otherwise result in 32 bits)
         */
        blackboxWriteUnsignedVB((vbatReference - blackboxCurrent->vbatLatest) & 0x3FFF);
    }

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC)) {
        // 12bit value directly from ADC
        blackboxWriteUnsignedVB(blackboxCurrent->amperageLatest);
    }

#ifdef MAG
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
            for (x = 0; x < XYZ_AXIS_COUNT; x++) {
                blackboxWriteSignedVB(blackboxCurrent->magADC[x]);
            }
        }
#endif

#ifdef BARO
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO)) {
            blackboxWriteSignedVB(blackboxCurrent->BaroAlt);
        }
#endif

#ifdef SONAR
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SONAR)) {
            blackboxWriteSignedVB(blackboxCurrent->sonarRaw);
        }
#endif

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        blackboxWriteUnsignedVB(blackboxCurrent->rssi);
    }

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->gyroADC[x]);
    }

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->accSmooth[x]);
    }

    //Motors can be below minthrottle when disarmed, but that doesn't happen much
    blackboxWriteUnsignedVB(blackboxCurrent->motor[0] - flightLog->sysConfig.minthrottle);

    //Motors tend to be similar to each other
    for (x = 1; x < motorCount; x++) {
        blackboxWriteSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);
    }

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
        blackboxWriteSignedVB(blackboxHistory[0]->servo[5] - 1500);
    }

    //Rotate our history buffers:

    //The current state becomes the new "before" state
    blackboxHistory[1] = blackboxHistory[0];
    //And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2] = blackboxHistory[0];
    //And advance the current state over to a blank space ready to be filled
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

static void writeInterframe(void)
{
    int x;
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    blackboxMainState_t *blackboxLast = blackboxHistory[1];

    blackboxWrite('P');

    //No need to store iteration count since its delta is always 1

    /*
     * Since the difference between the difference between successive times will be nearly zero (due to consistent
     * looptime spacing), use second-order differences.
     */
    blackboxWriteS32EliasDelta((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteS32EliasDelta(blackboxCurrent->axisPID_P[x] - blackboxLast->axisPID_P[x]);
    }

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteS32EliasDelta(blackboxCurrent->axisPID_I[x] - blackboxLast->axisPID_I[x]);
    }
    
    /*
     * The PID D term is frequently set to zero for yaw, which makes the result from the calculation
     * always zero. So don't bother recording D results when PID D terms are zero.
     */
    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x)) {
            blackboxWriteS32EliasDelta(blackboxCurrent->axisPID_D[x] - blackboxLast->axisPID_D[x]);
        }
    }

    /*
     * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
     * can pack multiple values per byte:
     */
    for (x = 0; x < 4; x++) {
        blackboxWriteS32EliasDelta(blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x]);
    }

    //Check for sensors that are updated periodically (so deltas are normally zero)
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
        blackboxWriteS32EliasDelta((int32_t) blackboxCurrent->vbatLatest - blackboxLast->vbatLatest);
    }

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC)) {
        blackboxWriteS32EliasDelta((int32_t) blackboxCurrent->amperageLatest - blackboxLast->amperageLatest);
    }

#ifdef MAG
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
        for (x = 0; x < XYZ_AXIS_COUNT; x++) {
            blackboxWriteS32EliasDelta(blackboxCurrent->magADC[x] - blackboxLast->magADC[x]);
        }
    }
#endif

#ifdef BARO
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO)) {
        blackboxWriteS32EliasDelta(blackboxCurrent->BaroAlt - blackboxLast->BaroAlt);
    }
#endif

#ifdef SONAR
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SONAR)) {
        blackboxWriteS32EliasDelta(blackboxCurrent->sonarRaw - blackboxLast->sonarRaw);
    }
#endif

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        blackboxWriteS32EliasDelta((int32_t) blackboxCurrent->rssi - blackboxLast->rssi);
    }

    //Since gyros, accs and motors are noisy, base the prediction on the average of the history:
    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteS32EliasDelta(blackboxHistory[0]->gyroADC[x] - (blackboxHistory[1]->gyroADC[x] + blackboxHistory[2]->gyroADC[x]) / 2);
    }

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteS32EliasDelta(blackboxHistory[0]->accSmooth[x] - (blackboxHistory[1]->accSmooth[x] + blackboxHistory[2]->accSmooth[x]) / 2);
    }

    for (x = 0; x < motorCount; x++) {
        blackboxWriteS32EliasDelta(blackboxHistory[0]->motor[x] - (blackboxHistory[1]->motor[x] + blackboxHistory[2]->motor[x]) / 2);
    }

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
        blackboxWriteS32EliasDelta(blackboxCurrent->servo[5] - blackboxLast->servo[5]);
    }

    // Flush the bit cache to align the stream to a byte boundary
    blackboxFlushBits();

    //Rotate our history buffers
    blackboxHistory[2] = blackboxHistory[1];
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

/* Write the contents of the global "slowHistory" to the log as an "S" frame. Because this data is logged so
 * infrequently, delta updates are not reasonable, so we log independent frames. */
static void writeSlowFrame(void)
{
    blackboxWrite('S');

    blackboxWriteUnsignedVB(slowHistory.flightModeFlags);
    blackboxWriteUnsignedVB(slowHistory.stateFlags);
    blackboxWriteUnsignedVB(slowHistory.failsafePhase);
}

/**
 * Load rarely-changing values from the FC into the given structure
 */
static void loadSlowState(int64_t *frame)
{
    slowHistory.flightModeFlags = frame[0];
    slowHistory.stateFlags = frame[1];
    slowHistory.failsafePhase = frame[2];
}

/**
 * Fill the current state of the blackbox using values read from the flight controller
 */
static void loadMainState(int64_t *frame)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    mainFieldIndexes_t *index = &flightLog->mainFieldIndexes;
    int i;

    blackboxIteration = frame[index->loopIteration];
    blackboxCurrent->time = frame[index->time];

    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_P[i] = frame[index->pid[0][i]];
    }
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_I[i] = frame[index->pid[1][i]];
    }
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_D[i] = index->pid[2][i] > -1 ? frame[index->pid[2][i]] : 0;
    }

    for (i = 0; i < 4; i++) {
        blackboxCurrent->rcCommand[i] = frame[index->rcCommand[i]];
    }

    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->gyroADC[i] = frame[index->gyroADC[i]];
    }

    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->accSmooth[i] = frame[index->accSmooth[i]];
    }

    for (i = 0; i < motorCount; i++) {
        blackboxCurrent->motor[i] = frame[index->motor[i]];
    }

    blackboxCurrent->vbatLatest = index->vbatLatest > -1 ? frame[index->vbatLatest] : 0;
    blackboxCurrent->amperageLatest = index->amperageLatest > -1 ? frame[index->amperageLatest] : 0;

#ifdef MAG
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->magADC[i] = index->magADC[i] > -1 ? frame[index->magADC[i]] : 0;
    }
#endif

#ifdef BARO
    blackboxCurrent->BaroAlt = index->BaroAlt > -1 ? frame[index->BaroAlt] : 0;
#endif

#ifdef SONAR
    // Store the raw sonar value without applying tilt correction
    blackboxCurrent->sonarRaw = index->sonarRaw > -1 ? frame[index->sonarRaw] : 0;
#endif

    blackboxCurrent->rssi = index->rssi > -1 ? frame[index->rssi] : 0;

#ifdef USE_SERVOS
    //Tail servo for tricopters
    blackboxCurrent->servo[5] = index->servo[5] > -1 ? frame[index->servo[5]] : 0;
#endif
}

/*
 * Treat each decoded frame as if it were a set of freshly read flight data ready to be
 * encoded.
 */
void onFrameReady(flightLog_t *fl, bool frameValid, int64_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize)
{
    uint32_t start = blackboxWrittenBytes;
    unsigned int encodedFrameSize;

    (void) fl;
    (void) fieldCount;
    (void) frameOffset;
    (void) frameSize;

    if (frameValid) {
        switch (frameType) {
            case 'I':
                loadMainState(frame);
                writeIntraframe();

                encodedFrameSize = blackboxWrittenBytes - start;

                encodedStats.frame['I'].validCount++;
                encodedStats.frame['I'].bytes += encodedFrameSize;
                encodedStats.frame['I'].sizeCount[encodedFrameSize]++;
            break;
            case 'P':
                loadMainState(frame);
                writeInterframe();

                encodedFrameSize = blackboxWrittenBytes - start;

                encodedStats.frame['P'].validCount++;
                encodedStats.frame['P'].bytes += encodedFrameSize;
                encodedStats.frame['P'].sizeCount[encodedFrameSize]++;
            break;
            case 'G':
            case 'H':
                //TODO implement me
            break;
            case 'S':
                loadSlowState(frame);
                writeSlowFrame();

                encodedFrameSize = blackboxWrittenBytes - start;

                encodedStats.frame['S'].validCount++;
                encodedStats.frame['S'].bytes += encodedFrameSize;
                encodedStats.frame['S'].sizeCount[encodedFrameSize]++;
            break;
            default:
                fprintf(stderr, "Unknown frame type %c\n", (char) frameType);
        }
    }
}

void parseCommandlineOptions(int argc, char **argv)
{
    int c;

    while (1)
    {
        static struct option long_options[] = {
            {"debug", no_argument, &optionDebug, 1},
            {0, 0, 0, 0}
        };

        int option_index = 0;

        c = getopt_long (argc, argv, "", long_options, &option_index);

        /* Detect the end of the options. */
        if (c == -1)
            break;
    }

    if (optind < argc)
        optionFilename = argv[optind];
}

// Print out a chart listing the numbers of frames in each size category
void printFrameSizeComparison(flightLogStatistics_t *oldStats, flightLogStatistics_t *newStats)
{
    // First determine the size bounds:
    int smallestSize = 255, largestSize = 0;

    bool frameTypeExists[256];

    for (int frameType = 0; frameType <= 255; frameType++) {
        frameTypeExists[frameType] = oldStats->frame[frameType].validCount || newStats->frame[frameType].validCount;
        if (frameTypeExists[frameType]) {
            for (int i = 0; i < 256; i++) {
                if (oldStats->frame[frameType].sizeCount[i] || newStats->frame[frameType].sizeCount[i]) {
                    if (i < smallestSize)
                        smallestSize = i;
                    if (i > largestSize)
                        largestSize = i;
                }
            }
        }
    }

    fprintf(stderr, "\nFrame sizes\n");

    fprintf(stderr, "  ");
    for (int frameType = 0; frameType <= 255; frameType++) {
        if (frameTypeExists[frameType]) {
            fprintf(stderr, "       Old       New");
        }
    }
    fprintf(stderr, "\n");

    fprintf(stderr, "Size");
    for (int frameType = 0; frameType <= 255; frameType++) {
        if (frameTypeExists[frameType]) {
            fprintf(stderr, "   %c count   %c count", (char) frameType, (char) frameType);
        }
    }
    fprintf(stderr, "\n");

    for (int i = smallestSize; i <= largestSize; i++) {
        fprintf(stderr, "%4d ", i);
        for (int frameType = 0; frameType <= 255; frameType++) {
            if (frameTypeExists[frameType]) {
                fprintf(stderr, "%9d %9d ", oldStats->frame[frameType].sizeCount[i], newStats->frame[frameType].sizeCount[i]);
            }
        }
        fprintf(stderr, "\n");
    }
}

void printStats(flightLogStatistics_t *stats)
{
    uint32_t intervalMS = (uint32_t) ((stats->field[FLIGHT_LOG_FIELD_INDEX_TIME].max - stats->field[FLIGHT_LOG_FIELD_INDEX_TIME].min) / 1000);
    uint32_t totalBytes = stats->totalBytes;
    uint32_t totalFrames = stats->frame['I'].validCount + stats->frame['P'].validCount;

    for (int i = 0; i < 256; i++) {
        uint8_t frameType = (uint8_t) i;

        if (stats->frame[frameType].validCount) {
            fprintf(stderr, "%c frames %7d %6.1f bytes avg %8d bytes total\n", (char) frameType, stats->frame[frameType].validCount,
                (float) stats->frame[frameType].bytes / stats->frame[frameType].validCount, stats->frame[frameType].bytes);
        }
    }

    if (totalFrames)
        fprintf(stderr, "Frames %9d %6.1f bytes avg %8d bytes total\n", totalFrames, (double) totalBytes / totalFrames, totalBytes);
    else
        fprintf(stderr, "Frames %8d\n", 0);

    if (stats->totalCorruptFrames)
        fprintf(stderr, "%d frames failed to decode (%.2f%%)\n", stats->totalCorruptFrames, (double) stats->totalCorruptFrames / (stats->totalCorruptFrames + stats->frame['I'].validCount + stats->frame['P'].validCount) * 100);

    fprintf(stderr, "IntervalMS %u Total bytes %u\n", intervalMS, stats->totalBytes);

    if (intervalMS > 0) {
        fprintf(stderr, "Data rate %4uHz %6u bytes/s %10u baud\n",
                (unsigned int) (((int64_t) totalFrames * 1000) / intervalMS),
                (unsigned int) (((int64_t) stats->totalBytes * 1000) / intervalMS),
                (unsigned int) ((((int64_t) stats->totalBytes * 1000 * 8) / intervalMS + 100 - 1) / 100 * 100)); /* Round baud rate up to nearest 100 */
    }
}

/**
 * Transmit the header information for the given field definitions. Transmitted header lines look like:
 *
 * H Field I name:a,b,c
 * H Field I predictor:0,1,2
 *
 * For all header types, provide a "mainFrameChar" which is the name for the field and will be used to refer to it in the
 * header (e.g. P, I etc). For blackboxDeltaField_t fields, also provide deltaFrameChar, otherwise set this to zero.
 *
 * Provide an array 'conditions' of FlightLogFieldCondition enums if you want these conditions to decide whether a field
 * should be included or not. Otherwise provide NULL for this parameter and NULL for secondCondition.
 *
 * Set xmitState.headerIndex to 0 and xmitState.u.fieldIndex to -1 before calling for the first time.
 *
 * secondFieldDefinition and secondCondition element pointers need to be provided in order to compute the stride of the
 * fieldDefinition and secondCondition arrays.
 *
 * Returns true if there is still header left to transmit (so call again to continue transmission).
 */
static bool sendFieldDefinition(char mainFrameChar, char deltaFrameChar, const void *fieldDefinitions,
        const void *secondFieldDefinition, int fieldCount, const uint8_t *conditions, const uint8_t *secondCondition)
{
    const blackboxFieldDefinition_t *def;
    unsigned int headerCount;
    static bool needComma = false;
    size_t definitionStride = (char*) secondFieldDefinition - (char*) fieldDefinitions;
    size_t conditionsStride = (char*) secondCondition - (char*) conditions;

    if (deltaFrameChar) {
        headerCount = BLACKBOX_DELTA_FIELD_HEADER_COUNT;
    } else {
        headerCount = BLACKBOX_SIMPLE_FIELD_HEADER_COUNT;
    }

    /*
     * We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
     * the whole header.
     */

    // On our first call we need to print the name of the header and a colon
    if (xmitState.u.fieldIndex == -1) {
        if (xmitState.headerIndex >= headerCount) {
            return false; //Someone probably called us again after we had already completed transmission
        }

        uint32_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[xmitState.headerIndex]);

        if (blackboxDeviceReserveBufferSpace(charsToBeWritten) != BLACKBOX_RESERVE_SUCCESS) {
            return true; // Try again later
        }

        blackboxPrintf("H Field %c %s:", xmitState.headerIndex >= BLACKBOX_SIMPLE_FIELD_HEADER_COUNT ? deltaFrameChar : mainFrameChar, blackboxFieldHeaderNames[xmitState.headerIndex]);

        xmitState.u.fieldIndex++;
        needComma = false;
    }

    // We write at most this many bytes per call:
    const uint32_t WRITE_CHUNK_SIZE = 32;
    // The longest we expect an integer to be as a string (it's currently 2 but add a little room to grow here):
    const uint32_t LONGEST_INTEGER_STRLEN = 4;

    int32_t bufferRemain;

    if (blackboxDeviceReserveBufferSpace(WRITE_CHUNK_SIZE) != BLACKBOX_RESERVE_SUCCESS) {
        return true; // Device is busy right now, try later
    }
    
    bufferRemain = WRITE_CHUNK_SIZE - 1; // Leave 1 byte spare so we can easily terminate line

    for (; xmitState.u.fieldIndex < fieldCount; xmitState.u.fieldIndex++) {
        def = (const blackboxFieldDefinition_t*) ((const char*)fieldDefinitions + definitionStride * xmitState.u.fieldIndex);

        if (!conditions || testBlackboxCondition(conditions[conditionsStride * xmitState.u.fieldIndex])) {
            int32_t bytesToWrite = 0;

            // First estimate the length of the string we want to print
            if (needComma) {
                bytesToWrite++;
            }

            // The first header is a field name
            if (xmitState.headerIndex == 0) {
                bytesToWrite += strlen(def->name);

                // Do we need to print an index in brackets after the name?
                if (def->fieldNameIndex != -1) {
                    bytesToWrite += strlen("[]") + LONGEST_INTEGER_STRLEN;
                }
            } else {
                //The other headers are integers
                bytesToWrite += LONGEST_INTEGER_STRLEN;
            }

            // Now perform the write if the buffer is large enough
            
            if (bytesToWrite > bufferRemain) {
                // Ran out of space!
                return true;
            }

            if (needComma) {
                blackboxWrite(',');
                bufferRemain--;
            } else {
                needComma = true;
            }

            // The first header is a field name
            if (xmitState.headerIndex == 0) {
                bufferRemain -= blackboxPrint(def->name);

                // Do we need to print an index in brackets after the name?
                if (def->fieldNameIndex != -1) {
                    bufferRemain -= blackboxPrintf("[%d]", def->fieldNameIndex);
                }
            } else {
                //The other headers are integers
                bufferRemain -= blackboxPrintf("%d", def->arr[xmitState.headerIndex - 1]);
            }
        }
    }

    // Did we complete this line?
    if (xmitState.u.fieldIndex == fieldCount) {
        blackboxWrite('\n');
        xmitState.headerIndex++;
        xmitState.u.fieldIndex = -1;
    }

    return xmitState.headerIndex < headerCount;
}

/**
 * Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0. Returns
 * true iff transmission is complete, otherwise call again later to continue transmission.
 */
static bool blackboxWriteSysinfo()
{
    blackboxPrintf("H Firmware type:Cleanflight\n");
    blackboxPrintf("H Firmware revision:xxxxxxx\n");
    blackboxPrintf("H Firmware date:" __DATE__ " " __TIME__ "\n");
    blackboxPrintf("H P interval:%d/%d\n", flightLog->frameIntervalPNum, flightLog->frameIntervalPDenom);
    blackboxPrintf("H rcRate:%d\n", flightLog->sysConfig.rcRate);
    blackboxPrintf("H minthrottle:%d\n", flightLog->sysConfig.minthrottle);
    blackboxPrintf("H maxthrottle:%d\n", flightLog->sysConfig.maxthrottle);
    blackboxPrintf("H gyro.scale:0x%x\n", floatToUint(flightLog->sysConfig.gyroScale));
    blackboxPrintf("H acc_1G:%u\n", flightLog->sysConfig.acc_1G);
    blackboxPrintf("H vbatscale:%u\n", flightLog->sysConfig.vbatscale);
    blackboxPrintf("H vbatcellvoltage:%u,%u,%u\n", flightLog->sysConfig.vbatmincellvoltage, flightLog->sysConfig.vbatwarningcellvoltage, flightLog->sysConfig.vbatmaxcellvoltage);
    blackboxPrintf("H vbatref:%u\n", flightLog->sysConfig.vbatref);
    blackboxPrintf("H currentMeter:%d,%d\n", flightLog->sysConfig.currentMeterOffset, flightLog->sysConfig.currentMeterScale);
    
    return true;
}

static void blackboxLogHeaders() {
    for (int i = 0; blackboxHeader[i] != '\0'; i++) {
        blackboxWrite(blackboxHeader[i]);
    }

    xmitState.headerIndex = 0;
    xmitState.u.fieldIndex = -1;
    while (sendFieldDefinition('I', 'P', blackboxMainFields, blackboxMainFields + 1, ARRAY_LENGTH(blackboxMainFields),
        &blackboxMainFields[0].condition, &blackboxMainFields[1].condition)) {
    }

#ifdef GPS
    if (log->frameDefs['H'].fieldCount > 0) {
        xmitState.headerIndex = 0;
        xmitState.u.fieldIndex = -1;
        while (sendFieldDefinition('H', 0, blackboxGpsHFields, blackboxGpsHFields + 1, ARRAY_LENGTH(blackboxGpsHFields),
            NULL, NULL)) {
        }
    }

    if (log->frameDefs['G'].fieldCount > 0) {
        xmitState.headerIndex = 0;
        xmitState.u.fieldIndex = -1;
        while (sendFieldDefinition('G', 0, blackboxGpsGFields, blackboxGpsGFields + 1, ARRAY_LENGTH(blackboxGpsGFields),
            &blackboxGpsGFields[0].condition, &blackboxGpsGFields[1].condition)) {
        }
    }
#endif

    if (flightLog->frameDefs['S'].fieldCount > 0) {
        xmitState.headerIndex = 0;
        xmitState.u.fieldIndex = -1;
        while (sendFieldDefinition('S', 0, blackboxSlowFields, blackboxSlowFields + 1, ARRAY_LENGTH(blackboxSlowFields),
            NULL, NULL)) {
        }
    }

    blackboxWriteSysinfo();
}


void onMetadataReady(flightLog_t *fl)
{
    int i;

    (void) fl;

    motorCount = 0;

    for (i = 0; i < flightLog->frameDefs['I'].fieldCount; i++) {
        if (strncmp(flightLog->frameDefs['I'].fieldName[i], "motor[", strlen("motor[")) == 0) {
            int motorIndex = atoi(flightLog->frameDefs['I'].fieldName[i] + strlen("motor["));

            if (motorIndex + 1 > motorCount)
                motorCount = motorIndex + 1;
        }
    }

    vbatReference = flightLog->sysConfig.vbatref;
    blackboxBuildConditionCache();

    blackboxLogHeaders();
}

int main(int argc, char **argv)
{
    FILE *input;

    parseCommandlineOptions(argc, argv);

    if (!optionFilename) {
        fprintf(stderr, "Missing log filename argument\n");
        return -1;
    }

    input = fopen(optionFilename, "rb");

    if (!input) {
        fprintf(stderr, "Failed to open input file!\n");
        return -1;
    }

    blackboxHistory[0] = &blackboxHistoryRing[0];
    blackboxHistory[1] = &blackboxHistoryRing[1];
    blackboxHistory[2] = &blackboxHistoryRing[2];

    flightLog = flightLogCreate(fileno(input));

    flightLogParse(flightLog, 0, onMetadataReady, onFrameReady, NULL, 0);

    encodedStats.totalBytes = blackboxWrittenBytes;
    encodedStats.field[FLIGHT_LOG_FIELD_INDEX_TIME].min = flightLog->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].min;
    encodedStats.field[FLIGHT_LOG_FIELD_INDEX_TIME].max = flightLog->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].max;

    fprintf(stderr, "Logged time %u seconds\n", (uint32_t)((flightLog->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].max - flightLog->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].min) / 1000000));

    fprintf(stderr, "\nOriginal statistics\n");
    printStats(&flightLog->stats);

    fprintf(stderr, "\nNew statistics\n");
    printStats(&encodedStats);

    printFrameSizeComparison(&flightLog->stats, &encodedStats);

    flightLogDestroy(flightLog);

    return 0;
}
