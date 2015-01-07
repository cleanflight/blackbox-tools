#ifndef PARSER_H_
#define PARSER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "blackbox_fielddefs.h"

#define FLIGHT_LOG_MAX_LOGS_IN_FILE 31
#define FLIGHT_LOG_MAX_FIELDS 128
#define FLIGHT_LOG_MAX_FRAME_LENGTH 256

#define FLIGHT_LOG_FIELD_INDEX_ITERATION 0
#define FLIGHT_LOG_FIELD_INDEX_TIME 1

typedef enum FirmwareType {
	FIRMWARE_TYPE_BASEFLIGHT = 0,
	FIRMWARE_TYPE_CLEANFLIGHT
} FirmwareType;

typedef struct flightLogFrameStatistics_t {
    uint32_t bytes;
    uint32_t validCount, desyncCount, corruptCount;
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

	flightLogFieldStatistics_t field[FLIGHT_LOG_MAX_FIELDS];
	flightLogFrameStatistics_t frame[256];
} flightLogStatistics_t;

struct flightLogPrivate_t;

typedef struct flightLog_t {
	flightLogStatistics_t stats;

	int minthrottle, maxthrottle;
	unsigned int rcRate, yawRate;

	// Calibration constants from the hardware sensors:
	uint16_t acc_1G;
	float gyroScale;

    uint8_t vbatscale;
    uint8_t vbatmaxcellvoltage;
    uint8_t vbatmincellvoltage;
    uint8_t vbatwarningcellvoltage;

    uint16_t vbatref;

	FirmwareType firmwareType;

	//Information about log sections:
	const char *logBegin[FLIGHT_LOG_MAX_LOGS_IN_FILE + 1];
	int logCount;

	unsigned int frameIntervalI;
	unsigned int frameIntervalPNum, frameIntervalPDenom;

	int mainFieldSigned[FLIGHT_LOG_MAX_FIELDS];
	int gpsFieldSigned[FLIGHT_LOG_MAX_FIELDS];

	int mainFieldCount;
	char *mainFieldNames[FLIGHT_LOG_MAX_FIELDS];

	int gpsFieldCount;
	char *gpsFieldNames[FLIGHT_LOG_MAX_FIELDS];

	struct flightLogPrivate_t *private;
} flightLog_t;

typedef struct flightLogEvent_syncBeep_t {
    uint32_t time;
} flightLogEvent_syncBeep_t;

typedef union flightLogEventData_t
{
    flightLogEvent_syncBeep_t syncBeep;
} flightLogEventData_t;

typedef struct flightLogEvent_t
{
    FlightLogEvent event;
    flightLogEventData_t data;
} flightLogEvent_t;

typedef void (*FlightLogMetadataReady)(flightLog_t *log);
typedef void (*FlightLogFrameReady)(flightLog_t *log, bool frameValid, int32_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize);
typedef void (*FlightLogEventReady)(flightLog_t *log, flightLogEvent_t *event);

flightLog_t* flightLogCreate(int fd);

int flightLogEstimateNumCells(flightLog_t *log);
unsigned int flightLogVbatToMillivolts(flightLog_t *log, uint16_t vbat);

bool flightLogParse(flightLog_t *log, int logIndex, FlightLogMetadataReady onMetadataReady, FlightLogFrameReady onFrameReady, FlightLogEventReady onEvent, bool raw);
void flightLogDestroy(flightLog_t *log);

#endif
