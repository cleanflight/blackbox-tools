#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//For msvcrt to define M_PI:
#define _USE_MATH_DEFINES
#include <math.h>

#include <sys/stat.h>

// Support for memory-mapped files:
#ifdef WIN32
	#include <windows.h>
	#include <io.h>
#else
	// Posix-y systems:
	#include <sys/mman.h>
#endif

#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "parser.h"
#include "tools.h"

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
	char *fieldNamesCombined;

	//Information about fields which we need to decode them properly
	flightLogFrameDefs_t frameDefs[256];

	int dataVersion;
	int motor0Index;

	// Blackbox state:
	int32_t blackboxHistoryRing[3][FLIGHT_LOG_MAX_FIELDS];
	int32_t* mainHistory[3];
	flightLogEvent_t lastEvent;

	bool mainStreamIsValid;

	// Event handlers:
	FlightLogMetadataReady onMetadataReady;
	FlightLogFrameReady onFrameReady;
	FlightLogEventReady onEvent;

	// Log data stream:
	int fd;

	//The start of the entire log file:
	const char *logData;

	//The section of the file which is currently being examined:
	const char *logStart, *logEnd, *logPos;

	//Set to true if we attempt to read from the log when it is already exhausted
	bool eof;
} flightLogPrivate_t;

typedef void (*FlightLogFrameParse)(flightLog_t *log, bool raw);
typedef void (*FlightLogFrameComplete)(flightLog_t *log, char frameType, const char *frameStart, const char *frameEnd, bool raw);

typedef struct flightLogFrameType_t {
    uint8_t marker;
    FlightLogFrameParse parse;
    FlightLogFrameComplete complete;
} flightLogFrameType_t;

static void parseIntraframe(flightLog_t *log, bool raw);
static void parseInterframe(flightLog_t *log, bool raw);
static void parseGPSFrame(flightLog_t *log, bool raw);
static void parseGPSHomeFrame(flightLog_t *log, bool raw);
static void parseEventFrame(flightLog_t *log, bool raw);

static void completeIntraframe(flightLog_t *log, char frameType, const char *frameStart, const char *frameEnd, bool raw);
static void completeInterframe(flightLog_t *log, char frameType, const char *frameStart, const char *frameEnd, bool raw);
static void completeEventFrame(flightLog_t *log, char frameType, const char *frameStart, const char *frameEnd, bool raw);

static const flightLogFrameType_t frameTypes[] = {
    {.marker = 'I', .parse = parseIntraframe,   .complete = completeIntraframe},
    {.marker = 'P', .parse = parseInterframe,   .complete = completeInterframe},
    {.marker = 'G', .parse = parseGPSFrame,     .complete = 0},
    {.marker = 'H', .parse = parseGPSHomeFrame, .complete = 0},
    {.marker = 'E', .parse = parseEventFrame,   .complete = completeEventFrame}
};

static int readChar(flightLog_t *log)
{
	if (log->private->logPos < log->private->logEnd) {
		int result = (uint8_t) *log->private->logPos;
		log->private->logPos++;
		return result;
	}

	log->private->eof = true;
	return EOF;
}

static void unreadChar(flightLog_t *log, int c)
{
	(void) c;

	log->private->logPos--;
}

static void parseFieldNames(flightLog_t *log, const char *names)
{
	char *start, *end;
	bool done = false;

	log->private->fieldNamesCombined = strdup(names);

	start = log->private->fieldNamesCombined;

	while (!done && *start) {
		end = start;

		do {
			end++;
		} while (*end != ',' && *end != 0);

		log->mainFieldNames[log->mainFieldCount++] = start;

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

static void parseHeaderLine(flightLog_t *log)
{
	char *fieldName, *fieldValue;
	const char *lineStart, *lineEnd, *separatorPos;
	int i, c;
	char valueBuffer[1024];
	union {
		float f;
		uint32_t u;
	} floatConvert;

	if (*log->private->logPos != ' ')
		return;

	//Skip the space
	log->private->logPos++;

	lineStart = log->private->logPos;
	separatorPos = 0;

	for (i = 0; i < 1024; i++) {
		c = readChar(log);

		if (c == ':' && !separatorPos)
			separatorPos = log->private->logPos - 1;

		if (c == '\n')
			break;

		if (c == EOF || c == '\0')
			// Line ended before we saw a newline or it has binary stuff in there that shouldn't be there
			return;
	}

	if (!separatorPos)
		return;

	lineEnd = log->private->logPos;

	//Make a duplicate copy of the line so we can null-terminate the two parts
	memcpy(valueBuffer, lineStart, lineEnd - lineStart);

	fieldName = valueBuffer;
	valueBuffer[separatorPos - lineStart] = '\0';

	fieldValue = valueBuffer + (separatorPos - lineStart) + 1;
	valueBuffer[lineEnd - lineStart - 1] = '\0';

	if (strcmp(fieldName, "Field I name") == 0) {
		parseFieldNames(log, fieldValue);

		for (i = 0; i < log->mainFieldCount; i++) {
			if (strcmp(log->mainFieldNames[i], "motor[0]") == 0) {
				log->private->motor0Index = i;
				break;
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

static uint32_t readUnsignedVB(flightLog_t *log)
{
	int i, c, shift = 0;
	uint32_t result = 0;

	// 5 bytes is enough to encode 32-bit unsigned quantities
	for (i = 0; i < 5; i++) {
		c = readChar(log);

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

static int32_t readSignedVB(flightLog_t *log)
{
	uint32_t i = readUnsignedVB(log);

	// Apply ZigZag decoding to recover the signed value
	return (i >> 1) ^ -(int32_t) (i & 1);
}

static void readTag2_3S32(flightLog_t *log, int32_t *values)
{
	uint8_t leadByte;
	uint8_t byte1, byte2, byte3, byte4;
	int i;

	leadByte = readChar(log);

	// Check the selector in the top two bits to determine the field layout
	switch (leadByte >> 6) {
		case 0:
			// 2-bit fields
			values[0] = signExtend2Bit((leadByte >> 4) & 0x03);
			values[1] = signExtend2Bit((leadByte >> 2) & 0x03);
			values[2] = signExtend2Bit(leadByte & 0x03);
		break;
		case 1:
			// 4-bit fields
			values[0] = signExtend4Bit(leadByte & 0x0F);

			leadByte = readChar(log);

			values[1] = signExtend4Bit(leadByte >> 4);
			values[2] = signExtend4Bit(leadByte & 0x0F);
		break;
		case 2:
			// 6-bit fields
			values[0] = signExtend6Bit(leadByte & 0x3F);

			leadByte = readChar(log);
			values[1] = signExtend6Bit(leadByte & 0x3F);

			leadByte = readChar(log);
			values[2] = signExtend6Bit(leadByte & 0x3F);
		break;
		case 3:
			// Fields are 8, 16 or 24 bits, read selector to figure out which field is which size

			for (i = 0; i < 3; i++) {
				switch (leadByte & 0x03) {
					case 0: // 8-bit
						byte1 = readChar(log);

						// Sign extend to 32 bits
						values[i] = (int32_t) (int8_t) (byte1);
					break;
					case 1: // 16-bit
						byte1 = readChar(log);
						byte2 = readChar(log);

						// Sign extend to 32 bits
						values[i] = (int32_t) (int16_t) (byte1 | (byte2 << 8));
					break;
					case 2: // 24-bit
						byte1 = readChar(log);
						byte2 = readChar(log);
						byte3 = readChar(log);

						values[i] = signExtend24Bit(byte1 | (byte2 << 8) | (byte3 << 16));
					break;
					case 3: // 32-bit
						byte1 = readChar(log);
						byte2 = readChar(log);
						byte3 = readChar(log);
						byte4 = readChar(log);

						values[i] = (int32_t) (byte1 | (byte2 << 8) | (byte3 << 16) | (byte4 << 24));
					break;
				}

				leadByte >>= 2;
			}
		break;
	}
}

static void readTag8_4S16_v1(flightLog_t *log, int32_t *values)
{
	uint8_t selector, combinedChar;
	uint8_t char1, char2;
	int i;

	enum {
		FIELD_ZERO  = 0,
		FIELD_4BIT  = 1,
		FIELD_8BIT  = 2,
		FIELD_16BIT = 3
	};

	selector = readChar(log);

	//Read the 4 values from the stream
	for (i = 0; i < 4; i++) {
		switch (selector & 0x03) {
			case FIELD_ZERO:
				values[i] = 0;
			break;
			case FIELD_4BIT: // Two 4-bit fields
				combinedChar = (uint8_t) readChar(log);

				values[i] = signExtend4Bit(combinedChar & 0x0F);

				i++;
				selector >>= 2;

				values[i] = signExtend4Bit(combinedChar >> 4);
			break;
			case FIELD_8BIT: // 8-bit field
				//Sign extend...
				values[i] = (int32_t) (int8_t) readChar(log);
			break;
			case FIELD_16BIT: // 16-bit field
				char1 = readChar(log);
				char2 = readChar(log);

				//Sign extend...
				values[i] = (int16_t) (char1 | (char2 << 8));
			break;
		}

		selector >>= 2;
	}
}

static void readTag8_4S16_v2(flightLog_t *log, int32_t *values)
{
	uint8_t selector;
	uint8_t char1, char2;
	uint8_t buffer;
	int nibbleIndex;

	int i;

	enum {
		FIELD_ZERO  = 0,
		FIELD_4BIT  = 1,
		FIELD_8BIT  = 2,
		FIELD_16BIT = 3
	};

	selector = readChar(log);

	//Read the 4 values from the stream
	nibbleIndex = 0;
	for (i = 0; i < 4; i++) {
		switch (selector & 0x03) {
			case FIELD_ZERO:
				values[i] = 0;
			break;
			case FIELD_4BIT:
				if (nibbleIndex == 0) {
					buffer = (uint8_t) readChar(log);
					values[i] = signExtend4Bit(buffer >> 4);
					nibbleIndex = 1;
				} else {
					values[i] = signExtend4Bit(buffer & 0x0F);
					nibbleIndex = 0;
				}
			break;
			case FIELD_8BIT:
				if (nibbleIndex == 0) {
					//Sign extend...
					values[i] = (int32_t) (int8_t) readChar(log);
				} else {
					char1 = buffer << 4;
					buffer = (uint8_t) readChar(log);

					char1 |= buffer >> 4;
					values[i] = (int32_t) (int8_t) char1;
				}
			break;
			case FIELD_16BIT:
				if (nibbleIndex == 0) {
					char1 = (uint8_t) readChar(log);
					char2 = (uint8_t) readChar(log);

					//Sign extend...
					values[i] = (int16_t) (uint16_t) ((char1 << 8) | char2);
				} else {
					/*
					 * We're in the low 4 bits of the current buffer, then one byte, then the high 4 bits of the next
					 * buffer.
					 */
					char1 = (uint8_t) readChar(log);
					char2 = (uint8_t) readChar(log);

					values[i] = (int16_t) (uint16_t) ((buffer << 12) | (char1 << 4) | (char2 >> 4));

					buffer = char2;
				}
			break;
		}

		selector >>= 2;
	}
}

static void readTag8_8SVB(flightLog_t *log, int32_t *values, int valueCount)
{
    uint8_t header;

    if (valueCount == 1) {
        values[0] = readSignedVB(log);
    } else {
        header = (uint8_t) readChar(log);

        for (int i = 0; i < 8; i++, header >>= 1)
            values[i] = (header & 0x01) ? readSignedVB(log) : 0;
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
 * Attempt to parse the Intraframe at the current log position into the history buffer at mainHistory[0].
 */
static void parseIntraframe(flightLog_t *log, bool raw)
{
    flightLogPrivate_t *private = log->private;
	int i;
	int *current, *previous;

	current = private->mainHistory[0];
	previous = private->mainHistory[1];

	if (previous) {
        for (uint32_t frameIndex = previous[FLIGHT_LOG_FIELD_INDEX_ITERATION] + 1; !shouldHaveFrame(log, frameIndex); frameIndex++) {
            log->stats.intentionallyAbsentIterations++;
        }
    }

	for (i = 0; i < log->mainFieldCount; i++) {
		uint32_t value;

		switch (private->frameDefs['I'].encoding[i]) {
			case FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB:
				value = (uint32_t) readSignedVB(log);
			break;
			case FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB:
				value = readUnsignedVB(log);
			break;
			case FLIGHT_LOG_FIELD_ENCODING_NEG_14BIT:
			    value = (uint32_t) -signExtend14Bit(readUnsignedVB(log));
			break;
			default:
				fprintf(stderr, "Unsupported I-field encoding %d\n", private->frameDefs['I'].encoding[i]);
				exit(-1);
		}

		if (!raw) {
			//Not many predictors can be used in I-frames since they can't depend on any other frame
			switch (private->frameDefs['I'].predictor[i]) {
				case FLIGHT_LOG_FIELD_PREDICTOR_0:
					//No-op
				break;
				case FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE:
					value += log->minthrottle;
				break;
				case FLIGHT_LOG_FIELD_PREDICTOR_1500:
					value += 1500;
				break;
				case FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0:
					if (private->motor0Index < 0) {
						fprintf(stderr, "Attempted to base I-field prediction on motor0 before it was read\n");
						exit(-1);
					}
					value += (uint32_t) current[private->motor0Index];
				break;
				case FLIGHT_LOG_FIELD_PREDICTOR_VBATREF:
				    value += log->vbatref;
				break;
				default:
					fprintf(stderr, "Unsupported I-field predictor %d\n", private->frameDefs['I'].predictor[i]);
					exit(-1);
			}
		}

		current[i] = (int32_t) value;
	}
}

/**
 * Take the raw value for an inter-field, apply the prediction that is configured for it, and return it.
 */
static int32_t applyInterPrediction(flightLog_t *log, int fieldIndex, int predictor, uint32_t value)
{
    int32_t *previous = log->private->mainHistory[1];
    int32_t *previous2 = log->private->mainHistory[2];

    /*
     * If we don't have a previous state to refer to (e.g. because previous frame was corrupt) don't apply
     * a prediction. Instead just show the raw values.
     */
    if (!previous)
        predictor = FLIGHT_LOG_FIELD_PREDICTOR_0;

	switch (predictor) {
		case FLIGHT_LOG_FIELD_PREDICTOR_0:
			// No correction to apply
		break;
		case FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS:
			value += (uint32_t) previous[fieldIndex];
		break;
		case FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE:
			value += 2 * (uint32_t) previous[fieldIndex] - (uint32_t) previous2[fieldIndex];
		break;
		case FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2:
			if (log->mainFieldSigned[fieldIndex])
				value += (uint32_t) ((int32_t) ((uint32_t) previous[fieldIndex] + (uint32_t) previous2[fieldIndex]) / 2);
			else
				value += ((uint32_t) previous[fieldIndex] + (uint32_t) previous2[fieldIndex]) / 2;
		break;
		default:
			fprintf(stderr, "Unsupported P-field predictor %d\n", predictor);
			exit(-1);
	}

	return (int32_t) value;
}

/**
 * Attempt to parse the interframe at the current log position into the history buffer at mainHistory[0].
 */
static void parseInterframe(flightLog_t *log, bool raw)
{
	int i, j;
	int groupCount;

	int32_t *current = log->private->mainHistory[0];
	int32_t *previous = log->private->mainHistory[1];

	uint32_t frameIndex;
	uint32_t skippedFrames = 0;

	if (previous) {
        //Work out how many frames we skipped to get to this one, based on the log sampling rate
        for (frameIndex = previous[FLIGHT_LOG_FIELD_INDEX_ITERATION] + 1; !shouldHaveFrame(log, frameIndex); frameIndex++)
            skippedFrames++;
    }

	log->stats.intentionallyAbsentIterations += skippedFrames;

	i = 0;
	while (i < log->mainFieldCount) {
		uint32_t value;
		uint32_t values[8];

		if (log->private->frameDefs['P'].predictor[i] == FLIGHT_LOG_FIELD_PREDICTOR_INC) {
		    current[i] = skippedFrames + 1;

		    if (previous)
		        current[i] += previous[i];

	        i++;
		} else {
			switch (log->private->frameDefs['P'].encoding[i]) {
				case FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB:
					value = (uint32_t) readSignedVB(log);
				break;
				case FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB:
					value = readUnsignedVB(log);
				break;
				case FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16:
					if (log->private->dataVersion < 2)
						readTag8_4S16_v1(log, (int32_t*)values);
					else
						readTag8_4S16_v2(log, (int32_t*)values);

					//Apply the predictors for the fields:
					for (j = 0; j < 4; j++, i++)
					    current[i] = applyInterPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : log->private->frameDefs['P'].predictor[i], values[j]);

					continue;
				break;
				case FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32:
					readTag2_3S32(log, (int32_t*)values);

					//Apply the predictors for the fields:
					for (j = 0; j < 3; j++, i++)
					    current[i] = applyInterPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : log->private->frameDefs['P'].predictor[i], values[j]);

					continue;
				break;
				case FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB:
				    //How many fields are in this encoded group? Check the subsequent field encodings:
				    for (j = i + 1; j < i + 8 && j < log->mainFieldCount; j++)
				        if (log->private->frameDefs['P'].encoding[j] != FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB)
				            break;

				    groupCount = j - i;

				    readTag8_8SVB(log, (int32_t*) values, groupCount);

				    for (j = 0; j < groupCount; j++, i++)
                        current[i] = applyInterPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : log->private->frameDefs['P'].predictor[i], values[j]);

                    continue;
				break;
				case FLIGHT_LOG_FIELD_ENCODING_NULL:
				    //Nothing to read
				    value = 0;
				break;
				default:
					fprintf(stderr, "Unsupported P-field encoding %d\n", log->private->frameDefs['P'].encoding[i]);
					exit(-1);
			}

			current[i] = applyInterPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : log->private->frameDefs['P'].predictor[i], value);
			i++;
		}
	}
}

/**
 * TODO
 */
static void parseGPSFrame(flightLog_t *log, bool raw)
{
	(void) raw;

	readUnsignedVB(log);
	readSignedVB(log);
	readSignedVB(log);
	readUnsignedVB(log);
	readUnsignedVB(log);
}

/**
 * TODO
 */
static void parseGPSHomeFrame(flightLog_t *log, bool raw)
{
	(void) raw;

	readSignedVB(log);
	readSignedVB(log);
}

/**
 * Attempt to parse an event frame at the current location into the log->private->lastEvent struct.
 * Return false if the event couldn't be parsed (e.g. unknown event ID), or true if it might have been
 * parsed successfully.
 */
static void parseEventFrame(flightLog_t *log, bool raw)
{
    (void) raw;

    uint8_t eventType = readChar(log);

    flightLogEventData_t *data = &log->private->lastEvent.data;
    log->private->lastEvent.event = eventType;

    switch (eventType) {
        case FLIGHT_LOG_EVENT_SYNC_BEEP:
            data->syncBeep.time =readUnsignedVB(log);
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

/**
 * Just like strstr, but for binary strings. Not available on all platforms, so reimplemented here.
 */
void* memmem(const void *haystack, size_t haystackLen, const void *needle, size_t needleLen)
{
	if (needleLen <= haystackLen) {
		const char* c_haystack = (char*)haystack;
		const char* c_needle = (char*)needle;

		for (const char *pos = c_haystack; pos <= c_haystack + haystackLen - needleLen; pos++) {
			if (*pos == *c_needle && memcmp(pos, c_needle, needleLen) == 0)
				return (void*)pos;
		}
	}

	return NULL;
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
	const char *mapped;
	struct stat stats;
	size_t fileSize;

	const char *logSearchStart;
	int logIndex;

	flightLog_t *log;
	flightLogPrivate_t *private;

	// Map the log into memory
	if (fd < 0 || fstat(fd, &stats) < 0) {
		fprintf(stderr, "Failed to use log file: %s\n", strerror(errno));
		return 0;
	}

	fileSize = stats.st_size;

	if (fileSize == 0) {
		fprintf(stderr, "Error: This log is zero-bytes long!\n");
		return 0;
	}

#ifdef WIN32
	intptr_t fileHandle = _get_osfhandle(fd);
	HANDLE mapping = CreateFileMapping((HANDLE) fileHandle, NULL, PAGE_READONLY, 0, 0, NULL);

	if (mapping == NULL) {
		fprintf(stderr, "Failed to map log file into memory\n");
		return 0;
	}

	mapped = MapViewOfFile(mapping, FILE_MAP_READ, 0, 0, fileSize);

	if (mapped == NULL) {
		fprintf(stderr, "Failed to map log file into memory: %s\n", strerror(errno));

		return 0;
	}
#else
	mapped = mmap(0, fileSize, PROT_READ, MAP_PRIVATE, fd, 0);

	if (mapped == MAP_FAILED) {
		fprintf(stderr, "Failed to map log file into memory: %s\n", strerror(errno));

		return 0;
	}
#endif

	log = (flightLog_t *) malloc(sizeof(*log));
	private = (flightLogPrivate_t *) malloc(sizeof(*private));

	memset(log, 0, sizeof(*log));
	memset(private, 0, sizeof(*private));

    //First check how many logs are in this one file (each time the FC is rearmed, a new log is appended)
    logSearchStart = mapped;

    for (logIndex = 0; logIndex < FLIGHT_LOG_MAX_LOGS_IN_FILE && logSearchStart < mapped + fileSize; logIndex++) {
		log->logBegin[logIndex] = memmem(logSearchStart, (mapped + fileSize) - logSearchStart, LOG_START_MARKER, strlen(LOG_START_MARKER));

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
	log->logBegin[log->logCount] = mapped + fileSize;

	private->logData = mapped;
    private->fd = fd;

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

static void completeIntraframe(flightLog_t *log, char frameType, const char *frameStart, const char *frameEnd, bool raw)
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
        log->private->onFrameReady(log, private->mainStreamIsValid, private->mainHistory[0], frameType, log->mainFieldCount, frameStart - private->logData, frameEnd - frameStart);

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

static void completeInterframe(flightLog_t *log, char frameType, const char *frameStart, const char *frameEnd, bool raw)
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
        log->private->onFrameReady(log, private->mainStreamIsValid, private->mainHistory[0], frameType, log->mainFieldCount, frameStart - private->logData, frameEnd - frameStart);

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

static void completeEventFrame(flightLog_t *log, char frameType, const char *frameStart, const char *frameEnd, bool raw)
{
    (void) frameType;
    (void) frameStart;
    (void) frameEnd;
    (void) raw;

    if (log->private->onEvent)
        log->private->onEvent(log, &log->private->lastEvent);
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
	free(log->private->fieldNamesCombined);
	log->private->fieldNamesCombined = NULL;
	log->mainFieldCount = 0;
	log->gpsFieldCount = 0;
	flightLoginvalidateStream(log);

	log->private->mainHistory[0] = log->private->blackboxHistoryRing[0];
    log->private->mainHistory[1] = NULL;
    log->private->mainHistory[2] = NULL;

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
	private->lastEvent.event = -1;

	private->onMetadataReady = onMetadataReady;
	private->onFrameReady = onFrameReady;
	private->onEvent = onEvent;

	//Set parsing ranges up for the log the caller selected
	private->logStart = log->logBegin[logIndex];
    private->logPos = private->logStart;
    private->logEnd = log->logBegin[logIndex + 1];
    private->eof = false;

	while (1) {
		int command = readChar(log);

		switch (parserState) {
			case PARSER_STATE_HEADER:
				switch (command) {
					case 'H':
						parseHeaderLine(log);
					break;
					case EOF:
						fprintf(stderr, "Data file contained no events\n");
						return false;
					default:
					    frameType = getFrameType(command);

					    if (frameType) {
                            unreadChar(log, command);

                            if (log->mainFieldCount == 0) {
                                fprintf(stderr, "Data file is missing field name definitions\n");
                                return false;
                            }

                            parserState = PARSER_STATE_DATA;
                            lastFrameType = NULL;
                            frameStart = private->logPos;

                            if (onMetadataReady)
                                onMetadataReady(log);
                        } // else skip garbage which apparently precedes the first data frame
                    break;
				}
			break;
			case PARSER_STATE_DATA:
				if (lastFrameType) {
					unsigned int lastFrameSize = private->logPos - frameStart;

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
                            lastFrameType->complete(log, lastFrameType->marker, frameStart, private->logPos, raw);

					} else {
						//The previous frame was corrupt

					    //We need to resynchronise before we can deliver another main frame:
                        log->private->mainStreamIsValid = false;
                        log->stats.frame[lastFrameType->marker].corruptCount++;
                        log->stats.totalCorruptFrames++;

                        //Let the caller know there was a corrupt frame (don't give them a pointer to the frame data because it is totally worthless)
                        if (onFrameReady)
                            onFrameReady(log, false, 0, lastFrameType->marker, 0, frameStart - private->logData, lastFrameSize);

                        /*
                         * Start the search for a frame beginning after the first byte of the previous corrupt frame.
                         * This way we can find the start of the next frame after the corrupt frame if the corrupt frame
                         * was truncated.
                         */
                        private->logPos = frameStart;
                        lastFrameType = NULL;
                        prematureEof = false;
                        private->eof = false;
                        continue;
                    }
				}

                if (command == EOF)
                    goto done;

				frameType = getFrameType((uint8_t) command);
				frameStart = private->logPos;

				if (frameType) {
				    frameType->parse(log, raw);
				} else {
				    private->mainStreamIsValid = false;
				}

				//We shouldn't read an EOF during reading a frame (that'd imply the frame was truncated)
				if (private->eof)
					prematureEof = true;

                lastFrameType = frameType;
			break;
		}
	}

	done:
	log->stats.totalBytes = private->logEnd - private->logStart;

	return true;
}

void flightLogDestroy(flightLog_t *log)
{
#ifdef WIN32
	UnmapViewOfFile(log->private->logData);
#else
	munmap((void*)log->private->logData, log->private->logEnd - log->private->logData);
#endif

	free(log->private->fieldNamesCombined);
	free(log->private);
	free(log);

	//TODO clean up mainFieldNames
}
