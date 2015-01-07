/**
 * This tool reads in a flight log and re-encodes it using a private copy of the encoder. This allows experiments
 * to be run on improving the encoder's efficiency, and allows any changes to the encoder to be verified (by comparing
 * decoded logs against the ones produced original encoder).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WIN32
	#include "getopt.h"
#else
	#include <getopt.h>
#endif

#include "parser.h"

#define MAG
#define BARO
#define XYZ_AXIS_COUNT 3
#define MAX_MOTORS 8
#define MAX_SERVOS 8

#define BLACKBOX_I_INTERVAL 32

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

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
	"H Blackbox version:1\n"
	"H Data version:2\n"
	"H I interval:" STR(BLACKBOX_I_INTERVAL) "\n";

static const char* const blackboxMainHeaderNames[] = {
    "I name",
    "I signed",
    "I predictor",
    "I encoding",
    "P predictor",
    "P encoding"
};

/* All field definition structs should look like this (but with longer arrs): */
typedef struct blackboxFieldDefinition_t {
    const char *name;
    uint8_t arr[1];
} blackboxFieldDefinition_t;

typedef struct blackboxMainFieldDefinition_t {
    const char *name;
    uint8_t isSigned;
    uint8_t Ipredict;
    uint8_t Iencode;
    uint8_t Ppredict;
    uint8_t Pencode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxMainFieldDefinition_t;

/**
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
static const blackboxMainFieldDefinition_t blackboxMainFields[] = {
    /* loopIteration doesn't appear in P frames since it always increments */
    {"loopIteration", UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(INC),           .Pencode = FLIGHT_LOG_FIELD_ENCODING_NULL, CONDITION(ALWAYS)},
    /* Time advances pretty steadily so the P-frame prediction is a straight line */
    {"time",          UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(STRAIGHT_LINE), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP[0]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP[1]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP[2]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    /* I terms get special packed encoding in P frames: */
    {"axisI[0]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisI[1]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisI[2]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisD[0]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_0)},
    {"axisD[1]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_1)},
    {"axisD[2]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_2)},
    /* rcCommands are encoded together as a group in P-frames: */
    {"rcCommand[0]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"rcCommand[1]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"rcCommand[2]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    /* Throttle is always in the range [minthrottle..maxthrottle]: */
    {"rcCommand[3]",  UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},

    {"vbatLatest",    UNSIGNED, .Ipredict = PREDICT(VBATREF), .Iencode = ENCODING(NEG_14BIT), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(ALWAYS)},
#ifdef MAG
    {"magADC[0]",     SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
    {"magADC[1]",     SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
    {"magADC[2]",     SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
#endif
#ifdef BARO
    {"BaroAlt",       SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_BARO},
#endif

    /* Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact */
    {"gyroData[0]",   SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroData[1]",   SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroData[2]",   SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth[0]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth[1]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth[2]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    /* Motors only rarely drops under minthrottle (when stick falls below mincommand), so predict minthrottle for it and use *unsigned* encoding (which is large for negative numbers but more compact for positive ones): */
    {"motor[0]",      UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_1)},
    /* Subsequent motors base their I-frame values on the first one, P-frame values on the average of last two frames: */
    {"motor[1]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_2)},
    {"motor[2]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_3)},
    {"motor[3]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_4)},
    {"motor[4]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_5)},
    {"motor[5]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_6)},
    {"motor[6]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_7)},
    {"motor[7]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_8)},
    {"servo[5]",      UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(TRICOPTER)}
};

typedef struct blackboxValues_t {
	uint32_t time;

	int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];

	int16_t rcCommand[4];
	int16_t gyroData[3];
	int16_t accSmooth[3];
	int16_t motor[MAX_MOTORS];
	int16_t servo[MAX_SERVOS];
	uint16_t vbatLatest;

    int32_t BaroAlt;
    int16_t magADC[XYZ_AXIS_COUNT];
} blackboxValues_t;

typedef struct mcfgStandin_t {
	uint16_t minthrottle, maxthrottle;
} mcfgStandin_t;

// Simulation of data that mw.c would normally provide:
mcfgStandin_t mcfg = {
	.minthrottle = 1150, .maxthrottle = 1850
};

uint16_t vbatReference = 4095;
int numberMotor = 0;

// Program options
int optionDebug;
char *optionFilename = 0;

uint32_t blackboxIteration, writtenBytes;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxValues_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackboxValues_t* blackboxHistory[3];

flightLogStatistics_t encodedStats;

void blackboxWrite(uint8_t ch)
{
	putc(ch, stdout);

	writtenBytes++;
}

// Print the null-terminated string 's' to the serial port and return the number of bytes written
static int blackboxPrint(const char *s)
{
    const char *pos = s;

    while (*pos) {
        blackboxWrite(*pos);
        pos++;
    }

    return pos - s;
}

//printf() to the blackbox serial port with no blocking shenanigans (so it's caller's responsibility to not write too fast!)
static void blackboxPrintf(char *format, ...)
{
    char buffer[512];

    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);

    blackboxPrint(buffer);

    va_end(args);
}

/**
 * Write an unsigned integer to the blackbox serial port using variable byte encoding.
 */
static void writeUnsignedVB(uint32_t value)
{
	//While this isn't the final byte (we can only write 7 bits at a time)
	while (value > 127) {
		blackboxWrite((uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
		value >>= 7;
	}
	blackboxWrite(value);
}

/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
static void writeSignedVB(int32_t value)
{
	//ZigZag encode to make the value always positive
	writeUnsignedVB((uint32_t)((value << 1) ^ (value >> 31)));
}

/**
 * Write a 2 bit tag followed by 3 signed fields of 2, 4, 6 or 32 bits
 */
static void writeTag2_3S32(int32_t *values) {
	static const int NUM_FIELDS = 3;

	//Need to be enums rather than const ints if we want to switch on them (due to being C)
	enum {
		BITS_2  = 0,
		BITS_4  = 1,
		BITS_6  = 2,
		BITS_32 = 3
	};

	enum {
		BYTES_1  = 0,
		BYTES_2  = 1,
		BYTES_3  = 2,
		BYTES_4  = 3
	};

	int x;
	int selector = BITS_2, selector2;

	/*
	 * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
	 * below:
	 *
	 * Selector possibilities
	 *
	 * 2 bits per field  ss11 2233,
	 * 4 bits per field  ss00 1111 2222 3333
	 * 6 bits per field  ss11 1111 0022 2222 0033 3333
	 * 32 bits per field sstt tttt followed by fields of various byte counts
	 */
	for (x = 0; x < NUM_FIELDS; x++) {
		//Require more than 6 bits?
		if (values[x] >= 32 || values[x] < -32) {
			selector = BITS_32;
			break;
		}

		//Require more than 4 bits?
		if (values[x] >= 8 || values[x] < -8) {
			 if (selector < BITS_6)
				 selector = BITS_6;
		} else if (values[x] >= 2 || values[x] < -2) { //Require more than 2 bits?
			if (selector < BITS_4)
				selector = BITS_4;
		}
	}

	switch (selector) {
		case BITS_2:
			blackboxWrite((selector << 6) | ((values[0] & 0x03) << 4) | ((values[1] & 0x03) << 2) | (values[2] & 0x03));
		break;
		case BITS_4:
			blackboxWrite((selector << 6) | (values[0] & 0x0F));
			blackboxWrite((values[1] << 4) | (values[2] & 0x0F));
		break;
		case BITS_6:
			blackboxWrite((selector << 6) | (values[0] & 0x3F));
			blackboxWrite((uint8_t)values[1]);
			blackboxWrite((uint8_t)values[2]);
		break;
		case BITS_32:
			/*
			 * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
			 *
			 * Selector2 field possibilities
			 * 0 - 8 bits
			 * 1 - 16 bits
			 * 2 - 24 bits
			 * 3 - 32 bits
			 */
			selector2 = 0;

			//Encode in reverse order so the first field is in the low bits:
			for (x = NUM_FIELDS - 1; x >= 0; x--) {
				selector2 <<= 2;

				if (values[x] < 128 && values[x] >= -128)
					selector2 |= BYTES_1;
				else if (values[x] < 32768 && values[x] >= -32768)
					selector2 |= BYTES_2;
				else if (values[x] < 8388608 && values[x] >= -8388608)
					selector2 |= BYTES_3;
				else
					selector2 |= BYTES_4;
			}

			//Write the selectors
			blackboxWrite((selector << 6) | selector2);

			//And now the values according to the selectors we picked for them
			for (x = 0; x < NUM_FIELDS; x++, selector2 >>= 2) {
				switch (selector2 & 0x03) {
					case BYTES_1:
						blackboxWrite(values[x]);
					break;
					case BYTES_2:
						blackboxWrite(values[x]);
						blackboxWrite(values[x] >> 8);
					break;
					case BYTES_3:
						blackboxWrite(values[x]);
						blackboxWrite(values[x] >> 8);
						blackboxWrite(values[x] >> 16);
					break;
					case BYTES_4:
						blackboxWrite(values[x]);
						blackboxWrite(values[x] >> 8);
						blackboxWrite(values[x] >> 16);
						blackboxWrite(values[x] >> 24);
					break;
				}
			}
		break;
	}
}

/**
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
static void writeTag8_4S16(int32_t *values) {

    //Need to be enums rather than const ints if we want to switch on them (due to being C)
    enum {
    	FIELD_ZERO  = 0,
    	FIELD_4BIT  = 1,
    	FIELD_8BIT  = 2,
    	FIELD_16BIT = 3
	};

    uint8_t selector, buffer;
    int nibbleIndex;
    int x;

    selector = 0;
    //Encode in reverse order so the first field is in the low bits:
    for (x = 3; x >= 0; x--) {
        selector <<= 2;

        if (values[x] == 0)
            selector |= FIELD_ZERO;
        else if (values[x] < 8 && values[x] >= -8)
            selector |= FIELD_4BIT;
        else if (values[x] < 128 && values[x] >= -128)
            selector |= FIELD_8BIT;
        else
            selector |= FIELD_16BIT;
    }

    blackboxWrite(selector);

    nibbleIndex = 0;
    buffer = 0;
    for (x = 0; x < 4; x++, selector >>= 2) {
        switch (selector & 0x03) {
        	case FIELD_ZERO:
        		//No-op
        	break;
            case FIELD_4BIT:
            	if (nibbleIndex == 0) {
            		//We fill high-bits first
            		buffer = values[x] << 4;
            		nibbleIndex = 1;
            	} else {
            		blackboxWrite(buffer | (values[x] & 0x0F));
            		nibbleIndex = 0;
            	}
            break;
            case FIELD_8BIT:
            	if (nibbleIndex == 0)
            		blackboxWrite(values[x]);
            	else {
            		//Write the high bits of the value first (mask to avoid sign extension)
					blackboxWrite(buffer | ((values[x] >> 4) & 0x0F));
					//Now put the leftover low bits into the top of the next buffer entry
					buffer = values[x] << 4;
            	}
            break;
            case FIELD_16BIT:
            	if (nibbleIndex == 0) {
            		//Write high byte first
					blackboxWrite(values[x] >> 8);
					blackboxWrite(values[x]);
				} else {
					//First write the highest 4 bits
					blackboxWrite(buffer | ((values[x] >> 12) & 0x0F));
					// Then the middle 8
					blackboxWrite(values[x] >> 4);
					//Only the smallest 4 bits are still left to write
					buffer = values[x] << 4;
				}
            break;
        }
    }
    //Anything left over to write?
    if (nibbleIndex == 1)
    	blackboxWrite(buffer);
}

/**
 * Write `valueCount` fields from `values` to the Blackbox using signed variable byte encoding. A 1-byte header is
 * written first which specifies which fields are non-zero (so this encoding is compact when most fields are zero).
 *
 * valueCount must be 8 or less.
 */
static void writeTag8_8SVB(int32_t *values, int valueCount)
{
    uint8_t header;
    int i;

    if (valueCount > 0) {
        //If we're only writing one field then we can skip the header
        if (valueCount == 1) {
            writeSignedVB(values[0]);
        } else {
            //First write a one-byte header that marks which fields are non-zero
            header = 0;

            // First field should be in low bits of header
            for (i = valueCount - 1; i >= 0; i--) {
                header <<= 1;

                if (values[i] != 0)
                    header |= 0x01;
            }

            blackboxWrite(header);

            for (i = 0; i < valueCount; i++)
                if (values[i] != 0)
                    writeSignedVB(values[i]);
        }
    }
}

static bool testBlackboxCondition(FlightLogFieldCondition condition)
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
            return (unsigned int) numberMotor >= condition - FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1 + 1;
        case FLIGHT_LOG_FIELD_CONDITION_TRICOPTER:
            return numberMotor == 3;

        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_P_0:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_P_1:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_P_2:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_I_0:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_I_1:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_I_2:
            return true;

        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1:
	        return true;
	        
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2:
            return false;

        case FLIGHT_LOG_FIELD_CONDITION_MAG:
#ifdef MAG
        	return true;
#else
            return false;
#endif

        case FLIGHT_LOG_FIELD_CONDITION_BARO:
#ifdef BARO
			return true;
#else
            return false;
#endif
        case FLIGHT_LOG_FIELD_CONDITION_NEVER:
            return false;
        default:
            return false;
    }
}

static void writeIntraframe(void)
{
	blackboxValues_t *blackboxCurrent = blackboxHistory[0];
	int x;

	blackboxWrite('I');

	writeUnsignedVB(blackboxIteration);
	writeUnsignedVB(blackboxCurrent->time);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        writeSignedVB(blackboxCurrent->axisPID_P[x]);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        writeSignedVB(blackboxCurrent->axisPID_I[x]);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x))
            writeSignedVB(blackboxCurrent->axisPID_D[x]);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->rcCommand[x]);

	writeUnsignedVB(blackboxCurrent->rcCommand[3] - mcfg.minthrottle); //Throttle lies in range [minthrottle..maxthrottle]

    /*
     * Our voltage is expected to decrease over the course of the flight, so store our difference from
     * the reference:
     *
     * Write 14 bits even if the number is negative (which would otherwise result in 32 bits)
     */
    writeUnsignedVB((vbatReference - blackboxCurrent->vbatLatest) & 0x3FFF);

#ifdef MAG
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
            for (x = 0; x < XYZ_AXIS_COUNT; x++)
                writeSignedVB(blackboxCurrent->magADC[x]);
        }
#endif

#ifdef BARO
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO))
            writeSignedVB(blackboxCurrent->BaroAlt);
#endif

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
		writeSignedVB(blackboxCurrent->gyroData[x]);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
		writeSignedVB(blackboxCurrent->accSmooth[x]);

	//Motors can be below minthrottle when disarmed, but that doesn't happen much
	writeUnsignedVB(blackboxCurrent->motor[0] - mcfg.minthrottle);

	//Motors tend to be similar to each other
	for (x = 1; x < numberMotor; x++)
		writeSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER))
		writeSignedVB(blackboxHistory[0]->servo[5] - 1500);

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
    int32_t deltas[5];

	blackboxValues_t *blackboxCurrent = blackboxHistory[0];
	blackboxValues_t *blackboxLast = blackboxHistory[1];

	blackboxWrite('P');

	//No need to store iteration count since its delta is always 1

	/*
	 * Since the difference between the difference between successive times will be nearly zero, use
	 * second-order differences.
	 */
	writeSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        writeSignedVB(blackboxCurrent->axisPID_P[x] - blackboxLast->axisPID_P[x]);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        deltas[x] = blackboxCurrent->axisPID_I[x] - blackboxLast->axisPID_I[x];

	/* 
	 * The PID I field changes very slowly, most of the time +-2, so use an encoding
	 * that can pack all three fields into one byte in that situation.
	 */
	writeTag2_3S32(deltas);
	
    /*
     * The PID D term is frequently set to zero for yaw, which makes the result from the calculation
     * always zero. So don't bother recording D results when PID D terms are zero.
     */
    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x))
            writeSignedVB(blackboxCurrent->axisPID_D[x] - blackboxLast->axisPID_D[x]);

	/*
	 * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
	 * can pack multiple values per byte:
	 */
    for (x = 0; x < 4; x++)
        deltas[x] = blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x];

	writeTag8_4S16(deltas);
	
    //Check for sensors that are updated periodically (so deltas are normally zero) VBAT, MAG, BARO
    int optionalFieldCount = 0;

    deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->vbatLatest - blackboxLast->vbatLatest;

#ifdef MAG
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
        for (x = 0; x < XYZ_AXIS_COUNT; x++)
            deltas[optionalFieldCount++] = blackboxCurrent->magADC[x] - blackboxLast->magADC[x];
    }
#endif

#ifdef BARO
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO))
        deltas[optionalFieldCount++] = blackboxCurrent->BaroAlt - blackboxLast->BaroAlt;
#endif
    writeTag8_8SVB(deltas, optionalFieldCount);

	//Since gyros, accs and motors are noisy, base the prediction on the average of the history:
    for (x = 0; x < XYZ_AXIS_COUNT; x++)
		writeSignedVB(blackboxHistory[0]->gyroData[x] - (blackboxHistory[1]->gyroData[x] + blackboxHistory[2]->gyroData[x]) / 2);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
		writeSignedVB(blackboxHistory[0]->accSmooth[x] - (blackboxHistory[1]->accSmooth[x] + blackboxHistory[2]->accSmooth[x]) / 2);

	for (x = 0; x < numberMotor; x++)
		writeSignedVB(blackboxHistory[0]->motor[x] - (blackboxHistory[1]->motor[x] + blackboxHistory[2]->motor[x]) / 2);

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER))
		writeSignedVB(blackboxCurrent->servo[5] - blackboxLast->servo[5]);

	//Rotate our history buffers
	blackboxHistory[2] = blackboxHistory[1];
	blackboxHistory[1] = blackboxHistory[0];
	blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

/*
 * Treat each decoded frame as if it were a set of freshly read flight data ready to be
 * encoded.
 */
void onFrameReady(flightLog_t *log, bool frameValid, int32_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize)
{
	int x, src;
	uint32_t start = writtenBytes;
	unsigned int encodedFrameSize;
	blackboxValues_t *blackboxCurrent = blackboxHistory[0];

	(void) log;
	(void) frameOffset;
	(void) frameSize;
	(void) fieldCount;

	if (frameValid) {
		blackboxIteration = (uint32_t) frame[0];

		//Load the decoded frame into the buffer ready to be encoded
		src = 1;

		blackboxCurrent->time = (uint32_t) frame[src++];

		for (x = 0; x < XYZ_AXIS_COUNT; x++)
			blackboxCurrent->axisPID_P[x] = frame[src++];

		for (x = 0; x < XYZ_AXIS_COUNT; x++)
			blackboxCurrent->axisPID_I[x] = frame[src++];

		// We sometimes don't write every axisD[], so check if it is actually available
		for (x = 0; x < XYZ_AXIS_COUNT; x++)
		    if (strncmp(log->mainFieldNames[src], "axisD[", strlen("axisD[")) == 0
		        && log->mainFieldNames[src][strlen("axisD[")] == x + '0') {
			blackboxCurrent->axisPID_D[x] = frame[src++];
		} else
		    blackboxCurrent->axisPID_D[x] = 0;

		for (x = 0; x < 4; x++)
			blackboxCurrent->rcCommand[x] = frame[src++];

		if (strcmp(log->mainFieldNames[src], "vbatLatest") == 0)
		    blackboxCurrent->vbatLatest = frame[src++];

        if (strncmp(log->mainFieldNames[src], "magADC[", strlen("magADC[")) == 0)
            for (x = 0; x < XYZ_AXIS_COUNT; x++)
                blackboxCurrent->magADC[x] = frame[src++];

        if (strcmp(log->mainFieldNames[src], "BaroAlt") == 0)
            blackboxCurrent->BaroAlt = frame[src++];

		for (x = 0; x < XYZ_AXIS_COUNT; x++)
			blackboxCurrent->gyroData[x] = frame[src++];

		for (x = 0; x < XYZ_AXIS_COUNT; x++)
			blackboxCurrent->accSmooth[x] = frame[src++];

		for (x = 0; x < numberMotor; x++)
			blackboxCurrent->motor[x] = frame[src++];
		
		if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER))
			blackboxCurrent->servo[5] = frame[src++];

		if (frameType == 'I') {
			writeIntraframe();

			encodedFrameSize = writtenBytes - start;

			encodedStats.frame['I'].validCount++;
			encodedStats.frame['I'].bytes += encodedFrameSize;
			encodedStats.frame['I'].sizeCount[encodedFrameSize]++;
		} else if (frameType == 'P'){
			writeInterframe();

			encodedFrameSize = writtenBytes - start;

            encodedStats.frame['P'].validCount++;
            encodedStats.frame['P'].bytes += encodedFrameSize;
            encodedStats.frame['P'].sizeCount[encodedFrameSize]++;
		} else {
			fprintf(stderr, "Unknown frame type %c\n", (char) frameType);
			exit(-1);
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

static void sendFieldDefinition(const char * const *headerNames, unsigned int headerCount, const void *fieldDefinitions,
        const void *secondFieldDefinition, int fieldCount, const uint8_t *conditions, const uint8_t *secondCondition)
{
    const blackboxFieldDefinition_t *def;
    static bool needComma = false;
    size_t definitionStride = (char*) secondFieldDefinition - (char*) fieldDefinitions;
    size_t conditionsStride = (char*) secondCondition - (char*) conditions;

    for (unsigned int headerXmitIndex = 0; headerXmitIndex < headerCount; headerXmitIndex++) {
        blackboxPrint("H Field ");
        blackboxPrint(headerNames[headerXmitIndex]);
        blackboxPrint(":");

        needComma = false;

        for (int fieldXmitIndex = 0; fieldXmitIndex < fieldCount; fieldXmitIndex++) {
            def = (const blackboxFieldDefinition_t*) ((const char*)fieldDefinitions + definitionStride * fieldXmitIndex);

            if (!conditions || testBlackboxCondition(conditions[conditionsStride * fieldXmitIndex])) {
                if (needComma) {
                    blackboxWrite(',');
                } else
                    needComma = true;

                // The first header is a field name
                if (headerXmitIndex == 0) {
                    blackboxPrint(def->name);
                } else {
                    //The other headers are integers
                    if (def->arr[headerXmitIndex - 1] >= 10) {
                        blackboxWrite(def->arr[headerXmitIndex - 1] / 10 + '0');
                        blackboxWrite(def->arr[headerXmitIndex - 1] % 10 + '0');
                    } else {
                        blackboxWrite(def->arr[headerXmitIndex - 1] + '0');
                    }
                }
            }
        }

        blackboxWrite('\n');
    }
}

void blackboxWriteHeader(flightLog_t *log)
{
    union floatConvert_t {
        float f;
        uint32_t u;
    } floatConvert;

    int i;

    for (i = 0; blackboxHeader[i] != '\0'; i++)
        blackboxWrite(blackboxHeader[i]);

    sendFieldDefinition(blackboxMainHeaderNames, ARRAY_LENGTH(blackboxMainHeaderNames), blackboxMainFields, blackboxMainFields + 1,
            ARRAY_LENGTH(blackboxMainFields), &blackboxMainFields[0].condition, &blackboxMainFields[1].condition);

    blackboxPrintf("H Firmware type:Cleanflight\n");
    blackboxPrintf("H rcRate:%d\n", log->rcRate);
    blackboxPrintf("H minthrottle:%d\n", log->minthrottle);
    blackboxPrintf("H maxthrottle:%d\n", log->maxthrottle);
    floatConvert.f = log->gyroScale;
    blackboxPrintf("H gyro.scale:0x%x\n", floatConvert.u);
    blackboxPrintf("H vbatscale:%u\n", log->vbatscale);
    blackboxPrintf("H vbatcellvoltage:%u,%u,%u\n", log->vbatmincellvoltage, log->vbatwarningcellvoltage, log->vbatmaxcellvoltage);
    blackboxPrintf("H vbatref:%u\n", log->vbatref);
    blackboxPrintf("H acc_1G:%u\n", log->acc_1G);
}

void onMetadataReady(flightLog_t *log)
{
	int i;

	numberMotor = 0;

	for (i = 0; i < log->mainFieldCount; i++) {
		if (strncmp(log->mainFieldNames[i], "motor[", strlen("motor[")) == 0) {
			int motorIndex = atoi(log->mainFieldNames[i] + strlen("motor["));

			if (motorIndex + 1 > numberMotor)
				numberMotor = motorIndex + 1;
		}
	}

	vbatReference = log->vbatref;

	blackboxWriteHeader(log);
}

int main(int argc, char **argv)
{
	FILE *input;
	flightLog_t *log;

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

	log = flightLogCreate(fileno(input));

	flightLogParse(log, 0, onMetadataReady, onFrameReady, NULL, 0);

	encodedStats.totalBytes = writtenBytes;
	encodedStats.field[FLIGHT_LOG_FIELD_INDEX_TIME].min = log->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].min;
	encodedStats.field[FLIGHT_LOG_FIELD_INDEX_TIME].max = log->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].max;

	fprintf(stderr, "Logged time %u seconds\n", (uint32_t)((log->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].max - log->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].min) / 1000000));

	fprintf(stderr, "\nOriginal statistics\n");
	printStats(&log->stats);

	fprintf(stderr, "\nNew statistics\n");
	printStats(&encodedStats);

	printFrameSizeComparison(&log->stats, &encodedStats);

	flightLogDestroy(log);

	return 0;
}
