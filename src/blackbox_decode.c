#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>

#include <errno.h>
#include <fcntl.h>

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

typedef enum Unit {
    UNIT_RAW = 0,
    UNIT_METERS_PER_SECOND,
    UNIT_KILOMETERS_PER_HOUR,
    UNIT_MILES_PER_HOUR
} Unit;

static const char* const UNIT_NAME[] = {
    "raw",
    "m/s",
    "km/h",
    "mi/h"
};

typedef struct decodeOptions_t {
    int help, raw, limits, debug, toStdout;
    int logNumber;
    const char *outputPrefix;
    Unit unitGPSSpeed;
} decodeOptions_t;

decodeOptions_t options = {
    .help = 0, .raw = 0, .limits = 0, .debug = 0, .toStdout = 0,
    .logNumber = -1,
    .outputPrefix = 0,

    .unitGPSSpeed = UNIT_METERS_PER_SECOND
};

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

//We'll use field names to identify GPS field units so the values can be formatted for display
typedef enum {
    GPS_FIELD_TYPE_INTEGER,
    GPS_FIELD_TYPE_DEGREES_TIMES_10, // for headings
    GPS_FIELD_TYPE_COORDINATE_DEGREES_TIMES_10000000,
    GPS_FIELD_TYPE_METERS_PER_SECOND_TIMES_100,
    GPS_FIELD_TYPE_METERS
} GPSFieldType;

static gpsGFieldIndexes_t gpsGFieldIndexes;
static gpsHFieldIndexes_t gpsHFieldIndexes;
static GPSFieldType gpsFieldTypes[FLIGHT_LOG_MAX_FIELDS];

static uint32_t lastFrameTime = (uint32_t) -1;

static FILE *csvFile = 0, *eventFile = 0, *gpsCsvFile = 0;
static char *eventFilename = 0, *gpsCsvFilename = 0;
static gpxWriter_t *gpx = 0;

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
            fprintf(eventFile, "%u: Sync beep\n", event->data.syncBeep.time);
        break;
        case FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_START:
            fprintf(eventFile, "%u: Autotune cycle start,%d,%d,%u,%u,%u\n", lastFrameTime,
                event->data.autotuneCycleStart.phase, event->data.autotuneCycleStart.cycle,
                event->data.autotuneCycleStart.p, event->data.autotuneCycleStart.i, event->data.autotuneCycleStart.d);
        break;
        case FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_RESULT:
            fprintf(eventFile, "%u: Autotune cycle result,%u,%u,%u,%u\n", lastFrameTime,
                event->data.autotuneCycleResult.overshot,
                event->data.autotuneCycleResult.p, event->data.autotuneCycleResult.i, event->data.autotuneCycleResult.d);
        break;
        case FLIGHT_LOG_EVENT_LOG_END:
            fprintf(eventFile, "%u: Log clean end\n", lastFrameTime);
        break;
        default:
            fprintf(eventFile, "%u: Unknown event %u\n", lastFrameTime, event->event);
        break;
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
            // Print GPS fieldname header
            fprintf(gpsCsvFile, "time");

            for (int i = 0; i < log->gpsFieldCount; i++) {
                fprintf(gpsCsvFile, ", %s", log->gpsFieldNames[i]);

                if (gpsFieldTypes[i] == GPS_FIELD_TYPE_METERS_PER_SECOND_TIMES_100)
                    fprintf(gpsCsvFile, " (%s)", UNIT_NAME[options.unitGPSSpeed]);
            }
            fprintf(gpsCsvFile, "\n");
        }
    }
}

double convertMetersPerSecondToUnit(double meterspersec, Unit unit)
{
    static const double MILES_PER_METER = 0.00062137;

    switch (unit) {
        case UNIT_KILOMETERS_PER_HOUR:
            return meterspersec * 60 * 60 / 1000;
        break;

        case UNIT_MILES_PER_HOUR:
            return meterspersec * MILES_PER_METER * 60 * 60;
        break;

        case UNIT_METERS_PER_SECOND:
            return meterspersec;

        case UNIT_RAW:
            fprintf(stderr, "Attempted to convert speed to raw units but this data is already cooked\n");
            exit(-1);
        break;
        default:
            fprintf(stderr, "Bad speed unit in conversion\n");
            exit(-1);
    }
}

void outputGPSFrame(flightLog_t *log, int32_t *frame)
{
    int i;
    int32_t degrees;
    uint32_t fracDegrees;
    uint32_t gpsFrameTime;

    // If we're not logging every loop iteration, we include a timestamp field in the GPS frame:
    if (gpsGFieldIndexes.time != -1) {
        gpsFrameTime = frame[gpsGFieldIndexes.time];
    } else {
        // Otherwise this GPS frame was recorded at the same time as the main stream frame we read before the GPS frame:
        gpsFrameTime = lastFrameTime;
    }

    // We need at least lat/lon/altitude from the log to write a useful GPX track
    if (gpsGFieldIndexes.GPS_coord[0] != -1 && gpsGFieldIndexes.GPS_coord[0] != -1 && gpsGFieldIndexes.GPS_altitude != -1) {
        gpxWriterAddPoint(gpx, lastFrameTime, frame[gpsGFieldIndexes.GPS_coord[0]], frame[gpsGFieldIndexes.GPS_coord[1]], frame[gpsGFieldIndexes.GPS_altitude]);
    }

    createGPSCSVFile(log);

    if (gpsCsvFile) {
        fprintf(gpsCsvFile, "%u", gpsFrameTime);

        for (i = 0; i < log->gpsFieldCount; i++) {
            //We've already printed the time:
            if (i == gpsGFieldIndexes.time)
                continue;

            fprintf(gpsCsvFile, ", ");

            switch (gpsFieldTypes[i]) {
                case GPS_FIELD_TYPE_COORDINATE_DEGREES_TIMES_10000000:
                    degrees = frame[i] / 10000000;
                    fracDegrees = abs(frame[i]) % 10000000;

                    fprintf(gpsCsvFile, "%d.%07u", degrees, fracDegrees);
                break;
                case GPS_FIELD_TYPE_DEGREES_TIMES_10:
                    fprintf(gpsCsvFile, "%d.%01u", frame[i] / 10, abs(frame[i]) % 10);
                break;
                case GPS_FIELD_TYPE_METERS_PER_SECOND_TIMES_100:
                    if (options.unitGPSSpeed == UNIT_RAW) {
                        fprintf(gpsCsvFile, "%d", frame[i]);
                    } else if (options.unitGPSSpeed == UNIT_METERS_PER_SECOND) {
                        fprintf(gpsCsvFile, "%d.%02u", frame[i] / 100, abs(frame[i]) % 100);
                    } else {
                        fprintf(gpsCsvFile, "%.2f", convertMetersPerSecondToUnit(frame[i] / 100.0, options.unitGPSSpeed));
                    }
                break;
                case GPS_FIELD_TYPE_METERS:
                    fprintf(gpsCsvFile, "%d", frame[i]);
                break;
                case GPS_FIELD_TYPE_INTEGER:
                default:
                    fprintf(gpsCsvFile, "%d", frame[i]);
            }

        }
        fprintf(gpsCsvFile, "\n");
    }
}

void onFrameReady(flightLog_t *log, bool frameValid, int32_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize)
{
    int i;

    if (frameType == 'G') {
        if (frameValid) {
            outputGPSFrame(log, frame);
        }
    } else if (frameType == 'P' || frameType == 'I') {
        if (frameValid || (frame && options.raw)) {
            lastFrameTime = (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_TIME];

            for (i = 0; i < fieldCount; i++) {
                if (i == 0) {
                    if (frameValid)
                        fprintf(csvFile, "%u", (uint32_t) frame[i]);
                    else
                        fprintf(csvFile, "X");
                } else {
                    if (log->mainFieldSigned[i] || options.raw)
                        fprintf(csvFile, ", %3d", frame[i]);
                    else
                        fprintf(csvFile, ", %3u", (uint32_t) frame[i]);
                }
            }

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
    }
}

void resetGPSFieldIdents()
{
    for (int i = 0; i < FLIGHT_LOG_MAX_FIELDS; i++)
        gpsFieldTypes[i] = GPS_FIELD_TYPE_INTEGER;
}

/**
 * Check which GPS fields are present and log the indexes of well-known fields in `gps{G|H}FieldIndexes`.
 *
 * Sets the units/display format we should use for each GPS field in `gpsFieldTypes`.
 */
void identifyGPSFields(flightLog_t *log)
{
    int i;

    // Start by marking all fields as not-present (index -1)
    gpsGFieldIndexes.time = -1;
    gpsGFieldIndexes.GPS_altitude = -1;
    gpsGFieldIndexes.GPS_coord[0] = -1;
    gpsGFieldIndexes.GPS_coord[1] = -1;
    gpsGFieldIndexes.GPS_ground_course = -1;
    gpsGFieldIndexes.GPS_numSat = -1;
    gpsGFieldIndexes.GPS_speed = -1;

    gpsHFieldIndexes.GPS_home[0] = -1;
    gpsHFieldIndexes.GPS_home[1] = -1;

    for (i = 0; i < log->gpsFieldCount; i++) {
        const char *fieldName = log->gpsFieldNames[i];

        if (strcmp(fieldName, "GPS_coord[0]") == 0) {
            gpsGFieldIndexes.GPS_coord[0] = i;
            gpsFieldTypes[i] = GPS_FIELD_TYPE_COORDINATE_DEGREES_TIMES_10000000;
        } else if (strcmp(fieldName, "GPS_coord[1]") == 0) {
            gpsGFieldIndexes.GPS_coord[1] = i;
            gpsFieldTypes[i] = GPS_FIELD_TYPE_COORDINATE_DEGREES_TIMES_10000000;
        } else if (strcmp(fieldName, "GPS_altitude") == 0) {
            gpsGFieldIndexes.GPS_altitude = i;
            gpsFieldTypes[i] = GPS_FIELD_TYPE_METERS;
        } else if (strcmp(fieldName, "GPS_speed") == 0) {
            gpsGFieldIndexes.GPS_speed = i;
            gpsFieldTypes[i] = GPS_FIELD_TYPE_METERS_PER_SECOND_TIMES_100;
        } else if (strcmp(fieldName, "GPS_ground_course") == 0) {
            gpsGFieldIndexes.GPS_ground_course = i;
            gpsFieldTypes[i] = GPS_FIELD_TYPE_DEGREES_TIMES_10;
        } else if (strcmp(fieldName, "GPS_numSat") == 0) {
            gpsGFieldIndexes.GPS_numSat = i;
        } else {
            gpsFieldTypes[i] = GPS_FIELD_TYPE_INTEGER;
        }
    }

    for (i = 0; i < log->gpsHomeFieldCount; i++) {
        const char *fieldName = log->gpsHomeFieldNames[i];

        if (strcmp(fieldName, "GPS_home[0]") == 0)
            gpsHFieldIndexes.GPS_home[0] = i;
        else if (strcmp(fieldName, "GPS_home[1]") == 0)
            gpsHFieldIndexes.GPS_home[1] = i;
    }
}

void writeMainCSVHeader(flightLog_t *log)
{
    int i;

    for (i = 0; i < log->mainFieldCount; i++) {
        if (i > 0)
            fprintf(csvFile, ", ");

        fprintf(csvFile, "%s", log->mainFieldNames[i]);
    }
    fprintf(csvFile, "\n");
}

void onMetadataReady(flightLog_t *log)
{
    if (log->mainFieldCount == 0) {
        fprintf(stderr, "No fields found in log, is it missing its header?\n");
    } else {
        writeMainCSVHeader(log);
        identifyGPSFields(log);
    }
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

    uint8_t frameTypes[] = {'I', 'P', 'H', 'G', 'E'};

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
            (unsigned int) ((((int64_t) stats->totalBytes * 1000 * 8) / intervalMS + 100 - 1) / 100 * 100)); /* Round baud rate up to nearest 100 */
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

        for (i = 0; i < log->mainFieldCount; i++) {
            fprintf(stderr, "%14s %12" PRId64 " %12" PRId64 " %12" PRId64 "\n",
                log->mainFieldNames[i],
                stats->field[i].min,
                stats->field[i].max,
                stats->field[i].max - stats->field[i].min
            );
        }
    }

    fprintf(stderr, "\n");
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

    int success = flightLogParse(log, logIndex, onMetadataReady, onFrameReady, onEvent, options.raw);

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
        "   --help                  This page\n"
        "   --index <num>           Choose the log from the file that should be decoded (or omit to decode all)\n"
        "   --limits                Print the limits and range of each field\n"
        "   --stdout                Write log to stdout instead of to a file\n"
        "   --unit-gps-speed <unit> GPS speed unit (mps|kph|mph), default is mps (meters per second)\n"
        "   --debug                 Show extra debugging information\n"
        "   --raw                   Don't apply predictions to fields (show raw field deltas)\n"
        "\n", argv0
    );
}

bool parseUnit(const char *text, Unit *unit)
{
    if (strcmp(text, "kph") == 0 || strcmp(text, "kmph") == 0  || strcmp(text, "km/h") == 0 || strcmp(text, "km/hr") == 0) {
        *unit = UNIT_KILOMETERS_PER_HOUR;
    } else if (strcmp(text, "mps") == 0 || strcmp(text, "m/s") == 0) {
        *unit = UNIT_METERS_PER_SECOND;
    } else if (strcmp(text, "mph") == 0 || strcmp(text, "mi/h") == 0 || strcmp(text, "mi/hr") == 0) {
        *unit = UNIT_MILES_PER_HOUR;
    } else
        return false;

    return true;
}

void parseCommandlineOptions(int argc, char **argv)
{
    int c;

    enum {
        SETTING_PREFIX = 1,
        SETTING_INDEX,
        SETTING_UNIT_GPS_SPEED,
    };

    while (1)
    {
        static struct option long_options[] = {
            {"help", no_argument, &options.help, 1},
            {"raw", no_argument, &options.raw, 1},
            {"debug", no_argument, &options.debug, 1},
            {"limits", no_argument, &options.limits, 1},
            {"stdout", no_argument, &options.toStdout, 1},
            {"prefix", required_argument, 0, SETTING_PREFIX},
            {"index", required_argument, 0, SETTING_INDEX},
            {"unit-gps-speed", required_argument, 0, SETTING_UNIT_GPS_SPEED},
            {0, 0, 0, 0}
        };

        int option_index = 0;

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
                if (!parseUnit(optarg, &options.unitGPSSpeed)) {
                    fprintf(stderr, "Bad GPS speed unit\n");
                    exit(-1);
                }
            break;
        }
    }
}

int main(int argc, char **argv)
{
    flightLog_t *log;
    int fd;
    int logIndex;

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
