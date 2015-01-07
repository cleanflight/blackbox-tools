#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

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

typedef struct decodeOptions_t {
	int help, raw, limits, debug, toStdout;
	int logNumber;
	const char *outputPrefix;
} decodeOptions_t;

decodeOptions_t options = {
	.help = 0, .raw = 0, .limits = 0, .debug = 0, .toStdout = 0,
	.logNumber = -1,
	.outputPrefix = 0
};

uint32_t lastFrameIndex = (uint32_t) -1;

FILE *outputFile;

void onFrameReady(flightLog_t *log, bool frameValid, int32_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize)
{
	int i;

	if (frame)
		lastFrameIndex = (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_ITERATION];

	if (frameValid) {
		for (i = 0; i < fieldCount; i++) {
			if (i == 0) {
				fprintf(outputFile, "%u", (uint32_t) frame[i]);
			} else {
				if (log->mainFieldSigned[i] || options.raw)
					fprintf(outputFile,", %3d", frame[i]);
				else
					fprintf(outputFile,", %3u", (uint32_t) frame[i]);
			}
		}

		if (options.debug) {
			fprintf(outputFile,", %c, offset %d, size %d\n", (char) frameType, frameOffset, frameSize);
		} else
			fprintf(outputFile,"\n");
	} else if (options.debug) {
		// Print to stdout so that these messages line up with our other output on stdout (stderr isn't synchronised to it)
		if (frame) {
			/*
			 * We'll assume that the frame's iteration count is still fairly sensible (if an earlier frame was corrupt,
			 * the frame index will be smaller than it should be)
			 */
			fprintf(outputFile,"%c Frame unusuable due to prior corruption %u, offset %d, size %d\n", (char) frameType, lastFrameIndex, frameOffset, frameSize);
		} else {
			//We have no frame index for this frame, so just assume it was the one after the previously decoded frame
			lastFrameIndex++;
			fprintf(outputFile,"Failed to decode %c frame %u, offset %d, size %d\n", (char) frameType, lastFrameIndex, frameOffset, frameSize);
		}
	}
}

void onMetadataReady(flightLog_t *log)
{
	int i;

	if (log->mainFieldCount == 0) {
		fprintf(stderr, "No fields found in log, is it missing its header?\n");
		exit(-1);
	}

	for (i = 0; i < log->mainFieldCount; i++) {
		if (i > 0)
			fprintf(outputFile,", ");

		fprintf(outputFile,"%s", log->mainFieldNames[i]);
	}
	fprintf(outputFile,"\n");
}

void printStats(flightLog_t *log, int logIndex, bool raw, bool limits)
{
	flightLogStatistics_t *stats = &log->stats;
	uint32_t intervalMS = (uint32_t) ((stats->field[FLIGHT_LOG_FIELD_INDEX_TIME].max - stats->field[FLIGHT_LOG_FIELD_INDEX_TIME].min) / 1000);

	uint32_t goodBytes = stats->frame['I'].bytes + stats->frame['P'].bytes;
	uint32_t goodFrames = stats->frame['I'].validCount + stats->frame['P'].validCount;
	uint32_t totalFrames = (uint32_t) (stats->field[FLIGHT_LOG_FIELD_INDEX_ITERATION].max - stats->field[FLIGHT_LOG_FIELD_INDEX_ITERATION].min + 1);
	int32_t missingFrames = totalFrames - goodFrames - stats->intentionallyAbsentIterations + stats->frame['P'].desyncCount;

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

	if (intervalMS > 0 && !raw)
        fprintf(stderr, ", start %02d:%02d.%03d, end %02d:%02d.%03d, duration %02d:%02d.%03d\n\n",
            startTimeMins, startTimeSecs, startTimeMS,
            endTimeMins, endTimeSecs, endTimeMS,
            runningTimeMins, runningTimeSecs, runningTimeMS
        );

	fprintf(stderr, "Statistics\n");

	for (i = 0; i < (int) sizeof(frameTypes); i++) {
	    uint8_t frameType = frameTypes[i];

	    if (stats->frame[frameType].validCount + stats->frame[frameType].desyncCount) {
	        fprintf(stderr, "%c frames %7d %6.1f bytes avg %8d bytes total\n", (char) frameType, stats->frame[frameType].validCount + stats->frame[frameType].desyncCount,
                (float) stats->frame[frameType].bytes / (stats->frame[frameType].validCount + stats->frame[frameType].desyncCount), stats->frame[frameType].bytes);
        }
    }

	if (goodFrames)
		fprintf(stderr, "Frames %9d %6.1f bytes avg %8d bytes total\n", goodFrames, (float) goodBytes / goodFrames, goodBytes);
	else
		fprintf(stderr, "Frames %8d\n", 0);

	if (intervalMS > 0 && !raw) {
		fprintf(stderr, "Data rate %4uHz %6u bytes/s %10u baud\n",
            (unsigned int) (((int64_t) goodFrames * 1000) / intervalMS),
            (unsigned int) (((int64_t) stats->totalBytes * 1000) / intervalMS),
            (unsigned int) ((((int64_t) stats->totalBytes * 1000 * 8) / intervalMS + 100 - 1) / 100 * 100)); /* Round baud rate up to nearest 100 */
	} else {
		fprintf(stderr, "Data rate: Unknown, no timing information available.\n");
	}

	if (totalFrames && (stats->totalCorruptFrames || stats->frame['P'].desyncCount || missingFrames || stats->intentionallyAbsentIterations)) {
		fprintf(stderr, "\n");

		if (stats->totalCorruptFrames || stats->frame['P'].desyncCount) {
			fprintf(stderr, "%d frames failed to decode, rendering %d loop iterations unreadable. ", stats->totalCorruptFrames, stats->frame['P'].desyncCount + stats->frame['P'].corruptCount + stats->frame['I'].desyncCount + stats->frame['I'].corruptCount);
			if (!missingFrames)
				fprintf(stderr, "\n");
		}
		if (missingFrames) {
			fprintf(stderr, "%d iterations are missing in total (%ums, %.2f%%)\n",
				missingFrames,
				(unsigned int) (missingFrames * (intervalMS / totalFrames)), (double) missingFrames / totalFrames * 100);
		}
		if (stats->intentionallyAbsentIterations) {
			fprintf(stderr, "%d loop iterations weren't logged because of your blackbox_rate settings (%ums, %.2f%%)\n",
				stats->intentionallyAbsentIterations,
				(unsigned int) (stats->intentionallyAbsentIterations * (intervalMS / totalFrames)), (double) stats->intentionallyAbsentIterations / totalFrames * 100);
		}
	}

	if (limits) {
		fprintf(stderr, "\n\n    Field name          Min          Max        Range\n");
		fprintf(stderr,     "-----------------------------------------------------\n");

		for (i = 0; i < log->mainFieldCount; i++) {
			fprintf(stderr, "%14s %12" PRId64 " %12" PRId64 " %12" PRId64 "\n",
				log->mainFieldNames[i],
				stats->field[i].max,
				stats->field[i].min,
				stats->field[i].max - stats->field[i].min
			);
		}
	}

    fprintf(stderr, "\n");
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
		"   --help           This page\n"
		"   --index <num>    Choose the log from the file that should be decoded (or omit to decode all)\n"
		"   --limits         Print the limits and range of each field\n"
		"   --stdout         Write log to stdout instead of to a file\n"
		"   --debug          Show extra debugging information\n"
		"   --raw            Don't apply predictions to fields (show raw field deltas)\n"
		"\n", argv0
	);
}

void parseCommandlineOptions(int argc, char **argv)
{
	int c;

	while (1)
	{
		static struct option long_options[] = {
			{"help", no_argument, &options.help, 1},
			{"raw", no_argument, &options.raw, 1},
			{"debug", no_argument, &options.debug, 1},
			{"limits", no_argument, &options.limits, 1},
			{"stdout", no_argument, &options.toStdout, 1},
			{"prefix", required_argument, 0, 'p'},
			{"index", required_argument, 0, 'i'},
			{0, 0, 0, 0}
		};

		int option_index = 0;

		c = getopt_long (argc, argv, "", long_options, &option_index);

		if (c == -1)
			break;

		switch (c) {
			case 'i':
				options.logNumber = atoi(optarg);
			break;
			case 'o':
				options.outputPrefix = optarg;
			break;
		}
	}
}

int decodeFlightLog(flightLog_t *log, const char *filename, int logIndex)
{
	if (options.toStdout) {
		outputFile = stdout;
	} else {
		char *outputFilename = 0;
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

		int outputFilenameLen = outputPrefixLen + strlen(".00.csv") + 1;
		outputFilename = malloc(outputFilenameLen);

		snprintf(outputFilename, outputFilenameLen, "%.*s.%02d.csv", outputPrefixLen, outputPrefix, logIndex + 1);

		outputFile = fopen(outputFilename, "wb");

		if (!outputFile) {
			fprintf(stderr, "Failed to create output file %s\n", outputFilename);

			free(outputFilename);
			return -1;
		}

		fprintf(stderr, "Decoding log '%s' to '%s'...\n", filename, outputFilename);

		free(outputFilename);
	}

	int success = flightLogParse(log, logIndex, onMetadataReady, onFrameReady, NULL, options.raw);

	if (success)
		printStats(log, logIndex, options.raw, options.limits);

	if (!options.toStdout)
		fclose(outputFile);

	return success ? 0 : -1;
}

int main(int argc, char **argv)
{
	flightLog_t *log;
	int fd;
	int logIndex;

	parseCommandlineOptions(argc, argv);

	if (options.help) {
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
