#include <stdlib.h>
#include <string.h>

#include "gpxwriter.h"

#define GPS_DEGREES_DIVIDER 10000000L

static const char GPX_FILE_HEADER[] =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<gpx creator=\"Blackbox flight data recorder\" version=\"1.1\" xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\""
        " xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n"
    "<metadata><name>Blackbox flight log</name></metadata>\n";

static const char GPX_FILE_TRAILER[] =
    "</gpx>";

void gpxWriterAddPreamble(gpxWriter_t *gpx)
{
    gpx->file = fopen(gpx->filename, "wb");

    fprintf(gpx->file, GPX_FILE_HEADER);
}

/**
 * Add a point to the current track.
 *
 * Time is in microseconds since device power-on. Lat and lon are degrees multiplied by GPS_DEGREES_DIVIDER. Altitude
 * is in meters.
 */
void gpxWriterAddPoint(gpxWriter_t *gpx, flightLog_t *log, int64_t time, int32_t lat, int32_t lon, int16_t altitude)
{
    char negSign[] = "-";
    char noSign[] = "";

    if (!gpx)
        return;

    if (gpx->state == GPXWRITER_STATE_EMPTY) {
        gpxWriterAddPreamble(gpx);

        fprintf(gpx->file, "<trk><name>Blackbox flight log</name><trkseg>\n");

        gpx->startTime = log->sysConfig.logStartTime.tm_hour * 3600 + log->sysConfig.logStartTime.tm_min * 60 + log->sysConfig.logStartTime.tm_sec;
        gpx->state = GPXWRITER_STATE_WRITING_TRACK;
    }

    int32_t latDegrees = lat / GPS_DEGREES_DIVIDER;
    int32_t lonDegrees = lon / GPS_DEGREES_DIVIDER;

    uint32_t latFracDegrees = abs(lat) % GPS_DEGREES_DIVIDER;
    uint32_t lonFracDegrees = abs(lon) % GPS_DEGREES_DIVIDER;

    char *latSign = ((lat < 0) && (latDegrees == 0)) ? negSign : noSign;
    char *lonSign = ((lon < 0) && (lonDegrees == 0)) ? negSign : noSign;

    fprintf(gpx->file, "  <trkpt lat=\"%s%d.%07u\" lon=\"%s%d.%07u\"><ele>%d</ele>", latSign, latDegrees, latFracDegrees, lonSign, lonDegrees, lonFracDegrees, altitude);

    if (time != -1) {
        //We'll just assume that the timespan is less than 24 hours, and make up a date
        uint32_t hours, mins, secs, frac;

        time += gpx->startTime * 1000000;

        frac = (uint32_t)(time % 1000000);
        secs = (uint32_t)(time / 1000000);

        mins = secs / 60;
        secs %= 60;

        hours = mins / 60;
        mins %= 60;

        fprintf(gpx->file, "<time>%04u-%02u-%02uT%02u:%02u:%02u.%06uZ</time>",
            log->sysConfig.logStartTime.tm_year + 1900, log->sysConfig.logStartTime.tm_mon + 1, log->sysConfig.logStartTime.tm_mday,
            hours, mins, secs, frac);
    }
    fprintf(gpx->file, "</trkpt>\n");
}

gpxWriter_t* gpxWriterCreate(const char *filename)
{
    gpxWriter_t *result = malloc(sizeof(*result));

    result->filename = strdup(filename);
    result->state = GPXWRITER_STATE_EMPTY;
    result->file = NULL;

    return result;
}

void gpxWriterDestroy(gpxWriter_t* gpx)
{
    if (!gpx)
        return;

    if (gpx->state == GPXWRITER_STATE_WRITING_TRACK) {
        fprintf(gpx->file, "</trkseg></trk>\n");
    }

    if (gpx->state != GPXWRITER_STATE_EMPTY) {
        fprintf(gpx->file, GPX_FILE_TRAILER);
        fclose(gpx->file);
    }

    free(gpx->filename);
    free(gpx);
}
