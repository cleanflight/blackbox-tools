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
void gpxWriterAddPoint(gpxWriter_t *gpx, uint32_t time, int32_t lat, int32_t lon, int16_t altitude)
{
    if (!gpx)
        return;

    if (gpx->state == GPXWRITER_STATE_EMPTY) {
        gpxWriterAddPreamble(gpx);

        fprintf(gpx->file, "<trk><name>Blackbox flight log</name><trkseg>\n");

        gpx->state = GPXWRITER_STATE_WRITING_TRACK;
    }

    int32_t latDegrees = lat / GPS_DEGREES_DIVIDER;
    int32_t lonDegrees = lon / GPS_DEGREES_DIVIDER;

    uint32_t latFracDegrees = abs(lat) % GPS_DEGREES_DIVIDER;
    uint32_t lonFracDegrees = abs(lon) % GPS_DEGREES_DIVIDER;

    fprintf(gpx->file, "  <trkpt lat=\"%d.%07u\" lon=\"%d.%07u\"><ele>%d</ele>", latDegrees, latFracDegrees, lonDegrees, lonFracDegrees, altitude);
    if (time != (uint32_t) -1) {
        //We'll just assume that the timespan is less than 24 hours, and make up a date
        int hours, mins, secs, frac;

        frac = time % 1000000;
        secs = time / 1000000;

        mins = secs / 60;
        secs %= 60;

        hours = mins / 60;
        mins %= 60;

        fprintf(gpx->file, "<time>2000-01-01T%02d:%02d:%02d.%06uZ</time>", hours, mins, secs, frac);
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
