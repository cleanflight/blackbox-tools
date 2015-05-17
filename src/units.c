#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "units.h"

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

/**
 * Case-insentive string equality test.
 */
static bool striequals(const char *first, const char *second)
{
    while (1) {
        if (tolower(*first) != tolower(*second)) {
            return false;
        }
        if (*first == '\0') {
            return true;
        }

        first++;
        second++;
    }
}

/**
 * Return true on successful conversion and store the resulting unit into the given pointer.
 */
bool unitFromName(const char *text, Unit *unit)
{
    if (striequals(text, "kph") || striequals(text, "kmph")  || striequals(text, "km/h") || striequals(text, "km/hr")) {
        *unit = UNIT_KILOMETERS_PER_HOUR;
    } else if (striequals(text, "mps") || striequals(text, "m/s")) {
        *unit = UNIT_METERS_PER_SECOND;
    } else if (striequals(text, "mph") || striequals(text, "mi/h") || striequals(text, "mi/hr")) {
        *unit = UNIT_MILES_PER_HOUR;
    } else if (striequals(text, "mv")) {
        *unit = UNIT_MILLIVOLTS;
    } else if (striequals(text, "ma")) {
        *unit = UNIT_MILLIAMPS;
    } else if (striequals(text, "v")) {
        *unit = UNIT_VOLTS;
    } else if (striequals(text, "a")) {
        *unit = UNIT_AMPS;
    } else if (striequals(text, "m")) {
        *unit = UNIT_METERS;
    } else if (striequals(text, "cm")) {
        *unit = UNIT_CENTIMETERS;
    } else if (striequals(text, "ft")) {
        *unit = UNIT_FEET;
    } else if (striequals(text, "deg/s")) {
        *unit = UNIT_DEGREES_PER_SECOND;
    } else if (striequals(text, "rad/s")) {
        *unit = UNIT_RADIANS_PER_SECOND;
    } else if (striequals(text, "g")) {
        *unit = UNIT_GS;
    } else if (striequals(text, "m/s2")) {
        *unit = UNIT_METERS_PER_SECOND_SQUARED;
    } else if (striequals(text, "raw")) {
        *unit = UNIT_RAW;
    } else if (striequals(text, "ms")) {
        *unit = UNIT_MILLISECONDS;
    } else if (striequals(text, "s")) {
        *unit = UNIT_SECONDS;
    } else if (striequals(text, "us")) {
        *unit = UNIT_MICROSECONDS;
    } else if (striequals(text, "flags")) {
        *unit = UNIT_FLAGS;
    } else {
        return false;
    }

    return true;
}
