#ifndef UNITS_H_
#define UNITS_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum Unit {
    UNIT_RAW = 0,
    UNIT_METERS_PER_SECOND,
    UNIT_KILOMETERS_PER_HOUR,
    UNIT_MILES_PER_HOUR,
    UNIT_DEGREES_PER_SECOND,
    UNIT_RADIANS_PER_SECOND,
    UNIT_METERS_PER_SECOND_SQUARED,
    UNIT_GS,
    UNIT_MILLIVOLTS,
    UNIT_MILLIAMPS,
    UNIT_VOLTS,
    UNIT_AMPS,
    UNIT_METERS,
    UNIT_CENTIMETERS,
    UNIT_FEET,
    UNIT_MICROSECONDS,
    UNIT_MILLISECONDS,
    UNIT_SECONDS,
    UNIT_FLAGS
} Unit;

// Unit conversion defines:
#define FEET_PER_METER 3.28084
// In meters per second squared
#define ACCELERATION_DUE_TO_GRAVITY 9.80665

static const char* const UNIT_NAME[] = {
    "raw",
    "m/s",
    "km/h",
    "mi/h",
    "deg/s",
    "rad/s",
    "m/s/s",
    "g",
    "mV",
    "mA",
    "V",
    "A",
    "m",
    "cm",
    "ft",
    "us",
    "ms",
    "s",
    "flags"
};

double convertMetersPerSecondToUnit(double meterspersec, Unit unit);
bool unitFromName(const char *text, Unit *unit);

#endif
