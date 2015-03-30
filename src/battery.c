#include <stdint.h>

#include "battery.h"

#define SECONDS_PER_HOUR 3600u
#define MICROSECONDS_PER_SECOND 1000000u
// Careful, it nearly overflows a uint32_t:
#define MICROSECONDS_PER_HOUR (SECONDS_PER_HOUR * MICROSECONDS_PER_SECOND)

#define MILLIAMPS_PER_CENTIAMP 10

/**
 * Reset/initialise the state of the passed current meter.
 */
void currentMeterInit(currentMeterState_t *state)
{
    state->lastTime = 0;

    state->energyMilliampHours = 0;
    state->currentMilliamps = 0;
}

/**
 * Update the state of the current meter by estimating the current using the RC throttle position and current time you provide.
 *
 * Time is an absolute time in micro-seconds (not a delta).
 */
void currentMeterUpdateVirtual(currentMeterState_t *state, int16_t currentMeterOffset, int16_t currentMeterScale, uint32_t throttle, uint32_t time)
{
    int32_t throttleOffset, throttleFactor;

    // Current consumption due to idling while armed (zero-throttle current usage):
    state->currentMilliamps = (int32_t)currentMeterOffset * MILLIAMPS_PER_CENTIAMP;

    // Current consumption based on throttle position:
    throttleOffset = (int32_t)throttle - 1000;
    throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50);

    state->currentMilliamps += throttleFactor * (int32_t)currentMeterScale  / 100;

    if (state->lastTime != 0) {
        state->energyMilliampHours += ((double) state->currentMilliamps * (time - state->lastTime)) / MICROSECONDS_PER_HOUR;
    }

    state->lastTime = time;
}

void currentMeterUpdateMeasured(currentMeterState_t *state, int16_t amperageMilliamps, uint32_t time)
{
    state->currentMilliamps = amperageMilliamps;

    if (state->lastTime != 0) {
         state->energyMilliampHours += ((double) state->currentMilliamps * (time - state->lastTime)) / MICROSECONDS_PER_HOUR;
    }

    state->lastTime = time;
}
