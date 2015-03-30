#ifndef BATTERY_H_
#define BATTERY_H_

typedef struct currentMeterState_t {
    uint32_t lastTime;

    double energyMilliampHours;
    int32_t currentMilliamps;
} currentMeterState_t;

void currentMeterInit(currentMeterState_t *state);
void currentMeterUpdateVirtual(currentMeterState_t *state, int16_t currentMeterOffset, int16_t currentMeterScale, uint32_t throttle, uint32_t time);
void currentMeterUpdateMeasured(currentMeterState_t *state, int16_t amperageMilliamps, uint32_t time);

#endif
