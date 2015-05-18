#include "blackbox_fielddefs.h"

const char * const FLIGHT_LOG_FLIGHT_MODE_NAME[] = {
    "ANGLE_MODE",
    "HORIZON_MODE",
    "MAG",
    "BARO",
    "GPS_HOME",
    "GPS_HOLD",
    "HEADFREE",
    "AUTOTUNE",
    "PASSTHRU",
    "SONAR"
};

const char * const FLIGHT_LOG_FLIGHT_STATE_NAME[] = {
    "GPS_FIX_HOME",
    "GPS_FIX",
    "CALIBRATE_MAG",
    "SMALL_ANGLE",
    "FIXED_WING"
};

const char * const FLIGHT_LOG_FAILSAFE_PHASE_NAME[] = {
    "IDLE",
    "RX_LOSS_DETECTED",
    "LANDING",
    "LANDED"
};
