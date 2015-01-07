/**
 * This IMU code is used for attitude estimation, and is directly derived from Baseflight's imu.c.
 */
#include <stdint.h>

//For msvcrt to define M_PI:
#define _USE_MATH_DEFINES

#include <math.h>

#include "imu.h"

#define RAD    (M_PI / 180.0f)

enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE
};

typedef enum {
    X = 0,
    Y,
    Z
} sensor_axis_e;

#define abs(x) ((x) > 0 ? (x) : -(x))

//Settings that would normally be set by the user in MW config:
static const uint16_t gyro_cmpf_factor = 600;
static const float accz_lpf_cutoff = 5.0f;
static const float magneticDeclination = 0.0f;

//IMU fields:
static float fc_acc;

void imuInit(void)
{
    fc_acc = (float) (0.5f / (M_PI * accz_lpf_cutoff)); // calculate RC time constant used in the accZ lpf
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)gyro_cmpf_factor + 1.0f))

static t_fp_vector EstG;

static void normalizeVector(struct fp_vector *src, struct fp_vector *dest)
{
    float length;

    length = sqrtf(src->X * src->X + src->Y * src->Y + src->Z * src->Z);
    if (length != 0) {
        dest->X = src->X / length;
        dest->Y = src->Y / length;
        dest->Z = src->Z / length;
    }
}

static void rotateVector(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta[ROLL]);
    sinx = sinf(delta[ROLL]);
    cosy = cosf(delta[PITCH]);
    siny = sinf(delta[PITCH]);
    cosz = cosf(delta[YAW]);
    sinz = sinf(delta[YAW]);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}

t_fp_vector calculateAccelerationInEarthFrame(int16_t accSmooth[3], attitude_t *attitude, uint16_t acc_1G)
{
    float rpy[3];
    t_fp_vector result;

    // Rotate the accel values into the earth frame
    rpy[0] = -attitude->roll;
    rpy[1] = -attitude->pitch;
    rpy[2] = -attitude->heading;

    result.V.X = accSmooth[0];
    result.V.Y = accSmooth[1];
    result.V.Z = accSmooth[2];

    rotateVector(&result.V, rpy);

    result.V.Z -= acc_1G;

    return result;
}

// baseflight calculation by Luggi09 originates from arducopter
static float calculateHeading(t_fp_vector *vec, float angleradRoll, float angleradPitch)
{
    float cosineRoll = cosf(angleradRoll);
    float sineRoll = sinf(angleradRoll);
    float cosinePitch = cosf(angleradPitch);
    float sinePitch = sinf(angleradPitch);
    float Xh = vec->A[X] * cosinePitch + vec->A[Y] * sineRoll * sinePitch + vec->A[Z] * sinePitch * cosineRoll;
    float Yh = vec->A[Y] * cosineRoll - vec->A[Z] * sineRoll;
    float hd = (float) (atan2f(Yh, Xh) + magneticDeclination / 10.0f * RAD);

    if (hd < 0)
        hd += (float) (2 * M_PI);

    return hd;
}

void getEstimatedAttitude(int16_t gyroData[3], int16_t accSmooth[3], uint32_t currentTime, uint16_t acc_1G, float gyroScale, attitude_t *attitude)
{
    int32_t accMag = 0;
    static t_fp_vector EstN = { .A = { 1.0f, 0.0f, 0.0f } };
    static uint32_t previousTime = 0;
    uint32_t deltaTime;
    float scale, deltaGyroAngle[3];

    if (previousTime == 0) {
    	deltaTime = 1;
    } else {
		deltaTime = currentTime - previousTime;
	}

	scale = deltaTime * gyroScale;
	previousTime = currentTime;

    // Initialization
    for (int axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyroData[axis] * scale;

        accMag += (int32_t)accSmooth[axis] * accSmooth[axis];
    }
    accMag = accMag * 100 / ((int32_t)acc_1G * acc_1G);

    rotateVector(&EstG.V, deltaGyroAngle);

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) {
        for (int axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)gyro_cmpf_factor + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    // Attitude of the estimated vector
    attitude->roll = atan2f(EstG.V.Y, EstG.V.Z);
    attitude->pitch = atan2f(-EstG.V.X, sqrtf(EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z));

	rotateVector(&EstN.V, deltaGyroAngle);
	normalizeVector(&EstN.V, &EstN.V);
	attitude->heading = calculateHeading(&EstN, attitude->roll, attitude->pitch);
}
