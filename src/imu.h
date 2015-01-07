#ifndef IMU_H_
#define IMU_H_

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

// Angles in radians:
typedef struct attitude_t {
	float roll;
	float pitch;
	float heading;
} attitude_t;

void imuInit(void);
void getEstimatedAttitude(int16_t gyroData[3], int16_t accSmooth[3], uint32_t currentTime, uint16_t acc_1G, float gyroScale, attitude_t *attitude);
t_fp_vector calculateAccelerationInEarthFrame(int16_t accSmooth[3], attitude_t *attitude, uint16_t acc_1G);

#endif
