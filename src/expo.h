#ifndef EXPO_H_
#define EXPO_H_

typedef struct expoCurve_t expoCurve_t;

expoCurve_t *expoCurveCreate(int offset, double power, double inputRange, double outputRange, int steps);
void expoCurveDestroy(expoCurve_t *curve);
double expoCurveLookup(expoCurve_t *curve, double input);

#endif
