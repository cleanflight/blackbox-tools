#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "expo.h"

struct expoCurve_t {
    int offset;
    double *curve;
    double inputScale;
    int steps;
};

double expoCurveLookup(expoCurve_t *curve, double input)
{
    double normalisedInput, valueInCurve;
    int prevStepIndex;

    input += curve->offset;

    normalisedInput = input * curve->inputScale;

    //Straight line
    if (curve->steps == 1)
        return normalisedInput * curve->curve[0];

    valueInCurve = fabs(normalisedInput);
    prevStepIndex = (int) valueInCurve;

    /* If the input value lies beyond the stated input range, use the final
     * two points of the curve to extrapolate out (the "curve" out there is a straight line, though)
     */
    if (prevStepIndex > curve->steps - 2) {
        prevStepIndex = curve->steps - 2;
    }

    //Straight-line interpolation between the two curve points
    double proportion = valueInCurve - prevStepIndex;
    double result = curve->curve[prevStepIndex] + (curve->curve[prevStepIndex + 1] - curve->curve[prevStepIndex]) * proportion;

    if (input < 0)
        return -result;
    return result;
}

/**
 * Offset is subtracted from input values, then they're divided by inputRange. They're raised to the power you
 * supply. The output range for inputs in the range [-inputRange...inputRange] will lie in [-outputRange...outputRange].
 *
 * Steps changes the resolution of the approximation of the power curve, more steps approximates it better. Use 1 or 2
 * steps if you want to model a straight line (power is forced to 1.0).
 */
expoCurve_t *expoCurveCreate(int offset, double power, double inputRange, double outputRange, int steps)
{
    expoCurve_t *result;

    if (steps <= 2 || power == 1.0) {
        //Curve is actually a straight line
        steps = 1;
        power = 1.0;
    }

    result = malloc(sizeof(*result));
    result->offset = offset;
    result->steps = steps;
    result->curve = malloc(steps * sizeof(*result->curve));

    if (steps == 1) {
        //Straight line
        result->inputScale = 1.0 / inputRange;
        result->curve[0] = outputRange;
    } else {
        double stepSize = 1.0 / (steps - 1);

        result->inputScale = (steps - 1) / inputRange;

        for (int i = 0; i < steps; i++) {
            result->curve[i] = pow(i * stepSize, power) * outputRange;
        }
    }

    return result;
}

void expoCurveDestroy(expoCurve_t *curve)
{
    free(curve->curve);
    free(curve);
}
