#include <stdint.h>
#include <stdio.h>
#include <assert.h>

#include "../src/expo.h"

int main(void)
{
	expoCurve_t *curve = expoCurveCreate(0, 0.700, 750, 1.0, 10);

	assert(expoCurveLookup(curve, 0) == 0.0);
	assert(expoCurveLookup(curve, -750) == -1.0);
	assert(expoCurveLookup(curve, 750) == 1.0);

	//Straight line
	curve = expoCurveCreate(0, 1.0, 500, 1.0, 1);
	assert(expoCurveLookup(curve, 0) == 0.0);
	assert(expoCurveLookup(curve, -500) == -1.0);
	assert(expoCurveLookup(curve, 500) == 1.0);
	assert(expoCurveLookup(curve, -250) == -0.5);
	assert(expoCurveLookup(curve, 250) == 0.5);


	printf("Done\n");

	return 0;
}
