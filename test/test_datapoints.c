#include <stdint.h>
#include <stdio.h>
#include <assert.h>

#include "../src/datapoints.h"

#define NUM_EXAMPLE_VALS 8


int main(void)
{
	char *fieldNames[] = {"Test"};
	int32_t val;

	//First some basic tests about locating frames
	{
		datapoints_t *points = datapointsCreate(1, fieldNames, 2);

		val = 42;
		datapointsAddFrame(points, 3, &val);
		val = 113;
		datapointsAddFrame(points, 6, &val);

		assert(datapointsFindFrameAtTime(points, 0) == -1);
		assert(datapointsFindFrameAtTime(points, 3) == 0);
		assert(datapointsFindFrameAtTime(points, 4) == 0);
		assert(datapointsFindFrameAtTime(points, 6) == 1);
		assert(datapointsFindFrameAtTime(points, 8) == 1);
		assert(!datapointsGetGapStartsAtIndex(points, 0));

		datapointsDestroy(points);
	}

	//Test smoothing partitioning by making every value its own partition
	{
		datapoints_t *points;

		int exampleVals[NUM_EXAMPLE_VALS] = {3, 7, 1, 28, 105, -1, 8, 13};

		points = datapointsCreate(1, fieldNames, NUM_EXAMPLE_VALS + 3 /* Make it a little bigger than we'll fill to test that too */);

		for (int i = 0; i < NUM_EXAMPLE_VALS; i++) {
			val = exampleVals[i];
			datapointsAddFrame(points, i, &val);
			datapointsAddGap(points);
		}

		datapointsSmoothField(points, 0, 2);

		for (int i = 0; i < NUM_EXAMPLE_VALS; i++) {
			assert(datapointsGetFieldAtIndex(points, i, 0, &val));
			assert(val == exampleVals[i]);
		}

		datapointsDestroy(points);
	}

	//Mixture of partition sizes
	{
		datapoints_t *points;

		int exampleVals[NUM_EXAMPLE_VALS] = {3, 7, 1, 28, 105, -1,  8, 13};
		int gaps[NUM_EXAMPLE_VALS] =        {0, 1, 1,  1,   0,  0,  0,  0};
		int smoothed[NUM_EXAMPLE_VALS] =    {5, 5, 1, 28,  37, 31, 31,  6};

		points = datapointsCreate(1, fieldNames, NUM_EXAMPLE_VALS);

		for (int i = 0; i < NUM_EXAMPLE_VALS; i++) {
			val = exampleVals[i];
			datapointsAddFrame(points, i, &val);

			if (gaps[i])
				datapointsAddGap(points);
		}

		datapointsSmoothField(points, 0, 2);

		for (int i = 0; i < NUM_EXAMPLE_VALS; i++) {
			assert(datapointsGetFieldAtIndex(points, i, 0, &val));
			assert(val == smoothed[i]);
		}

		datapointsDestroy(points);
	}

	printf("Done\n");

	return 0;
}
