#include <math.h>

#include "stats.h"

/*
 * Compute summary statistics from a series of data in an online fashion.
 *
 * Derived from http://www.johndcook.com/blog/standard_deviation/ which in turn is dervied from
 * Donald Knuth’s Art of Computer Programming, Vol 2, page 232, 3rd edition
 */

int seriesStats_getCount(seriesStats_t *stats)
{
    return stats->count;
}

void seriesStats_init(seriesStats_t *stats)
{
    stats->count = 0;
}

void seriesStats_append(seriesStats_t *stats, double val)
{
    if (stats->count == 0) {
        stats->m = val;
        stats->s = 0.0;
    } else {
        double oldM = stats->m;

        stats->m = oldM + (val - oldM) / stats->count;
        stats->s = stats->s + (val - oldM) * (val - stats->m);
    }

    stats->count++;
}

double seriesStats_getMean(seriesStats_t *stats)
{
    return stats->count > 0 ? stats->m : 0.0;
}

double seriesStats_getVariance(seriesStats_t *stats)
{
    return stats->count > 1 ? stats->s / (stats->count - 1) : 0.0;
}

double seriesStats_getStandardDeviation(seriesStats_t *stats)
{
    return sqrt(seriesStats_getVariance(stats));
}
