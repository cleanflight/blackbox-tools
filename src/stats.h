#ifndef STATS_H_
#define STATS_H_

typedef struct seriesStats_t {
    double m, s;
    int count;
} seriesStats_t;

void seriesStats_init(seriesStats_t *stats);

void seriesStats_append(seriesStats_t *stats, double val);
int seriesStats_getCount(seriesStats_t *stats);
double seriesStats_getMean(seriesStats_t *stats);
double seriesStats_getVariance(seriesStats_t *stats);
double seriesStats_getStandardDeviation(seriesStats_t *stats);

#endif
