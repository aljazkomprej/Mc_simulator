/* measure_time.h */

#ifndef MEASURE_TIME_H
#define MEASURE_TIME_H

typedef struct
{
	uint16_t start;
	uint16_t stop;
	uint16_t dtime;
	uint16_t max;
	uint16_t min;
} measure_time_t;
extern  measure_time_t mt1;


extern void init_cctimer3_for_time_measurement(void);
extern void measure_time_init(measure_time_t *mt);
extern void measure_time_start(measure_time_t *mt);
extern void measure_time_stop(measure_time_t *mt);


#endif /* MEASURE_TIME_H */