
#ifndef __POWER_PROFILING_H__
#define __POWER_PROFILING_H__

#include <stdint.h>

extern void power_profiling_start(void);
extern void power_profiling_stop(void);
extern void power_profiling_set_period(uint32_t us);
extern int  power_profiling_read(uint8_t *data, uint16_t size);

#endif // __POWER_PROFILING_H__
