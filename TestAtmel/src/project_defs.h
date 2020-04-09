#include <interrupt.h>
#include <cycle_counter.h>

#ifndef PROJECT_DEFS_H_
#define PROJECT_DEFS_H_

// 8 MHz clock ticks; wraps after 8.9 minutes; 125 ns resolution
// cpu_cy_2_us()
#define CLOCK_MHZ 8
#define getTicks() Get_sys_count()

#define safe(code) 	\
	AVR32_ENTER_CRITICAL_REGION() \
	code; \
	AVR32_LEAVE_CRITICAL_REGION()

#define NUM_ACTIONS 40
#define NUM_TE 40

#define N_EVENTS 800
#define FIRST_EVENT (const char *)secs(5)

#endif
