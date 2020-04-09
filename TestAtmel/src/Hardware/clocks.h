#include "ttypes.h"

#ifndef CLOCKS_H_
#define CLOCKS_H_

// at 1KHz 16 bit time yeilds max of 65 seconds; 32 bits yeilds 48.5 days
// one second of raw time for reference and conversions Crystal is 32.768KHz; prescaler /32
#define ONE_SECOND 1024

Long raw_time();
void show_timer();
void init_clocks(void);
void set_alarm(Long t);
void over_due();

#endif
