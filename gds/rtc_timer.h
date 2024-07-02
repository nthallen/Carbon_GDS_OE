#ifndef RTC_TIMER_H_INCLUDED
#define RTC_TIMER_H_INCLUDED
#include "hal_timer.h"
#include "subbus.h"

//#define RTC_USE_MAX_DURATION_REFERENCE

// int32_t uDACS_timer_start(struct timer_descriptor *const descr);
#define RTC_BASE_ADDR 0x40
#define RTC_ELAPSED_OFFSET 0
#define RTC_CUR_STATE_DURATION_OFFSET 2
#define RTC_MAX_STATE_DURATION_OFFSET 3

#ifdef RTC_USE_MAX_DURATION_REFERENCE

extern uint16_t rtc_max_state_duration_ref_value;
#define RTC_MAX_DURATION_REF_OFFSET 4
#define RTC_HIGH_ADDR 0x44

#else

#define RTC_HIGH_ADDR 0x43

#endif

#define RTC_COUNTS_PER_SECOND 100000
#define RTC_COUNTS_PER_MSEC (RTC_COUNTS_PER_SECOND/1000)
extern subbus_driver_t sb_rtc;
extern uint32_t rtc_current_count;

#endif

