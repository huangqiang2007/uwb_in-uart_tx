#ifndef __INLCUDE_TIMER_H_
#define __INLCUDE_TIMER_H_

#include <stdint.h>
#include <stdlib.h>

/*
 * CMD feedback timeout judge
 *
 * represent 30 * 1ms = 1s timeout
 * */
#define CMD_FEEDBACK_TIMEOUT 5 /* receive wait response timeout 10ms*/
volatile uint32_t g_cmd_feedback_timeout; /* the count of 1ms unit */

/*
 * wakeup duration
 *
 * 600000 * 1ms = 10 minutes
 * */
#define WAKUP_DURATION 610000 /* wake up time 10 minutes */
volatile uint32_t g_wakup_timeout; /* the count of 1ms unit */

/*
 * slave does not receive new CMD during the below duration, enter into sleep mode.
 *
 * 30000 * 1ms = 5 minutes
 * */
#define IDLE_JUDGE 30000
volatile uint32_t g_idle_judge;

void setupTimer0(void);
void setupTimer1(void);
extern void Delay_ms(uint32_t ms);
extern void Delay_us(uint32_t us);
extern void timerInit(void);

#endif /* INLCUDE_TIMER_H_ */
