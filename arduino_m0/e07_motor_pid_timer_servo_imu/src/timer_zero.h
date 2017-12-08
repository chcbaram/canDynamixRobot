#ifndef _TIMER_ZERO_H_
#define _TIMER_ZERO_H_


#include <Arduino.h>

#define TIMER_ZERO_MAX_CH   2


#define TIMER_ZERO_CH1      0
#define TIMER_ZERO_CH2      1


void timerZeroBegin(void);
void timerZeroSet(uint8_t ch, uint32_t ms, void (*f)());
void timerZeroStart(uint8_t ch);
void timerZeroStop(uint8_t ch);

#endif

