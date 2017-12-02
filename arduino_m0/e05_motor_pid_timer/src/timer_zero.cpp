#include "timer_zero.h"


#define TIMER_NUMBER      TC3



typedef struct
{
  bool     enable;
  uint32_t time_period;
  void     (*callBackFunc)();
} timer_zero_t;



static timer_zero_t timer_zero[TIMER_ZERO_MAX_CH];


void timerZeroBegin(void)
{
  uint32_t i;


  for (i=0; i<TIMER_ZERO_MAX_CH; i++)
  {
    timer_zero[i].enable = false;   
    timer_zero[i].time_period = 0;   
    timer_zero[i].callBackFunc = NULL; 
  }

  // Enable clock for TC
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  // The type cast must fit with the selected timer mode
  TcCount16* TC = (TcCount16*) TC3; // get timer struct

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_NFRQ; // Set TC as normal Normal Frq
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64;   // Set perscaler
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
 
 
  // Interrupts
  TC->INTENSET.reg = 0;                // disable all interrupts
  
  // Enable InterruptVector
  NVIC_EnableIRQ(TC3_IRQn);

  // Enable TC
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void timerZeroSet(uint8_t ch, uint32_t ms, void (*f)())
{
  uint32_t time_us;
  float clk_freq;
  float clk_time;
  TcCount16* TC = (TcCount16*) TIMER_NUMBER; 


  if (ch >= TIMER_ZERO_MAX_CH)
  {
    return;
  }
  
  time_us = ms * 1000; 
  clk_freq = 48.0 / 64; 
  clk_time = 1.0 / clk_freq;


  switch(ch)
  {
    case TIMER_ZERO_CH1:
      TC->CC[0].reg = (uint16_t)(time_us / clk_time);
      while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync          
      timer_zero[TIMER_ZERO_CH1].time_period  = (uint16_t)(time_us / clk_time);
      timer_zero[TIMER_ZERO_CH1].callBackFunc = f;
      timer_zero[TIMER_ZERO_CH1].enable       = true;
      break;

    case TIMER_ZERO_CH2:
      TC->CC[1].reg = (uint16_t)(time_us / clk_time);
      while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync      
      timer_zero[TIMER_ZERO_CH2].time_period  = (uint16_t)(time_us / clk_time);
      timer_zero[TIMER_ZERO_CH2].callBackFunc = f;
      timer_zero[TIMER_ZERO_CH2].enable       = true;
      break;      
  }
}

void timerZeroStart(uint8_t ch)
{
  TcCount16* TC = (TcCount16*) TIMER_NUMBER; 


  if (ch >= TIMER_ZERO_MAX_CH)
  {
    return;
  }

  switch(ch)
  {
    case TIMER_ZERO_CH1:
      TC->INTENSET.bit.MC0 = 1; 
      break;

    case TIMER_ZERO_CH2:
      TC->INTENSET.bit.MC1 = 1; 
      break;      
  }  
}

void timerZeroStop(uint8_t ch)
{
  TcCount16* TC = (TcCount16*) TIMER_NUMBER; 


  if (ch >= TIMER_ZERO_MAX_CH)
  {
    return;
  }

  switch(ch)
  {
    case TIMER_ZERO_CH1:
      TC->INTENSET.bit.MC0 = 0; 
      break;

    case TIMER_ZERO_CH2:
      TC->INTENSET.bit.MC1 = 0; 
      break;      
  }    
}


void TC3_Handler()
{
  uint32_t i;
  TcCount16* TC = (TcCount16*) TIMER_NUMBER; 

  
  if (TC->INTFLAG.bit.MC0 == 1) 
  {  
    TC->INTFLAG.bit.MC0 = 1;    

    if (timer_zero[0].enable == true)
    {
      TC->CC[0].reg = (TC->CC[0].reg + timer_zero[0].time_period) % 0xFFFF;

      if (timer_zero[0].callBackFunc != NULL)
      {
        (*timer_zero[0].callBackFunc)();
      }
    }
  }

  if (TC->INTFLAG.bit.MC1 == 1) 
  {  
    TC->INTFLAG.bit.MC1 = 1;    

    if (timer_zero[1].enable == true)
    {
      TC->CC[1].reg = (TC->CC[1].reg + timer_zero[1].time_period) % 0xFFFF;

      if (timer_zero[1].callBackFunc != NULL)
      {
        (*timer_zero[1].callBackFunc)();
      }
    }
  }  
}