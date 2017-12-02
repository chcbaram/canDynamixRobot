#include "timer_zero.h"

#define LOG_SERIAL    SerialUSB



volatile uint32_t timer_cnt[2] = {0, };




void ledTest(void)
{
  digitalWrite(9, !digitalRead(9));
  timer_cnt[0]++;
}

void ledTest2(void)
{
  digitalWrite(10, !digitalRead(10));
  timer_cnt[1]++;
}


void setup()
{
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  timerZeroBegin();
  timerZeroSet(TIMER_ZERO_CH1, 10, ledTest);
  timerZeroSet(TIMER_ZERO_CH2,  5, ledTest2);


  timerZeroStart(TIMER_ZERO_CH1);
  timerZeroStart(TIMER_ZERO_CH2);

  // Wait for synchronization.
  LOG_SERIAL.begin(115200);
}

void loop()
{
  static uint8_t state = 0;
  static uint32_t pre_time = millis();
  static uint32_t pre_time_delay = millis();
  static uint8_t  pwm = 0;

  if (millis()-pre_time >= 100)
  {
    pre_time = millis();

    LOG_SERIAL.print("Timer ch1 ");
    LOG_SERIAL.print(timer_cnt[0]);
    LOG_SERIAL.print("\t  Timer ch2 ");
    LOG_SERIAL.print(timer_cnt[1]);
    LOG_SERIAL.println();
  }
}


