
#define LOG_SERIAL    SerialUSB



typedef struct
{
  int32_t  counter;

  int32_t  speed;
  int32_t  start_counter;

  int32_t  on_time;
  uint32_t start_time;

  uint8_t  enc_pin[2];
  uint8_t  mot_pin[2];
} motor_t;


motor_t motor;


void setup()
{
  motor.counter = 0;
  motor.on_time = 0;
  motor.speed = 0;
  motor.mot_pin[0] = 4;
  motor.mot_pin[1] = 3;
  motor.enc_pin[0] = 2;
  motor.enc_pin[1] = 5;
  
  pinMode(motor.enc_pin[0], INPUT_PULLUP);
  pinMode(motor.enc_pin[1], INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motor.enc_pin[0]), motor_enc_count_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor.enc_pin[1]), motor_enc_count_b, CHANGE);

  pinMode(motor.mot_pin[0], OUTPUT);
  pinMode(motor.mot_pin[1], OUTPUT);

  analogWrite(motor.mot_pin[0], 0);
  analogWrite(motor.mot_pin[1], 0);

  LOG_SERIAL.begin(115200);
}

void loop()
{
  static uint8_t state = 0;
  static uint32_t pre_time = millis();
  static uint32_t pre_time_delay = millis();
  static uint8_t  pwm = 0;

  switch(state)
  {
    case 0:
      if (millis()-pre_time_delay >= 2000)
      {
        pre_time_delay = millis();
        state = 1;
        pwm = 50;
        analogWrite(motor.mot_pin[0], pwm);
      }
      break;

    case 1:
      if (millis()-pre_time_delay >= 2000)
      {
        pre_time_delay = millis();
        state = 2;
        pwm = 100;
        analogWrite(motor.mot_pin[0], pwm);
      }
      break;

    case 2:
      if (millis()-pre_time_delay >= 2000)
      {
        pre_time_delay = millis();
        state = 3;
        pwm = 150;
        analogWrite(motor.mot_pin[0], pwm);
      }
      break;
    case 3:
      if (millis()-pre_time_delay >= 2000)
      {
        pre_time_delay = millis();
        state = 4;
        pwm = 200;
        analogWrite(motor.mot_pin[0], pwm);
      }
      break;
    case 4:
      if (millis()-pre_time_delay >= 2000)
      {
        pre_time_delay = millis();
        state = 5;
        pwm = 250;
        analogWrite(motor.mot_pin[0], pwm);
      }
      break;
    case 5:
      if (millis()-pre_time_delay >= 2000)
      {
        pre_time_delay = millis();
        state = 5;
        pwm = 0;
        analogWrite(motor.mot_pin[0], pwm);
      }
      break;
  }

  if (millis()-pre_time >= 100)
  {
    pre_time = millis();

    motor.speed = motor.counter - motor.start_counter;
    motor.start_counter = motor.counter;

    LOG_SERIAL.print(pwm);
    LOG_SERIAL.print("  ");
    LOG_SERIAL.print(motor.speed);
    LOG_SERIAL.print("  ");
    //Serial.println(motor.counter);
    LOG_SERIAL.println();
  }
}


void motor_enc_count_a(void)
{
  uint8_t enc_bit = 0;

  if (digitalRead(motor.enc_pin[0]) == HIGH)
  {
    enc_bit |= (1<<0);
  }
  if (digitalRead(motor.enc_pin[1]) == HIGH)
  {
    enc_bit |= (1<<1);
  }

  switch(enc_bit)
  {
    case 0x01:
    case 0x02:
      motor.counter--;
      break;

    default:
      motor.counter++;
      break;
  }
}

void motor_enc_count_b(void)
{
  uint8_t enc_bit = 0;

  if (digitalRead(motor.enc_pin[0]) == HIGH)
  {
    enc_bit |= (1<<0);
  }
  if (digitalRead(motor.enc_pin[1]) == HIGH)
  {
    enc_bit |= (1<<1);
  }

  switch(enc_bit)
  {
    case 0x00:
    case 0x03:
      motor.counter--;
      break;

    default:
      motor.counter++;
      break;
  }
}
