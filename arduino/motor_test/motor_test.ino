




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
  motor.enc_pin[0] = 3;
  motor.enc_pin[1] = 7;
  motor.mot_pin[0] = 9;
  motor.mot_pin[1] = 10;
  
  pinMode(motor.enc_pin[0], INPUT_PULLUP);
  pinMode(motor.enc_pin[1], INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motor.enc_pin[0]), motor_isr, CHANGE);

  analogWrite(motor.mot_pin[0], 0);
  analogWrite(motor.mot_pin[1], 0);

  Serial.begin(115200);
}

void loop()
{
  static uint8_t state = 0;
  static uint32_t pre_time = millis();
  static uint32_t pre_time_delay = millis();
  static uint8_t  pwm = 0;
  /*
  if (Serial.available())
  {
    uint8_t ch;

    ch = Serial.read();

    if (ch == '1')
    {
      analogWrite(motor.mot_pin[0], 50);
    }
    else if (ch == '2')
    {
      analogWrite(motor.mot_pin[0], 100);
    }
    else if (ch == '3')
    {
      analogWrite(motor.mot_pin[0], 150);
    }
    else if (ch == '4')
    {
      analogWrite(motor.mot_pin[0], 200);
    }
    else if (ch == '5')
    {
      analogWrite(motor.mot_pin[0], 250);
    }
    else if (ch == '0')
    {
      analogWrite(motor.mot_pin[0], 0);
    }
  }
  */
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

    Serial.print(pwm);
    Serial.print("  ");
    //Serial.print(motor.on_time);
    Serial.print("  ");
    Serial.print(motor.speed);
    Serial.print("  ");
    //Serial.println(motor.counter);
    Serial.println();
  }
}


void motor_isr(void)
{
  motor_enc_count();
}

void motor_enc_count(void)
{
  uint8_t enc_bit = 0;

    if (digitalRead(motor.enc_pin[0]) == HIGH)
    {
      motor.start_time = micros();
      enc_bit |= (1<<0);
    }
    else
    {
      motor.on_time = micros() - motor.start_time;
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
