
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
  motor.enc_pin[0] = 6;
  motor.enc_pin[1] = 7;
  motor.mot_pin[0] = 3;
  motor.mot_pin[1] = 4;
  
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

  if (millis()-pre_time >= 100)
  {
    pre_time = millis();

    LOG_SERIAL.print(motor.counter);
    LOG_SERIAL.print("  ");
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
