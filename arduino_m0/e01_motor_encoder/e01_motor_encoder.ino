
#define LOG_SERIAL    SerialUSB


#define LEFT            0
#define RIGHT           1



#define MOT_L_PWM_P     3
#define MOT_L_PWM_M     4
#define ENC_L_A         2
#define ENC_L_B         5

#define MOT_R_PWM_P     6
#define MOT_R_PWM_M     8
#define ENC_R_A         7
#define ENC_R_B         9


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


motor_t motor[2];


uint8_t motorGetEncBit(motor_t *p_motor);
void motorUpdateEncPhaseA(motor_t *p_motor);
void motorUpdateEncPhaseB(motor_t *p_motor);


void motorLeftEncPhaseA_ISR(void);
void motorLeftEncPhaseB_ISR(void);
void motorRightEncPhaseA_ISR(void);
void motorRightEncPhaseB_ISR(void);


void setup()
{
  motor[LEFT].counter = 0;
  motor[LEFT].on_time = 0;
  motor[LEFT].speed = 0;
  motor[LEFT].mot_pin[0] = MOT_L_PWM_M;
  motor[LEFT].mot_pin[1] = MOT_L_PWM_P;
  motor[LEFT].enc_pin[0] = ENC_L_A;
  motor[LEFT].enc_pin[1] = ENC_L_B;

  
  motor[RIGHT].counter = 0;
  motor[RIGHT].on_time = 0;
  motor[RIGHT].speed = 0;
  motor[RIGHT].mot_pin[0] = MOT_R_PWM_P;
  motor[RIGHT].mot_pin[1] = MOT_R_PWM_M;
  motor[RIGHT].enc_pin[0] = ENC_R_B;
  motor[RIGHT].enc_pin[1] = ENC_R_A;


  for (int i=0; i<2; i++)
  {
    pinMode(motor[i].enc_pin[0], INPUT_PULLUP);
    pinMode(motor[i].enc_pin[1], INPUT_PULLUP);

    pinMode(motor[i].mot_pin[0], OUTPUT);
    pinMode(motor[i].mot_pin[1], OUTPUT);

    analogWrite(motor[i].mot_pin[0], 60);
    analogWrite(motor[i].mot_pin[1], 0);
  }  

  attachInterrupt(digitalPinToInterrupt(motor[LEFT].enc_pin[0]),  motorLeftEncPhaseA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor[LEFT].enc_pin[1]),  motorLeftEncPhaseB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor[RIGHT].enc_pin[0]), motorRightEncPhaseA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor[RIGHT].enc_pin[1]), motorRightEncPhaseB_ISR, CHANGE);




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

    LOG_SERIAL.print("L ");
    LOG_SERIAL.print(motor[LEFT].counter);
    LOG_SERIAL.print("\t  R ");
    LOG_SERIAL.print(motor[RIGHT].counter);
    LOG_SERIAL.println();
  }
}


void motorLeftEncPhaseA_ISR(void)
{
  motorUpdateEncPhaseA(&motor[LEFT]);
}

void motorLeftEncPhaseB_ISR(void)
{
  motorUpdateEncPhaseB(&motor[LEFT]);
}

void motorRightEncPhaseA_ISR(void)
{
  motorUpdateEncPhaseA(&motor[RIGHT]);
}

void motorRightEncPhaseB_ISR(void)
{
  motorUpdateEncPhaseB(&motor[RIGHT]);
}


uint8_t motorGetEncBit(motor_t *p_motor)
{
  uint8_t enc_bit = 0;
  
  if (digitalRead(p_motor->enc_pin[0]) == HIGH)
  {
    enc_bit |= (1<<0);
  }
  if (digitalRead(p_motor->enc_pin[1]) == HIGH)
  {
    enc_bit |= (1<<1);
  }

  return enc_bit;
}

void motorUpdateEncPhaseA(motor_t *p_motor)
{
  switch(motorGetEncBit(p_motor))
  {
    case 0x01:
    case 0x02:
      p_motor->counter--;
      break;

    default:
      p_motor->counter++;
      break;
  }
}

void motorUpdateEncPhaseB(motor_t *p_motor)
{
  switch(motorGetEncBit(p_motor))
  {
    case 0x00:
    case 0x03:
      p_motor->counter--;
      break;

    default:
      p_motor->counter++;
      break;
  }
}
