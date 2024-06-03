#include <Encoder.h>
#include <util/atomic.h>

#define enca_right 2
#define encb_right 10

#define enca_left 3
#define encb_left 11

long prevt = 0;
float v1_global = 0;


float v1_global_2 = 0;

long prevt_2 = 0;

float rightEncoder = 0;
float leftEncoder = 0;




void initialize_encoder()
{
  pinMode(enca_right, INPUT);
  
  pinMode(encb_right, INPUT);


  pinMode(enca_left, INPUT);

  pinMode(encb_left, INPUT);
  

  attachInterrupt(digitalPinToInterrupt(enca_right), readencoder_right, RISING);
  attachInterrupt(digitalPinToInterrupt(enca_left), readencoder_left, RISING);
}


float time()
{
  long currt = micros();
  float deltat = ((float)(currt - prevt)) / 1.0e6;
  prevt = currt;
  return deltat;
}

float time_2()
{
  long currt_2 = micros();
  float deltat_2 = ((float)(currt_2 - prevt_2)) / 1.0e6;
  prevt_2 = currt_2;
  return deltat_2;
}

void readencoder_right()
{
  int b = digitalRead(encb_right);
  int incr = 0;
  if (b < 1)
  {
    incr = 1;
  }
  else
  {
    incr = -1;                     
  }
  float deltat_encoder = time();
  rightEncoder = incr / deltat_encoder;
}

void readencoder_left()
{
  int b_2 = digitalRead(encb_left);
  int incr_2 = 0;
  if (b_2 < 1)
  {
    incr_2 = 1;
  }
  else
  {
    incr_2 = -1;                
  }
  float deltat_encoder_2 = time_2();
  leftEncoder = incr_2 / deltat_encoder_2;
}