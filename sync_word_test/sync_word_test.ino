
#include "move.h"
#include "I2Cdev.h"
#include "MPU6050.h"
/*#include "Wire.h"
#define TIMER_RATE 10000 // 100 Hz
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

HardwareTimer Timer(TIMER_CH1);

int8_t Fall_flag = 0; //0 is okay 1 is forward fall -1 if backward fall
*/
void setup() 
{ 
  /*Wire.begin();
  accelgyro.initialize();
  Timer.stop();
  Timer.setPeriod(TIMER_RATE);           // in microseconds
  Timer.attachInterrupt(gyro_handler);
  Timer.start();*/
  Moves::initialization();
  delay(1000);
}

void loop() 
{
  //if(Fall_flag == 0)
  //{
  Moves::ready_for_walk(0,0);
  delay(4000);
  //Moves::GET_UP_FRONT();
  for(int i = 0;i<80;i++)
  {Moves::FFT_M(0,0);
  //Moves::FFT_M(0,0);
  //Moves::FFT_M(0,0);
  //Moves::FFT_M(0,0);
   //Moves::M_R(0,0);
    //delay(1000);
    //Moves::TURN_L(0,0);
    //Moves::FFT_M(0,0);
   //
  // Moves::FFT_U(1,0,0);
   //Moves::F_Shoot_R(0,0);
   //delay(1000);
   //Moves::GET_UP_BACK(0,0);
    //Moves::GET_UP_FRONT(0,0); 
  Moves::Check();

    
   }
   
  /*}
  else if(Fall_flag==1)
  {
    Moves::GET_UP_FRONT(0,0);  
  }
  else if(Fall_flag == -1)
  {
    Moves::GET_UP_BACK(0,0);  
  }*/
} 
/*
void gyro_handler(){
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if(ax > 6000)
  { 
    Fall_flag = 1; 
  }
  else if(ax < -6000)
  {
      Fall_flag = -1;  
  }
  else
  {
      Fall_flag = 0;  
  }  
  
}*/
