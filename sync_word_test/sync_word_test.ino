
#include "move.h"


void setup() 
{
  Moves::initialization();
  delay(1000);
}

void loop() 
{
  Moves::ready_for_walk(0,0);
  delay(5000);
  //Moves::GET_UP_FRONT();
  for(int i = 0;i<40;i++)
  {
    Moves::FFT_M(0,0);
   }
    for(int i = 0;i<10;i++)
  {
    Moves::FLT_M(0,0);
   }
    for(int i = 0;i<10;i++)
  {
    Moves::FRT_M(0,0);
   }
   delay(1000);
     for(int i = 0;i<10;i++)
  {
    Moves::M_R(0,0);
   }
    for(int i = 0;i<10;i++)
  {
    Moves::M_L(0,0);
   }
   delay(1000);
     for(int i = 0;i<10;i++)
  {
    Moves::TURN_R(0,0);
   }
    for(int i = 0;i<10;i++)
  {
    Moves::TURN_L(0,0);
   }
   
   
   
} 
