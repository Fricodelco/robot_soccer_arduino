
#include "move.h"


void setup() 
{
  Moves::initialization();
  delay(1000);
}

void loop() 
{
  Moves::ready_for_walk();
  delay(5000);
  //Moves::GET_UP_FRONT();
  for(int i = 0;i<40;i++)
  {
    Moves::FFT_M();
   }
    for(int i = 0;i<10;i++)
  {
    Moves::FLT_M();
   }
    for(int i = 0;i<10;i++)
  {
    Moves::FRT_M();
   }
   delay(1000);
     for(int i = 0;i<10;i++)
  {
    Moves::M_R();
   }
    for(int i = 0;i<10;i++)
  {
    Moves::M_L();
   }
   delay(1000);
     for(int i = 0;i<10;i++)
  {
    Moves::TURN_R();
   }
    for(int i = 0;i<10;i++)
  {
    Moves::TURN_L();
   }
   
   
   
} 
