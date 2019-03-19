#include "moves.h"
#define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

#define BAUDRATE  1000000
Bioloid::Bioloid(){
  DynamixelWorkbench dxl_wb;
  
}
void Bioloid::initialization()
{
  model_number = 0;
  dxl_wb.init(DEVICE_NAME, BAUDRATE);
  for (int i = 0; i<20; i++){
    dxl_id[i]=i+1;
  }
  for (int cnt = 0; cnt < 12; cnt++)
  {
    dxl_wb.ping(dxl_id[cnt], &model_number);
    dxl_wb.jointMode(dxl_id[cnt], 250, 0); //second point is speed 0-1023
  }
  dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position");
}
void Bioloid::setMotors(int m1,int m2,int m3,int m4,int m5,int m6,int m7,int m8,int m9,int m10,int m11,int m12,int m13,int m14,int m15,int m16,int m17,int m18,int m19,int m20,int delayy){
  bool result = false;
  goal_position[0]=map(m1,-150,150,0,1023);
  goal_position[1]=map(m2,-150,150,0,1023);
  goal_position[2]=map(m3,-150,150,0,1023);
  goal_position[3]=map(m4,-150,150,0,1023);
  goal_position[4]=map(m5,-150,150,0,1023);
  goal_position[5]=map(m6,-150,150,0,1023);
  goal_position[6]=map(m7,-150,150,0,1023);
  goal_position[7]=map(m8,-150,150,0,1023);
  goal_position[8]=map(m9,-150,150,0,1023);
  goal_position[9]=map(m10,-150,150,0,1023);
  goal_position[10]=map(m11,-150,150,0,1023);
  goal_position[11]=map(m12,-150,150,0,1023);
  goal_position[12]=map(m13,-150,150,0,1023);
  goal_position[13]=map(m14,-150,150,0,1023);
  goal_position[14]=map(m15,-150,150,0,1023);
  goal_position[15]=map(m16,-150,150,0,1023);
  goal_position[16]=map(m17,-150,150,0,1023);
  goal_position[17]=map(m18,-150,150,0,1023);
  goal_position[18]=map(m19,-150,150,0,1023);
  goal_position[19]=map(m20,-150,150,0,1023);
  result=dxl_wb.syncWrite(handler_index, &goal_position[0]);
  delay(delayy);
} 
void Bioloid::init(){
  setMotors(-81,80,-68,68,-14,14,-45,45,-1,1,-62,62,-100,100,51,-51,-1,1,0,0,100);
}
void Bioloid::ready_for_walk(){
  setMotors(-81,80,-68,68,-14,14,-45,45,-1,1,-70,58,-103,102,47,60,-1,1,0,0,100);
}
void Bioloid::F_R_S(){
  setMotors(-81,80,-68,68,-14,14,-45,45,-1,1,-64,64,-103,103,51,-51,-6,-7,0,0,100);
  setMotors(-81,80,-68,68,-14,14,-45,45,-7,4,-70,61,-132,96,65,-54,-10,-5,0,0,100);
  setMotors(-81,80,-68,68,-14,14,-45,45,-1,1,-70,58,-103,103,46,-60,-1,-1,0,0,100);
}
void Bioloid::F_L_S(){
  setMotors(-81,80,-68,68,-14,14,-45,45,-1,1,-61,61,-103,103,51,-51,7,16,0,0,100);
  setMotors(-81,80,-68,68,-14,14,-45,45,-4,7,-61,70,-103,132,51,-66,5,10,0,0,100);
  setMotors(-81,80,-68,68,-14,14,-45,45,-1,1,-58,70,-103,103,60,-47,-1,-1,0,0,100);
}
void Bioloid::FLT_L_M(){
  setMotors(-81,80,-68,68,-14,14,-45,45,-1,1,-70,58,-103,103,48,-57,7,16,0,0,100);
  setMotors(-81,80,-68,68,-14,14,-45,45,-4,7,-67,76,-103,132,54,-66,4,10,0,0,150);
  setMotors(-81,80,-68,68,-14,14,-45,15,-1,1,-58,64,-103,103,57,-48,-1,-1,0,0,150);
  setMotors(-81,80,-68,68,-14,14,-15,45,-1,1,-53,70,-103,103,62,-48,-19,-8,0,0,150);
  setMotors(-81,80,-68,68,-14,14,-45,45,-7,4,-76,67,-132,103,65,-51,-19,-5,0,0,150);
  setMotors(-81,80,-68,68,-14,14,-45,45,-1,1,-67,55,-103,103,48,-57,-1,-1,0,0,150);
}
