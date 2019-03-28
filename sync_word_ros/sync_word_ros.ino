
//#include "move.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Byte.h>
#include  <DynamixelWorkbench.h>

#define DEVICE_NAME "3" 

#define BAUDRATE  1000000

int32_t goal_position[20];

uint16_t model_number;
uint8_t dxl_id[20];
const uint8_t handler_index = 0;

DynamixelWorkbench dxl_wb;
#define ROS_RATE 50000
ros::NodeHandle_<ArduinoHardware, 3, 3, 150, 150> nh;
//ros::NodeHandle nh;
int16_t y = 0;
int16_t x = 0;
int16_t Motion_number = 0;
bool Ball_Detect = false;
//HardwareTimer Timer(TIMER_CH4);

std_msgs::Byte state_msg;
ros::Publisher pub_state("motion_ready", &state_msg);

bool start_flag = true;
void neckY(const std_msgs::Int16& errY)
{
  if(errY.data < 300){
    y = errY.data;
    Ball_Detect = true;}
  else
    Ball_Detect = false;
}
void neckX(const std_msgs::Int16& errX)
{
  if(errX.data < 300){
    x = errX.data;
    Ball_Detect = true;}
  else
    Ball_Detect = false;
}
void motion(const std_msgs::Int16& motion){
  Motion_number = motion.data;
}
ros::Subscriber<std_msgs::Int16> neck_subY("forward_y_err", neckY );
ros::Subscriber<std_msgs::Int16> neck_subX("forward_x_err", neckX );
ros::Subscriber<std_msgs::Int16> motion_sub("forward_action", motion );

HardwareTimer Timer(TIMER_CH4);
void setup() 
{
  pinMode(13, INPUT);
  model_number = 0;
  dxl_wb.init(DEVICE_NAME, BAUDRATE);
  for (int i = 0;
  i<20; i++){
    dxl_id[i]=i+1;
  }
  for (int cnt = 0; cnt < 6; cnt++)
  {
    dxl_wb.ping(dxl_id[cnt], &model_number); //arms
    dxl_wb.jointMode(dxl_id[cnt], 250, 0); 
  }
  for (int cnt = 6; cnt < 18; cnt++)
  {
    dxl_wb.ping(dxl_id[cnt], &model_number); //legs
    dxl_wb.jointMode(dxl_id[cnt], 320, 0); 
  }
  for (int cnt = 18; cnt < 20; cnt++)
  {
    dxl_wb.ping(dxl_id[cnt], &model_number); //neck
    dxl_wb.jointMode(dxl_id[cnt], 250, 0); 
  }
  dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position");
  
  // while(!Serial); // Wait for Opening Serial Monitor
  nh.getHardware()->setBaud(1000000); 
  nh.initNode(); 
  nh.subscribe(neck_subY);
  nh.subscribe(neck_subX);
  nh.subscribe(motion_sub);
  nh.advertise(pub_state);
  //ready_for_walk(0,y);
  Timer.stop();
  Timer.setPeriod(1000);           // in microseconds
  Timer.attachInterrupt(handler_nh);
  Timer.start();
  
  
  
}

void loop() 
{  
 if(digitalRead(13) == 0){
  state_msg.data = true;
  switch (Motion_number){
  case 0:
    ready_for_walk(x,y);
    pub_state.publish(&state_msg);
    break;
  case 1:
    FFT_M(x,y);
    pub_state.publish(&state_msg);
    break;
  case 2:
    FLT_M(x,y);
    pub_state.publish(&state_msg);
    break;
  case 3:
    FRT_M(x,y);
    pub_state.publish(&state_msg);
    break;
  case 4:
    TURN_L(x,y);
    pub_state.publish(&state_msg);
    break;
  case 5:
    TURN_R(x,y);
    pub_state.publish(&state_msg);
    break;
  case 6:
    F_Shoot_L(x,y);
    pub_state.publish(&state_msg);
    break;
  case 7:
    F_Shoot_R(x,y);
    pub_state.publish(&state_msg);
    break;
  case 8:
    M_L(x,y);
    pub_state.publish(&state_msg);
    break;
  case 9:
    M_R(x,y);
    pub_state.publish(&state_msg);
    break;
  case 10:
    ready_for_walk(x,y);
    pub_state.publish(&state_msg);
    //Search_Ball();
    break;
  case 11:
    GET_UP_FRONT(0,-20);
    pub_state.publish(&state_msg);
    break;
  case 12:
    GET_UP_BACK(0,-20);
    pub_state.publish(&state_msg);
    break;
  default:
    ready_for_walk(x,y);
    pub_state.publish(&state_msg);
    break;
  }
 }
 else{
  
  ready_for_walk(x,y);
  pub_state.publish(&state_msg);
 }
  //nh.spinOnce();
  
}
void handler_nh(void){
  nh.spinOnce();
}

void wait(int t)
{
  int t1 = 0;
  t1 = millis();
  while((millis()-t1)<=t){//nh.spinOnce();}
  }
  
}
void setMotors(int m1,int m2,int m3,int m4,int m5,int m6,int m7,int m8,int m9,int m10,int m11,int m12,int m13,int m14,int m15,int m16,int m17,int m18,int m19,int m20,int delayy){
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
  wait(delayy);
} 
void ready_for_walk(int w1, int w2){
  
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-70,70,-103,102,51,-51,-1,1,w2,w1,50);
}
void FFT_M(int w1, int w2){
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-70,58,-103,103, 48,-57,  7,12,w2,w1,40);
  setMotors(-90,90,-68,68,-14,14,-45,45,-4,7,-67,76,-103,137, 54,-66,  4,10,w2,w1,40);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-58,64,-103,103, 57,-48, 0,0,w2,w1,40);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-58,70,-103,103, 57,-48, -12,-7,w2,w1,40);
  setMotors(-90,90,-68,68,-14,14,-45,45,-7,4,-76,67,-137,103, 66,-54, -10,-4,w2,w1,40);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-64,58,-103,103, 48,-57,  0,0,w2,w1,40);
}
void FLT_M(int w1, int w2){
 
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-70,58,-103,103, 48,-57,  7,16,w2,w1,50);
  setMotors(-90,90,-68,68,-14,14,-45,45,-4,7,-67,76,-103,137, 54,-66,  4,10,w2,w1,50);
  setMotors(-90,90,-68,68,-14,14,-45,15,-1,1,-58,64,-103,103, 57,-48, -1,-1,w2,w1,60);
  setMotors(-90,90,-68,68,-14,14,-15,45,-1,1,-58,70,-103,103, 57,-48, -16,-7,w2,w1,60);
  setMotors(-90,90,-68,68,-14,14,-45,45,-7,4,-76,67,-137,103, 66,-54, -10,-4,w2,w1,50);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-64,58,-103,103, 48,-57,  -1,-1,w2,w1,50);
}
void FRT_M(int w1, int w2){

  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-58,70,-103,103, 57,-48, -16,-7,w2,w1,50);
  setMotors(-90,90,-68,68,-14,14,-45,45,-7,4,-76,67,-137,103, 66,-54, -10,-4,w2,w1,50);
  setMotors(-90,90,-68,68,-14,14,-15,45,-1,1,-69,58,-103,103, 48,-57,  -1,1,w2,w1,60);
  setMotors(-90,90,-68,68,-14,14,-45,15,-1,1,-76,58,-103,103, 45,-60,  7,16,w2,w1,60);
  setMotors(-90,90,-68,68,-14,14,-45,45,-4,7,-67,76,-103,137, 54,-66,  4,10,w2,w1,50);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-58,70,-103,103, 57,-48, -1,-1,w2,w1,50);
}
void  M_R(int w1, int w2){
  setMotors(-90,90,-68,68,-14,14,-51,51,-25,25,-74,65,-110,103, 60,-51, -7,-1,w2,w1,100);
  setMotors(-90,90,-68,68,-14,14,-51,51,-10,10,-65,65,-103,103, 51,-51, -7,-1,w2,w1,70);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-70,70,-103,103,51,-51,-1,1,w2,w1,100);
}
void M_L(int w1, int w2){
  setMotors(-90,90,-68,68,-14,14,-51,51,-25,25,-65,74,-103,110, 51,-60, -1,7,w2,w1,100);
  setMotors(-90,90,-68,68,-14,14,-51,51,-10,10,-65,65,-103,103, 51,-51, -1,7,w2,w1,70);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-70,70,-103,103,51,-51,-1,1,w2,w1,100);
}
void  TURN_R(int w1, int w2){
  setMotors(-90,90,-68,68,-14,14,-31,26,-1,1,-78,69,-117,103, 60,-51, -1,-1,w2,w1,70);
  setMotors(-90,90,-68,68,-14,14,-35,35,-1,1,-69,69,-103,103, 51,-51, -1,-1,w2,w1,100);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-72,72,-103,102,51,-51,-1,1,w2,w1,200);
}
void TURN_L(int w1, int w2){
  setMotors(-90,90,-68,68,-14,14,-26,31,-1,1,-69,78,-103,117, 51,-61, -1,-1,w2,w1,70);
  setMotors(-90,90,-68,68,-14,14,-35,35,-1,1,-69,69,-103,103, 51,-51, -1,-1,w2,w1,100);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-72,72,-103,102,51,-51,-1,1,w2,w1,200);
}
void F_Shoot_R(int w1, int w2){
 
  setMotors(45,-45,-80,80,-70,70,-45,45,-1,1,-53,65,-100,100, 67,-55, -10,-10,w2,w1,200);
  setMotors(45,-45,-80,80,-70,70,-40,45,-1,1,-110,60,-85,100, 12,-55, -10,-13,w2,w1,250);
  setMotors(45,-45,-80,80,-70,70,-45,45,-1,1,-70,70,-100,100,51,-51,-1,1,w2,w1,200);
}
void F_Shoot_L(int w1, int w2){
  setMotors(45,-45,-80,80,-70,70,-45,45,-1,1,-65,53,-100,100, 55,-67, 10,10,w2,w1,200);
  setMotors(45,-45,-80,80,-70,70,-45,40,-1,1,-60,110,-100,85, 55,-12, 13,10,w2,w1,250);
  setMotors(45,45,-80,80,-70,70,-45,45,-1,1,-70,70,-100,100,51,-51,-1,1,w2,w1,200);
}
void GET_UP_BACK(int w1, int w2){
  setMotors(-26,26,34,-34,0,0,-45,45,-1,1,25,-25,-94,94,8,-8,-1,1,w2,w1,1000);
  setMotors(10,-10,75,-75,3,-3,-45,45,-1,1,25,-25,-117,117,83,-83,-1,1,w2,w1,1000);
  setMotors(-42,42,36,-36,2,-2,-48,48,-1,1,25,-25,-106,106,83,-82,-1,1,w2,w1,1000); //dop move1
  setMotors(-87,87,2,-2,2,-2,-46,46,-1,1,16,-16,-100,100,83,-83,-1,1,w2,w1,1000);
  setMotors(-84,84,-32,32,-5,5,-45,45,-1,1,-17,17,-98,98,67,-67,-1,1,w2,w1,1000);//dop move2
  setMotors(-81,81,-68,68,-14,14,-45,45,-1,1,-61,61,-100,100,51,-51,-1,1,w2,w1,1000);  
}
void GET_UP_FRONT(int w1, int w2){
  setMotors(-81,81,0,0,14,-14,-46,46,-1,1,-48,48,-67,67,30,-30,-1,1,w2,w1,600);
  setMotors(-14,14,-4,4,-3,3,-46,46,-1,1,-48,48,-67,67,30,-30,-1,1,w2,w1,600);
  setMotors(29,-29,20,-20,-104,104,-46,46,-4,4,-133,133,-114,114,94,-94,-2,2,w2,w1,600);
  setMotors(0,0,-22,22,-64,64,-46,46,-2,2,-130,130,-125,125,77,-77,-1,1,w2,w1,500);//DOP MOVE 1
  setMotors(-34,34,-86,86,-8,8,-46,46,0,0,-126,126,-138,138,56,-56,0,0,w2,w1,700);
  setMotors(-81,81,-68,68,-14,14,-45,45,-1,1,-80,80,-110,110,51,-51,-1,1,w2,w1,1000);
}
void Search_Ball(){
  while((Ball_Detect == false) && (Motion_number == 10)){
    ready_for_walk(0,0); // head up
    wait(200);
      if(Ball_Detect == true){y=0;break;}


    ready_for_walk(0,-30); // head down
    wait(200);
      if(Ball_Detect == true){y=-30;break;}

          
    ready_for_walk(0,-60); //head down
      if(Ball_Detect == true){y=-60;break;}
    wait(200);
    if (x>0){
      if(HeadLeft() == true){break;}
      if(HeadRight() == true){break;}
      TURN_L(0,y);
      TURN_L(0,y);
      TURN_L(0,y);
      TURN_L(0,y); 
    }
    else{   
      if(HeadRight() == true){break;}
      if(HeadLeft() == true){break;}
      TURN_R(0,y);
      TURN_R(0,y);
      TURN_R(0,y);
      TURN_R(0,y); 
    }  
  }
}
bool HeadLeft(){
    ready_for_walk(30,0); //head left 
    wait(300);
      if(Ball_Detect == true){x=30,y=0;return true;}
    ready_for_walk(30,-30); // 
    wait(200);
      if(Ball_Detect == true){x=30,y=-30;return true;} 
    ready_for_walk(30,-60); // 
    wait(200);
      if(Ball_Detect == true){x=30,y=-60;return true;}
    return false; 
}

bool HeadRight(){
    ready_for_walk(-30,-20); //head right 
    wait(300);
      if(Ball_Detect == true){x=-30,y=-20;return true;}
    ready_for_walk(-30,-40); // 
    wait(200);
      if(Ball_Detect == true){x=-30,y=-40;return true;} 
    ready_for_walk(-30,-55); // 
    wait(200);
      if(Ball_Detect == true){x=-30,y=-55;return true;}
    return false;
}
void setMotorsEnc(int m1,int m2,int m3,int m4,int m5,int m6,int m7,int m8,int m9,int m10,int m11,int m12,int m13,int m14,int m15,int m16,int m17,int m18,int m19,int m20,int delayy){
  bool result = false;
  goal_position[0]=m1;
  goal_position[1]=m2;
  goal_position[2]=m3;
  goal_position[3]=m4;
  goal_position[4]=m5;
  goal_position[5]=m6;
  goal_position[6]=m7;
  goal_position[7]=m8;
  goal_position[8]=m9;
  goal_position[9]=m10;
  goal_position[10]=m11;
  goal_position[11]=m12;
  goal_position[12]=m13;
  goal_position[13]=m14;
  goal_position[14]=m15;
  goal_position[15]=m16;
  goal_position[16]=m17;
  goal_position[17]=m18;
  goal_position[18]=m19;
  goal_position[19]=m20;
  result=dxl_wb.syncWrite(handler_index, &goal_position[0]);
  wait(delayy);
}
