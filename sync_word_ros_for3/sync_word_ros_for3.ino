
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
int16_t y_old = 0;
int16_t x_old = 0;
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
    y_old = y;
    y = errY.data;
    Ball_Detect = true;}
    //dxl_wb.bulkWrite();}
  else
    Ball_Detect = false;
}
void neckX(const std_msgs::Int16& errX)
{
  if(errX.data < 300){
    x_old = x;
    x = errX.data;
    Ball_Detect = true;}
   // dxl_wb.bulkWrite();}
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
    dxl_wb.jointMode(dxl_id[cnt], 270, 0); 
  }
  for (int cnt = 19; cnt < 21; cnt++)
  {
    dxl_wb.ping(cnt, &model_number); //neck
    dxl_wb.jointMode(cnt, 500, 0); 
  }
  dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position");
  //dxl_wb.addBulkWriteParam(20, "Neck_X", x);
  //dxl_wb.addBulkWriteParam(19, "Neck_Y", y);
  //dxl_wb.bulkWrite();
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
  case 4:
    TURN_L(x,y,false);
    pub_state.publish(&state_msg);
    break;
  case 5:
    TURN_R(x,y,false);
    pub_state.publish(&state_msg);
    break;
  case 6:
    F_Shoot_L(x,y);
    pub_state.publish(&state_msg);
    wait(200);
    break;
  case 7:
    F_Shoot_R(x,y);
    pub_state.publish(&state_msg);
    wait(200);
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
    //ready_for_walk(x,y);
    //Head_Up_Down();
    pub_state.publish(&state_msg);
    Search_Ball();
    break;
  case 11:
    GET_UP_FRONT(x,y);
    pub_state.publish(&state_msg);
    wait(200);
    break;
  case 12:
    GET_UP_BACK(x,y);
    pub_state.publish(&state_msg);
    wait(200);
    break;
  case 13:
    CIRCLE_RIGHT(x,y);
    pub_state.publish(&state_msg);
    wait(200);
    break;
  case 14:
    CIRCLE_LEFT(x,y);
    pub_state.publish(&state_msg);
    wait(200);
    break;
  
  default:
    ready_for_walk(x,y);
    pub_state.publish(&state_msg);
    break;
  }
  Check_Alarm();
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
void Check_Alarm(){
  for(int cnt = 0;cnt<20;cnt++){
  const char *log = NULL;
  const ControlItem *control_item =  dxl_wb.getControlTable(dxl_id[cnt]);
  uint8_t the_number_of_control_item = dxl_wb.getTheNumberOfControlItem(dxl_id[cnt]);
  Serial.println("Ebanoe govno");
  uint16_t last_register_addr = control_item[the_number_of_control_item-1].address;
  uint16_t last_register_addr_length = control_item[the_number_of_control_item-1].data_length;
  uint32_t getAllRegisteredData[last_register_addr+last_register_addr_length];
  bool result;
  result = dxl_wb.readRegister(dxl_id[cnt], (uint16_t)0, last_register_addr+last_register_addr_length, getAllRegisteredData,&log);
  if(result == false || getAllRegisteredData[25] == 1){
      dxl_wb.init(DEVICE_NAME, BAUDRATE);
      for (int cnt = 0; cnt < 6; cnt++)
      {
        dxl_wb.ping(dxl_id[cnt], &model_number); //arms
        dxl_wb.jointMode(dxl_id[cnt], 250, 0); 
      }
      for (int cnt = 6; cnt < 18; cnt++)
      {
        dxl_wb.ping(dxl_id[cnt], &model_number); //legs
        dxl_wb.jointMode(dxl_id[cnt], 270, 0); 
      }
      for (int cnt = 19; cnt < 21; cnt++)
      {
        dxl_wb.ping(cnt, &model_number); //neck
        dxl_wb.jointMode(cnt, 500, 0); 
      }
      dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position");
      break;
    }
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
  goal_position[18]=map(y,-150,150,0,1023);
  goal_position[19]=map(x,-150,150,0,1023);
  result=dxl_wb.syncWrite(handler_index, &goal_position[0]);
  wait(delayy);
} 
void ready_for_walk(int w1, int w2){
  
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-67,67,-103,102,51,-51,-1,1,y,x,50);
}
void FFT_M(int w1, int w2){
  setMotors(-80,100,-68,68,-14,14,-45,45,-5,5,-67,53,-101,101, 51,-57,  4,10,y,x,100);
  setMotors(-80,100,-68,68,-14,14,-45,45,-9,12,-65,73,-101,125, 54,-66,  4,10,y,x,100);
  setMotors(-80,100,-68,68,-14,14,-45,45,-5,5,-53,67,-101,101, 57,-51, -5,5,y,x,100);

  setMotors(-100,80,-68,68,-14,14,-45,45,-5,5,-53,67,-101,101, 57,-51, -10,-4,y,x,100);
  setMotors(-100,80,-68,68,-14,14,-45,45,-12,9,-73,64,-125,101, 66,-54, -10,-4,y,x,100);
  setMotors(-100,80,-68,68,-14,14,-45,45,-5,5,-65,53,-101,101, 51,-57,  -5,5,y,x,100);
}

void  M_R(int w1, int w2){
  setMotors(-90,90,-68,68,-14,14,-51,51,-25,25,-74,65,-110,103, 60,-51, -7,-1,y,x,100);
  setMotors(-90,90,-68,68,-14,14,-51,51,-10,10,-65,65,-103,103, 51,-51, -7,-1,y,x,70);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-67,67,-103,103,51,-51,-1,1,y,x,100);
}
void M_L(int w1, int w2){
  setMotors(-90,90,-68,68,-14,14,-51,51,-25,25,-65,74,-103,110, 51,-60, -1,7,y,x,100);
  setMotors(-90,90,-68,68,-14,14,-51,51,-10,10,-65,65,-103,103, 51,-51, -1,7,y,x,70);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-67,67,-103,103,51,-51,-1,1,y,x,100);
}
void  TURN_R(int w1, int w2, bool ifsee){
  for(int i = 0;i<4;i++)
    if(ifsee == true)setMotors(-90,90,-68,68,-14,14,-45,45,-5,5,-67,67,-103,102,51,-51,-1,1,y,x,15);
  setMotors(-90,90,-68,68,-14,14,-36,21,-5,5,-75,66,-117,103, 60,-51, -1,-1,y,x,70);
  setMotors(-90,90,-68,68,-14,14,-35,35,-5,5,-66,66,-103,103, 51,-51, -1,-1,y,x,100);
  setMotors(-90,90,-68,68,-14,14,-45,45,-5,5,-69,69,-103,102,51,-51,-1,1,y,x,200);
  for(int i = 0;i<10;i++)
    if(ifsee == true)setMotors(-90,90,-68,68,-14,14,-45,45,-5,5,-67,67,-103,102,51,-51,-1,1,y,x,15);
}
void TURN_L(int w1, int w2, bool ifsee){
  for(int i = 0;i<4;i++)
    if(ifsee == true)setMotors(-90,90,-68,68,-14,14,-45,45,-5,5,-67,67,-103,102,51,-51,-1,1,y,x,15);
  setMotors(-90,90,-68,68,-14,14,-21,36,-5,5,-66,75,-103,117, 51,-61, -1,-1,y,x,70);
  setMotors(-90,90,-68,68,-14,14,-35,35,-5,5,-66,66,-103,103, 51,-51, -1,-1,y,x,100);
  setMotors(-90,90,-68,68,-14,14,-45,45,-5,5,-69,69,-103,102,51,-51,-1,1,y,x,200);
  for(int i = 0;i<10;i++)
    if(ifsee == true)setMotors(-90,90,-68,68,-14,14,-45,45,-5,5,-67,67,-103,102,51,-51,-1,1,y,x,15);
}
void F_Shoot_R(int w1, int w2){
  setMotors(-90,90,-68,68,-14,14,-45,45,-5,5,-65,65,  -103,102,51,-51,-1,1,y,x,1200);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,5,-65,63,-102,100,   40,-51, -8,-5,y,x,300);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,12,-65,63,-102,100,20,-51, -8,-11,y,x,1000);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,15,-75,65,-120,100, -10,-51, -10,-11,y,x,600);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,15,-105,63,-90,102, -30,-51, -10,-11,y,x,700);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,15,-100,63,-90,102, 0,-51, -10,-11,y,x,200);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,12,-80,63,-120,100, 30,-51, -10,-11,y,x,400);
  setMotors(-90,90,-68,68,-14,14,-45,45,-5,5,-65,65,-100,100,51,-51,-1,1,y,x,1500);

}
void F_Shoot_L(int w1, int w2){
  setMotors(-90,90,-68,68,-14,14,-45,45,-5,5,-67,67,-103,102,51,-51,-1,1,y,x,1200);
  setMotors(-90,90,-68,68,-14,14,-45,45,-5,1,-67,67,-102,100, 51,-40,5,8,y,x,700);
  setMotors(-90,90,-68,68,-14,14,-45,45,-12,1,-67,67,-102,100, 51,-15,8,8,y,x,700);
  setMotors(-90,90,-68,68,-14,14,-45,45,-15,1,-65,79,-100,120, 51,-10, 13,10,y,x,600);
  setMotors(-90,90,-68,68,-14,14,-45,45,-15,1,-65,107,-102,90, 51,30, 13,10,y,x,700);
  setMotors(-90,90,-68,68,-14,14,-45,45,-15,1,-63,97,-102,90, 51,-30, 13,10,y,x,200);
  setMotors(-90,90,-68,68,-14,14,-45,45,-12,1,-63,75,-100,120, 51,-30, 11,10,y,x,200);
  setMotors(-90,90,-68,68,-14,14,-45,45,-5,5,-65,65,-100,100,51,-51,-1,1,y,x,1500);
}
void GET_UP_BACK(int w1, int w2){
  setMotors(10,-10,75,-75,3,-3,-46,46,-1,1,26,-25,-117,117,83,-83,-1,1,y,x,500);
  setMotors(-26,26,34,-34,0,0,-45,45,-1,1,25,-25,-94,94,8,-8,-1,1,y,x,500);
  setMotors(10,-10,75,-75,3,-3,-45,45,-1,1,25,-25,-117,117,83,-83,-1,1,y,x,500);
  setMotors(-42,42,36,-36,2,-2,-48,48,-1,1,25,-25,-106,106,83,-82,-1,1,y,x,500); //dop move1
  setMotors(-87,87,2,-2,2,-2,-46,46,-1,1,25,-25,-100,100,83,-83,-1,1,y,x,300);
  setMotors(-85,85,-18,18,-2,2,-46,46,-1,1,0,0,-97,97,73,-73,-1,1,y,x,200);//dop move 
  setMotors(-84,84,-32,32,-5,5,-45,45,-1,1,-25,25,-98,98,67,-67,-1,1,y,x,100);//dop move2
  setMotors(-82,82,-54,54,-11,11,-45,45,-1,1,-44,44,-100,100,57,-57,-1,1,y,x,100);
  setMotors(-81,81,-68,68,-14,14,-45,45,-1,1,-60,60,-100,100,51,-51,-1,1,y,x,600);   
}
void GET_UP_FRONT(int w1, int w2){
  setMotors(-81,81,0,0,14,-14,-46,46,-1,1,-48,48,-67,67,30,-30,-1,1,y,x,600);
  setMotors(-14,14,-4,4,-3,3,-46,46,-1,1,-48,48,-67,67,30,-30,-1,1,y,x,500);
  setMotors(29,-29,57,-57,-104,104,-46,46,-4,4,-133,133,-114,114,94,-94,-2,2,y,x,500); //dop move 1 aprilya
  setMotors(29,-29,0,0,-104,104,-46,46,-4,4,-133,133,-114,114,94,-94,-2,2,y,x,500); 
  setMotors(0,0,-22,22,-64,64,-46,46,-2,2,-130,130,-125,125,77,-77,-1,1,y,x,600);//DOP MOVE 1 
  setMotors(-25,25,-90,90,0,0,-46,46,0,0,-126,126,-138,138,56,-56,0,0,y,x,1000);
  setMotors(-45,45,-71,71,-16,16,-46,46,0,0,-105,105,-142,142,55,-55,0,0,y,x,600);//dop move 
  setMotors(-58,58,-70,70,-15,15,-45,45,0,0,-93,93,-125,125,54,-54,0,0,y,x,500);//dop move
  //setMotors(-81,81,-71,71,-16,16,-46,46,0,0,-60,60,-130,130,60,-60,0,0,y,x,2000);// dop mov
 // setMotors(-51,51,-71,71,-16,16,-45,45,0,0,-103,103,-124,124,54,-54,0,0,w2,w1,50);//dop move
  setMotors(-81,81,-68,68,-14,14,-45,45,-1,1,-60,60,-100,100,51,-51,-1,1,y,x,600);
}
void CIRCLE_RIGHT(int w1, int w2){
  setMotors(-90,90,-68,68,-14,14,-51,51,-23,23,-75,65,-115,103, 65,-51, -7,-1,w2,w1,200);
  setMotors(-90,90,-68,68,-14,14,-51,51,-10,10,-65,75,-103,115, 51,-65, -7,-1,w2,w1,100);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-67,75,-103,115,51,-57,-1,1,w2,w1,150);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-67,67,-103,103,51,-51,-1,1,w2,w1,400);
  
  setMotors(-90,90,-68,68,-14,14,-12,19,-1,1,-67,75,-103,117, 51,-61, -1,-1,w2,w1,70);
  setMotors(-90,90,-68,68,-14,14,-30,30,-1,1,-67,67,-103,103, 51,-51, -1,-1,w2,w1,100);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-67,67,-103,102,51,-51,-1,1,w2,w1,300);
  
  setMotors(-90,90,-68,68,-14,14,-12,19,-1,1,-67,75,-103,117, 51,-61, -1,-1,w2,w1,70);
  setMotors(-90,90,-68,68,-14,14,-30,30,-1,1,-67,67,-103,103, 51,-51, -1,-1,w2,w1,100);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-67,67,-103,102,51,-51,-1,1,w2,w1,300);
}
void CIRCLE_LEFT(int w1, int w2){
  setMotors(-90,90,-68,68,-14,14,-51,51,-23,23,-65,75,-103,115, 51,-65, -1,7,y,x,200);
  setMotors(-90,90,-68,68,-14,14,-51,51,-10,10,-75,65,-115,103, 65,-51, -1,7,y,x,100);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-75,65,-115,103,57,-51,-1,1,y,x,150);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-67,67,-103,103,51,-51,-1,1,w2,w1,400);

  setMotors(-90,90,-68,68,-14,14,-12,19,-1,1,-75,67,-117,103, 60,-51, -1,-1,y,x,70);
  setMotors(-90,90,-68,68,-14,14,-30,30,-1,1,-67,67,-103,103, 51,-51, -1,-1,y,x,100);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-69,69,-103,102,51,-51,-1,1,y,x,300);
  
  setMotors(-90,90,-68,68,-14,14,-12,19,-1,1,-75,67,-117,103, 60,-51, -1,-1,y,x,70);
  setMotors(-90,90,-68,68,-14,14,-30,30,-1,1,-67,67,-103,103, 51,-51, -1,-1,y,x,100);
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-67,67,-103,102,51,-51,-1,1,y,x,300);
}
void Set_Head(int w1, int w2){
  
  setMotors(-90,90,-68,68,-14,14,-45,45,-1,1,-70,70,-103,102,51,-51,-1,1,w2,w1,20);
}
void Head_Up_Down(){
  
  if(y>=15){
    while(Ball_Detect==false && y <= 58){
      y = y + 2;
      Set_Head(x,y);
    }
    if(Ball_Detect == true && y < 50){
      y=y+10;
      
        ready_for_walk(x,y);
      //wait(100);
    }
    while(Ball_Detect==false && y > -30){
      y = y - 2;
      Set_Head(x,y);
    }
    if(Ball_Detect==true && y > -20){
      y=y-10;
      
        ready_for_walk(x,y);
      //wait(100);
    }
  }
  else{
    while(Ball_Detect==false && y > -30){
      y = y - 2;
      Set_Head(x,y);
    }
    if(Ball_Detect==true && y > -20){
      y=y-10;
      
        ready_for_walk(x,y);
      //wait(100);
    }
     while(Ball_Detect==false && y < 58){
      y = y + 2;
      Set_Head(x,y);
    }
    if(Ball_Detect == true && y < 50){
      y=y+10;
      
        ready_for_walk(x,y);
    }
  }
  
}
bool Search_Ball(){
    Head_Up_Down();  
    if (Ball_Detect == false && x_old>=0 ){
      if(x<=35){if(HeadLeft() == true){x=0;TURN_L(x,y,false);TURN_L(x,y,true);return true;}}
      if(HeadRight() == true){x=0;TURN_R(x,y,false);TURN_R(x,y,true);return true;}
      x = 0;
      x_old = 10;
      TURN_L(x,y,false);
      TURN_L(x,y,false);
      TURN_L(x,y,false);
      TURN_L(x,y,false); 
    }
    else if(Ball_Detect == false){   
      if(x>=-35){if(HeadRight() == true){x=0;TURN_R(x,y,false);TURN_R(x,y,true);return true;}}
      if(HeadLeft() == true){x=0;TURN_L(x,y,false);TURN_L(x,y,true);return true;}
      x=0;
      x_old=-10;
      TURN_R(x,y,false);
      TURN_R(x,y,false);
      TURN_R(x,y,false);
      TURN_R(x,y,false);
    }
}
bool HeadLeft(){
    x=45;
    ready_for_walk(x,y); 
    Head_Up_Down();
    if(Ball_Detect == true)
      return true;
    else
      return false; 
}

bool HeadRight(){
    x=-45;
    ready_for_walk(x,y); //head right 
    Head_Up_Down();
    if(Ball_Detect == true)
      return true;
    else
      return false; 
}
