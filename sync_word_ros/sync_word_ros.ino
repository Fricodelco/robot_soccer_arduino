
#include <DynamixelWorkbench.h>
#include <ros.h>
#include <std_msgs/Int16.h>

#define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

#define BAUDRATE  1000000
#define DXL_ID_1  1
#define DXL_ID_2  2

DynamixelWorkbench dxl_wb;
int8_t motion_n = 0;

uint16_t model_number = 0;
  
uint8_t dxl_id[2] = {DXL_ID_1, DXL_ID_2};

int32_t goal_position[2] = {0, 1023};

const uint8_t handler_index = 0;

ros::NodeHandle nh;

void motion_decode(const std_msgs::Int16& motion_msg)
{
   switch(motion_msg.data) {
    case 0:
      motion_n=0;
    case 1:
      motion_n=1;
   }
    
}

ros::Subscriber<std_msgs::Int16> motion_sub("motion_main", motion_decode );

void setup() 
{
  //Serial.begin(57600); //for debug
  // while(!Serial); // Wait for Opening Serial Monitor
  
  dxl_wb.init(DEVICE_NAME, BAUDRATE);
  
  for (int cnt = 0; cnt < 2; cnt++)
  {
    dxl_wb.ping(dxl_id[cnt], &model_number);
    dxl_wb.jointMode(dxl_id[cnt], 0, 0); //second point is speed 0-1023
  }
  dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position");

  nh.getHardware()->setBaud(115200);
  nh.initNode(); 
  nh.subscribe(motion_sub);
  
}

void loop() 
{  
  /*bool result = false;
  result = dxl_wb.syncWrite(handler_index, &goal_position[0]);
  if (result == false)
  {
    Serial.println("Failed to sync write position");
  }
   */
  switch (motion_n)
  {
    case 0:
      motion_forward();
    case 1:
      motion_back();  
  }
}


void motion_forward(){
  bool result = false;
  goal_position[0]=1023;
  goal_position[1]=0;
  result=dxl_wb.syncWrite(handler_index, &goal_position[0]);
}
void motion_back(){
  bool result = false;
  goal_position[0]=0;
  goal_position[1]=1023;
  result=dxl_wb.syncWrite(handler_index, &goal_position[0]);
}
