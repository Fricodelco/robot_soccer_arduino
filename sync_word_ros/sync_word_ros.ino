
#include "move.h"
#include <ros.h>
#include <std_msgs/Int16.h>


#define ROS_RATE 50000

ros::NodeHandle nh;
int y = 0;

HardwareTimer Timer(TIMER_CH4);

void neckY(const std_msgs::Int16& errY)
{
   if((errY.data < 555) && (abs(errY.data)>10))
   {
      y = y-0.4*errY.data; 
   }
    
}
void motion(const std_msgs::Int16& motion){
  if(motion.data==-1){
      Moves::ready_for_walk(0,y);
  }
}
ros::Subscriber<std_msgs::Int16> neck_sub("errY", neckY );
ros::Subscriber<std_msgs::Int16> motion_sub("goalkeeper_action", motion );

void setup() 
{
  //Serial.begin(57600); //for debug
  Timer.stop();
  Timer.setPeriod(ROS_RATE);           // in microseconds
  Timer.attachInterrupt(handler_ros);
  Timer.start();
  Moves::initialization();
  // while(!Serial); // Wait for Opening Serial Monitor
  nh.initNode(); 
  nh.subscribe(neck_sub);
  nh.subscribe(motion_sub);
 

  
  
  
}

void loop() 
{  
  digitalWrite(BOARD_LED_PIN, LOW);
  //Moves::ready_for_walk(0,y);
}

void handler_ros(){
  //digitalWrite(BOARD_LED_PIN, HIGH);
  //delay(50);
  nh.spinOnce();
 
}
