
#include "move.h"
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
int y = 0;
void neckY(const std_msgs::Int16& errY)
{
   if((errY.data < 555) && (abs(errY.data)>10))
   {
      y = y-0.4*errY.data; 
   }
    
}

ros::Subscriber<std_msgs::Int16> neck_sub("motion_main", neckY );

void setup() 
{
  //Serial.begin(57600); //for debug
  // while(!Serial); // Wait for Opening Serial Monitor
  nh.initNode(); 
  nh.subscribe(neck_sub);
  Moves::initialization();
  delay(1000);
 

  
  
  
}

void loop() 
{  
  
  Moves::ready_for_walk(0,y);
  nh.spinOnce();
}
