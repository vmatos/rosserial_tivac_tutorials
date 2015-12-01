/*********************************************************************
 *
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *  rosserial_tivac array test - Just like array_test on 
 * rosserial_arduino
 *
 *********************************************************************/

#include <stdbool.h>
#include <stdint.h>
// TivaC specific includes
extern "C"
{
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
}
// ROS includes
#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// ROS nodehandle
ros::NodeHandle nh;

geometry_msgs::Pose sum_msg;
ros::Publisher p("sum", &sum_msg);

void messageCb(const geometry_msgs::PoseArray& msg){
  sum_msg.position.x = 0;
  sum_msg.position.y = 0;
  sum_msg.position.z = 0;
  for(int i = 0; i < msg.poses_length; i++)
  {
    sum_msg.position.x += msg.poses[i].position.x;
    sum_msg.position.y += msg.poses[i].position.y;
    sum_msg.position.z += msg.poses[i].position.z;
  }
}

ros::Subscriber<geometry_msgs::PoseArray> s("poses",messageCb);

int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  // TivaC system clock configuration. Set to 80MHz.
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  // ROS nodehandle initialization and topic registration
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);

  while (1)
  {
    // Publish message to be transmitted.
    p.publish(&sum_msg);

    // Handle all communications and callbacks.
    nh.spinOnce();

    // Delay for a bit.
    nh.getHardware()->delay(100);
  }
}
