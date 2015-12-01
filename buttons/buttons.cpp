/*********************************************************************
 *
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *  rosserial_tivac buttons tutorial
 *
 *  On this demo your TivaC Launchpad will publish the user buttons
 * state on the topic '/button_state'.
 *  This tutorial demonstrates the use of a custom message Buttons.msg
 *
 * Full guide: http://wiki.ros.org/rosserial_tivac/Tutorials
 *
 *********************************************************************/

#include <stdbool.h>
#include <stdint.h>
// TivaC specific includes
extern "C"
{
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include "buttons.h"
}
// ROS includes
#include <ros.h>
// Custom ROS message
#include "rosserial_tivac_tutorials/Buttons.h"

// ROS nodehandle
ros::NodeHandle nh;

rosserial_tivac_tutorials::Buttons button_msg;
ros::Publisher button_publisher("button_state", &button_msg);

int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  // TivaC system clock configuration. Set to 80MHz.
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  uint8_t button_debounced_delta;
  uint8_t button_raw_state;
  ButtonsInit();

  // ROS nodehandle initialization and topic registration
  nh.initNode();
  nh.advertise(button_publisher);

  while (1)
  {
    uint8_t button_debounced_state = ButtonsPoll(&button_debounced_delta, &button_raw_state);
    // Publish message to be transmitted.
    button_msg.sw1.data = button_debounced_state & LEFT_BUTTON;
    button_msg.sw2.data = button_debounced_state & RIGHT_BUTTON;
    button_publisher.publish(&button_msg);

    // Handle all communications and callbacks.
    nh.spinOnce();

    // Delay for a bit.
    nh.getHardware()->delay(100);
  }
}
