/*********************************************************************
 *
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *  rosserial_tivac rgb_led tutorial
 *
 *  On this demo your TivaC Launchpad listens to the topic '/led' and
 * changes the colors and intensity accordingly.
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
  #include <rgb.h>
}
// ROS includes
#include <ros.h>
#include <std_msgs/ColorRGBA.h>

uint32_t current_color[3] = {0xFFFE, 0xFFFE, 0xFFFE};
float current_intensity = 1.f;
void color_cb(const std_msgs::ColorRGBA& msg)
{
  current_color[0] = msg.r * 0xFFFE;
  current_color[1] = msg.g * 0xFFFE;
  current_color[2] = msg.b * 0xFFFE;
  current_intensity = msg.a;
  RGBSet(current_color, current_intensity);
}

// ROS nodehandle and subscriber
ros::NodeHandle nh;
ros::Subscriber<std_msgs::ColorRGBA> rgb_sub("led", &color_cb);

int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  // TivaC system clock configuration. Set to 80MHz.
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  // Setup RGB driver
  RGBInit(0);
  RGBSet(current_color, current_intensity);

  // ROS nodehandle initialization and topic registration
  nh.initNode();
  nh.subscribe(rgb_sub);
  bool nh_prev_state = false;

  while (1)
  {
    // If subscribed, enable RGB driver
    if (nh.connected() && !nh_prev_state)
    {
      RGBEnable();
      nh_prev_state = true;
    }
    if (!nh.connected() && nh_prev_state)
    {
      RGBDisable();
      nh_prev_state = false;
    }

    // Handle all communications and callbacks.
    nh.spinOnce();

    // Delay for a bit.
    nh.getHardware()->delay(1000);
  }
}
