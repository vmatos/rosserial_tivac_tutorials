/*********************************************************************
 *
 *  Copyright (c) 2016 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *  rosserial_tivac getparam tutorial
 *
 *  On this demo your TivaC Launchpad will publish the param string
 * over the topic "/param".
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
  #include <utils/ustdlib.h>
}
// ROS includes
#include <ros.h>

// ROS nodehandle
ros::NodeHandle nh;
char loginfo_buffer[100];


int int_buf = -2;
float float_buf = -1.0;
char string_buf[50] = "n/a";
char *string_buf_ptr[1] = {string_buf};

int int_array_buf[3] = {-1, -2, -3};
float float_array_buf[3] = {-1.0, -2.0, -3.0};
char string_array_buf1[50] = "n/a1";
char string_array_buf2[50] = "n/a2";
char string_array_buf3[50] = "n/a3";
char *string_array_buf_ptr[3] = {string_array_buf1, string_array_buf2, string_array_buf3};


int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  // TivaC system clock configuration. Set to 80MHz.
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  // ROS nodehandle initialization and topic registration
  nh.initNode();
  
  // Wait for connection to establish
  while (!nh.connected())
  {
    nh.spinOnce();
    nh.getHardware()->delay(10);
  }
  
  // Get int parameter
  bool param_int_success = false;
  while (!param_int_success)
  {
    param_int_success = nh.getParam("~int", (int*)&int_buf, 1);
    nh.spinOnce();
    nh.getHardware()->delay(10);
  }
  usprintf(loginfo_buffer, "[TIVA] Got param int: %d", int_buf);
  nh.loginfo(loginfo_buffer);
  
  // Get float parameter
  bool param_float_success = false;
  while (!param_float_success)
  {
    param_float_success = nh.getParam("~float", (float*)&float_buf, 1);
    nh.spinOnce();
    nh.getHardware()->delay(10);
  }
  usprintf(loginfo_buffer, "[TIVA] Got param float: %f", float_buf);
  nh.loginfo(loginfo_buffer);
  
  // Get string parameter
  bool param_string_success = false;
  while (!param_string_success)
  {
    param_string_success = nh.getParam("~string", string_buf_ptr);
    nh.spinOnce();
    nh.getHardware()->delay(10);
  }
  usprintf(loginfo_buffer, "[TIVA] Got param string: %s", string_buf_ptr[0]);
  nh.loginfo(loginfo_buffer);
  
  // Get int array
  bool param_int_array_success = false;
  while (!param_int_array_success)
  {
    param_int_array_success = nh.getParam("~array_int", int_array_buf, 3);
    nh.spinOnce();
    nh.getHardware()->delay(10);
  }
  usprintf(loginfo_buffer, "[TIVA] Got param array_int: [%d, %d, %d]", int_array_buf[0], int_array_buf[1], int_array_buf[2]);
  nh.loginfo(loginfo_buffer);
  
  // Get float array
  bool param_float_array_success = false;
  while (!param_float_array_success)
  {
    param_float_array_success = nh.getParam("~array_float", float_array_buf, 3);
    nh.spinOnce();
    nh.getHardware()->delay(10);
  }
  usprintf(loginfo_buffer, "[TIVA] Got param array_float: [%f, %f, %f]", float_array_buf[0], float_array_buf[1], float_array_buf[2]);
  nh.loginfo(loginfo_buffer);
  
  // Get string array
  bool param_string_array_success = false;
  while (!param_string_array_success)
  {
    param_string_array_success = nh.getParam("~array_string", string_array_buf_ptr, 3);
    nh.spinOnce();
    nh.getHardware()->delay(10);
  }
  usprintf(loginfo_buffer, "[TIVA] Got param array_string: [%s, %s, %s]", string_array_buf1, string_array_buf2, string_array_buf3);
  nh.loginfo(loginfo_buffer);
  
  // We just want to make rosserial_server throw a warning.
  nh.getParam("~array_mix", &int_buf, 1);
  nh.spinOnce();
  nh.getHardware()->delay(10);

  while (1)
  {
    // Handle all communications and callbacks.
    nh.spinOnce();

    // Delay for a bit.
    nh.getHardware()->delay(100);
  }
}
