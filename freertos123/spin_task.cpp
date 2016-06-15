#include <stdbool.h>
#include <stdint.h>
#include "ros_freertos.h"
extern "C"
{
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
}

#define tskSPIN_PRIORITY 1

static ros::NodeHandle *nh_;

// ros spin() like task
static void spinTask(void *pvParameters)
{
  portTickType ui32WakeTime;
  // Get the current tick count.
  ui32WakeTime = xTaskGetTickCount();

  while (1)
  {
    // rosserial callback handling
    nh_->spinOnce();

    // Toggle for spin heartbeat
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, MAP_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3)^GPIO_PIN_3);

    vTaskDelayUntil(&ui32WakeTime, 100);
  }
}

// spin task initialization
uint32_t spinInitTask(ros::NodeHandle *nh)
{
  // Enable green LED for spin heartbeat
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_4);

  nh_ = nh;

  // Init spin task
  if (xTaskCreate(spinTask, (const portCHAR *)"spin", 100, NULL, tskIDLE_PRIORITY + tskSPIN_PRIORITY, NULL) != pdTRUE)
  {
    return 1;
  }
  return 0;
}
