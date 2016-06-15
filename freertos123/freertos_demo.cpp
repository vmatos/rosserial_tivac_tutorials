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
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
}
#include "spin_task.h"
#include "publish_task.h"
#include "subscribe_task.h"

// ROS nodehandle
ros::NodeHandle nh;

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
extern "C"
{
void __error__(char *pcFilename, uint32_t ui32Line)
{
  UARTprintf("Error at line: %d\n", ui32Line);
  UARTprintf(pcFilename);
}
}
#endif

extern "C" 
{
// This hook is called by FreeRTOS when an stack overflow error is detected.
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
  MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
  
#ifdef USE_USBCON
  UARTprintf("Stack overflow\n");
#endif
  
  // Loop forever.  Interrupts are disabled on entry to this function,
  // so no processor interrupts will interrupt this loop.
  while (1)
  {
  }
}
}

// Initialize FreeRTOS and start the initial set of tasks.
int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  
  // TivaC system clock configuration. Set to 80MHz.
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  
  MAP_IntEnable(FAULT_NMI);
  MAP_IntEnable(FAULT_MPU);
  MAP_IntEnable(FAULT_BUS);
  MAP_IntEnable(FAULT_USAGE);

  // ROS nodehandle initialization and topic registration
  nh.initNode();
  
#ifdef USE_USBCON  
  UARTprintf("\n\nWelcome to the EK-TM4C123GXL FreeRTOS Demo!\n");
#endif

  // Start ROS spin task, responsible for handling callbacks and communications
  if (spinInitTask(&nh))
  {
#ifdef USE_USBCON
    UARTprintf("Couldn't create ROS spin task.\n");
#endif
    while (1);
  }
#ifdef USE_USBCON
  else 
  {
    UARTprintf("Created ROS spin task.\n");
  }
#endif

  // Register and init publish task
  if (publishInitTask(&nh))
  {
#ifdef USE_USBCON
    UARTprintf("Couldn't create publish task.\n");
#endif
    while (1);
  }
#ifdef USE_USBCON
  else 
  {
    UARTprintf("Created publish task.\n");
  }
#endif

  // Register and init subscribe task
  if (subscribeInitTask(&nh))
  {
#ifdef USE_USBCON
    UARTprintf("Couldn't create subscribe task.\n");
#endif
    while (1);
  }
#ifdef USE_USBCON
  else 
  {
    UARTprintf("Created subscribe task.\n");
  }
#endif

  // Start the scheduler.  This should not return.
#ifdef USE_USBCON
  UARTprintf("Starting scheduller.\n");  
#endif
  vTaskStartScheduler();

  // In case the scheduler returns for some reason, print an error and loop forever.
  while (1)
  {
#ifdef USE_USBCON
    UARTprintf("Scheduler returned!\n");
#endif
  }
}

#if configUSE_MALLOC_LOCK_UNLOCK == 1
extern "C"
{
/*-----------------------------------------------------------*/
void __malloc_lock(struct _reent* REENT)
{
	vTaskSuspendAll();
}

void __malloc_unlock(struct _reent* REENT)
{
	xTaskResumeAll();	
}
}
#endif
