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
#include "queue.h"
}
#include "std_msgs/String.h"

#define tskSUBSCRIBE_PRIORITY 2

static ros::NodeHandle *nh_;

xQueueHandle g_pStrQueue;
xQueueHandle g_pStrLenQueue;
const int STR_QUEUE_SIZE = 5;

// Callback for receiving string message from topic, sending it to the queue to be processed
static void str_cb(const std_msgs::String& msg)
{
  if (xQueueSendToBack(g_pStrQueue, &msg, 500) != pdPASS)
  {
    // Error. Queue is full
#ifdef USE_USBCON
    UARTprintf("String message queue full.\n");
#endif
  }
}

ros::Subscriber<std_msgs::String> str_sub("str", &str_cb);

// subscriber task - Processes data from the string message queue
static void subscribeTask(void *pvParameters)
{
  portTickType ui32WakeTime;
  // Get the current tick count.
  ui32WakeTime = xTaskGetTickCount();

  std_msgs::String incoming_msg;
  uint32_t msg_len;
  while (1)
  {
    if (xQueueReceive(g_pStrQueue, &incoming_msg, 0) == pdPASS)
    {
      msg_len = strlen(incoming_msg.data);
      xQueueSendToBack(g_pStrLenQueue, &msg_len, 500);
    }
    vTaskDelayUntil(&ui32WakeTime, 100);
  }
}

// Initialization of the subscriber task
// Also registers subscriber onto the node handle.
uint32_t subscribeInitTask(ros::NodeHandle *nh)
{
  nh_ = nh;
  nh_->subscribe(str_sub);

  g_pStrQueue = xQueueCreate(STR_QUEUE_SIZE, sizeof(std_msgs::String));
  g_pStrLenQueue = xQueueCreate(STR_QUEUE_SIZE, sizeof(uint32_t));

  // Init spin task
  if (xTaskCreate(subscribeTask, (const portCHAR *)"subscrive", 100, NULL, tskIDLE_PRIORITY + tskSUBSCRIBE_PRIORITY, NULL) != pdTRUE)
  {
    return 1;
  }
  return 0;
}
