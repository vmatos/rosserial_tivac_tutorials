/*
 * Copyright (c) 2015, Robosavvy Ltd.
 * All rights reserved.
 * Author: Vitor Matos
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 * following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *   3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*****************************************************************************
//
// Bare minimum hardware resources allocated for rosserials communication.
// * 2 LEDs if desired
// * One UART port, interrupt driven transfers
// * Two RingBuffers of TX_BUFFER_SIZE and RX_BUFFER_SIZE
//
//*****************************************************************************

#ifndef ROS_LIB_TIVAC_HARDWARE_FREERTOS_H
#define ROS_LIB_TIVAC_HARDWARE_FREERTOS_H

#include <stdbool.h>
#include <stdint.h>
extern "C"
{
  #include "inc/hw_types.h"
  #include "inc/hw_memmap.h"
  #include "inc/hw_ints.h"
  #include "driverlib/sysctl.h"
  #include "driverlib/gpio.h"
  #include "driverlib/rom.h"
  #include "driverlib/rom_map.h"
  #include "driverlib/systick.h"
  #include "driverlib/pin_map.h"
  #include "driverlib/uart.h"
  #include "utils/ringbuf.h"
  #include "FreeRTOS.h"
  #include "task.h"
}

#ifdef TM4C123GXL
#define LED1        GPIO_PIN_3  // Green LED
#define LED2        GPIO_PIN_2  // Blue LED
#define LED_PORT    GPIO_PORTF_BASE
#define LED_PERIPH  SYSCTL_PERIPH_GPIOF
#endif

#ifdef TM4C1294XL
#define LED1        GPIO_PIN_1  // D1 LED
#define LED2        GPIO_PIN_0  // D2 LED
#define LED_PORT    GPIO_PORTN_BASE
#define LED_PERIPH  SYSCTL_PERIPH_GPION
#ifndef TM4C129FREQ
#error "Must define system clock frequency on: TM4C129FREQ"
#endif
#endif

#ifndef ROSSERIAL_BAUDRATE
#define ROSSERIAL_BAUDRATE 57600
#endif

extern tRingBufObject rxBuffer;
extern tRingBufObject txBuffer;

class TivaCHardware
{
  public:
    TivaCHardware() {}

    void init()
    {
#ifdef TM4C123GXL
      this->ui32SysClkFreq = MAP_SysCtlClockGet();
#endif
#ifdef TM4C1294XL
      this->ui32SysClkFreq = TM4C129FREQ;
#endif

      // Setup LEDs
#ifdef LED_COMM
      MAP_SysCtlPeripheralEnable(LED_PERIPH);
      MAP_GPIOPinTypeGPIOOutput(LED_PORT, LED2);
#endif

      // Enable the peripherals for UART0
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
      // Set GPIO A0 and A1 as UART pins.
      MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
      MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
      MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
      // Configure UART0
      MAP_UARTConfigSetExpClk(UART0_BASE, this->ui32SysClkFreq, ROSSERIAL_BAUDRATE,
          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
      // Supposedely MCU resets with FIFO 1 byte depth. Just making sure.
      MAP_UARTFIFODisable(UART0_BASE);
      // UART buffers to transmit and receive
      RingBufInit(&rxBuffer, this->ui8rxBufferData, RX_BUFFER_SIZE);
      RingBufInit(&txBuffer, this->ui8txBufferData, TX_BUFFER_SIZE);

      // Enable RX and TX interrupt
      UARTIntRegister(UART0_BASE, TivaCHardware::UARTIntHandler);
      MAP_IntEnable(INT_UART0);
      MAP_UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);
      MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_TX);

      // Enable processor interrupts.
      MAP_IntMasterEnable();
    }

    // read a byte from the serial port. -1 = failure
    int read()
    {
      if (!RingBufEmpty(&rxBuffer))
        return RingBufReadOne(&rxBuffer);
      else
        return -1;
    }

    // write data to the connection to ROS
    void write(uint8_t* data, int length)
    {
      // Trigger sending buffer, if not already sending
      if (RingBufEmpty(&txBuffer))
      {
        RingBufWrite(&txBuffer, data, length);
        MAP_UARTCharPutNonBlocking(UART0_BASE, RingBufReadOne(&txBuffer));
      }
      else
      {
        RingBufWrite(&txBuffer, data, length);
      }
    }

    // returns milliseconds since start of program
    uint32_t time()
    {
      return (uint32_t) xTaskGetTickCount();
    }

    // UART buffer structures
    uint8_t ui8rxBufferData[RX_BUFFER_SIZE];
    uint8_t ui8txBufferData[TX_BUFFER_SIZE];

    // UART interrupt handler
    // For each received byte, pushes it into the buffer.
    // For each transmitted byte, read the next available from the buffer.
    static void UARTIntHandler()
    {
      uint32_t ui32Status;
      // Get the interrrupt status.
      ui32Status = MAP_UARTIntStatus(UART0_BASE, true);
      // Clear the asserted interrupts.
      MAP_UARTIntClear(UART0_BASE, ui32Status);

      // Since we are using no RX, or TX FIFO, only one char at a time for each interrupt call
      // RX the next character from the UART and put it on RingBuffer.
      // We should verify if buffer is not full. Let's assume not, for faster interrupt routine.
      if (ui32Status & UART_INT_RX)
        RingBufWriteOne(&rxBuffer, MAP_UARTCharGetNonBlocking(UART0_BASE));

      // TX the next available char on the buffer
      if (ui32Status & UART_INT_TX)
        if (!RingBufEmpty(&txBuffer))
          MAP_UARTCharPutNonBlocking(UART0_BASE, RingBufReadOne(&txBuffer));

    #ifdef LED_COMM
      // Blink the LED to show a character transfer is occuring.
      MAP_GPIOPinWrite(LED_PORT, LED2, MAP_GPIOPinRead(LED_PORT, LED2)^LED2);
    #endif
    }

    // System frequency
    uint32_t ui32SysClkFreq;
    uint32_t getSysClkFreq(void)
    {
      return this->ui32SysClkFreq;
    }
};
#endif  // ROS_LIB_TIVAC_HARDWARE_FREERTOS_H
