cmake_minimum_required(VERSION 2.8.3)
project(freertos123)

# Include rosserial libraries for TivaC
include_directories(${ROS_LIB_DIR})

# Per project based definitions and options
add_definitions(-DLED_COMM)
add_definitions(-DROSSERIAL_BAUDRATE=57600)
add_definitions(-DTX_BUFFER_SIZE=256)
add_definitions(-DRX_BUFFER_SIZE=256)

# Generate target for TivaC
generate_tivac_firmware(
  USB
  STARTUP startup_gcc.c
  SRCS 
    freertos_demo.cpp
    spin_task.cpp
    publish_task.cpp
    subscribe_task.cpp
    ${TIVA_WARE_PATH}/utils/uartstdio.c
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/list.c
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/tasks.c
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/queue.c
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/portable/MemMang/heap_3.c
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
  INCS 
    .
    ${TIVA_WARE_PATH}/inc
    ${TIVA_WARE_PATH}/utils
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/include 
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/portable/MemMang/
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/portable/GCC/ARM_CM4F
  BOARD tm4c123gxl
)
