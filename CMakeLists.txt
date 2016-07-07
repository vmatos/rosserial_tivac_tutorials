cmake_minimum_required(VERSION 2.8.3)
project(rosserial_tivac_tutorials)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rosserial_client
  rosserial_tivac
  std_msgs
)
add_message_files(
  FILES Buttons.msg
)
add_service_files(
  FILES ColorRGBA.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_tivac
  SCRIPT make_libraries_tiva
)
# Because we need to generate our custom messages before exporting our rosserial ros_lib
add_dependencies(${PROJECT_NAME}_ros_lib ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Chatter tutorial
rosserial_configure_client(
  DIRECTORY chatter
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(chatter chatter.axf ALL)
rosserial_add_client_target(chatter flash)
rosserial_add_client_target(chatter size)
rosserial_add_client_target(chatter dump)

# Chatter tutorial for TivaC Connected
rosserial_configure_client(
  DIRECTORY chatter129
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(chatter129 chatter129.axf ALL)
rosserial_add_client_target(chatter129 flash)
rosserial_add_client_target(chatter129 size)
rosserial_add_client_target(chatter129 dump)

# RGB LED tutorial
rosserial_configure_client(
  DIRECTORY rgb_led
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(rgb_led rgb_led.axf ALL)
rosserial_add_client_target(rgb_led flash)
rosserial_add_client_target(rgb_led size)

# RGB LED service tutorial
rosserial_configure_client(
  DIRECTORY rgb_srv
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(rgb_srv rgb_srv.axf ALL)
rosserial_add_client_target(rgb_srv flash)
rosserial_add_client_target(rgb_srv size)

# Custom message - Button publisher
rosserial_configure_client(
  DIRECTORY buttons
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(buttons buttons.axf ALL)
rosserial_add_client_target(buttons flash)
rosserial_add_client_target(buttons size)

# How to get a parameter from ROS master
rosserial_configure_client(
  DIRECTORY getparam
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(getparam getparam.axf ALL)
rosserial_add_client_target(getparam flash)
rosserial_add_client_target(getparam size)

# Time and TF example
rosserial_configure_client(
  DIRECTORY time_tf
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(time_tf time_tf.axf ALL)
rosserial_add_client_target(time_tf flash)
rosserial_add_client_target(time_tf size)

# Array test tutorial
rosserial_configure_client(
  DIRECTORY array_test
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(array_test array_test.axf ALL)
rosserial_add_client_target(array_test flash)

# freeRTOS for tm4c123gxl
rosserial_configure_client(
  DIRECTORY freertos123
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(freertos123 freertos123.axf ALL)
rosserial_add_client_target(freertos123 flash)
rosserial_add_client_target(freertos123 size)
rosserial_add_client_target(freertos123 dump)
