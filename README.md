# rosserial_tivac_tutorials
Tutorials and examples for rosserial_tivac package.

## Building procedure:

```
catkin_make rosserial_tivac_tutorials_generate_messages
source devel/setup.bash
catkin_make
```

## Tutorials and examples:

### chatter
Small demo for the TM4C123GXL Launchpad.
It uses the UART on the debug USB port to communicate with rosserial.
Publishes on topic `/chatter` a string every few milliseconds.

### chatter129
Small demo prepared for the TM4C1294XL Connected Launchpad.
It uses the UART on the debug USB port to communicate with rosserial.
Publishes on topic `/chatter` a string every few milliseconds.

### buttons
User button state publisher for the TM4C123GXL Launchpad.
Enumerates CDC device class on the device USB port to communicate with rosserial.
Publishes on topic `/button_state` a custom message `rosserial_tivac_tutorials/Buttons`.

### getparam
For the TM4C123GXL Launchpad.
It uses the UART on the debug USB port to communicate with rosserial.
Gets private parameter 'my_param' on the rosserial node.
Publishes on topic `/param` a float, with the default value is -1.0 if parameter is not found.


### rgb_led
Subscriber demo for the TM4C123GXL Launchpad.
Enumerates CDC device class on the device USB port to communicate with rosserial.
Subscribes to topic `/led` of message `std_msgs/ColorRGBA` to change intensity and colors, value range from [0 1.0].

### rgb_srv
Service server demo for the TM4C123GXL Launchpad.
Enumerates CDC device class on the device USB port to communicate with rosserial.
Provides a service on topic `/led` of message `rosserial_tivac_tutorials/ColorRGBA` to change intensity and colors, value range from [0 1.0], and always responds with `true`.

### time_tf
Time and TF demo for the TM4C123GXL Launchpad.
Enumerates CDC device class on the device USB port to communicate with rosserial.
Publishes on topic `/tf` a static transform, timestamped with the current time.

### FreeRTOS on TM4C123GXL
Demo FreeRTOS application using `rosserial_tivac`.
This demo creates 3 tasks.

1. spin task - handles ros::spinOnce() periodically at 100ms.
2. subscribe task - reads from the received message queue and processes data. Sends length of string to processed data queue.
3. publish task - publishes data from processed data queue.
