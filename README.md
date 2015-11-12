# rosserial_tivac_tutorials
Tutorials and examples for rosserial_tivac package.

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
