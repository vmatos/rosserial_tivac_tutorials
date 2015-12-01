#!/bin/bash

# start xterm with openocd in the background
xterm -e openocd -f board/ek-lm4f120xl.cfg &

# save the PID of the background process
XTERM_PID=$!

# wait a bit to be sure the hardware is ready
sleep 2

# execute some initialisation commands via gdb
arm-none-eabi-gdb --batch --command=init.gdb $HOME/ros/tivac_ws/devel/share/rosserial_tivac_tutorials/freertos123.axf

# start the gdb gui
nemiver --remote=localhost:3333 --gdb-binary="$(which arm-none-eabi-gdb)" $HOME/ros/tivac_ws/devel/share/rosserial_tivac_tutorials/freertos123.axf

# close xterm when the user has exited nemiver
kill $XTERM_PID
