# Get into gdb
    
    openocd -f board/ek-lm4f120xl.cfg

    cd ~/ros/tivac_ws/devel/share/rosserial_tivac_tutorials
    arm-none-eabi-gdb -tui --eval-command="target remote localhost:3333" freertos123.axf

    (gdb) target extended-remote :3333

or :
    
    ddd --eval-command="target remote localhost:3333" --debugger arm-none-eabi-gdb  freertos123.axf

then: 

    target extended-remote :3333
    monitor reset halt
    load
    monitor reset init

    b UsageISR




info frame to show the stack frame info

To read the memory at given addresses you should take a look at x

x/x $esp for hex x/d $esp for signed x/u $esp for unsigned etc. x uses the format syntax, you could also take a look at the current instruction via x/i $eip etc.

# Openocd + nemiver
    
    ./nemiver.sh
  
