1. Open project by selecting the root `CMakeLists.txt` in your desired IDE. CLion
will automatically run the configure step.

![img.png](../.assets/clion_1.png)

2. Edit your targets by navigating to `Run > Edit Configurations...`. Remove unneeded
targets by selecting multiple at a time:

![img.png](../.assets/clion_4.png)

3. Add a new `OpenOCD` configuration:

![img.png](../.assets/clion_5.png)

4. Configure the desired target as shown:

   1. Note that this process will have to be repeated a few times for each robot target

![img.png](../.assets/clion_6.png)

5. Ensure the desired robot is selected in your IDE:

![img.png](../.assets/clion_7.png)

6. Build and run using your IDE. Ensure board is connected before running.

   Expected output is similar to the following:
```
...
** Programming Started **
Info : device id = 0x10006421
Info : flash size = 512 kbytes
** Programming Finished **
...
shutdown command invoked
```

7. Install the following plugin to view serial output:

![img.png](../.assets/clion_8.png)

8. Navigate to `Tools > Serial Port Monitor > Settings` and select the correct serial port and baud rate:

![img.png](../.assets/clion_9.png)

9. Open the serial port monitor by navigating to `Tools > Serial Port Monitor > /dev/ttyACM0` and observe the output.
    1. You may have to disable the "Hex View" option for cleaner output:

![img.png](../.assets/clion_10.png)