**This setup guide assumes WSL is already set up. 
If not, follow [this guide](https://docs.microsoft.com/en-us/windows/wsl/install-win10) to install WSL.**

After downloading WSL you need to turn on "windows for linux subsystem". You can achieve this by going to your settings and find "Turn Windows features on or off". After clicking this, you will be taken to a page where you can turn linux on. 

![Turn Windows Features on or off](.md/os/window)

### Base development packages
```shell
sudo apt install                  \
  build-essential                 \
  cmake                           \
  python3-dev                     \
  python3-pip                     \
  gcc-arm-none-eabi               \
  libnewlib-arm-none-eabi         \
  libstdc++-arm-none-eabi-newlib  \
  openocd                         \
  ninja-build
```

### mbed-tools & Dependencies
```shell
pip install   \
  mbed-tools  \
  prettytable \
  future      \
  jinja2      \
  intelhex
```

If an error pops up about not being able to downlaod it, use 
```shell
pip install libaryname--braek-system-packages
```
and download each one of them one by one. 
### Enable mebed-tools

Type `nano ~/.bashrc` (or `nano ~/.bash_profile`) in the terminal, append the following to enable `mbed-tools` via command line:

```shell
export PATH="${PATH}:/home/${USER}/.local/bin"
```
 
### Flashing STM32 (Better for flashing but optional) 

Author: Michael Owens

Flashing is where WSL begins to diverge from normal Linux. We have to pass through the USB device to WSL and then make sure we have the right installer.

1. Follow [this guide](https://learn.microsoft.com/en-us/windows/wsl/connect-usb#attach-a-usb-device) to install `usbipd-win`

2. Start windows powershell / windows terminal (not cmd, don't be cringe) and try running `usbipd list`. The output should look something like this:

   ```
   PS C:\Users\legor> usbipd list
   Connected:
   BUSID  VID:PID    DEVICE                                                        STATE
   2-3    0b05:19b6  USB Input Device                                              Not shared
   3-2    0483:374b  ST-Link Debug, USB Mass Storage Device, USB Serial Device...  Attached
   3-3    13d3:56eb  USB2.0 HD UVC WebCam, USB2.0 IR UVC WebCam, Camera DFU De...  Not shared
   4-1    0489:e0e2  MediaTek Bluetooth Adapter                                    Not shared
   ```

3. We can see here that the bus id of the st link programmer for the board is `3-2`, so we should run `usbipd wsl attach --busid 3-2` (replace the busid with yours)

4. Finally, open your WSL shell. Run `lsusb`, the output should look like this, and now we know WSL has the device connected.

   ```
   ubuntu@my-pc:~/TR-mbed6$ lsusb
   Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
   Bus 001 Device 003: ID 0483:374b STMicroelectronics ST-LINK/V2.1
   Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
   ```

5. Flash the board with OpenOCD:

```shell
sudo openocd -f board/st_nucleo_f4.cfg -c "program cmake-build-debug/robots/TestBench/TR-TestBench.elf verify reset exit"
```

### Return to [main setup guide](../../README.md#setup) to continue setup.
