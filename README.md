```
  __________                        __             __   __   ___   ____ ___  _____
 /_  __/ __ \            ____ ___  / /_  ___  ____/ /  / /  |__ \ / __ \__ \|__  /
  / / / /_/ /  ______   / __ `__ \/ __ \/ _ \/ __  /  / /   __/ // / / /_/ / /_ < 
 / / / _, _/  /_____/  / / / / / / /_/ /  __/ /_/ /  / /   / __// /_/ / __/___/ / 
/_/ /_/ |_|           /_/ /_/ /_/_.___/\___/\__,_/  / /   /____/\____/____/____/  
                                                   /_/                            
```

## Prerequisites

The following setup guide is written assuming Ubuntu. This setup guide is applicable to other
operating systems, but may take additional setup (i.e. WSL for Windows, or HomeBrew for MacOS).

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

Use pip to install `mbed` and various dependencies:

```shell
pip install   \
  mbed-tools  \
  prettytable \
  future      \
  jinja2      \
  intelhex
```

In `~/.bashrc` (or `~/.bash_profile`), append the following to enable `mbed-tools` via command line:

```shell
export PATH="${PATH}:/home/${USER}/.local/bin"
```

Optional: Extra dependencies for compiling micro-ros:

```shell
pip install   \
  catkin_pkg  \
  lark-parser \
  empy        \
  colcon-common-extensions
```

Now you can skip to the Setup section.

## Prerequisites [Windows]

This setup is for Windows. 

```
winget install Kitware.CMake
winget install Python.Python.3.10
winget install Ninja-build.Ninja
winget install Arm.GnuArmEmbeddedToolchain
```

Then we pip install the rest

```
pip install mbed-tools prettytable future jinja2 intelhex
```

The rest of the setup is the same, follow below.

---

## Setup

1. Clone this repo:
   
   ```shell
   git clone https://github.com/Triton-Robotics/TR-mbed.git
   ```

2. Navigate to project root directory and initialize `mbed` project:
   
   ```shell
   cd TR-mbed && mbed-tools deploy
   ```

**Optional: At this point, you may choose to diverge from the CLI setup guide and set up an IDE
instead. Check out the setup instructions for [CLion](.md/clion.md) or [VSCode](.md/vscode.md).**

3. Configure `CMake` project. This should only be done once per project, or after editing
   any `CMakeLists.txt`:

```shell
cmake -S . -B cmake_build/NUCLEO_F446RE/develop/GCC_ARM -GNinja
```

4. Build the desired target using your IDE or via CLI:

```shell
cmake --build cmake_build/NUCLEO_F446RE/develop/GCC_ARM --target TR-Sentry -j $(nproc)
```

Viable targets for build are: `TR-Engineer`, `TR-Infantry`, `TR-Sentry`, `TR-Hero`, and `TR-TestBench`

5. Locate the generated executable:
   
   eg. The built executable for Sentry will be present in `cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Sentry/TR_Sentry.elf`

6. Flash the executable to the target device and open a serial terminal:
   
   1. For flashing device in WSL, see the [Appendix](#appendix-wsl-steps)

```shell
openocd -f board/st_nucleo_f4.cfg -c "program cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Sentry/TR-Sentry.elf verify reset exit"

mbed-tools sterm -b 115200
```

If OpenOCD fails with the error `Error: libusb_open() failed with LIBUSB_ERROR_ACCESS`, you may need to add a `udev` rule
to allow OpenOCD to access the ST-Link programmer:

```shell
cd /etc/udev/rules.d

# Download openocd udev rules
sudo wget https://raw.githubusercontent.com/openocd-org/openocd/master/contrib/60-openocd.rules

# Reload udev rules
sudo udevadm control --reload
```

---

## Appendix: WSL Steps

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
sudo openocd -f board/st_nucleo_f4.cfg -c "program cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Sentry/TR-Sentry.elf verify reset exit"
```

### 