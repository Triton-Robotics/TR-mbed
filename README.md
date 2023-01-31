## Prerequisites

The following setup guide is written assuming Ubuntu and CLion as the IDE. This setup guide is applicable to other 
operating systems and IDEs, but may take additional setup.

```shell
sudo apt install \
  python3-dev \
  python3-pip \
  gcc-arm-none-eabi \
  ninja-build
```

Use pip to install `mbed` and various dependencies:
```shell
pip install \
  mbed-tools \
  prettytable \
  future \
  jinja2 \
  intelhex
```

In `~/.bash_profile`, append the following to enable `mbed-tools` via command line:
```shell
export PATH="${PATH}:/home/${USER}/.local/bin"
```

Extra dependencies for compiling micro-ros (Optional):
```shell
pip install \
  catkin_pkg \
  lark-parser \
  empy \
  colcon-common-extensions
```
---
## Setup
1. Clone this repo:
```shell
git clone <add link here later>
```

2. Initialize `mbed` project:
```shell
mbed-tools deploy
```

3. Configure project for `CMake`:
```shell
mbed-tools configure -m NUCLEO_F446RE -t GCC_ARM
```

3. Open project by selecting the root `CMakeLists.txt` in your desired IDE.
   
    Alternatively, skip to step 6 for CLI usage. 

![img.png](.assets/img.png)

4. Upon opening the project, ensure the `CMake` directory is set to the following:

![img.png](.assets/img2.png)

5. Ensure the desired robot is selected in your IDE:

![img.png](.assets/img3.png)

6. Build the desired target using your IDE or via CLI:
   
   Note: You may have to modify the number after the `-j` argument to the number of processes your CPU can handle.
   This can be found via the `nproc` command.

```shell
cmake -S . -B cmake_build/NUCLEO_F446RE/develop/GCC_ARM -GNinja

cmake --build cmake_build/NUCLEO_F446RE/develop/GCC_ARM --target TR-Sentry -j 12
```

Viable targets for build are: `TR-Engineer`, `TR-Infantry`, `TR-Sentry`, `TR-Hero`, and `TR-TestBench`

7. Locate the generated binary:

    eg. The built binary for Sentry will be present in `cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Sentry/TR_Sentry.bin`


8. Copy the binary to the target device and open a terminal:

```shell
cp cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Sentry/TR_Sentry.bin /media/${USER}/NOD_F446RE/

mbed-tools sterm
```