```
  __________                  __             __   ___   ____ ___   ______
 /_  __/ __ \      ____ ___  / /_  ___  ____/ /  |__ \ / __ \__ \ / ____/
  / / / /_/ /_____/ __ `__ \/ __ \/ _ \/ __  /   __/ // / / /_/ //___ \  
 / / / _, _/_____/ / / / / / /_/ /  __/ /_/ /   / __// /_/ / __/____/ /  
/_/ /_/ |_|     /_/ /_/ /_/_.___/\___/\__,_/   /____/\____/____/_____/   
                                                                                               
```

---

| Table of Contents                   |
|-------------------------------------|
| [Prerequisites](#prerequisites)     |
| [Setup](#setup)                     |
| [Flashing](#flashing-stm32)         |
| [Troubleshooting](#troubleshooting) |
| [Documentation](#documentation)     |

---

## Prerequisites

**We have setup guides for the major operating systems.**

### **Please choose the appropriate setup guide for your operating system:**
- ### [Linux](.md/os/linux.md)
- ### [Windows Subsystem for Linux (WSL)](.md/os/wsl.md)
- ### [MacOS](.md/os/macos.md)

---

## Setup
1. Clone this repo:

```shell
git clone https://github.com/Triton-Robotics/TR-mbed.git
```

### At this point, you may choose to diverge from the CLI setup guide and set up an IDE instead. Check out the setup instructions for [CLion](.md/ide/clion.md) or [VSCode](.md/ide/vscode.md). We higly recommend [CLion](.md/ide/clion.md).

2. Configure `CMake` project. This should only be done once per project, or after editing
   any `CMakeLists.txt`:

```shell
cmake -S . -B cmake-build-debug -GNinja
```

3. Build the desired target using your IDE or via CLI:

```shell
cmake --build cmake-build-debug --target TR-TestBench -j $(nproc)
```

Viable targets for build are: `TR-Engineer`, `TR-Infantry`, `TR-Sentry`, `TR-Hero`, and `TR-TestBench`

---


## Flashing and building code

### Finding the Makefile

1. Go into the Makefiles folder, and copy the appropriate makefile for your operating system outside the folder.

2. Now, you can build your code with the following command:

```shell
make testbench-build
```

Note: If the makefile does not work with your system, contact an embed member or team lead for assistance since it might be an issue with the drive (your computer will not work with E drive, to which you have to change it to D in the makefile).

After completing all of this you can make and flash your code to the nucleo using `make testbench-jlink`, `make hero-jlink`, `make infantry-jlink`, and `make sentry-jlink`, or build with `make infantry-build`, `make sentry-build`, `make hero-build`, or `make testbench-build`. You should see a block with many numbers when the code is flashed. If you see a red text regarding "missing embed-tools" it is likely becuase you don't have the serial monitor downloaded which is not required to make and flash code. 


### Flashing to JLink

1. Download the [JLink](https://www.segger.com/downloads/jlink/) software on your device.

2. Flash using the makefile, similar to the cmake setup:

```shell
make testbench-jlink
```

## Flashing STM32 (Old Method)

1. Locate the generated executable:

   eg. The built executable for TestBench will be present in `cmake-build-debug/robots/TestBench/TR-TestBench.elf`


2. Flash the executable to the target device:

```shell
openocd -f board/st_nucleo_f4.cfg -c "program cmake-build-debug/robots/TestBench/TR-TestBench.elf verify reset exit"
```

> For flashing device in WSL, see the [WSL guide](.md/os/wsl.md#flashing-stm32).

3. View serial output:

```shell
mbed-tools sterm -b 115200
```

---

## Troubleshooting

See the [Troubleshooting](.md/TROUBLESHOOTING.md) guide for common issues and solutions.

---

## Debugging

See the [Debugging](.md/debug/stm32cubeide.md) guide on how to run the debugger using STM32CubeIDE

See the [Ozone Debugger](.md/debug/ozone_debugger.md) guide on how to run the debugger using the Ozone Debugger.

--- 

## Documentation

See our [generated doxygen](https://triton-robotics.github.io/TR-mbed/).
