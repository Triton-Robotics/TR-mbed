```
  __________                        __             __   __   ___   ____ ___  _____
 /_  __/ __ \            ____ ___  / /_  ___  ____/ /  / /  |__ \ / __ \__ \|__  /
  / / / /_/ /  ______   / __ `__ \/ __ \/ _ \/ __  /  / /   __/ // / / /_/ / /_ < 
 / / / _, _/  /_____/  / / / / / / /_/ /  __/ /_/ /  / /   / __// /_/ / __/___/ / 
/_/ /_/ |_|           /_/ /_/ /_/_.___/\___/\__,_/  / /   /____/\____/____/____/  
                                                   /_/                            
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
Note: Go to the Embed Discord and search up makefile. After you find it, download it and upload it to your local TR-Embed folder. Most likely your computer will not work with E drive, to which you have to change it to D in the makefile. If you have problems with this, contact a embed member or the team lead. 

After completing all of this you can make and flash your code to the nucleo using `make testbench`, `make hero`, `make infantry`, and `make sentry`. You should see a block with many numbers when the code is flashed. If you see a red text regarding "missing embed-tools" it is likely becuase you don't have the serial monitor downloaded which is not required to make and flash code. 

## Flashing STM32

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

## Documentation

See our [generated doxygen](https://triton-robotics.github.io/TR-mbed/).
