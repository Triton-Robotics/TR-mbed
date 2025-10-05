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
Note: Go to the Embed Discord and search up makefile. After you find it, download it and upload it to your local TR-Embed folder. Most likely your computer will not work with E drive, to which you have to change it to D in the makefile. If you have problems with this, contact a embed member or the team lead. 

After completing all of this you can make and flash your code to the nucleo using `make testbench`, `make hero`, `make infantry`, and `make sentry`. You should see a block with many numbers when the code is flashed. If you see a red text regarding "missing embed-tools" it is likely becuase you don't have the serial monitor downloaded which is not required to make and flash code. 

### Flashing to JLink

1. Download the [JLink](https://www.segger.com/downloads/jlink/) software and the [Ozone](https://www.segger.com/downloads/jlink/#Ozone) debugger on your devices.

2. Flash using the makefile, similar to the cmake setup:

```shell
make testbench-jlink
```

### Debugging with Ozone

1. To debug with Ozone, you need to open the Ozone debugger, then create a new project.

2. In devices, you should select the STM32F446RE device, then press Next:

<img width="599" height="673" alt="image" src="https://github.com/user-attachments/assets/09b85499-3bea-4122-bb26-876b41bdf92e" />

3. Now, for the Connection settings, select SWD as the Target Interface, USB as the Host Interface, and 4MHz as the Interface Speed

<img width="600" height="682" alt="image" src="https://github.com/user-attachments/assets/c0949765-56de-4f36-9fa7-2495f13ceefb" />

4. Finally, select the correct .elf file from your cmake-build-debug folder in the repository to configure Ozone

<img width="596" height="679" alt="image" src="https://github.com/user-attachments/assets/79d86231-8c65-461d-842a-af0ef8510bf2" />

5. Press Finish since we do not need to change the Optional Settings.

Congrats, you have Ozone set up!

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

## Debugging

See the [Debugging](.md/debug/stm32cubeide.md) guide on how to run the debugger using STM32CubeIDE

--- 

## Documentation

See our [generated doxygen](https://triton-robotics.github.io/TR-mbed/).
