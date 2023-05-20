```
  __________                        __             __   __   ___   ____ ___  _____
 /_  __/ __ \            ____ ___  / /_  ___  ____/ /  / /  |__ \ / __ \__ \|__  /
  / / / /_/ /  ______   / __ `__ \/ __ \/ _ \/ __  /  / /   __/ // / / /_/ / /_ < 
 / / / _, _/  /_____/  / / / / / / /_/ /  __/ /_/ /  / /   / __// /_/ / __/___/ / 
/_/ /_/ |_|           /_/ /_/ /_/_.___/\___/\__,_/  / /   /____/\____/____/____/  
                                                   /_/                            
```

## Prerequisites

**We strongly recommend using Linux (Ubuntu or any other flavor) for development.** If this is not possible,
we've created setup guides for the major operating systems below:

- [Linux](.md/os/linux.md)
- [Windows](.md/os/windows.md)
- [Windows Subsystem for Linux (WSL)](.md/os/wsl.md)
- [MacOS](.md/os/macos.md)

---

## Setup
1. Clone this repo:

```shell
git clone https://github.com/Triton-Robotics/TR-mbed.git
```

> At this point, you may choose to diverge from the CLI setup guide and set up an IDE instead. Check out the setup instructions for [CLion](.md/ide/clion.md) or [VSCode](.md/ide/vscode.md).

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

If OpenOCD fails with the error `Error: libusb_open() failed with LIBUSB_ERROR_ACCESS`, you may need to add a `udev` rule
to allow OpenOCD to access the ST-Link programmer:

```shell
cd /etc/udev/rules.d

# Download openocd udev rules
sudo wget https://raw.githubusercontent.com/openocd-org/openocd/master/contrib/60-openocd.rules

# Reload udev rules
sudo udevadm control --reload
```