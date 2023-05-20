This setup guide uses a package manager called `winget`. If not already installed,
`winget` can be installed from the [Microsoft Store](https://apps.microsoft.com/store/detail/app-installer/9NBLGGH4NNS1?hl=en-gb&gl=gb).

### Base development packages
```shell
winget install Kitware.CMake
winget install Python.Python.3.10
winget install Ninja-build.Ninja
winget install Arm.GnuArmEmbeddedToolchain
```
> Note that there is currently no package hosted for `OpenOCD` on `winget` repositories. For the time being,
> you may install `OpenOCD` from the [official website](https://gnutoolchains.com/arm-eabi/openocd/).

**Make sure to close and re-open Powershell to use newly installed CLI utilities.**

### mbed-tools & Dependencies
```shell
pip install   \
  mbed-tools  \
  prettytable \
  future      \
  jinja2      \
  intelhex
```

### Return to [main setup guide](../../README.md#setup) to continue setup.