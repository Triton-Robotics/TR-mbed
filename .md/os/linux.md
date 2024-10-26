The following setup guide is written assuming Ubuntu. This setup guide is applicable to other
distros, but the package names may vary per distro.

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

In `~/.bashrc` (or `~/.bash_profile`), append the following to enable `mbed-tools` via command line:

```shell
export PATH="${PATH}:/home/${USER}/.local/bin"
```

### Return to [main setup guide](../../README.md#setup) to continue setup.