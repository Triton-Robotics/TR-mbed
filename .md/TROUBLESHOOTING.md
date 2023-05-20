### Compilation Errors
General compilation errors pertaining to a `mbed` failure usually correlates to a missing header file.
This can be fixed by removing build and IDE caches:
```shell
# Remove IDE specific caches
rm -rf .idea/ .vscode/

# Remove build folders
rm -rf mbed-os/ cmake_build/
```

Re-configure and build the project as dictated by the [Setup](../README.md#setup) guide.

### OpenOCD

If OpenOCD fails with the error `Error: libusb_open() failed with LIBUSB_ERROR_ACCESS`, you may need to add a `udev` rule
to allow OpenOCD to access the ST-Link programmer:

```shell
cd /etc/udev/rules.d

# Download openocd udev rules
sudo wget https://raw.githubusercontent.com/openocd-org/openocd/master/contrib/60-openocd.rules

# Reload udev rules
sudo udevadm control --reload
```

### STM32 Nucleo Status Lights:

Solid Light green: Nucleo is disconnected

- Power Cycle

Solid Red Light: Nominal

Blinking Red Light: Not enough voltage