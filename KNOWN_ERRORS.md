# Known Issues

## Lights:

Solid Light green - Nucleo is disconnected

- Power Cycle

Solid Red Light - Good

Blinking Red Light - Not enough voltage

- Give it more voltage

---

## Flashing Code Issues:

Hard Fault - You're calling a function in two places at once (Thread Related)

- Fix the code so that is not the case

STLINK Error

- Restart Computer

- Manually re-configure the device as NUCLEO F446RE

Icode

* Restart Computer

`ERROR: Mbed OS was not found due to the following error: This MbedOS copy does not contain a targets.json file.  
You may need to resolve the mbed-os.lib reference. You can do this by performing a 'deploy'.`

* Delete the files mbed-os.lib, mbed_app.json and the folder mbed-os. Save your main code (it will be overwritten), and then type in mbed-tools new . (inside of the docker environment).

* It will re-install mbed-os, and then you can paste your main code back in.

## Functionality Issu# Known Issues

## Lights:

Solid Light green - Nucleo is disconnected

- Power Cycle

Solid Red Light - Good

Blinking Red Light - Not enough voltage

- Give it more voltage

---

## Flashing Code Issues:

Hard Fault - You're calling a function in two places at once (Thread Related)

- Fix the code so that is not the case

STLINK Error

- Restart Computer

- Manually re-configure the device as NUCLEO F446RE

Icode

- Restart Computer

`ERROR: Mbed OS was not found due to the following error: This MbedOS copy does not contain a targets.json file. 
You may need to resolve the mbed-os.lib reference. You can do this by performing a 'deploy'.`

- Delete the files mbed-os.lib, mbed_app.json and the folder mbed-os. Save your main code (it will be overwritten), and then type in mbed-tools new . (inside of the docker environment).

- It will re-install mbed-os, and then you can paste your main code back in.

## Functionality Issues

No Serial

- Windows: Restart Mbed

- All Platforms: Check if 5V wire from old devboard is attached. Often external power will prevent the nucleo from power cycling and stop comms.

no remote - flip wires
