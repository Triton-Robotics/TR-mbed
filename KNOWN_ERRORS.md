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



## Functionality Issues

No Serial

* Windows: Restart Mbed

* All Platforms: Check if 5V wire from old devboard is attached. Often external power will prevent the nucleo from power cycling and stop comms.

no remote - flip wires
