# OPEL-automatic-revmatch-project

Hi! This is a project on creating a device that by plugging into a car's CAN buses and thottle pedal signals, is able to match the
engine RPMs of the lower gear to the wheel speed during a downshift, known as "Revmatching".
The operation of the device is controlled by a PIC16F877A MCU from Microchip.

### Basic idea
The basic idea is as follows: 
Steering wheel button pressed -> Wait for clutch disengagement -> Raise engine RPM to match the lower gear at the current speed and keep it there
-> Wait for clutch engagement -> Stop.

### CAN bus data
From the 3 CAN buses of my 2005 OPEL Vectra C we are interested in 2 of them: 95kbps low speed bus from where driver input can be read 
(such as steering wheel buttons for activating revmatching mode) and the 500kbps bus from where vehicle telematics data can be read
(such as engine RPM, vehicle speed, clucth state etc.). The vehicles CAN buses will be interfaced with a MCP2515 CAN controller IC
and a MCP2551 CAN transciever IC for each of the 2 buses.

### Controlling the throttle
Throttle is controlled by tapping into the (2) throttle pedal potentiometer output wires and with a signal relay we can disconnect
the thottle pedal outputs and start driving the signal voltage with a capacitor smooothed PWM signal to be interpreted by the ECU.
Throttle value is calculated using PID control.

### Additional features
Additional features of the device include displaying current gear on the dashboard with a 7-segment led and a led array for the current RPM. 
These are driven by 2 8-bit shift registers that are interfaced by the MCU with a single differential pair using a RS485 transciever.
Another feature is "welcome actions" where upon unlocking the car doors, horn is briefly activated 3 times
and high beams flashed. This is done by replicating driver input by sending a CAN frame that corresponds to the activation of their driver controls.

