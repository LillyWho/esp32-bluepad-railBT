# Bluepad32 railBT firmware

## This project aims to create a microcontroller that controls a train using an ESP32, an H-Bridge and a Bluetooth Gamepad via Bluepad32.

I made this as a cheaper alternative to premade G scale RC solutions, which can cost upwards of 70€.
The parts I used are cheap and off the shelf components readily available from DIY electronics sites and ebay.

~~In my case, I used a standard ESP32, an LN239 H-Bridge and a Google Stadia controller (any other controller supported by Bluepad32 would work but would require editing) to control an EZTEC Toy train that runs on G Gauge.
For the pinout, please refer to the constants set in the sketch.~~ This was true, however I've modified the firmware to suit the features of the Polulu DRV8874 h-bridge board. This means that the firmware now depends on a custom library which can be found [here](https://github.com/LillyWho/DRV8874_H). The DRV8874 provides benefits such as not being reliant on a 5V voltage line, motor drive features such as coasting/braking to given pwm. Due to the lack of a 5V DC-DC converter on board of the h-bridge you will now have to supply your own to drive the ESP32. The design relies on the following: [DC converter](https://www.ebay.de/itm/284112436345)
Please check the PCB folder for a KiCad project that includes both a mainboard for engines, and a breakout board for coaches. The mainboard relies on RJ-45 connectors to establish communication and a shared 24V line between the two engines. The breakout board is meant to be put into coaches, to control lighting. The coaches might gain more functionality once I switch the MU control to CANBus as it allows for more than two devices talking in serial.

Please note that all button mappings that aren't A/B, X/Y may be specific to the Stadia controller and thus may have to be adjusted by reading the console printout of the Bluepad test programme and editing the code accordingly, and that the maximum reported value of the thumbstick axis also needs to be adjusted according to the maximum that your specific model returns. It's not a complex process however. In the future I will shift the input functions over to a library making it so that different button mappings for different controllers can simply be swapped out by including a different library.

### Three modes are supported: 
- Mode 0 for direct control of direction and speed via the left thumb stick
- Mode 1 for throttle mode via the left stick
- Mode 2 for control via the D-pad

## Control scheme: 

- (...) button: Control Mode Down
- Menu button: Control Mode up
- X: Toggle Emergency brake (controller vibrates when applying the brake)
- A: Toggle headlight

  
## Other features:
- MU control via serial connection (the current implementation is designed to have the trailing engine run in reverse, as is the case with any push-pull consist with power cars)
- Directional headlight, can be enabled/disabled via a constant

## Planned features
- MU with more than one extra engine
- Move MU support to CANBus for multi engine support (maybe?)
- Inertia simulation
- Sound via the WAVTrigger board
- Library for driving an ultrasonic humidifier board in sync with wheel spin to simulate steam exhaust synced to cylinder beats
- button mapping swappable via libraries
- hall effect sensor support for inductive train control, triggering a SPAD on existing DCC layouts
## Demo video (outdated version)
https://www.youtube.com/watch?v=-wW9WL53EOc
