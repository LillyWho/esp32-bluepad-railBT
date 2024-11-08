# Bluepad32 railBT firmware

## This project aims to create a microcontroller that controls a train using an ESP32, an H-Bridge and a Bluetooth Gamepad via Bluepad32.

I made this as a cheaper alternative to premade G scale RC solutions, which can cost upwards of 70€.
The parts I used are cheap and off the shelf components readily available from DIY electronics sites and ebay.

In my case, I used a standard ESP32, an LN239 H-Bridge and a Google Stadia controller (any other controller supported by Bluepad32 would work) to control an EZTEC Toy train that runs on G Gauge.
For the pinout, please refer to the constants set in the sketch.

Three modes are supported: 
- Mode 0 for direct control of direction and speed via the left thumb stick
- Mode 1 for throttle mode via the left stick
- Mode 2 for control via the D-pad

## Control scheme: 

- (...) button: Control Mode Down
- Menu button: Control Mode up
- X: Toggle Emergency brake (controller vibrates when applying the brake)
- A: Toggle headlight

## Planned features

- Inertia simulation
- Sound via the WAVTrigger board
- Driving an ultrasonic humidifier board in sync with wheel spin to simulate steam exhaust synced to cylinder beats
