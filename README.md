# Bluepad32 railBT firmware

## This project aims to create a microcontroller that controls a train using an ESP32, an H-Bridge and a Bluetooth Gamepad via Bluepad32.

I made this as a cheaper alternative to premade G scale RC solutions, which can cost upwards of 70€.
The parts I used are cheap and off the shelf components readily available from DIY electronics sites and ebay.

In my case, I used a standard ESP32, an LN239 H-Bridge and a Google Stadia controller (any other controller supported by Bluepad32 would work) to control an EZTEC Toy train that runs on G Gauge.
For the pinout, please refer to the constants set in the sketch.

Please note that all button mappings that aren't A/B, X/Y may be specific to the Stadia controller and thus may have to be adjusted by reading the console printout of the Bluepad test programme, and that the maximum reported value of the thumbstick axis also needs to be adjusted according to the maximum that your specific model returns. It's not a complex process however.

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
- Directional headlights
- Pairing multiple engines together, using the built-in Wifi module as an AP that subsequent engines can join and receive commands from

## Demo video
https://www.youtube.com/watch?v=-wW9WL53EOc
