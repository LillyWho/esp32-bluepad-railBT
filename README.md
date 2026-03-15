# Bluepad32 railBT firmware

## This project aims to programme a microcontroller that controls a train using an ESP32, an H-Bridge and a Bluetooth Gamepad via Bluepad32.

I made this as a cheaper alternative to premade G scale RC solutions, which can cost upwards of 70€.
The parts I used are cheap and off the shelf components readily available from DIY electronics sites and ebay.
I've been designing and constantly upgrading a custom mainboard that houses all the components, reducing the need for manual wiring significantly. See the pcb folder.

# Features:
- Directional headlights (can be disabled if needed in favour of a simple headlight)
- Power pickup from battery, or track (AC/DC/DCC agnostic via full bridge converter, two inputs are provided to be switched via six-pole switch)
- RJ-45 socket for delivering 24V power and RX/TX to another engine (current implementation is similar to RS-232 serial and untested, with i2c or CANBus protocols being considered)
- Inertia simulation (in early testing, doesn't fully work yet) like on a commercial DCC decoder (user-configurable, see CVAR.h in the include folder)
- High grade traction properties thanks to Polulu DRV8874 brushed DC motor controller (Zero resistance braking, coasting, etc, see Polulu website)
- 24V rail for traction and lighting, and 5V rail for microcontroller and peripheries

### Three drive modes are supported: 
- Mode 0 for direct control of direction and speed via the left thumb stick (for fine grain shunting)
- Mode 1 for throttle mode via the left stick
- Mode 2 for control via the D-pad (for those who don't like having to hold the thumb stick during shunting)

## Control scheme: 

- (...) / Select button: Control Mode Down
- Menu / Start button: Control Mode up
- X: Toggle Emergency brake (controller vibrates when applying the brake)
- A: Toggle headlight

## Planned features
- MU with more than one extra engine
- Move MU support to CANBus or other applicable protocol for multi engine support
- Sound via the WAVTrigger board (planned as targeting the WavTrigger Pro for its sound pitching capabilities)
- Library for driving an ultrasonic humidifier board in sync with wheel spin to simulate steam exhaust synced to cylinder beats
- button mapping swappable via libraries
- hall effect sensor support for inductive train control, triggering a SPAD on existing DCC layouts
## Demo video (outdated early prototype version)
https://www.youtube.com/watch?v=-wW9WL53EOc
