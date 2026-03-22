# Bluepad32 railBT firmware

#### _This project aims to programme a microcontroller that controls a train using an ESP32, an H-Bridge and a Bluetooth Gamepad via Bluepad32._

I made this as a cheaper alternative to premade G scale RC solutions, which can cost upwards of 70€.
The parts I used are cheap and off the shelf components readily available from DIY electronics sites and ebay.
I've been designing and constantly upgrading a custom mainboard that houses all the components, reducing the need for manual wiring significantly. See the pcb folder.

# Features:
- Directional headlights (can be disabled if needed in favour of a simple headlight)
- Power pickup from battery, or track (AC/DC/DCC agnostic via full bridge converter, two inputs are provided to be switched via six-pole switch)
- RJ-45 socket for delivering 24V power and RX/TX to another engine (current implementation is similar to RS-232 serial and untested, with i2c or CANBus protocols being considered)
- Inertia simulation (in early testing, doesn't fully work yet) like on a commercial DCC decoder (user-configurable, see CVAR.h in the include folder)
- High grade traction properties thanks to Polulu DRV8874 brushed DC motor controller (braking to traction percentage, zero-resistance coasting, etc, see [Polulu website](https://www.pololu.com/product/4035]))
- 24V rail for traction and lighting, and 5V rail for microcontroller and peripheries
- Support for ultrasonic humidifier boards. The circuit and firmware allow for rapidly switching the board on and off in tune with the speed of the engine to simulate steam exhaust blast.

### Three drive modes are supported: 
- Mode 0 for direct control of direction and speed via the left thumb stick (for fine grain shunting)
- Mode 1 for throttle mode via the left stick
- Mode 2 for control via the D-pad (for those who don't like having to hold the thumb stick during shunting)

## Control scheme: 

- (...) / Select button: Control Mode Down
- Menu / Start button: Control Mode up
- X: Toggle Emergency brake (controller vibrates when applying the brake)
- A: Toggle headlight

## Parts list:
- XL1509 Buck converter module (a complete module, not the chip itself) or similar
- railBT Mainboard (optional but recommended, submit the kicad file to the custom PCB manufacturer of your choice)
- ESP32 (any generic variant will do)
- RJ-45 Jack
- 470microF 35V capacitor (or similar spec)
- A whole bunch of pin connectors to be broken down into various lengths
- Four two-hole pin sockets
- Two six-hole pin sockets
- One polarised capacitor for five Volts 2A
- Four MBR735 shotkey diodes or similar
- Three IRL540NPBF mosfets or similar
- A whole bunch of dupont jumper wires, in all different varieties - Just buy a large pack of all of them
## Planned features
- MU with more than one extra engine
- Move MU support to CANBus or other applicable protocol for multi engine support
- Sound via the WAVTrigger board (planned as targeting the WavTrigger Pro for its sound pitching capabilities)
- Library for driving an ultrasonic humidifier board in sync with wheel spin to simulate steam exhaust synced to cylinder beats
- button mapping swappable via libraries
- hall effect sensor support for inductive train control, triggering a SPAD on existing DCC layouts
## Demo video (outdated early prototype version)
https://www.youtube.com/watch?v=-wW9WL53EOc
