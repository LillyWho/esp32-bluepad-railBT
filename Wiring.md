# Wiring Setup

For wiring I recommend getting a pack of jumper wires female to male, which you can either strip on the male end or just solder/clamp into the terminals of the H-bridge.

The below diagram (sorry it's a bit shit) assumes that you use the L298N and only feed it 12V. If you want to supply more than 12V then you have to rewire the setup with a voltage divider that feeds 5V
to the bridge into the 5V pin and also the ESP32. Make sure to pull the voltage regulator jumper before applying power into both the 5V and 12V pins!

![wiring RC](https://github.com/user-attachments/assets/b6cd88c4-1232-4428-be83-a59958d309f8)
