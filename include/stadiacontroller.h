#ifndef stadiacontroller
#define stadiacontroller

#include <Arduino.h>

class gamepad {
public:
  gamepad();

  bool dpad_up();
  bool dpad_down();
  bool dpad_left();
  bool dpad_right();
  bool button_a();
  bool button_b();
  bool button_x();
  bool button_y();
  int axis1_y();
  int axis1_x();
  int axis2_y();
  int axis2_x();

private:
  int max_throttle = 1020;
  int max_brake = 1020;
  int max_axis = 509;
  int axis_deadzone = 16;
};

#endif // stadiacontroller
