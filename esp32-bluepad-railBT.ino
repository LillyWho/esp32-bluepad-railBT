#define debug true

// *********************************************************************************
// This project uses Bluepad32 as its input library. Please go to
// https://github.com/ricardoquesada/bluepad32 and read the documentation for
// installation instructions.
#include <Bluepad32.h>
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
// *********************************************************************************
#include <DRV8874.h>
// *********************************************************************************
// This project uses https://github.com/sbouhoun/smoother for smoothing joystick
// input. Please install the library using the Arduino IDE library manager. This
// project uses https://github.com/luisllamasbinaburo/Arduino-Interpolation/ .
// Please install the library using the Arduino IDE library manager.
#define enableAxisSmoothing \
  true  // CHANGEME: Set this to true or false depending on whether you want to \
        // apply smoothing to the joysticks for less erratic output. This depends \
        // on your gamepad and might improve or worsen the subjective feel of the \
        // sticks.
#ifdef enableAxisSmoothing
#include <smooth.h>
#define nbReadings
smoother analogSmooth;
#endif
// **********************************************************************************
#include <InterpolationLib.h>

#include "include/cvars.h"  // CHANGEME: Edit this file in the sketch's folder to customise your engine's behaviour like you would a DCC decoder!

#define supportSound true

#ifdef supportSound
//#define __WT_USE_SERIAL1__
//#define __WT_USE_SERIAL2__
#define __WT_USE_SERIAL3__

#include <wavtrigger-sounddecoder.h>

#endif
/////////////////////////////////////////////////////
// CHANGEME: Enable this if you want to enable the automatic pairing feature!
#define multiHeader true

#ifdef multiHeader
//#include <HardwareSerial.h>
HardwareSerial muComms = Serial2; // use UART port 2 for MU mode
bool head = false;
bool setupCommsComplete = false;
#endif

/////////////////////////////////////////////////////

// mode 0 = absolute analogue
// mode 1 = real analogue throttle
// mode 2 = d-pad incremental mode
int mode = 0;
bool directionPlus = false;
bool directionMinus = false;
float direction = 0;
int targetSpeed = 0;
int maxSpeed = CVAR_5;
long interval = 0;
long lastPuff = 0;
long lastPuffOff = 0;
bool doPuff = false;
bool lightsOn = false;
bool emergencyBrake = false;
bool steamerEnabled = false;
long lastTime = 0;
long deltaTime = 0;
int speed = 0;
//////////////////////////////////////////////////////
/////  The Stadia Gamepad returns the Y axis forwad on the sticks as negative,
/// which is counter-intuitive. So let's invert it if need be.
constexpr bool invertLeftStick = true;
constexpr bool invertRightStick = true;
constexpr bool smoothAxes = true;
/////  Also, let's define the maximum the sticks report so we can lerp
/// accordingly. Change this according to how your controller reports the axes
constexpr long maxYAxisL = 509;
constexpr long maxYAxisR = 509;
long yAxisL = 0;
long yAxisR = 0;
//////////////////////////////////////////////////////
bool dpadUp = false;
bool dpadDown = false;
bool dpadLeft = false;
bool dpadRight = false;
bool startButton = false;
bool selectButton = false;
bool aButton = false;
bool bButton = false;
bool xButton = false;
bool yButton = false;
//////////////////////////////////////////////////////
constexpr float maxAcceleration = 1.0; // Max rate of acceleration
constexpr float maxDeceleration = 2.0; // Max rate of deceleration
constexpr float inertia = 0.1; // Inertia factor (higher values = slower response)
//////////////////////////////////////////////////////
int ADebounce = 0;
int BDebounce = 0;
int YDebounce = 0;
int XDebounce = 0;
int StartDebounce = 0;
int SelectDebounce = 0;
long UpDebounce = 0;
long DownDebounce = 0;
long throttleTick = 0;
//////////////////////////////////////////////////////

constexpr int ledPin1 = 5;
constexpr int ledPin2 = 4;
constexpr int motorPin1 = 14;
constexpr int motorPin2 = 15;
constexpr int speedPin = 2;
bool headlightOn = false;
bool taillightOn = false;
String lastHeadlightState = "off";
//////////////////////////////////////////////////////
constexpr int throttleInterval = 50; // throttle repeat in ms, change according to your preference
constexpr int throttleSteps = 4;
//////////////////////////////////////////////////////

DRV8874 motor(motorPin1, motorPin2);

//////////////////////////////////////////////////////
double speedSteps[] = { 0, 50, 100 };
double throttleIncSteps[] = { 0, 50, 100 };
constexpr int increments = sizeof(throttleIncSteps) / sizeof(throttleIncSteps[0]);

double smoothThrottle(double throttleInput) {
    // static double previousInput = 0;
    std::array<double, 2> tempSpeedSteps; // Use double for type compatibility

    /*if (throttleInput != previousInput) {
        if (throttleInput > previousInput) {
            tempSpeedSteps = { previousInput, throttleInput };
        } else {
            tempSpeedSteps = { throttleInput, previousInput };
        }
    }*/

    // Pass the underlying C-style array using .data()
    return Interpolation::SmoothStep(tempSpeedSteps.data(), throttleIncSteps,
                                     increments, throttleInput, true);
}
void dpad(ControllerPtr ctl) {
    dpadDown = ctl->dpad() & 0x02;
    dpadUp = ctl->dpad() & 0x01;
    dpadRight = ctl->dpad() & 0x04;
    dpadLeft = ctl->dpad() & 0x08;
}
void motorControl(float input) {
    float prevInput = 0; // TODO: prevInput is never changed
    if (prevInput != input && input > 0) {
        if (prevInput > input) {
            motor.brakeForward(input);
        } else if (prevInput < input) {
            motor.forward(input);
        }
    } else if (prevInput != input && input < 0) {
        if (prevInput < input) {
            motor.brakeReverse(input);
        } else if (prevInput > input) {
            motor.reverse(input);
        }
    } else if (input != 0) {
        motor.coast();
    } else if (input == 0) {
        motor.brakeLow();
    }
}
float lerp(float input, float inMin, float inMax, float outMin, float outMax) {
    // Scale input value to the 0-1 range, then apply to the output range
    return outMin + (input - inMin) * (outMax - outMin) / (inMax - inMin);
}

// This callback gets called any time a new gamepad is connected.
// dpadUp to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties &&properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                          ctl->getModelName().c_str(), properties.vendor_id,
                          properties.product_id);
            myControllers[i] = ctl;
            return;
        }
    }

    Serial.println("CALLBACK: Controller connected, but could not find empty slot");
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            return;
        }
    }

    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, "
        "%4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(), // Controller Index
        ctl->dpad(), // D-pad
        ctl->buttons(), // bitmask of pressed buttons
        ctl->axisX(), // (-511 - 512) dpadLeft X Axis
        ctl->axisY(), // (-511 - 512) dpadLeft Y axis
        ctl->axisRX(), // (-511 - 512) dpadRight X axis
        ctl->axisRY(), // (-511 - 512) dpadRight Y axis
        ctl->brake(), // (0 - 1023): brake button
        ctl->throttle(), // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(), // bitmask of pressed "misc" buttons
        ctl->gyroX(), // Gyro X
        ctl->gyroY(), // Gyro Y
        ctl->gyroZ(), // Gyro Z
        ctl->accelX(), // Accelerometer X
        ctl->accelY(), // Accelerometer Y
        ctl->accelZ() // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...

    yAxisL = ctl->axisY();
    if (invertLeftStick)
        yAxisL = -yAxisL;
#ifdef enableAxisSmoothing
    yAxisL = analogSmooth.compute(yAxisL);
#endif

    yAxisR = ctl->axisRY();
    if (invertRightStick)
        yAxisR = -yAxisR;
#ifdef enableAxisSmoothing
    yAxisR = analogSmooth.compute(yAxisR);
#endif

    direction = yAxisL;
    dpad(ctl);

    B(ctl->b());
    A(ctl->a());
    Y(ctl->y());
    X(ctl->x(), ctl);
    Start(ctl->miscButtons() & 0x04);
    Select(ctl->miscButtons() & 0x02);
#ifdef debug
    dumpGamepad(ctl);
#endif
}

void processControllers() {
    for (auto myController: myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
                dpad(myController);
                if (multiHeader) {
                    head = true;
                }
            } else {
#ifdef debug
                Serial.println("Unsupported controller");
#endif
            }
        }
    }
}
#ifdef multiHeader
unsigned long lastMessageTime = 0;
constexpr unsigned long timeout = 500; // 5 second timeout

void setupMuComms() {
    String response; // Initialize response as an empty string

    // Check if there is data available in the serial buffer
    if (muComms.available() > 0) {
        response = muComms.readStringUntil('\n'); // Read the response until a newline
    }

    // Compare the response with "ack"
    if (head) {
        if (response == "ACK") {
            setupCommsComplete = true;
            return;
        }
        muComms.println("RING"); // Send the RING message
    } else {
        if (response == "RING") {
            muComms.println("ACK");
            setupCommsComplete = true;
        }
    }
}
void sendMuComms() {
    String response; // Initialize response as an empty string
    if (muComms.available() > 0) {
        response = muComms.readStringUntil('\n'); // Read the response until a newline
        if (!response.isEmpty()) {
            lastMessageTime = millis();
        }
    }

    String requiredResponse = "speed" + String(targetSpeed) + "ACK";
    if (response != requiredResponse) {
        muComms.println("speed" + String(targetSpeed));
    }
}
void receiveMuComms() {
    if (muComms.available() <= 0)
        return;

    String msg = muComms.readStringUntil('\n'); // Read the response until a newline
    if (msg.isEmpty())
        return;

    String lightDeLimiter = "LIGHT";
    String speedDeLimiter = "SPEED";

    if (msg.indexOf(lightDeLimiter) != -1) {
        String &&command = msg.substring(lightDeLimiter.length());
        headlightControl(command == "ON");
        return;
    }

    if (msg.indexOf(speedDeLimiter) != -1) {
        String &&command = msg.substring(speedDeLimiter.length());
        muComms.println(msg + "ACK");
        int speedCommand = command.toInt();
        float pwm = motor.positivePwm(speedCommand);
        motorControl(pwm);
        headlightOn = speedCommand > 0;
        taillightOn = speedCommand < 0;
    }
}
#endif // multiHeader

void headlightControl(bool enable) {
    if (!multiHeader) {
        digitalWrite(ledPin1, enable);
        return;
    }
    if (!enable) {
        digitalWrite(ledPin1, false);
        digitalWrite(ledPin2, false);
    } else {
        if (taillightOn) {
            lastHeadlightState = "tail";
            digitalWrite(ledPin1, false);
            digitalWrite(ledPin2, true);
        } else if (headlightOn) {
            lastHeadlightState = "head";
            digitalWrite(ledPin1, true);
            digitalWrite(ledPin2, false);
        } else if (lastHeadlightState == "tail" && !taillightOn) {
            digitalWrite(ledPin1, false);
            digitalWrite(ledPin2, true);
        } else if (lastHeadlightState == "head" && !headlightOn) {
            digitalWrite(ledPin1, true);
            digitalWrite(ledPin2, false);
        }
    }
}

void setupBT() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t *addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n",
                  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);
    Serial.println("RailBT Firmware booted. Please connect controller");
    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two
    // connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);
}

// Arduino setup function. Runs in CPU 1
void setup() {
#if multiHeader
    muComms.begin(115200);
#endif
    setupBT();
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(speedPin, OUTPUT);
    pinMode(ledPin1, OUTPUT);
    pinMode(ledPin2, OUTPUT);
}

void mode0() {
    targetSpeed = lerp(yAxisL, -509, 509, -255, 255);
    motorControl(motor.positivePwm(targetSpeed));
}
void mode1() {
    float directionLerp = lerp(yAxisL, -509, 509, -throttleSteps, throttleSteps);
    // Serial.println(directionLerp);
    if (std::abs(targetSpeed) == maxSpeed) {
        throttleTick = 0;
        return;
    } // reset tick variable, write to motor and exit

    if (direction == 0) {
        throttleTick = 0;
    } else if (direction != 0 && millis() - throttleTick > throttleInterval && ((directionPlus && targetSpeed < maxSpeed) || (directionMinus && targetSpeed > -maxSpeed))) {
        throttleTick = millis();
        targetSpeed = constrain(targetSpeed + directionLerp, -maxSpeed, maxSpeed);
    }
    motorControl(motor.positivePwm(targetSpeed));
}
void mode2() {
    if (dpadUp && targetSpeed < maxSpeed) {
        if (UpDebounce == 0 || millis() - UpDebounce > throttleInterval) {
            targetSpeed = constrain(targetSpeed + 4, -maxSpeed, maxSpeed);
            UpDebounce = millis();
        }
    } else if (dpadDown && targetSpeed > -maxSpeed) {
        if (DownDebounce == 0 || millis() - DownDebounce > throttleInterval) {
            targetSpeed = constrain(targetSpeed - 4, -maxSpeed, maxSpeed);
            DownDebounce = millis();
        }
    }
    if (!dpadUp) {
        UpDebounce = 0;
    }
    if (!dpadDown) {
        DownDebounce = 0;
    }
    motorControl(motor.positivePwm(targetSpeed));
}
void modeUp() {
    if (mode == 2)
        return;
    mode = min(mode + 1, 2); // Increment Mode, but make sure it cannot exceed 2
}
void modeDown() {
    if (mode == 0)
        return;
    mode = max(mode - 1, 0); // Decrement Mode, but make sure it cannot go below 0
}

void B(bool pressed) {
    if (!pressed) {
        BDebounce = 0;
        return;
    }
    if (BDebounce == 0) {
        BDebounce = millis();
    }
}
void A(bool pressed) {
    if (!pressed) {
        ADebounce = 0;
        return;
    }
    if (ADebounce == 0) {
        ADebounce = millis();
        lightsOn = !lightsOn;
    }
}
void Y(bool pressed) {
    if (!pressed) {
        YDebounce = 0;
        return;
    }
    if (YDebounce == 0) {
        YDebounce = millis();
        modeUp();
    }
}
void X(bool pressed, ControllerPtr ctl) {
    if (!pressed) {
        XDebounce = 0;
        return;
    }
    if (XDebounce == 0) {
        XDebounce = millis();
        emergencyBrake = !emergencyBrake;
        if (emergencyBrake) {
            ctl->playDualRumble(0 /* delayedStartMs */, 1000 /* durationMs */,
                                0x80 /* weakMagnitude */, 0x40 /* strongMagnitude */);
        }
    }
}
void Start(bool pressed) {
    if (!pressed) {
        StartDebounce = 0;
        return;
    }
    if (StartDebounce == 0) {
        StartDebounce = millis();
        mode = min(mode + 1, 2); // Increment Mode, but make sure it cannot exceed 2
    }
}
void Select(bool pressed) {
    if (!pressed) {
        SelectDebounce = 0;
        return;
    }
    if (SelectDebounce == 0) {
        SelectDebounce = millis();
        if (mode > 0) {
            mode = max(mode - 1, 0); // Decrement Mode, but make sure it cannot go below 0
        }
    }
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    if (BP32.update())
        processControllers();

    // The main loop must have some kind of "yield to lower priority task"
    // event. Otherwise, the watchdog will get triggered. If your main loop
    // doesn't have one, just add a simple `vTaskDelay(1)`. Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

#if multiHeader
    if (!setupCommsComplete) {
        setupMuComms();
        return;
    }
    if (!head) {
        receiveMuComms();
        return;
    }
#endif
    directionPlus = direction > 0;
    directionMinus = direction < 0;

    if (emergencyBrake) {
        targetSpeed = 0;
    } else {
        switch (mode) {
            default:
            case 0:
                mode0();
                break;
            case 1:
                mode1();
                break;
            case 2:
                mode2();
                break;
        }
    }
    headlightOn = targetSpeed > 0;
    taillightOn = targetSpeed < 0;
    headlightControl(lightsOn);

    analogWrite(speedPin, abs(targetSpeed));
    vTaskDelay(1);
}
