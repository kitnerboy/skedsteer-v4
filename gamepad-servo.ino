#include <Bluepad32.h>
#include <ESP32Servo.h>

// --- Pin Definitions based on updated netlist ---
// Drive Motor Driver (DRV8833 #2)
const int rightMotorPin1 = 10;
const int rightMotorPin2 = 7;
const int leftMotorPin1  = 21;
const int leftMotorPin2  = 20;

// Arm & Aux Motor Driver (DRV8833 #1)
const int armMotorPin1   = 0;
const int armMotorPin2   = 1;
const int auxLightsPin   = 3; // Aux lights (controlled via single pin, IN4 grounded)

// Servos
const int bucketServoPin = 5;
const int clawServoPin   = 6;

Servo bucketServo;
Servo clawServo;

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// Bluepad32 standard gamepad button bitmasks
constexpr uint32_t BTN_A  = 0x0001;
constexpr uint32_t BTN_X  = 0x0004;
constexpr uint32_t BTN_Y  = 0x0008;
constexpr uint32_t BTN_L1 = 0x0010;
constexpr uint32_t BTN_R1 = 0x0020;

// Smoothing variables for DC motors (acceleration ramping)
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
const int accelerationStep = 4;

// Servo control variables
float bucketServoPosition = 90.0; // Start at middle position (90 degrees)
const float bucketFixedSpeed = 2.0; // Fixed angle step per loop for digital bumpers R1/L1

float clawServoPosition = 90.0;   // Start at middle position (90 degrees)
const float clawFixedSpeed = 2.0;   // Fixed angle step per loop for buttons A/Y

// State variable for lights toggle
bool lightsOn = false;
unsigned long lastButtonPress = 0;

void setup() {
    USBSerial.begin(115200);

    // Configure motor and light control pins as outputs
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);
    pinMode(leftMotorPin1,  OUTPUT);
    pinMode(leftMotorPin2,  OUTPUT);

    pinMode(armMotorPin1, OUTPUT);
    pinMode(armMotorPin2, OUTPUT);
    pinMode(auxLightsPin, OUTPUT);

    // Attach servos and set initial positions
    bucketServo.attach(bucketServoPin);
    clawServo.attach(clawServoPin);
    bucketServo.write((int)bucketServoPosition);
    clawServo.write((int)clawServoPosition);

    // Initialize Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableVirtualDevice(false);

    delay(2000);
    USBSerial.println("MiniSkidi-V4 initialized with ESP32-C3-SuperMini pinout.");
}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processControllers();
    }
    vTaskDelay(10); // Loop delay to keep servo motion smooth and predictable
}

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            USBSerial.println("Controller connected\n");
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            USBSerial.println("Controller disconnected from index\n");
            // Stop all actuators safely on disconnect
            moveMotors(0, 0);
            moveArm(0);
            digitalWrite(auxLightsPin, LOW);
            return;
        }
    }
}

void moveMotors(int leftSpeed, int rightSpeed) {
    if (leftSpeed > 0) {
        analogWrite(leftMotorPin1, leftSpeed);
        digitalWrite(leftMotorPin2, LOW);
    } else {
        analogWrite(leftMotorPin1, 0);
        digitalWrite(leftMotorPin2, abs(leftSpeed));
    }

    if (rightSpeed > 0) {
        analogWrite(rightMotorPin1, rightSpeed);
        digitalWrite(rightMotorPin2, LOW);
    } else {
        analogWrite(rightMotorPin1, 0);
        digitalWrite(rightMotorPin2, abs(rightSpeed));
    }
}

void moveArm(int speed) {
    if (speed > 0) {
        analogWrite(armMotorPin1, speed);
        digitalWrite(armMotorPin2, LOW);
    } else if (speed < 0) {
        analogWrite(armMotorPin1, 0);
        analogWrite(armMotorPin2, abs(speed));
    } else {
        digitalWrite(armMotorPin1, LOW);
        digitalWrite(armMotorPin2, LOW);
    }
}

const int triggerDeadzone = 100; // Deadzone threshold for resting trigger noise/offset

int computeTriggerSpeed(int rawValue) {
    if (rawValue <= triggerDeadzone) {
        return 0;
    }
    // Map smoothly from deadzone edge to full speed
    return constrain(map(rawValue, triggerDeadzone, 1023, 0, 255), 0, 255);
}

void processGamepad(ControllerPtr ctl) {
    // --- 1. Drive Control (Left Stick) ---
    int LXValue = ctl->axisX();
    int LYValue = ctl->axisY();

    int targetDrive = -map(LYValue, -512, 511, -255, 255);
    int targetSteer = map(LXValue, -512, 511, -255, 255);

    int targetLeftSpeed  = constrain(targetDrive - targetSteer, -255, 255);
    int targetRightSpeed = constrain(targetDrive + targetSteer, -255, 255);

    if (abs(LXValue) < 15 && abs(LYValue) < 15) {
        targetLeftSpeed = 0;
        targetRightSpeed = 0;
    }

    // Acceleration ramping
    if (currentLeftSpeed < targetLeftSpeed)
        currentLeftSpeed = min(currentLeftSpeed + accelerationStep, targetLeftSpeed);
    else if (currentLeftSpeed > targetLeftSpeed)
        currentLeftSpeed = max(currentLeftSpeed - accelerationStep, targetLeftSpeed);

    if (currentRightSpeed < targetRightSpeed)
        currentRightSpeed = min(currentRightSpeed + accelerationStep, targetRightSpeed);
    else if (currentRightSpeed > targetRightSpeed)
        currentRightSpeed = max(currentRightSpeed - accelerationStep, targetRightSpeed);

    moveMotors(currentLeftSpeed, currentRightSpeed);

    // --- 2. Arm Control via Analog Triggers (R2 / L2) ---
    // ctl->throttle() is R2, ctl->brake() is L2 (values range from 0 to 1023)
    int r2Value = ctl->throttle();
    int l2Value = ctl->brake();

    int armSpeed = computeTriggerSpeed(r2Value) - computeTriggerSpeed(l2Value);
    armSpeed = constrain(armSpeed, -255, 255);
    moveArm(armSpeed);

    // --- 3. Bucket Servo Control via Digital Bumpers (R1 / L1) ---
    uint32_t buttons = ctl->buttons();
    bool r1Pressed = (buttons & BTN_R1) != 0;
    bool l1Pressed = (buttons & BTN_L1) != 0;

    float bucketStep = 0.0f;
    if (r1Pressed) {
        bucketStep += bucketFixedSpeed;
    }
    if (l1Pressed) {
        bucketStep -= bucketFixedSpeed;
    }

    if (bucketStep != 0.0f) {
        bucketServoPosition += bucketStep;
        bucketServoPosition = constrain(bucketServoPosition, 0.0f, 180.0f);
        bucketServo.write((int)bucketServoPosition);
    }

    // --- 4. Claw Servo Control via Face Buttons (A / Y) ---
    bool aPressed = (buttons & BTN_A) != 0;
    bool yPressed = (buttons & BTN_Y) != 0;

    float clawStep = 0.0f;
    if (aPressed) {
        clawStep += clawFixedSpeed;
    }
    if (yPressed) {
        clawStep -= clawFixedSpeed;
    }

    if (clawStep != 0.0f) {
        clawServoPosition += clawStep;
        clawServoPosition = constrain(clawServoPosition, 0.0f, 180.0f);
        clawServo.write((int)clawServoPosition);
    }

    // --- 5. Lights Toggle (Button X) ---
    if ((buttons & BTN_X) && (millis() - lastButtonPress > 400)) {
        lightsOn = !lightsOn;
        digitalWrite(auxLightsPin, lightsOn ? HIGH : LOW);
        lastButtonPress = millis();
    }
}

void processControllers() {
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            if (ctl->isGamepad()) {
                processGamepad(ctl);
            }
        }
    }
}