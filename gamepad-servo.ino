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

// Smoothing variables for DC motors (acceleration ramping)
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
const int accelerationStep = 4; 
const int dead_zone_value = 15;

// Arm fixed PWM speed for digital bumper control (0..255)
const int armFixedSpeed = 140;

// Servo control variables
float bucketServoPosition = 90.0; // Bucket servo (analog triggers L2/R2)
const float bucketServoSpeed = 2.0;  

float clawServoPosition = 90.0;   // Claw servo (discrete buttons)
const float clawFixedSpeed = 2.0;     // Fixed speed for discrete control

// State variable for lights toggle
bool lightsOn = false;
unsigned long lastButtonPress = 0;

constexpr uint32_t BTN_A  = 0x0001;
constexpr uint32_t BTN_B  = 0x0002;
constexpr uint32_t BTN_X  = 0x0004;
constexpr uint32_t BTN_Y  = 0x0008;

void setup() {
    USBSerial.begin(115200);
    
    // Configure motor and light control pins as outputs
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);
    pinMode(leftMotorPin1, OUTPUT);
    pinMode(leftMotorPin2, OUTPUT);
    
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

    USBSerial.println("MiniSkidi-V4 initialized with bumper arm control.");
}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processControllers();
    }
    vTaskDelay(10); // Loop delay for smooth servo updates
}

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            USBSerial.printf("Controller connected at index %d\n", i);
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            USBSerial.printf("Controller disconnected from index %d\n", i);
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

void setMotorPinPwm(int pin, int duty) {
    if (duty > 0) {
        pinMode(pin, OUTPUT);
        analogWrite(pin, constrain(duty, 0, 255));
    } else {
        // Switch unused motor input back to plain GPIO LOW to release PWM activity
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}

void moveArm(int speed) {
    if (speed > 0) {
        setMotorPinPwm(armMotorPin1, speed);
        setMotorPinPwm(armMotorPin2, 0);
    } else if (speed < 0) {
        setMotorPinPwm(armMotorPin1, 0);
        setMotorPinPwm(armMotorPin2, abs(speed));
    } else {
        setMotorPinPwm(armMotorPin1, 0);
        setMotorPinPwm(armMotorPin2, 0);
    }
}

void processGamepad(ControllerPtr ctl) {
    // --- 1. Drive Control (Left Stick) ---
    int LXValue = ctl->axisX();
    int LYValue = ctl->axisY();

    int targetDrive = (abs(LYValue) > 20) ? -map(LYValue, -512, 511, -255, 255) : 0;
    int targetSteer = (abs(LXValue) > 20) ? map(LXValue, -512, 511, -255, 255) : 0;

    int leftSpeed  = constrain(targetDrive + targetSteer, -255, 255);
    int rightSpeed = constrain(targetDrive - targetSteer, -255, 255);

    moveMotors(leftSpeed, rightSpeed);

    // --- 2. Arm Control via buttons A and Y (R1 / L1) ---
    uint32_t buttons = ctl->buttons();
    // Adjust bit masks if your Steam Controller maps bumpers to different bits
    bool armUpPressed = (buttons & BTN_A) != 0;
    bool armDownPressed = (buttons & BTN_Y) != 0;

    int armSpeed = 0;
    if (armUpPressed && !armDownPressed) {
        armSpeed = armFixedSpeed;
    } else if (armDownPressed && !armUpPressed) {
        armSpeed = -armFixedSpeed;
    }
    moveArm(armSpeed);

    // --- 3. Bucket Servo Control via buttons B and X ---
    float bucketStep = 0;
    if (buttons & BTN_B) {
        bucketStep += bucketServoSpeed;
    }
    if (buttons & BTN_X) {
        bucketStep -= bucketServoSpeed;
    }

    if (bucketStep != 0) {
        bucketServoPosition += bucketStep;
        bucketServoPosition = constrain(bucketServoPosition, 0.0, 180.0);
        bucketServo.write((int)bucketServoPosition);
    }

    // --- 4. Claw Servo Control (kept compatible with base sketch) ---
    // float clawStep = 0;
    // if (buttons & 0x0001) {
    //     clawStep += clawFixedSpeed;
    // }
    // if (buttons & 0x0002) {
    //     clawStep -= clawFixedSpeed;
    // }

    // if (clawStep != 0) {
    //     clawServoPosition += clawStep;
    //     clawServoPosition = constrain(clawServoPosition, 0.0, 180.0);
    //     clawServo.write((int)clawServoPosition);
    // }

    // --- 5. Lights Toggle (Button X / Square) ---
    // if ((buttons & 0x0004) && (millis() - lastButtonPress > 400)) {
    //     lightsOn = !lightsOn;
    //     digitalWrite(auxLightsPin, lightsOn ? HIGH : LOW);
    //     lastButtonPress = millis();
    // }
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
