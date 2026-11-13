#include <Bluepad32.h>
#include <ESP32Servo.h>

// --- Pin Definitions based on updated netlist ---
// Drive Motor Driver (DRV8833 #2)
const int rightMotorPin1 = 7;
const int rightMotorPin2 = 10;
const int leftMotorPin1  = 6;
const int leftMotorPin2  = 5;

// Arm & Aux Motor Driver (DRV8833 #1)
const int armMotorPin1   = 20;
const int armMotorPin2   = 21;
const int auxPin3        = 1;
const int auxPin4        = 0;

// Servos
const int bucketServoPin = 3;
const int clawServoPin   = 4;

Servo bucketServo;
Servo clawServo;

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// Smoothing variables for DC motors (acceleration ramping)
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
const int accelerationStep = 4; 

void setup() {
    USBSerial.begin(115200);
    
    // Configure motor control pins as outputs
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);
    pinMode(leftMotorPin1,  OUTPUT);
    pinMode(leftMotorPin2,  OUTPUT);
    
    pinMode(armMotorPin1, OUTPUT);
    pinMode(armMotorPin2, OUTPUT);
    pinMode(auxPin3,      OUTPUT);
    pinMode(auxPin4,      OUTPUT);

    // Attach servos
    bucketServo.attach(bucketServoPin);
    clawServo.attach(clawServoPin);

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
    vTaskDelay(1);
}

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            USBSerial.println("Controller connected at index %d\n", i);
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            USBSerial.println("Controller disconnected from index %d\n", i);
            // Stop all motors on disconnect
            moveMotors(0, 0);
            return;
        }
    }
}

void moveMotors(int leftSpeed, int rightSpeed) {
    // Simple motor control helper using PWM/Digital signals
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

void processGamepad(ControllerPtr ctl) {
    int LXValue = ctl->axisX();
    int LYValue = ctl->axisY();
    
    int targetDrive = -map(LYValue, -512, 511, -255, 255);
    int targetSteer = map(LXValue, -512, 511, -255, 255);
    
    int targetLeftSpeed  = constrain(targetDrive - targetSteer, -255, 255);
    int targetRightSpeed = constrain(targetDrive + targetSteer, -255, 255);
    
    // Deadzone check
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