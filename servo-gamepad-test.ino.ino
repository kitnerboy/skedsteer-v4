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
const int auxLightsPin   = 0; // Фара (управление одним пином, второй IN4 посажен на GND)

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

// Servo speed control variables
float bucketServoPosition = 90.0; // Start at middle position (90 degrees)
const float maxServoSpeed = 2.0;  // Maximum angle change per loop iteration

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

    // Attach servos and set initial position
    bucketServo.attach(bucketServoPin);
    clawServo.attach(clawServoPin);
    bucketServo.write((int)bucketServoPosition);

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
    vTaskDelay(10); // Small delay to control servo update rate and smoothness
}

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            USBSerial.println("Controller connected at index \n");
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            USBSerial.println("Controller disconnected from index\n");
            // Stop all motors on disconnect
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

    // --- 2. Arm Control (Right Stick Y axis) ---
    int RYValue = ctl->axisRY();
    int armSpeed = 0;
    if (abs(RYValue) > 20) {
        armSpeed = -map(RYValue, -512, 511, -255, 255);
    }
    moveArm(armSpeed);

    // --- 3. Servo Speed Control via Triggers (L2 / R2) ---
    // ctl->throttle() is R2, ctl->brake() is L2 (values range from 0 to 1023)
    int r2Value = ctl->throttle(); 
    int l2Value = ctl->brake();     

    float servoStep = 0;
    if (r2Value > 30) {
        // Right trigger moves servo in one direction, speed proportional to trigger press
        servoStep += map(r2Value, 0, 1023, 0, maxServoSpeed * 100) / 100.0;
    }
    if (l2Value > 30) {
        // Left trigger moves servo in the opposite direction
        servoStep -= map(l2Value, 0, 1023, 0, maxServoSpeed * 100) / 100.0;
    }

    // Update servo position and constrain within safe limits (0 - 180 degrees)
    if (servoStep != 0) {
        bucketServoPosition += servoStep;
        bucketServoPosition = constrain(bucketServoPosition, 0.0, 180.0);
        bucketServo.write((int)bucketServoPosition);
    }

    // --- 4. Lights Toggle (Button X / Square) ---
    uint32_t b = ctl->buttons();
    if ((b & 0x0004) && (millis() - lastButtonPress > 400)) {
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
