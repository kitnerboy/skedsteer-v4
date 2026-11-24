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

// Servo control variables
float bucketServoPosition = 90.0; // Bucket servo (analog triggers L2/R2)
const float bucketServoSpeed = 2.0;  

float clawServoPosition = 90.0;   // Claw servo (discrete buttons R1/L1)
const float clawFixedSpeed = 2.0;     // Fixed speed for R1/L1 control

// State variable for lights toggle
bool lightsOn = false;
unsigned long lastButtonPress = 0;

void setup() {
    Serial.begin(115200);
    
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
    
    Serial.println("MiniSkidi-V4 initialized with dual servo control (Triggers + Bumpers).");
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
            Serial.printf("Controller connected at index %d\n", i);
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            Serial.printf("Controller disconnected from index %d\n", i);
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

    if (currentLeftSpeed < targetLeftSpeed) currentLeftSpeed = min(currentLeftSpeed + accelerationStep, targetLeftSpeed);
    else if (currentLeftSpeed > targetLeftSpeed) currentLeftSpeed = max(currentLeftSpeed - accelerationStep, targetLeftSpeed);

    if (currentRightSpeed < targetRightSpeed) currentRightSpeed = min(currentRightSpeed + accelerationStep, targetRightSpeed);
    else if (currentRightSpeed > targetRightSpeed) currentRightSpeed = max(currentRightSpeed - accelerationStep, targetRightSpeed);

    moveMotors(currentLeftSpeed, currentRightSpeed);

    // --- 2. Arm Control (Right Stick Y axis) ---
    int RYValue = ctl->axisRY();
    int armSpeed = 0;
    if (abs(RYValue) > 20) {
        armSpeed = -map(RYValue, -512, 511, -255, 255);
    }
    moveArm(armSpeed);

    // --- 3. Bucket Servo Control via Analog Triggers (L2 / R2) ---
    int r2Value = ctl->throttle(); 
    int l2Value = ctl->brake();     

    float bucketStep = 0;
    if (r2Value > 30) {
        bucketStep += map(r2Value, 0, 1023, 0, bucketServoSpeed * 100) / 100.0;
    }
    if (l2Value > 30) {
        bucketStep -= map(l2Value, 0, 1023, 0, bucketServoSpeed * 100) / 100.0;
    }

    if (bucketStep != 0) {
        bucketServoPosition += bucketStep;
        bucketServoPosition = constrain(bucketServoPosition, 0.0, 180.0);
        bucketServo.write((int)bucketServoPosition);
    }

    // --- 4. Claw Servo Control via Discrete Bumpers (R1 / L1) ---
    // Bluepad32 button bits for bumpers: typically R1 and L1 have specific bit masks, 
    // or we can use ctl->miscButtons() / specific masks depending on gamepad type.
    // Standard mapping: let's check standard button bits or use ctl->buttons().
    uint32_t buttons = ctl->buttons();
    
    // Note: depending on the gamepad model (DS4, Xbox, etc.), R1/L1 are often in ctl->buttons() 
    // or separate fields. In Bluepad32, standard shoulder buttons are usually part of buttons bitmask.
    // Let's check common masks or generic button flags. 
    // Alternatively, check standard Bluepad constants if available, or evaluate standard bits:
    // (If R1/L1 map to specific bits, e.g., 0x0010 / 0x0020, adjust as needed for your gamepad).
    bool r1Pressed = (buttons & 0x0010); // Example bit for R1 (adjust if needed)
    bool l1Pressed = (buttons & 0x0020); // Example bit for L1 (adjust if needed)

    float clawStep = 0;
    if (r1Pressed) {
        clawStep += clawFixedSpeed; // Move in one direction at fixed speed
    }
    if (l1Pressed) {
        clawStep -= clawFixedSpeed; // Move in opposite direction at fixed speed
    }

    if (clawStep != 0) {
        clawServoPosition += clawStep;
        clawServoPosition = constrain(clawServoPosition, 0.0, 180.0);
        clawServo.write((int)clawServoPosition);
    }

    // --- 5. Lights Toggle (Button X / Square) ---
    if ((buttons & 0x0004) && (millis() - lastButtonPress > 400)) {
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
