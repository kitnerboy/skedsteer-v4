#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myController;

// Servo pin definitions (pins 0, 1, and 2 as requested)
#define SERVO_PIN_1 0  // Left Stick X-axis -> Pin 0
#define SERVO_PIN_2 1  // Left Stick Y-axis -> Pin 1
#define SERVO_PIN_3 2  // Right Stick Y-axis -> Pin 2

Servo servo1;
Servo servo2;
Servo servo3;

// Helper function to map Bluepad32 axis value (-512 to 511) to servo angle (0 to 180 degrees)
int mapAxisToAngle(int32_t axisValue) {
  if (axisValue < -512) axisValue = -512;
  if (axisValue > 511) axisValue = 511;
  return map(axisValue, -512, 511, 0, 180);
}

void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.println("CALLBACK: Controller connected");
    myController = ctl;
    BP32.enableNewBluetoothConnections(false);
    // Play a short rumble to indicate successful connection
    ctl->playDualRumble(0, 250, 0x80, 0x40);
  } else {
    Serial.println("CALLBACK: Another controller tried to connect, rejecting.");
    ctl->disconnect();
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.println("CALLBACK: Controller disconnected");
    myController = nullptr;
    BP32.enableNewBluetoothConnections(true);
  }
}

#define CHECK_CONTROL_VALUE(type, var, red_func, format) \
  do { \
    static type old_##var = 0; \
    type var = red_func(); \
    if (var != old_##var) { \
      Serial.printf("%s changed to " format "\n", #var, var); \
      old_##var = var; \
    } \
  } while (0)

void log_controls(ControllerPtr ctl){
  CHECK_CONTROL_VALUE(int, leftX, ctl->axisX, "0x%03d");
  CHECK_CONTROL_VALUE(int, leftY, ctl->axisY, "0x%03d");
  CHECK_CONTROL_VALUE(int, rightY, ctl->axisRY, "0x%03d");
  CHECK_CONTROL_VALUE(uint16_t, buttons, ctl->buttons, "0x%04X");
  CHECK_CONTROL_VALUE(uint16_t, miscButtons, ctl->miscButtons, "0x%04X");
  CHECK_CONTROL_VALUE(uint8_t, dpad, ctl->dpad, "0x%02X");
  CHECK_CONTROL_VALUE(uint32_t, throttle, ctl->throttle, "0x%03d");
  CHECK_CONTROL_VALUE(uint32_t, brake, ctl->brake, "0x%03d");  
}

void processGamepad(ControllerPtr ctl) {
  int angle1 = mapAxisToAngle(ctl->axisX());
  servo1.write(angle1);

  // 2. Left Stick Y-axis controls Servo 2 (Pin 1)
  int angle2 = mapAxisToAngle(ctl->axisY());
  servo2.write(angle2);

  // 3. Right Stick Y-axis controls Servo 3 (Pin 2)
  int angle3 = mapAxisToAngle(ctl->axisRY());
  servo3.write(angle3);

  log_controls(ctl);
}

void setup() {
  Serial.begin(115200);

  delay(2000);

  // Attach servos to pins 0, 1, and 2
  servo1.attach(SERVO_PIN_1);
  servo2.attach(SERVO_PIN_2);
  servo3.attach(SERVO_PIN_3);

  // Move servos to center position (90 degrees) on startup
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());

  // Setup Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {
  // Fetch controller data updates
  bool dataUpdated = BP32.update();

  if (dataUpdated && myController && myController->isConnected() && myController->isGamepad()) {
    processGamepad(myController);
  }

  // Yield to FreeRTOS task watchdog
  vTaskDelay(1);
}