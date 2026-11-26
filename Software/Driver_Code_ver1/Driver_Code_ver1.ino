#include <Bluepad32.h>

// ==========================================
// 1. IBT_2 MOTOR PINS (RPWM, LPWM)
// ==========================================
#define FL_RPWM 13 //motor 4
#define FL_LPWM 14

#define FR_RPWM 26 // motor 2
#define FR_LPWM 27

#define RL_RPWM 32 //motor 3
#define RL_LPWM 33

#define RR_RPWM 18 //motor 1
#define RR_LPWM 19

// ==========================================
// 2. MOTOR DIRECTION CALIBRATION (1 or -1)
// ==========================================
// Hardware polarity is already mirrored between left and right sides,
// so all four drivers share the same sign.
const int FL_DIR = 1;
const int FR_DIR = 1;
const int RL_DIR = 1;
const int RR_DIR = 1;

// ==========================================
// 3. TUNING PARAMETERS
// ==========================================
const int   DEADZONE     = 40;    // Ignore stick drift (-511 to 512)
const int   MAX_PWM      = 120;   // Keep at 120 for first test; raise to 220-255 once calibrated
const float STRAFE_SCALE = 1.1f;  // Boosts sideways speed to offset mecanum roller slip
const float TURN_SCALE   = 0.85f; // Slightly tames spin sensitivity for robosoccer control

ControllerPtr myController = nullptr;

// Drives a single IBT_2 driver (-1.0 = full reverse, +1.0 = full forward)
void setMotor(int rpwmPin, int lpwmPin, float speed, int dirMultiplier) {
  speed = constrain(speed * dirMultiplier, -1.0f, 1.0f);
  int pwmVal = (int)(fabs(speed) * MAX_PWM);

  if (speed > 0.0f) {
    analogWrite(rpwmPin, pwmVal);
    analogWrite(lpwmPin, 0);
  } else if (speed < 0.0f) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, pwmVal);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

void driveMecanum(float y, float x, float rx) {
  // Core Mecanum X-Configuration Inverse Kinematics
  // Strafe Left (x = -1)  -> FL(-), FR(+), RL(+), RR(-)
  // Strafe Right (x = +1) -> FL(+), FR(-), RL(-), RR(+)
  // Rotate CW (rx = +1)   -> FL(+), FR(-), RL(+), RR(-)
  float fl = y + x + rx;
  float fr = y - x - rx;
  float rl = y - x + rx;
  float rr = y + x - rx;

  // Normalize all 4 wheels proportionally so combined moves stay <= 1.0
  float maxMag = max(max(fabs(fl), fabs(fr)), max(fabs(rl), fabs(rr)));
  if (maxMag > 1.0f) {
    fl /= maxMag;
    fr /= maxMag;
    rl /= maxMag;
    rr /= maxMag;
  }

  setMotor(FL_RPWM, FL_LPWM, fl, FL_DIR);
  setMotor(FR_RPWM, FR_LPWM, fr, FR_DIR);
  setMotor(RL_RPWM, RL_LPWM, rl, RL_DIR);
  setMotor(RR_RPWM, RR_LPWM, rr, RR_DIR);
}

void stopAllMotors() {
  driveMecanum(0.0f, 0.0f, 0.0f);
}

void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.println("PS4 Controller Connected!");
    myController = ctl;
    ctl->setColorLED(0, 255, 0);
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.println("Controller Disconnected - Failsafe Stop!");
    myController = nullptr;
    stopAllMotors();
  }
}

void processGamepad(ControllerPtr ctl) {
  uint8_t dpad = ctl->dpad();

  // D-Pad Pure-Direction Mode (great for verifying wheel directions)
  if (dpad != 0) {
    float testY = 0.0f;
    float testX = 0.0f;
    if (dpad & DPAD_UP)    testY =  0.7f; // All 4 forward
    if (dpad & DPAD_DOWN)  testY = -0.7f; // All 4 backward
    if (dpad & DPAD_LEFT)  testX = -0.7f; // FL-, FR+, RL+, RR-
    if (dpad & DPAD_RIGHT) testX =  0.7f; // FL+, FR-, RL-, RR+
    driveMecanum(testY, testX, 0.0f);
    return;
  }

  // Analog Joystick Holonomic Control
  int rawY  = -ctl->axisY();   // Left Stick Up = Forward (+)
  int rawX  =  ctl->axisX();   // Left Stick Right = Strafe Right (+)
  int rawRX =  ctl->axisRX();  // Right Stick Right = Rotate CW (+)

  float y  = (abs(rawY)  > DEADZONE) ? (rawY  / 512.0f) : 0.0f;
  float x  = (abs(rawX)  > DEADZONE) ? (rawX  / 512.0f) * STRAFE_SCALE : 0.0f;
  float rx = (abs(rawRX) > DEADZONE) ? (rawRX / 512.0f) * TURN_SCALE   : 0.0f;

  driveMecanum(y, x, rx);
}

void setup() {
  Serial.begin(115200);

  int pins[] = {FL_RPWM, FL_LPWM, FR_RPWM, FR_LPWM, RL_RPWM, RL_LPWM, RR_RPWM, RR_LPWM};
  for (int i = 0; i < 8; i++) {
    pinMode(pins[i], OUTPUT);
    analogWrite(pins[i], 0);
  }

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  Serial.println("Mecanum Bluepad32 Ready. Pair PS4 controller (Share + PS).");
}

void loop() {
  bool dataUpdated = BP32.update();

  if (dataUpdated && myController && myController->isConnected() && myController->hasData()) {
    if (myController->isGamepad()) {
      processGamepad(myController);
    }
  } else if (myController == nullptr || !myController->isConnected()) {
    stopAllMotors();
  }

  vTaskDelay(1);
}