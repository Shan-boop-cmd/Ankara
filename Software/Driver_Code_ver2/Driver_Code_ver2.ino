#include <Bluepad32.h>

// ==========================================
// 1. PIN DEFINITIONS (MATCHED TO YOUR WIRING)
// ==========================================
#define LED_PIN   2   // Onboard ESP32 blue LED (ON = controller connected)

// Front Left (FL) -> Motor 4 (Driver 4)
#define FL_RPWM   23
#define FL_LPWM   25
#define FL_EN     33

// Front Right (FR) -> Motor 2 (Driver 2)
#define FR_RPWM   18
#define FR_LPWM   19
#define FR_EN     27

// Rear Left (RL) -> Motor 3 (Driver 3)
#define RL_RPWM   21
#define RL_LPWM   22
#define RL_EN     32

// Rear Right (RR) -> Motor 1 (Driver 1)
#define RR_RPWM   16
#define RR_LPWM   17
#define RR_EN     26

// ==========================================
// 2. MOTOR DIRECTION CALIBRATION (1 or -1)
// ==========================================
// Left/right motors are already mirrored in hardware wiring.
// Hold D-Pad UP to verify: if ALL 4 spin backward, flip all four to -1.
// If only one wheel spins backward, flip just that wheel to -1.
const int FL_DIR = 1;
const int FR_DIR = 1;
const int RL_DIR = 1;
const int RR_DIR = 1;

// ==========================================
// 3. TUNING PARAMETERS
// ==========================================
const int   DEADZONE        = 40;    // Ignore resting stick drift (-511..512)
const int   MAX_PWM         = 120;   // To set max speed, out of 255. 
const float DPAD_SPEED      = 0.75f; // Fixed speed for D-Pad test & emergency backup
const float EXPO            = 0.4f;  // 0.0 = linear, 0.4 = smoother low-speed stick response
const float STRAFE_SCALE    = 1.1f;  // Offsets mecanum sideways roller slip
const float TURN_SCALE      = 0.85f; // Tames rotation sensitivity
const float PRECISION_SCALE = 0.5f;  // Hold L1 for 50% speed

const int PWM_PINS[8] = {
  FL_RPWM, FL_LPWM, FR_RPWM, FR_LPWM,
  RL_RPWM, RL_LPWM, RR_RPWM, RR_LPWM
};

const int EN_PINS[4] = { FL_EN, FR_EN, RL_EN, RR_EN };

ControllerPtr myController = nullptr;

// Enables or disables all four IBT_2 drivers together
void setDriversEnabled(bool enable) {
  int state = enable ? HIGH : LOW;
  for (int i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], state);
  }
}

// Drives a single IBT_2 (-1.0 to +1.0) with zero-overlap direction switching
void setMotor(int rpwmPin, int lpwmPin, float speed, int dirMultiplier) {
  speed = constrain(speed * dirMultiplier, -1.0f, 1.0f);
  int pwmVal = (int)(fabs(speed) * MAX_PWM);

  if (pwmVal == 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  } else if (speed > 0.0f) {
    analogWrite(lpwmPin, 0);      // Clear opposite channel FIRST
    analogWrite(rpwmPin, pwmVal);
  } else {
    analogWrite(rpwmPin, 0);      // Clear opposite channel FIRST
    analogWrite(lpwmPin, pwmVal);
  }
}

// Mecanum X-configuration inverse kinematics
// Forward (+y):      FL+, FR+, RL+, RR+
// Strafe Left (-x):  FL-, FR+, RL+, RR-
// Strafe Right (+x): FL+, FR-, RL-, RR+
// Rotate CW (+rx):   FL+, FR-, RL+, RR-
void driveMecanum(float y, float x, float rx) {
  float fl = y + x + rx;
  float fr = y - x - rx;
  float rl = y - x + rx;
  float rr = y + x - rx;

  // Proportional normalization so combined moves stay within -1.0..1.0
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

// Hard failsafe: zeroes all PWMs AND disables all four H-bridges
void disableAllMotors() {
  driveMecanum(0.0f, 0.0f, 0.0f);
  setDriversEnabled(false);
}

// Smooth deadzone rescaling + expo curve (-511..512 -> -1.0..1.0)
float tuneStick(int raw) {
  if (abs(raw) <= DEADZONE) return 0.0f;

  float sign = (raw > 0) ? 1.0f : -1.0f;
  float mag  = (abs(raw) - DEADZONE) / (512.0f - DEADZONE);
  mag = constrain(mag, 0.0f, 1.0f);

  float curved = (1.0f - EXPO) * mag + EXPO * (mag * mag);
  return sign * curved;
}

void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    myController = ctl;
    driveMecanum(0.0f, 0.0f, 0.0f); // Guarantee 0 PWM before enabling
    setDriversEnabled(true);        // Pull all 4 R_EN/L_EN pins HIGH
    digitalWrite(LED_PIN, HIGH);
    ctl->setColorLED(0, 255, 0);
    Serial.println("PS4 Connected - All 4 Drivers ENABLED.");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    myController = nullptr;
    disableAllMotors();             // Zero PWM + pull all 4 EN pins LOW
    digitalWrite(LED_PIN, LOW);
    Serial.println("PS4 Disconnected - All 4 Drivers DISABLED.");
  }
}

void processGamepad(ControllerPtr ctl) {
  float scale = ctl->l1() ? PRECISION_SCALE : 1.0f;

  // ------------------------------------------
  // 1. D-PAD CHECK FIRST (Bench Test & Emergency Backup)
  // Overrides both sticks whenever any D-Pad button is held.
  // L2 / R2 provide emergency CCW / CW rotation while using D-Pad.
  // ------------------------------------------
  uint8_t dpad = ctl->dpad();
  if (dpad != 0) {
    float dy  = 0.0f;
    float dx  = 0.0f;
    float drx = 0.0f;

    if (dpad & DPAD_UP)    dy  += DPAD_SPEED;
    if (dpad & DPAD_DOWN)  dy  -= DPAD_SPEED;
    if (dpad & DPAD_RIGHT) dx  += DPAD_SPEED;
    if (dpad & DPAD_LEFT)  dx  -= DPAD_SPEED;
    if (ctl->r2())         drx += DPAD_SPEED * TURN_SCALE;
    if (ctl->l2())         drx -= DPAD_SPEED * TURN_SCALE;

    driveMecanum(dy * scale, dx * scale, drx * scale);
    return;
  }

  // ------------------------------------------
  // 2. ANALOG JOYSTICK CONTROL (Normal Driving)
  // ------------------------------------------
  float y  = tuneStick(-ctl->axisY());                 // Left Stick Up (+) / Down (-)
  float x  = tuneStick( ctl->axisX())  * STRAFE_SCALE; // Left Stick Right (+) / Left (-)
  float rx = tuneStick( ctl->axisRX()) * TURN_SCALE;   // Right Stick Right/CW (+) / Left/CCW (-)

  // Emergency standalone rotation with L2/R2 if right stick is dead/centered
  if (rx == 0.0f) {
    if (ctl->r2()) rx =  DPAD_SPEED * TURN_SCALE;
    if (ctl->l2()) rx = -DPAD_SPEED * TURN_SCALE;
  }

  driveMecanum(y * scale, x * scale, rx * scale);
}

void setup() {
  Serial.begin(115200);

  // Preset all 4 Enable pins LOW before setting OUTPUT (prevents boot twitch)
  for (int i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], LOW);
    pinMode(EN_PINS[i], OUTPUT);
  }

  // Preset all 8 PWM pins LOW before attaching PWM
  for (int i = 0; i < 8; i++) {
    digitalWrite(PWM_PINS[i], LOW);
    pinMode(PWM_PINS[i], OUTPUT);
    analogWrite(PWM_PINS[i], 0);
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  Serial.println("Mecanum Bot Ready. Drivers DISABLED until PS4 connects.");
}

void loop() {
  bool dataUpdated = BP32.update();

  if (dataUpdated && myController && myController->isConnected() && myController->hasData()) {
    if (myController->isGamepad()) {
      processGamepad(myController);
    }
  } else if (myController == nullptr || !myController->isConnected()) {
    disableAllMotors();
  }

  vTaskDelay(1);
}