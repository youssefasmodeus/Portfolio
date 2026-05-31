#include <Arduino.h>
#include <AccelStepper.h>
#include "BluetoothSerial.h"

// ==================== CONFIGURATION ====================
const float MAX_SPEED        = 1000.0f;
const float ACCELERATION     = 500.0f;
const float HOMING_SPEED     = 400.0f;
const long  HOMING_MAX_TRAVEL= 100000;
const long  HOMING_BACKOFF   = 200;

// ==================== PINS ====================
const int RELAY_PIN = 15;

// Motors: Step, Dir
const int X_STEP = 4,  X_DIR = 16;
const int R_STEP = 13, R_DIR = 23;
const int Z_STEP = 18, Z_DIR = 19;
const int Y_STEP = 21, Y_DIR = 22;

// Endstops (Active LOW)
const int X_MIN = 25, X_MAX = 26;
const int Y_MIN = 32, Y_MAX = -1; // Disabled per Script 2 (GPIO32 issue)
const int Z_MIN = 33, Z_MAX = 17;
const int R_MIN = 14, R_MAX = -1;

// ==================== GLOBALS ====================
BluetoothSerial SerialBT;

AccelStepper motor1(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper motor2(AccelStepper::DRIVER, R_STEP, R_DIR);
AccelStepper motor3(AccelStepper::DRIVER, Z_STEP, Z_DIR);
AccelStepper motor4(AccelStepper::DRIVER, Y_STEP, Y_DIR);

enum AxisState { IDLE, MOVING, LIMIT_HIT_MIN, LIMIT_HIT_MAX };

struct Axis {
  AccelStepper* motor;
  int minPin;
  int maxPin;
  AxisState state;
  const char* name;
  bool reverseDir;
  int homingDir;   // 1 for positive, -1 for negative
};

// Unified axis definitions
Axis axisX = { &motor1, X_MIN, X_MAX, IDLE, "X", false, -1 };
Axis axisR = { &motor2, R_MIN, R_MAX, IDLE, "R", false,  1 };
Axis axisZ = { &motor3, Z_MIN, Z_MAX, IDLE, "Z", false, -1 };
Axis axisY = { &motor4, Y_MIN, Y_MAX, IDLE, "Y", true,   1 };

// Iterable array for O(1) loop processing
const int NUM_AXES = 4;
Axis* axes[NUM_AXES] = { &axisX, &axisR, &axisZ, &axisY };

// Homing sequence configuration
Axis* homingSequence[] = { &axisX, &axisZ, &axisR, &axisY }; 
const int NUM_HOMING_AXES = 4;
enum HomingPhase { H_IDLE, H_APPROACH, H_BACKOFF, H_DONE };
HomingPhase hPhase = H_IDLE;
int homingIndex = 0;

uint32_t lastLimitPrintMs = 0;
const uint32_t LIMIT_PRINT_INTERVAL = 200;

// ==================== HELPERS ====================

bool isLimitPressed(int pin) {
  if (pin < 0) return false;
  return digitalRead(pin) == LOW;
}

void cancelMotion(AccelStepper* motor) {
  motor->setCurrentPosition(motor->currentPosition());
}

void pumpOn() {
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println("Pump OFF");    // Keeping your logic from Script 2
  SerialBT.println("Pump OFF");
}

void pumpOff() {
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("Pump ON");    // Keeping your logic from Script 2
  SerialBT.println("Pump ON");
}

// ==================== CORE MOTION & LIMITS ====================

bool canMove(Axis* axis, long steps) {
  if (steps == 0) return false;
  if (axis->state == IDLE || axis->state == MOVING) return true;

  int awayFromMin = axis->reverseDir ? -1 : 1;
  int awayFromMax = axis->reverseDir ? 1 : -1;

  if (axis->state == LIMIT_HIT_MIN) {
    if ((awayFromMin > 0 && steps > 0) || (awayFromMin < 0 && steps < 0)) {
      axis->state = IDLE;
      Serial.printf("%s clearing MIN limit\n", axis->name);
      return true;
    }
    Serial.printf("BLOCKED: %s at MIN limit\n", axis->name);
    return false;
  }

  if (axis->state == LIMIT_HIT_MAX) {
    if ((awayFromMax > 0 && steps > 0) || (awayFromMax < 0 && steps < 0)) {
      axis->state = IDLE;
      Serial.printf("%s clearing MAX limit\n", axis->name);
      return true;
    }
    Serial.printf("BLOCKED: %s at MAX limit\n", axis->name);
    return false;
  }
  return false;
}

void enforceAxisLimits() {
  bool anyMoving = false;
  bool limitsTriggered = false;

  for (int i = 0; i < NUM_AXES; i++) {
    Axis* ax = axes[i];
    long dtg = ax->motor->distanceToGo();
    
    if (dtg != 0) {
      anyMoving = true;
      ax->state = MOVING;

      if (ax->minPin >= 0 && isLimitPressed(ax->minPin)) {
        cancelMotion(ax->motor);
        ax->state = LIMIT_HIT_MIN;
        Serial.printf("\n*** %s MIN LIMIT HIT ***\n> ", ax->name);
      }
      else if (ax->maxPin >= 0 && isLimitPressed(ax->maxPin)) {
        cancelMotion(ax->motor);
        ax->state = LIMIT_HIT_MAX;
        Serial.printf("\n*** %s MAX LIMIT HIT ***\n> ", ax->name);
      }
    } else if (ax->state == MOVING) {
      ax->state = IDLE; // Auto-idle when complete
    }
  }

  // Real-time monitor printing
  uint32_t now = millis();
  if (anyMoving && (now - lastLimitPrintMs >= LIMIT_PRINT_INTERVAL) && hPhase == H_IDLE) {
    lastLimitPrintMs = now;
    String limitStr = "[LIMITS: ";
    for (int i = 0; i < NUM_AXES; i++) {
      if (isLimitPressed(axes[i]->minPin)) limitStr += String(axes[i]->name) + "_MIN ";
      if (isLimitPressed(axes[i]->maxPin)) limitStr += String(axes[i]->name) + "_MAX ";
    }
    if (limitStr.length() > 9) Serial.println(limitStr + "]");
  }
}

// ==================== HOMING SEQUENCE ====================

void startHoming() {
  if (hPhase != H_IDLE) {
    Serial.println("ERROR: Homing in progress");
    return;
  }
  Serial.println("\n=== HOMING SEQUENCE START ===");
  homingIndex = 0;
  hPhase = H_APPROACH;
  
  Axis* ax = homingSequence[homingIndex];
  ax->motor->setMaxSpeed(HOMING_SPEED);
  ax->motor->move(HOMING_MAX_TRAVEL * ax->homingDir);
  Serial.printf("Homing %s...\n", ax->name);
}

void processHoming() {
  if (hPhase == H_IDLE) return;

  Axis* ax = homingSequence[homingIndex];

  switch (hPhase) {
    case H_APPROACH:
      if (isLimitPressed(ax->minPin)) {
        cancelMotion(ax->motor);
        ax->motor->move(HOMING_BACKOFF * (ax->homingDir * -1)); // Back off opposite of approach
        hPhase = H_BACKOFF;
      } else if (ax->motor->distanceToGo() == 0) {
        Serial.printf("ERROR: %s homing failed - limit not found\n", ax->name);
        hPhase = H_IDLE;
      }
      break;

    case H_BACKOFF:
      if (ax->motor->distanceToGo() == 0) {
        ax->motor->setCurrentPosition(0);
        ax->motor->setMaxSpeed(MAX_SPEED);
        ax->state = IDLE;
        Serial.printf("✓ %s homed\n", ax->name);
        
        homingIndex++;
        if (homingIndex >= NUM_HOMING_AXES) {
          hPhase = H_DONE;
        } else {
          Axis* nextAx = homingSequence[homingIndex];
          nextAx->motor->setMaxSpeed(HOMING_SPEED);
          nextAx->motor->move(HOMING_MAX_TRAVEL * nextAx->homingDir);
          Serial.printf("Homing %s...\n", nextAx->name);
          hPhase = H_APPROACH;
        }
      }
      break;

    case H_DONE:
      Serial.println("=== HOMING COMPLETE ===\n> ");
      hPhase = H_IDLE;
      break;

    default: break;
  }
}

// ==================== PARSERS ====================

void parseXYZ(String msg) {
  msg.remove(0, 4);
  int c1 = msg.indexOf(',');
  int c2 = msg.indexOf(',', c1 + 1);
  if (c1 == -1 || c2 == -1) {
    Serial.println("ERROR: Invalid format");
    return;
  }

  float x = msg.substring(0, c1).toFloat();
  float y = msg.substring(c1 + 1, c2).toFloat();
  float z = msg.substring(c2 + 1).toFloat();

  if (x == 100.0f && y == 100.0f && z == 200.0f) {
    Serial.println("→ XYZ(100,100,200)");
    if (canMove(&axisX, 400)) { axisX.motor->move(400); axisX.state = MOVING; }
    if (canMove(&axisY, 300)) { axisY.motor->move(300); axisY.state = MOVING; }
    if (canMove(&axisZ, 400)) { axisZ.motor->move(400); axisZ.state = MOVING; }
    return;
  }

  if (x == 100.0f && y == -100.0f && z == 200.0f) {
    Serial.println("→ XYZ(100,-100,200)");
    if (canMove(&axisR, 675)) { axisR.motor->move(675); axisR.state = MOVING; }
    return;
  }
  Serial.println("ERROR: Unknown XYZ mapping");
}

void processBluetooth() {
  if (!SerialBT.available()) return;
  String msg = SerialBT.readStringUntil('\n');
  msg.trim();

  if (msg == "9") { startHoming(); }
  else if (msg == "7") { pumpOn(); }
  else if (msg == "8") { pumpOff(); }
  else if (msg.startsWith("XYZ,")) { parseXYZ(msg); }
  else { Serial.println("BT Unknown: " + msg); }
}

void processSerial() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim(); input.toLowerCase();
  Serial.println();

  if (input == "stop") {
    hPhase = H_IDLE;
    for (int i=0; i<NUM_AXES; i++) cancelMotion(axes[i]->motor);
    Serial.println("EMERGENCY STOP: All motion cancelled\n> ");
  } 
  else if (input == "pos") {
    Serial.println("=== POSITIONS ===");
    for(int i=0; i<NUM_AXES; i++) Serial.printf("%s: %ld steps\n", axes[i]->name, axes[i]->motor->currentPosition());
    Serial.print("> ");
  }
  else if (input == "status") {
    const char* sNames[] = {"IDLE", "MOVING", "LIMIT_MIN", "LIMIT_MAX"};
    Serial.println("=== AXIS STATES ===");
    for(int i=0; i<NUM_AXES; i++) {
      Serial.printf("%s: %s (dtg=%ld)\n", axes[i]->name, sNames[axes[i]->state], axes[i]->motor->distanceToGo());
    }
    Serial.print("> ");
  }
  else if (input == "home" || input == "hom") {
    startHoming();
  }
  else {
    int commaIdx = input.indexOf(',');
    if (commaIdx > 0) {
      if (hPhase != H_IDLE) {
        Serial.println("ERROR: Cannot move motors during homing. Use 'stop'.\n> ");
        return;
      }
      int motorNum = input.substring(0, commaIdx).toInt();
      long steps = input.substring(commaIdx + 1).toInt();
      
      Axis* targetAxis = nullptr;
      if (motorNum == 1) targetAxis = &axisX;
      else if (motorNum == 2) targetAxis = &axisR;
      else if (motorNum == 3) targetAxis = &axisZ;
      else if (motorNum == 4) targetAxis = &axisY;

      if (targetAxis && canMove(targetAxis, steps)) {
        targetAxis->motor->move(steps);
        targetAxis->state = MOVING;
        Serial.printf("%s axis: Moving %ld steps\n> ", targetAxis->name, steps);
      } else {
        Serial.println("ERROR: Invalid motor (1-4) or blocked.\n> ");
      }
    }
  }
  while (Serial.available()) Serial.read(); // Clear buffer
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_ROBOT");

  // Motor Init
  for (int i = 0; i < NUM_AXES; i++) {
    axes[i]->motor->setMaxSpeed(MAX_SPEED);
    axes[i]->motor->setAcceleration(ACCELERATION);
    if (axes[i]->minPin >= 0) pinMode(axes[i]->minPin, INPUT_PULLUP);
    if (axes[i]->maxPin >= 0) pinMode(axes[i]->maxPin, INPUT_PULLUP);
  }

  // Relay Init
  pinMode(RELAY_PIN, OUTPUT);
  pumpOff();

  Serial.println("\n========================================");
  Serial.println("  INDUSTRIAL ESP32 ROBOT READY");
  Serial.println("  Unified Limit Enforcement & BT Support");
  Serial.println("========================================\n");
  Serial.print("> ");
}

// ==================== LOOP ====================

void loop() {
  processBluetooth();
  processSerial();

  enforceAxisLimits();
  processHoming();

  for (int i = 0; i < NUM_AXES; i++) {
    axes[i]->motor->run();
  }
}
