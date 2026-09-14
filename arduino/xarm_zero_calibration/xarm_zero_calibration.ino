/*
 * =====================================================================================
 * Project: Dual xArm 1s Color-Sorting Robot (Semester 5 Mini-Project)
 * File:    arduino/xarm_zero_calibration/xarm_zero_calibration.ino
 * Target:  Arduino Uno R4 WiFi (Renesas RA4M1 32-bit ARM Cortex-M4)
 * Purpose: Interactive Motor Deviation Zeroing & Calibration Tool for xArm 1s
 * =====================================================================================
 * 
 * Hardware Connections:
 *   - Arduino Serial (USB): Connect to PC (115200 baud) for Serial Monitor CLI.
 *   - Arduino Serial1 (D0=RX, D1=TX): Connect to xArm 1s controller UART (115200 baud).
 *   - Power: Shared GND between Arduino and xArm controller.
 *   - Power Warning: DO NOT power xArm servos from Arduino 5V! Use dedicated 7.4V / 5A supply.
 * 
 * Kinematic Reference:
 *   - Servo 1: Base Azimuth / Yaw (Motor 1)
 *   - Servo 2: Shoulder Pitch (Motor 2)
 *   - Servo 3: Elbow Pitch (Motor 3)
 *   - Servo 4: Wrist Pitch (Motor 4)
 *   - Servo 5: Wrist Roll (Motor 5)
 *   - Servo 6: Gripper Claw (Motor 6)
 * 
 * Theoretical Center:
 *   - 500 ticks = 120.0 degrees (Midpoint of 0-1000 tick / 240-degree range).
 * =====================================================================================
 */

#include <Arduino.h>

// Baud rates
const unsigned long PC_BAUD   = 115200;
const unsigned long XARM_BAUD = 115200; // Default Hiwonder / LewanSoul controller baud

// Emergency Stop Button on Pin D3 (Hardware Interrupt INT1)
const int PIN_BTN_ESTOP = 3;
volatile bool isEStopped = false;

// Default zero tick centers
int servoPositions[7] = { 0, 500, 500, 500, 500, 500, 500 }; // 1-indexed (indices 1 to 6)
int servoDeviations[7] = { 0, 0, 0, 0, 0, 0, 0 };             // Deviation trims from 500

int selectedMotor = 1; // Currently selected motor for trimming (1 to 6)

// Hiwonder / LewanSoul Packet Command Constants
const byte CMD_SERVO_MOVE          = 0x03;
const byte CMD_GET_SERVO_POS       = 0x15;
const byte CMD_SERVO_DEVIATION     = 0x1B; // Write deviation to EEPROM

/**
 * Hardware Interrupt Service Routine for Emergency Stop
 */
void handleEStopISR() {
  isEStopped = true;
}

/**
 * Emergency Stop Handler: Immediately halts motion and cuts torque
 */
void triggerEmergencyStop() {
  isEStopped = true;
  Serial1.print("#000PDST!"); // ASCII stop
  
  // Send torque unload (CMD 0x0E, Param 0) to all servos
  for (byte id = 1; id <= 6; id++) {
    byte len = 4;
    byte cmd = 0x0E;
    byte param = 0x00; // Torque off
    byte checksum = ~(id + len + cmd + param);
    Serial1.write(0x55);
    Serial1.write(0x55);
    Serial1.write(id);
    Serial1.write(len);
    Serial1.write(cmd);
    Serial1.write(param);
    Serial1.write(checksum);
  }
  Serial1.print("#000PUL!");

  Serial.println(F("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
  Serial.println(F("           *** EMERGENCY STOP TRIGGERED! ***            "));
  Serial.println(F("  All motor torque CUT. Servos are now limp.            "));
  Serial.println(F("  Release button on D3 and type 'r' to re-enable torque."));
  Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
}

/**
 * Re-enable torque after E-stop is cleared
 */
void clearEmergencyStop() {
  if (digitalRead(PIN_BTN_ESTOP) == LOW) {
    Serial.println(F("[WARN] Cannot resume: E-Stop button on D3 is still pressed!"));
    return;
  }
  for (byte id = 1; id <= 6; id++) {
    byte len = 4;
    byte cmd = 0x0E;
    byte param = 0x01; // Torque on
    byte checksum = ~(id + len + cmd + param);
    Serial1.write(0x55);
    Serial1.write(0x55);
    Serial1.write(id);
    Serial1.write(len);
    Serial1.write(cmd);
    Serial1.write(param);
    Serial1.write(checksum);
  }
  Serial1.print("#000PLD!");
  isEStopped = false;
  Serial.println(F("[SAFETY] Torque restored. You may continue trimming deviations."));
}

/**
 * Send LewanSoul / Hiwonder binary packet to drive a single servo
 * Packet format: 0x55 0x55 [ID] [Length] [Cmd] [Params...] [Checksum]
 */
void sendServoMove(byte id, uint16_t position, uint16_t timeMs) {
  // Clamp safety limits [0, 1000]
  if (position > 1000) position = 1000;
  
  // Also send ASCII command format supported by xArm secondary controller boards:
  // Format: #<id>P<pos>T<time>!
  char asciiCmd[32];
  snprintf(asciiCmd, sizeof(asciiCmd), "#%03dP%04dT%04d!", id, position, timeMs);
  Serial1.print(asciiCmd);
  
  // Also send native binary packet in case controller is in transparent bus mode
  byte len = 7;
  byte posL = position & 0xFF;
  byte posH = (position >> 8) & 0xFF;
  byte timeL = timeMs & 0xFF;
  byte timeH = (timeMs >> 8) & 0xFF;
  
  byte checksum = ~(id + len + CMD_SERVO_MOVE + posL + posH + timeL + timeH);
  
  Serial1.write(0x55);
  Serial1.write(0x55);
  Serial1.write(id);
  Serial1.write(len);
  Serial1.write(CMD_SERVO_MOVE);
  Serial1.write(posL);
  Serial1.write(posH);
  Serial1.write(timeL);
  Serial1.write(timeH);
  Serial1.write(checksum);
}

/**
 * Move all 6 servos to their calibrated positions
 */
void applyAllPositions(uint16_t timeMs) {
  for (int i = 1; i <= 6; i++) {
    int target = servoPositions[i] + servoDeviations[i];
    if (target < 0) target = 0;
    if (target > 1000) target = 1000;
    sendServoMove(i, target, timeMs);
    delay(20);
  }
}

void printHeader() {
  Serial.println(F("\n========================================================"));
  Serial.println(F("     xArm 1s SERVO ZEROING & DEVIATION CALIBRATION      "));
  Serial.println(F("========================================================"));
  Serial.println(F("Commands:"));
  Serial.println(F("  1-6      : Select active Motor ID"));
  Serial.println(F("  + / -    : Adjust deviation by +1 / -1 tick"));
  Serial.println(F("  > / <    : Adjust deviation by +10 / -10 ticks"));
  Serial.println(F("  z        : Reset selected motor deviation to 0"));
  Serial.println(F("  Z        : Reset ALL motor deviations to 0"));
  Serial.println(F("  h        : Move all motors to theoretical CENTER (500 ticks)"));
  Serial.println(F("  p        : Print current deviation array for ROS 2 / C++"));
  Serial.println(F("  ! / e    : EMERGENCY STOP (Cut motor torque immediately)"));
  Serial.println(F("  r        : Resume / restore motor torque"));
  Serial.println(F("  ?        : Show this menu"));
  Serial.println(F("Safety:"));
  Serial.println(F("  - Pin D3 is wired to Emergency Stop (Hardware Interrupt)"));
  Serial.println(F("========================================================"));
}

void printStatus() {
  Serial.println(F("\n--- Current Servo Status ---"));
  for (int i = 1; i <= 6; i++) {
    int effective = servoPositions[i] + servoDeviations[i];
    float angleDeg = effective * (240.0 / 1000.0);
    
    if (i == selectedMotor) {
      Serial.print(F(" -> [M"));
    } else {
      Serial.print(F("    [M"));
    }
    Serial.print(i);
    Serial.print(F("] Base: "));
    Serial.print(servoPositions[i]);
    Serial.print(F(" | Dev: "));
    if (servoDeviations[i] >= 0) Serial.print(F("+"));
    Serial.print(servoDeviations[i]);
    Serial.print(F(" | Commanded: "));
    Serial.print(effective);
    Serial.print(F(" ("));
    Serial.print(angleDeg, 1);
    Serial.println(F(" deg)"));
  }
  Serial.print(F("Active Selection: Motor "));
  Serial.println(selectedMotor);
}

void printCalibrationOutput() {
  Serial.println(F("\n/* ---------------------------------------------------- */"));
  Serial.println(F("/* Copy and paste into ROS 2 / C++ / YAML config file:  */"));
  Serial.println(F("/* ---------------------------------------------------- */"));
  Serial.println(F("servo_deviations:"));
  for (int i = 1; i <= 6; i++) {
    Serial.print(F("  motor_"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(servoDeviations[i]);
  }
  Serial.println(F("\n// C++ / Arduino Array:"));
  Serial.print(F("const int SERVO_DEV[7] = {0, "));
  for (int i = 1; i <= 6; i++) {
    Serial.print(servoDeviations[i]);
    if (i < 6) Serial.print(F(", "));
  }
  Serial.println(F("};"));
  Serial.println(F("/* ---------------------------------------------------- */"));
}

void setup() {
  // Initialize PC USB Serial
  Serial.begin(PC_BAUD);
  while (!Serial && millis() < 3000); // Wait up to 3s for Serial Monitor on Uno R4

  // Initialize Hardware Serial1 (Pins 0/1) to xArm 1s
  Serial1.begin(XARM_BAUD);

  // Configure Emergency Stop Button on Pin D3
  pinMode(PIN_BTN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_ESTOP), handleEStopISR, FALLING);

  printHeader();
  
  // Command arm to safe center on startup (1500 ms gentle transit)
  Serial.println(F("\n[INIT] Moving all joints to nominal center (500 ticks) in 1500 ms..."));
  applyAllPositions(1500);
  delay(1600);
  printStatus();
}

void loop() {
  // Emergency stop gating
  if (isEStopped) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'r' || c == 'R') {
        clearEmergencyStop();
        printStatus();
      }
    }
    return;
  }

  if (Serial.available() > 0) {
    char c = Serial.read();

    // Emergency stop via serial
    if (c == '!' || c == 'e' || c == 'E') {
      triggerEmergencyStop();
      return;
    }
    // Select Motor 1-6
    else if (c >= '1' && c <= '6') {
      selectedMotor = c - '0';
      Serial.print(F("Selected Motor: "));
      Serial.println(selectedMotor);
    }
    // Fine Trim: +1 tick
    else if (c == '+') {
      servoDeviations[selectedMotor] += 1;
      int target = servoPositions[selectedMotor] + servoDeviations[selectedMotor];
      sendServoMove(selectedMotor, target, 50);
      printStatus();
    }
    // Fine Trim: -1 tick
    else if (c == '-') {
      servoDeviations[selectedMotor] -= 1;
      int target = servoPositions[selectedMotor] + servoDeviations[selectedMotor];
      sendServoMove(selectedMotor, target, 50);
      printStatus();
    }
    // Coarse Trim: +10 ticks
    else if (c == '>') {
      servoDeviations[selectedMotor] += 10;
      int target = servoPositions[selectedMotor] + servoDeviations[selectedMotor];
      sendServoMove(selectedMotor, target, 100);
      printStatus();
    }
    // Coarse Trim: -10 ticks
    else if (c == '<') {
      servoDeviations[selectedMotor] -= 10;
      int target = servoPositions[selectedMotor] + servoDeviations[selectedMotor];
      sendServoMove(selectedMotor, target, 100);
      printStatus();
    }
    // Reset selected motor
    else if (c == 'z') {
      servoDeviations[selectedMotor] = 0;
      sendServoMove(selectedMotor, 500, 300);
      Serial.print(F("Reset Motor "));
      Serial.print(selectedMotor);
      Serial.println(F(" deviation to 0."));
      printStatus();
    }
    // Reset all deviations
    else if (c == 'Z') {
      for (int i = 1; i <= 6; i++) servoDeviations[i] = 0;
      applyAllPositions(1000);
      Serial.println(F("Reset ALL motor deviations to 0."));
      printStatus();
    }
    // Move to 500 ticks center
    else if (c == 'h' || c == 'H') {
      Serial.println(F("Commanding all motors to 500 ticks (with current deviations)..."));
      applyAllPositions(1000);
    }
    // Print formatted array
    else if (c == 'p' || c == 'P') {
      printCalibrationOutput();
    }
    // Help menu
    else if (c == '?') {
      printHeader();
    }
  }
}
