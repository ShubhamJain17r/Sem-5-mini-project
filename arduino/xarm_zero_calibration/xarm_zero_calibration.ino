/*
 * =====================================================================================
 * Project: Dual xArm 1s Color-Sorting Robot (Semester 5 Mini-Project)
 * File:    arduino/xarm_zero_calibration/xarm_zero_calibration.ino
 * Target:  Arduino Uno R4 WiFi (Renesas RA4M1 32-bit ARM Cortex-M4)
 * Purpose: Dual-Arm Interactive Motor Deviation Zeroing & Calibration Tool
 * =====================================================================================
 * 
 * OFFICIAL HIWONDER xArm 1s MOTOR MAPPING (End-Effector to Base):
 *   - Motor 1: Claw Gripper (Mid-span ~ 500 ticks)
 *   - Motor 2: Wrist Roll (Horizontal ~ 500 ticks)
 *   - Motor 3: Wrist Pitch (Collinear with forearm ~ 500 ticks)
 *   - Motor 4: Elbow Pitch (90-degree right angle with upper arm ~ 500 ticks)
 *   - Motor 5: Shoulder Pitch (Perpendicular upright ~ 500 ticks)
 *   - Motor 6: Base Yaw (Chassis midline ~ 500 ticks)
 * 
 * HARDWARE CONNECTIONS (Breadboard-Free):
 *   - Arduino Serial (USB): Connect to PC (115200 baud) for Serial Monitor CLI.
 *   - Arduino Pin 0 (RX)  : Connect to active xArm controller TX.
 *   - Arduino Pin 1 (TX)  : Connect to active xArm controller RX.
 *   - Arduino GND         : Connect to active xArm controller GND (Common Ground).
 *   - Power Warning: DO NOT power xArm servos from Arduino 5V! Use dedicated 7.4V / 5A supply.
 * =====================================================================================
 */

#include <Arduino.h>

// Baud rates
const unsigned long PC_BAUD   = 115200;
unsigned long xArmBaud        = 9600; // Hiwonder secondary UART defaults to 9600 baud

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  #define xArmSerial Serial
#else
  #define xArmSerial Serial1
#endif

volatile bool isEStopped = false;

// Default zero tick centers
int servoPositions[7]  = { 0, 500, 500, 500, 500, 500, 500 }; // 1-indexed (indices 1 to 6)
int servoDeviations[7] = { 0, 0, 0, 0, 0, 0, 0 };             // Measured deviation trims

int selectedMotor = 6; // Default to Base Yaw (Motor 6)
int activeArm     = 1; // 1 = xarm_1 (Back / NW), 2 = xarm_2 (Front / SW)

// Hiwonder / LewanSoul Controller Command Constants
const byte CMD_SERVO_MOVE        = 0x03;
const byte CMD_MULT_SERVO_UNLOAD = 0x14; // Power off / unload motor torque

/**
 * Returns human-readable joint name for Hiwonder xArm 1s
 */
const char* getMotorName(int id) {
  switch(id) {
    case 1: return "Claw Gripper";
    case 2: return "Wrist Roll";
    case 3: return "Wrist Pitch";
    case 4: return "Elbow Pitch";
    case 5: return "Shoulder Pitch";
    case 6: return "Base Yaw";
    default: return "Unknown";
  }
}

/**
 * Toggle xArm Serial1 Baud Rate between 9600 and 115200
 */
void toggleBaudRate() {
  if (xArmBaud == 9600) {
    xArmBaud = 115200;
  } else {
    xArmBaud = 9600;
  }
  xArmSerial.begin(xArmBaud);
  Serial.print(F("\n[BAUD] Switched xArm Serial1 to: "));
  Serial.print(xArmBaud);
  Serial.println(F(" baud."));
}

/**
 * Emergency Stop Handler: Immediately halts motion and cuts torque
 */
void triggerEmergencyStop() {
  isEStopped = true;
  
  // Send multi-servo unload command (CMD 0x14) to cut torque on all 6 servos
  byte unloadFrame[11] = {
    0x55, 0x55,
    0x09,                  // Length = 9
    CMD_MULT_SERVO_UNLOAD, // 0x14
    0x06,                  // 6 servos
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06
  };
  xArmSerial.write(unloadFrame, 11);

  Serial.println(F("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
  Serial.println(F("           *** EMERGENCY STOP TRIGGERED! ***            "));
  Serial.println(F("  All motor torque CUT. Servos are now limp.            "));
  Serial.println(F("  Type 'r' in Serial Monitor to re-enable torque.       "));
  Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
}

/**
 * Re-enable torque after E-stop is cleared
 */
void clearEmergencyStop() {
  isEStopped = false;
  Serial.println(F("[SAFETY] Torque restored. Returning arm to calibrated center..."));
}

/**
 * Send binary packet to drive a single servo
 * Format: 0x55 0x55 [Length=8] [CMD 0x03] [Count=1] [TimeL] [TimeH] [ID] [PosL] [PosH]
 */
void sendServoMove(byte id, uint16_t position, uint16_t timeMs) {
  if (isEStopped) return;
  if (position > 1000) position = 1000;
  
  byte len = 8;
  byte posL = position & 0xFF;
  byte posH = (position >> 8) & 0xFF;
  byte timeL = timeMs & 0xFF;
  byte timeH = (timeMs >> 8) & 0xFF;
  
  byte frame[10] = {
    0x55, 0x55,
    len,
    CMD_SERVO_MOVE,
    0x01,
    timeL, timeH,
    id,
    posL, posH
  };
  
  xArmSerial.write(frame, 10);
}

/**
 * Move all 6 servos gently with 25ms inter-joint stagger to prevent power supply brownout
 */
void applyAllPositions(uint16_t timeMs) {
  if (isEStopped) return;
  
  // Move Base -> Shoulder -> Elbow -> Wrist Pitch -> Roll -> Claw
  for (int i = 6; i >= 1; i--) {
    if (isEStopped) return;
    int target = servoPositions[i] + servoDeviations[i];
    if (target < 0) target = 0;
    if (target > 1000) target = 1000;
    sendServoMove(i, target, timeMs);
    delay(25); // Gentle stagger avoids current surge & low-voltage buzzer alarm
  }
}

void printHeader() {
  Serial.println(F("\n========================================================"));
  Serial.println(F("     xArm 1s DUAL-ARM ZEROING & DEVIATION CALIBRATION   "));
  Serial.println(F("========================================================"));
  Serial.print(F("Active Target Profile: xArm "));
  Serial.print(activeArm);
  if (activeArm == 1) Serial.println(F(" (Back / NW Manipulator)"));
  else Serial.println(F(" (Front / SW Manipulator)"));
  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F("COMMANDS (Type in Serial Monitor @ 115200):"));
  Serial.println(F("  arm 1 / arm 2: Switch target profile (xArm 1 vs xArm 2)"));
  Serial.println(F("  1 - 6        : Select active Motor ID"));
  Serial.println(F("  + / -        : Adjust deviation by +1 / -1 tick"));
  Serial.println(F("  > / <        : Adjust deviation by +10 / -10 ticks"));
  Serial.println(F("  z            : Reset selected motor deviation to 0"));
  Serial.println(F("  Z            : Reset ALL motor deviations to 0"));
  Serial.println(F("  h            : Move all motors to theoretical CENTER (500 ticks)"));
  Serial.println(F("  b            : Toggle xArm Serial1 Baud Rate (9600 <-> 115200)"));
  Serial.println(F("  p            : Export calibrated YAML block for config files"));
  Serial.println(F("  !            : EMERGENCY STOP (Cut motor torque immediately)"));
  Serial.println(F("  r            : Resume / restore motor torque"));
  Serial.println(F("  ?            : Show this menu"));
  Serial.println(F("========================================================"));
}

void printStatus() {
  Serial.println(F("\n--- Current Servo Calibration Status ---"));
  Serial.print(F("Calibrating: xArm "));
  Serial.print(activeArm);
  if (activeArm == 1) Serial.println(F(" (Back / NW Manipulator)"));
  else Serial.println(F(" (Front / SW Manipulator)"));

  for (int i = 1; i <= 6; i++) {
    int effective = servoPositions[i] + servoDeviations[i];
    float angleDeg = effective * (240.0f / 1000.0f);
    
    if (i == selectedMotor) {
      Serial.print(F(" -> [M"));
    } else {
      Serial.print(F("    [M"));
    }
    Serial.print(i);
    Serial.print(F(" - "));
    Serial.print(getMotorName(i));
    Serial.print(F("] Base: "));
    Serial.print(servoPositions[i]);
    Serial.print(F(" | Dev: "));
    if (servoDeviations[i] >= 0) Serial.print(F("+"));
    Serial.print(servoDeviations[i]);
    Serial.print(F(" | Hardware: "));
    Serial.print(effective);
    Serial.print(F(" ticks ("));
    Serial.print(angleDeg, 1);
    Serial.println(F(" deg)"));
  }
  Serial.print(F("Active Selection: Motor "));
  Serial.print(selectedMotor);
  Serial.print(F(" ("));
  Serial.print(getMotorName(selectedMotor));
  Serial.println(F(")"));
  Serial.print(F("xArm Serial1 Baud Rate: "));
  Serial.println(xArmBaud);
}

void printCalibrationOutput() {
  Serial.println(F("\n# ----------------------------------------------------"));
  Serial.print(F("# Copy and paste into config/servo_calibration.yaml under xarm_"));
  Serial.print(activeArm);
  Serial.println(F(":"));
  Serial.println(F("# ----------------------------------------------------"));
  Serial.print(F("xarm_")); Serial.print(activeArm); Serial.println(F(":"));
  Serial.println(F("  status: \"verified\""));
  Serial.println(F("  servo_deviations:"));
  Serial.print(F("    motor_1_claw: "));        Serial.println(servoDeviations[1]);
  Serial.print(F("    motor_2_wrist_roll: "));  Serial.println(servoDeviations[2]);
  Serial.print(F("    motor_3_wrist_pitch: ")); Serial.println(servoDeviations[3]);
  Serial.print(F("    motor_4_elbow: "));       Serial.println(servoDeviations[4]);
  Serial.print(F("    motor_5_shoulder: "));    Serial.println(servoDeviations[5]);
  Serial.print(F("    motor_6_base_yaw: "));    Serial.println(servoDeviations[6]);
  Serial.println(F("\n  # C++ / Arduino Array:"));
  Serial.print(F("  const int SERVO_DEV_ARM")); Serial.print(activeArm); Serial.print(F("[7] = {0, "));
  for (int i = 1; i <= 6; i++) {
    Serial.print(servoDeviations[i]);
    if (i < 6) Serial.print(F(", "));
  }
  Serial.println(F("};"));
  Serial.println(F("# ----------------------------------------------------\n"));
}

void setup() {
  // Initialize PC USB Serial
  Serial.begin(PC_BAUD);
  while (!Serial && millis() < 3000); // Wait up to 3s for Serial Monitor on Uno R4

  // Initialize Hardware Serial (Pins 0/1) to xArm 1s
  xArmSerial.begin(xArmBaud);

  printHeader();
  
  // Command arm to safe center on startup (1500 ms gentle transit)
  Serial.print(F("\n[INIT] Connecting to xArm at "));
  Serial.print(xArmBaud);
  Serial.println(F(" baud. Moving all joints to nominal center (500 ticks) in 1500 ms..."));
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
        applyAllPositions(1000);
        printStatus();
      }
    }
    return;
  }

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    // Emergency stop via serial
    if (input.equals("!") || input.equalsIgnoreCase("ESTOP")) {
      triggerEmergencyStop();
      return;
    }
    // Arm profile selection
    else if (input.equalsIgnoreCase("ARM 1") || input.equalsIgnoreCase("A1")) {
      activeArm = 1;
      Serial.println(F(">> Target profile switched to: xArm 1 (Back / NW Manipulator)"));
      printStatus();
    }
    else if (input.equalsIgnoreCase("ARM 2") || input.equalsIgnoreCase("A2")) {
      activeArm = 2;
      Serial.println(F(">> Target profile switched to: xArm 2 (Front / SW Manipulator)"));
      printStatus();
    }
    // Select Motor 1-6
    else if (input.length() == 1 && input.charAt(0) >= '1' && input.charAt(0) <= '6') {
      selectedMotor = input.charAt(0) - '0';
      Serial.print(F("Selected Motor: "));
      Serial.print(selectedMotor);
      Serial.print(F(" ("));
      Serial.print(getMotorName(selectedMotor));
      Serial.println(F(")"));
    }
    // Fine Trim: +1 tick
    else if (input.equals("+")) {
      servoDeviations[selectedMotor] += 1;
      int target = servoPositions[selectedMotor] + servoDeviations[selectedMotor];
      sendServoMove(selectedMotor, target, 50);
      printStatus();
    }
    // Fine Trim: -1 tick
    else if (input.equals("-")) {
      servoDeviations[selectedMotor] -= 1;
      int target = servoPositions[selectedMotor] + servoDeviations[selectedMotor];
      sendServoMove(selectedMotor, target, 50);
      printStatus();
    }
    // Coarse Trim: +10 ticks
    else if (input.equals(">")) {
      servoDeviations[selectedMotor] += 10;
      int target = servoPositions[selectedMotor] + servoDeviations[selectedMotor];
      sendServoMove(selectedMotor, target, 100);
      printStatus();
    }
    // Coarse Trim: -10 ticks
    else if (input.equals("<")) {
      servoDeviations[selectedMotor] -= 10;
      int target = servoPositions[selectedMotor] + servoDeviations[selectedMotor];
      sendServoMove(selectedMotor, target, 100);
      printStatus();
    }
    // Reset selected motor
    else if (input.equalsIgnoreCase("z")) {
      servoDeviations[selectedMotor] = 0;
      sendServoMove(selectedMotor, 500, 300);
      Serial.print(F("Reset Motor "));
      Serial.print(selectedMotor);
      Serial.println(F(" deviation to 0."));
      printStatus();
    }
    // Reset all deviations
    else if (input.equalsIgnoreCase("Z") || input.equalsIgnoreCase("RESET")) {
      for (int i = 1; i <= 6; i++) servoDeviations[i] = 0;
      applyAllPositions(1000);
      Serial.println(F("Reset ALL motor deviations to 0."));
      printStatus();
    }
    // Move to 500 ticks center
    else if (input.equalsIgnoreCase("h") || input.equalsIgnoreCase("HOME")) {
      Serial.println(F("Commanding all motors to 500 ticks (with current deviations)..."));
      applyAllPositions(1000);
    }
    // Toggle baud rate
    else if (input.equalsIgnoreCase("b")) {
      toggleBaudRate();
    }
    // Print formatted array
    else if (input.equalsIgnoreCase("p") || input.equalsIgnoreCase("EXPORT")) {
      printCalibrationOutput();
    }
    // Help menu
    else if (input.equals("?")) {
      printHeader();
    }
  }
}
