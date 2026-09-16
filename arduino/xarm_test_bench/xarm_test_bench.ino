/*
 * =====================================================================================
 * Project: Dual xArm 1s Color-Sorting Robot (Semester 5 Mini-Project)
 * File:    arduino/xarm_test_bench/xarm_test_bench.ino
 * Target:  Arduino Uno R4 WiFi (Renesas RA4M1 32-bit ARM Cortex-M4)
 * Purpose: Multi-Mode Interactive Hardware Test Bench & Diagnostic Harness
 * =====================================================================================
 * 
 * Pinout Configuration:
 *   - Joystick X-Axis    : Analog Pin A0 (Drives active joint forward / backward)
 *   - Joystick Y-Axis    : Analog Pin A1 (Fine speed / secondary trim)
 *   - Potentiometer      : Analog Pin A2 (Motor Selector: 1 to 6)
 *   - Pushbutton         : Digital Pin D2 (Pose Cycle Button - INPUT_PULLUP)
 *   - LED Green (Ready)  : Digital Pin D8
 *   - LED Yellow (Motion): Digital Pin D9
 *   - LED Red (Limit/Warn): Digital Pin D10
 *   - xArm Serial1       : Pin 0 (RX), Pin 1 (TX) @ 115200 baud
 *   - USB Serial         : PC Serial Monitor @ 115200 baud
 * 
 * Safety Limits & Envelopes:
 *   - Motor 1 (Base Yaw)   : Ticks 150 to 850 (Prevents hitting 240° mechanical hard stop)
 *   - Motor 2 (Shoulder)   : Ticks 200 to 850
 *   - Motor 3 (Elbow)      : Ticks 150 to 850
 *   - Motor 4 (Wrist Pitch): Ticks 150 to 850
 *   - Motor 5 (Wrist Roll) : Ticks 200 to 800
 *   - Motor 6 (Claw)       : Ticks 150 (closed) to 800 (open)
 * =====================================================================================
 */

#include <Arduino.h>

// --- PIN ASSIGNMENTS ---
const int PIN_JOY_X      = A0;
const int PIN_JOY_Y      = A1;
const int PIN_POT_SELECT = A2;
const int PIN_BTN_POSE   = 2;
const int PIN_BTN_ESTOP  = 3; // Hardware Interrupt INT1 (Emergency Stop - Active LOW)
const int PIN_LED_READY  = 8;
const int PIN_LED_MOTION = 9;
const int PIN_LED_WARN   = 10; // Rapid flash when E-Stopped

// --- EMERGENCY STOP LATCH STATE ---
volatile bool isEStopped = false;
volatile bool eStopHandled = false;

// --- BAUD RATES & SERIAL PORT ---
const unsigned long PC_BAUD   = 115200;
const unsigned long XARM_BAUD = 115200;

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  // For Arduino Uno R3: Hardware Serial (Pins 0/1). Disconnect TX Pin 1 during USB uploading!
  #define xArmSerial Serial
#else
  // For Arduino Uno R4 / Mega 2560 / ESP32: Hardware Serial1
  #define xArmSerial Serial1
#endif

// --- MOTOR LIMITS (TICKS) ---
const int MIN_TICK[7] = { 0, 150, 200, 150, 150, 200, 150 };
const int MAX_TICK[7] = { 0, 850, 850, 850, 850, 800, 800 };

// --- CURRENT MOTOR STATE (TICKS) ---
int currentTicks[7] = { 0, 500, 500, 500, 500, 500, 300 }; // Default Home State

// --- PREDEFINED SAFETY POSES ---
// Pose 0: HOME / Tucked Transit
const int POSE_HOME[7]      = { 0, 500, 720, 280, 500, 500, 300 };
// Pose 1: Near-Field Pick Standoff (R = 140 mm, Steep Pitch)
const int POSE_NEAR_PICK[7] = { 0, 500, 580, 480, 350, 500, 650 };
// Pose 2: Far-Field Pick Standoff (R = 300 mm, Horizontal Pitch)
const int POSE_FAR_PICK[7]  = { 0, 500, 480, 680, 620, 500, 650 };
// Pose 3: NW / SW Drop Pose (205° Swing)
const int POSE_DROP_SWING[7]= { 0, 820, 650, 350, 500, 500, 700 };

const char* POSE_NAMES[4] = {
  "1. Home / Tucked Transit",
  "2. Near-Field Standoff (R = 140mm)",
  "3. Far-Field Horizontal (R = 300mm)",
  "4. Drop Zone Swing (205 deg)"
};

int activePoseIndex = 0;
int activeMotor = 1;

// Timing and debouncing
unsigned long lastButtonTime = 0;
unsigned long lastJogTime = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long JOG_INTERVAL_MS = 40; // 25 Hz jog update rate

/**
 * Hardware Interrupt Service Routine for Emergency Stop (Pin D3)
 */
void handleEStopISR() {
  isEStopped = true;
}

/**
 * Emergency Stop Handler: Cuts torque, sends stop packets, and sets latch
 */
void triggerEmergencyStop() {
  isEStopped = true;

  // Immediately extinguish active indicators and set warning LED
  digitalWrite(PIN_LED_READY, LOW);
  digitalWrite(PIN_LED_MOTION, LOW);
  digitalWrite(PIN_LED_WARN, HIGH);

  // Send binary torque unload command (CMD 0x1D or 0x0E, Param 0 = torque off)
  for (byte id = 1; id <= 6; id++) {
    byte len = 4;
    byte cmd = 0x1D;
    byte param = 0x00; // 0 = torque off / limp mode
    byte checksum = ~(id + len + cmd + param) & 0xFF;
    xArmSerial.write(0x55);
    xArmSerial.write(0x55);
    xArmSerial.write(id);
    xArmSerial.write(len);
    xArmSerial.write(cmd);
    xArmSerial.write(param);
    xArmSerial.write(checksum);
  }

  Serial.println(F("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
  Serial.println(F("           *** EMERGENCY STOP TRIGGERED! ***            "));
  Serial.println(F("  All motor motion halted. Servos unloaded (Torque OFF). "));
  Serial.println(F("  Check mechanism for jams, wire snags, or collisions.  "));
  Serial.println(F("  To resume: Release E-Stop button and type 'RESUME'     "));
  Serial.println(F("  or press D2 button to re-enable torque and re-home.   "));
  Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
}

/**
 * Clear E-Stop latch, re-enable torque, and safely re-home
 */
void clearEmergencyStop() {
  if (digitalRead(PIN_BTN_ESTOP) == LOW) {
    Serial.println(F("[WARN] Cannot resume: Physical E-Stop button on D3 is still pressed!"));
    return;
  }

  Serial.println(F("\n[SAFETY] Clearing E-Stop latch. Re-enabling servo torque..."));

  // Re-enable torque on all servos (CMD 0x1D, Param 1 = torque on)
  for (byte id = 1; id <= 6; id++) {
    byte len = 4;
    byte cmd = 0x1D;
    byte param = 0x01;
    byte checksum = ~(id + len + cmd + param) & 0xFF;
    xArmSerial.write(0x55);
    xArmSerial.write(0x55);
    xArmSerial.write(id);
    xArmSerial.write(len);
    xArmSerial.write(cmd);
    xArmSerial.write(param);
    xArmSerial.write(checksum);
  }
  delay(100);

  isEStopped = false;
  eStopHandled = false;
  digitalWrite(PIN_LED_WARN, LOW);
  digitalWrite(PIN_LED_READY, HIGH);

  Serial.println(F("[SAFETY] Torque restored. Returning arm to safe Home pose..."));
  executePose(POSE_HOME, 1500);
}

/**
 * Send binary packet to drive a single servo via xArm controller
 */
void sendServoMove(byte id, uint16_t position, uint16_t timeMs) {
  if (isEStopped) return; // Drop motion commands while E-Stopped

  // Clamp to calibrated limits
  if (id >= 1 && id <= 6) {
    if (position < MIN_TICK[id]) position = MIN_TICK[id];
    if (position > MAX_TICK[id]) position = MAX_TICK[id];
  }
  
  byte len = 8; // Length = 8 for 1 servo move
  byte posL = position & 0xFF;
  byte posH = (position >> 8) & 0xFF;
  byte timeL = timeMs & 0xFF;
  byte timeH = (timeMs >> 8) & 0xFF;

  // Checksum = ~(Length + Cmd + Count + TimeL + TimeH + ID + PosL + PosH) & 0xFF
  byte checksum = ~(len + 0x03 + 1 + timeL + timeH + id + posL + posH) & 0xFF;

  xArmSerial.write(0x55);
  xArmSerial.write(0x55);
  xArmSerial.write(len);        // Byte 2: Length (8)
  xArmSerial.write(0x03);       // Byte 3: Command CMD_SERVO_MOVE (0x03)
  xArmSerial.write(0x01);       // Byte 4: Servo Count (1)
  xArmSerial.write(timeL);      // Byte 5: Time LSB
  xArmSerial.write(timeH);      // Byte 6: Time MSB
  xArmSerial.write(id);         // Byte 7: Servo ID
  xArmSerial.write(posL);       // Byte 8: Position LSB
  xArmSerial.write(posH);       // Byte 9: Position MSB
  xArmSerial.write(checksum);   // Byte 10: Checksum
}

/**
 * Move all 6 servos synchronously in a single clean multi-servo packet frame
 */
void executePose(const int targetPose[7], uint16_t timeMs) {
  if (isEStopped) return;

  digitalWrite(PIN_LED_READY, LOW);
  digitalWrite(PIN_LED_MOTION, HIGH);

  Serial.print(F("\n[MOTION] Executing Multi-Servo Pose in "));
  Serial.print(timeMs);
  Serial.println(F(" ms..."));

  // Build Multi-Servo Binary Frame (26 bytes for 6 servos)
  byte count = 6;
  byte len = (count * 3) + 5; // 23 bytes
  byte totalLen = len + 3;    // 26 bytes total frame
  byte frame[26];

  frame[0] = 0x55;
  frame[1] = 0x55;
  frame[2] = len;             // Length = 23
  frame[3] = 0x03;            // CMD_SERVO_MOVE
  frame[4] = count;           // Servo Count = 6
  frame[5] = timeMs & 0xFF;   // Time LSB
  frame[6] = (timeMs >> 8) & 0xFF; // Time MSB

  uint16_t sum = len + 0x03 + count + frame[5] + frame[6];
  uint8_t idx = 7;

  for (int i = 1; i <= 6; i++) {
    int pos = targetPose[i];
    if (pos < MIN_TICK[i]) pos = MIN_TICK[i];
    if (pos > MAX_TICK[i]) pos = MAX_TICK[i];
    currentTicks[i] = pos;

    byte posL = pos & 0xFF;
    byte posH = (pos >> 8) & 0xFF;

    frame[idx++] = i;     // Servo ID
    frame[idx++] = posL;  // Pos LSB
    frame[idx++] = posH;  // Pos MSB

    sum += i + posL + posH;
  }

  frame[idx] = (~sum) & 0xFF; // Checksum

  xArmSerial.write(frame, totalLen);

  // Non-blocking delay loop checking E-Stop
  unsigned long startT = millis();
  while (millis() - startT < timeMs) {
    if (isEStopped) {
      digitalWrite(PIN_LED_MOTION, LOW);
      return;
    }
    delay(20);
  }

  digitalWrite(PIN_LED_MOTION, LOW);
  digitalWrite(PIN_LED_READY, HIGH);
}

/**
 * Read the Potentiometer on A2 to determine which motor is active
 */
void updateSelectedMotor() {
  int potVal = analogRead(PIN_POT_SELECT); // 0 to 1023
  // Divide into 6 clean buckets
  int newMotor = map(potVal, 0, 1024, 1, 7);
  if (newMotor > 6) newMotor = 6;

  if (newMotor != activeMotor) {
    activeMotor = newMotor;
    Serial.print(F(">> Active Motor changed to: Motor "));
    Serial.print(activeMotor);
    Serial.print(F(" (Current Tick: "));
    Serial.print(currentTicks[activeMotor]);
    Serial.println(F(")"));
  }
}

/**
 * Read Joystick X on A0 to jog active motor
 */
void handleAnalogJog() {
  int joyX = analogRead(PIN_JOY_X); // 0 (left) to 1023 (right), center ~512
  int deadband = 60;
  int delta = joyX - 512;

  if (abs(delta) > deadband) {
    digitalWrite(PIN_LED_MOTION, HIGH);
    
    // Scale jog speed according to stick deflection
    int stepSize = map(abs(delta), deadband, 512, 1, 12);
    if (delta < 0) stepSize = -stepSize;

    int newTick = currentTicks[activeMotor] + stepSize;

    // Check limits
    if (newTick < MIN_TICK[activeMotor]) {
      newTick = MIN_TICK[activeMotor];
      digitalWrite(PIN_LED_WARN, HIGH);
    } else if (newTick > MAX_TICK[activeMotor]) {
      newTick = MAX_TICK[activeMotor];
      digitalWrite(PIN_LED_WARN, HIGH);
    } else {
      digitalWrite(PIN_LED_WARN, LOW);
    }

    if (newTick != currentTicks[activeMotor]) {
      currentTicks[activeMotor] = newTick;
      sendServoMove(activeMotor, currentTicks[activeMotor], JOG_INTERVAL_MS);
    }
  } else {
    digitalWrite(PIN_LED_MOTION, LOW);
    digitalWrite(PIN_LED_WARN, LOW);
  }
}

/**
 * Check pose cycle pushbutton on D2 with 300 ms debounce
 */
void handlePoseButton() {
  if (digitalRead(PIN_BTN_POSE) == LOW) {
    if (millis() - lastButtonTime > 350) {
      lastButtonTime = millis();
      activePoseIndex = (activePoseIndex + 1) % 4;

      Serial.print(F("\n[BUTTON] Switching to: "));
      Serial.println(POSE_NAMES[activePoseIndex]);

      if (activePoseIndex == 0) executePose(POSE_HOME, 1200);
      else if (activePoseIndex == 1) executePose(POSE_NEAR_PICK, 1500);
      else if (activePoseIndex == 2) executePose(POSE_FAR_PICK, 1800);
      else if (activePoseIndex == 3) executePose(POSE_DROP_SWING, 2000);
    }
  }
}

/**
 * Diagnostic Claw Grip Test Routine
 */
void runClawGripTest() {
  Serial.println(F("\n========================================================"));
  Serial.println(F("              CLAW GRIP VERIFICATION TEST               "));
  Serial.println(F("========================================================"));
  Serial.println(F("1. Opening claw to 650 ticks..."));
  sendServoMove(6, 650, 800);
  delay(1000);

  Serial.println(F("2. Insert a 25mm test cube (or marker/pen) between fingers now."));
  Serial.println(F("   Clamping in 3 seconds..."));
  delay(3000);

  Serial.println(F("3. Clamping with moderate torque (Tick 250)..."));
  sendServoMove(6, 250, 800);
  delay(1200);

  Serial.println(F("4. Grip Evaluation:"));
  Serial.println(F("   - If closed completely (~150-180 ticks): EMPTY CLAW (MISSED)."));
  Serial.println(F("   - If stopped on cube (~380-460 ticks): SUCCESSFUL GRASP."));
  Serial.println(F("   - If wider than 600 ticks: JAMMED / WRONG DIRECTION."));
  Serial.println(F("========================================================\n"));
}

/**
 * Process text commands sent from Serial Monitor
 */
void handleSerialCLI() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.equalsIgnoreCase("HELP") || line.equals("?")) {
      Serial.println(F("\n--- CLI Command Menu ---"));
      Serial.println(F("  HOME        : Move to safe tucked transit pose"));
      Serial.println(F("  POSE1       : Near-field pick standoff (R=140mm)"));
      Serial.println(F("  POSE2       : Far-field horizontal reach (R=300mm)"));
      Serial.println(F("  POSE3       : NW/SW Drop pose (205 deg)"));
      Serial.println(F("  GRIP        : Run Claw Grip diagnostic routine"));
      Serial.println(F("  STATUS      : Print all 6 motor tick positions"));
      Serial.println(F("  ESTOP / !   : Trigger EMERGENCY STOP (Cuts motor torque immediately)"));
      Serial.println(F("  RESUME      : Clear E-Stop latch and re-enable motors"));
      Serial.println(F("  SET <id> <pos> <time> : Example: SET 1 600 500"));
      Serial.println(F("------------------------\n"));
    }
    else if (line.equalsIgnoreCase("ESTOP") || line.equalsIgnoreCase("STOP") || line.equals("!")) {
      triggerEmergencyStop();
    }
    else if (line.equalsIgnoreCase("RESUME") || line.equalsIgnoreCase("CLEAR") || line.equalsIgnoreCase("RESET")) {
      clearEmergencyStop();
    }
    else if (line.equalsIgnoreCase("HOME")) {
      executePose(POSE_HOME, 1200);
    }
    else if (line.equalsIgnoreCase("POSE1")) {
      executePose(POSE_NEAR_PICK, 1500);
    }
    else if (line.equalsIgnoreCase("POSE2")) {
      executePose(POSE_FAR_PICK, 1800);
    }
    else if (line.equalsIgnoreCase("POSE3")) {
      executePose(POSE_DROP_SWING, 2000);
    }
    else if (line.equalsIgnoreCase("GRIP")) {
      runClawGripTest();
    }
    else if (line.equalsIgnoreCase("STATUS")) {
      Serial.println(F("\n--- Current Servo Positions ---"));
      for (int i = 1; i <= 6; i++) {
        float deg = currentTicks[i] * 0.24f;
        Serial.print(F("Motor "));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(currentTicks[i]);
        Serial.print(F(" ticks ("));
        Serial.print(deg, 1);
        Serial.println(F(" deg)"));
      }
    }
    else if (line.startsWith("SET ")) {
      // Parse: SET <id> <pos> <time>
      int firstSpace = line.indexOf(' ', 4);
      if (firstSpace > 0) {
        int id = line.substring(4, firstSpace).toInt();
        int secondSpace = line.indexOf(' ', firstSpace + 1);
        int pos = 0;
        int t = 500;
        if (secondSpace > 0) {
          pos = line.substring(firstSpace + 1, secondSpace).toInt();
          t = line.substring(secondSpace + 1).toInt();
        } else {
          pos = line.substring(firstSpace + 1).toInt();
        }

        if (id >= 1 && id <= 6 && pos >= 0 && pos <= 1000) {
          currentTicks[id] = pos;
          sendServoMove(id, pos, t);
          Serial.print(F("Moved Motor "));
          Serial.print(id);
          Serial.print(F(" to "));
          Serial.print(pos);
          Serial.println(F(" ticks."));
        } else {
          Serial.println(F("Error: Invalid ID (1-6) or Position (0-1000)."));
        }
      }
    }
  }
}

void setup() {
  Serial.begin(PC_BAUD);
  while (!Serial && millis() < 3000);

  xArmSerial.begin(XARM_BAUD);

  // Configure Pins
  pinMode(PIN_BTN_POSE, INPUT_PULLUP);
  pinMode(PIN_BTN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_LED_READY, OUTPUT);
  pinMode(PIN_LED_MOTION, OUTPUT);
  pinMode(PIN_LED_WARN, OUTPUT);

  // Attach Hardware Interrupt for Emergency Stop (triggers on falling edge when button is pressed to GND)
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_ESTOP), handleEStopISR, FALLING);

  digitalWrite(PIN_LED_READY, HIGH);
  digitalWrite(PIN_LED_MOTION, LOW);
  digitalWrite(PIN_LED_WARN, LOW);

  Serial.println(F("========================================================"));
  Serial.println(F("     xArm 1s MULTI-MODE TEST BENCH & DIAGNOSTICS        "));
  Serial.println(F("========================================================"));
  Serial.println(F("Controls:"));
  Serial.println(F("  - Potentiometer (A2) : Selects Motor (1 to 6)"));
  Serial.println(F("  - Joystick X (A0)    : Live Analog Jog"));
  Serial.println(F("  - Pushbutton (D2)    : Cycles Predefined Safety Poses"));
  Serial.println(F("  - E-Stop Button (D3) : EMERGENCY STOP (Hardware Interrupt)"));
  Serial.println(F("  - Serial Monitor     : Type HELP for CLI, ! for E-Stop"));
  Serial.println(F("========================================================"));

  // Initialize arm into safe home pose
  executePose(POSE_HOME, 1500);
}

void loop() {
  // --- 0. EMERGENCY STOP LATCH CHECK ---
  if (isEStopped) {
    if (!eStopHandled) {
      eStopHandled = true;
      triggerEmergencyStop();
    }

    // Rapid flash Warning LED (4 Hz) while E-Stopped
    digitalWrite(PIN_LED_WARN, (millis() / 125) % 2 == 0 ? HIGH : LOW);
    digitalWrite(PIN_LED_READY, LOW);
    digitalWrite(PIN_LED_MOTION, LOW);

    // Allow clearing E-Stop via D2 button (only if physical D3 button is released)
    if (digitalRead(PIN_BTN_POSE) == LOW && digitalRead(PIN_BTN_ESTOP) == HIGH) {
      delay(250);
      clearEmergencyStop();
    }

    // Allow Serial Monitor CLI to process RESUME
    handleSerialCLI();
    return; // Block all motion while E-Stopped
  }

  // --- NORMAL OPERATION ---
  // 1. Process Pushbutton for pose cycling
  handlePoseButton();

  // 2. Read Potentiometer for active motor selection
  updateSelectedMotor();

  // 3. Process Analog Joystick Jogging at fixed interval
  if (millis() - lastJogTime >= JOG_INTERVAL_MS) {
    lastJogTime = millis();
    handleAnalogJog();
  }

  // 4. Process Serial Monitor CLI
  handleSerialCLI();

  // 5. Periodic telemetry output to Serial Monitor (every 1.5 seconds)
  if (millis() - lastTelemetryTime >= 1500) {
    lastTelemetryTime = millis();
    Serial.print(F("[LIVE] Active: M"));
    Serial.print(activeMotor);
    Serial.print(F(" | Pos: "));
    Serial.print(currentTicks[activeMotor]);
    Serial.print(F(" ticks ("));
    Serial.print(currentTicks[activeMotor] * 0.24f, 1);
    Serial.println(F(" deg)"));
  }
}
