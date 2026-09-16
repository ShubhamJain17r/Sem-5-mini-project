/*
 * =====================================================================================
 * Project: Dual xArm 1s Color-Sorting Robot (Semester 5 Mini-Project)
 * File:    arduino/xarm_direct_test/xarm_direct_test.ino
 * Target:  Arduino Uno R4 WiFi
 * Purpose: Breadboard-Free Keyboard Teleoperation, Reach Testing & Multi-Step Pick-and-Place
 * =====================================================================================
 * 
 * OFFICIAL HIWONDER xArm 1s MOTOR MAPPING (End-Effector to Base):
 *   - Motor 1: Claw Gripper (150 = fully closed, 750 = fully open)
 *   - Motor 2: Wrist Roll
 *   - Motor 3: Wrist Pitch
 *   - Motor 4: Elbow Pitch
 *   - Motor 5: Shoulder Pitch
 *   - Motor 6: Base Yaw (Center = 500 ticks / 120 deg)
 * 
 * CALIBRATED DEVIATIONS (Applied automatically to theoretical coordinates):
 *   Motor 1 (Claw)       : +1
 *   Motor 2 (Wrist Roll) : +3
 *   Motor 3 (Wrist Pitch): -3
 *   Motor 4 (Elbow)      : -13
 *   Motor 5 (Shoulder)   : 0
 *   Motor 6 (Base)       : 0
 * 
 * REACH TESTING WAYPOINTS:
 *   - Position A (Pick Zone) : Forward-Right Reach (Base=320, Shoulder=500, Elbow=640)
 *   - Position B (Place Zone): Left NW Drop Bin (Base=820, Shoulder=560, Elbow=460)
 *   - Angular Span: ~120 degrees base yaw rotation + full forward elbow extension
 * =====================================================================================
 */

#include <Arduino.h>

// --- BAUD RATES & SERIAL ---
const unsigned long PC_BAUD   = 115200;
unsigned long xArmBaud        = 9600; // Hiwonder secondary UART runs at 9600 baud!

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  #define xArmSerial Serial
#else
  #define xArmSerial Serial1
#endif

// --- CALIBRATED SERVO DEVIATIONS (MEASURED) ---
const int SERVO_DEV[7] = { 0, 1, 3, -3, -13, 0, 0 };

// --- MOTOR LIMITS (TICKS) ---
// M1: Claw | M2: Roll | M3: Wrist Pitch | M4: Elbow | M5: Shoulder | M6: Base
const int MIN_TICK[7] = { 0, 150, 200, 150, 150, 200, 150 };
const int MAX_TICK[7] = { 0, 800, 800, 850, 850, 850, 850 };

// --- CURRENT THEORETICAL MOTOR STATE (TICKS) ---
// Default to Home: M1(Claw)=700, M2(Roll)=500, M3(WP)=500, M4(El)=280, M5(Sh)=720, M6(Base)=500
int currentTicks[7] = { 0, 700, 500, 500, 280, 720, 500 };

// --- PREDEFINED POSES & PICK-AND-PLACE WAYPOINTS ---
// Indices: [0]=unused, [1]=Claw, [2]=Roll, [3]=WPitch, [4]=Elbow, [5]=Shoulder, [6]=Base
const int POSE_HOME[7]           = { 0, 700, 500, 500, 280, 720, 500 }; // Safe Tucked Transit (Claw open)
const int POSE_PICK_APPROACH[7]  = { 0, 700, 500, 520, 500, 600, 320 }; // Pos A Hover (Right Azimuth @ 320)
const int POSE_PICK_GRASP[7]     = { 0, 700, 500, 580, 640, 500, 320 }; // Pos A Reach (Shoulder=500, Elbow=640)
const int POSE_PICK_LIFT[7]      = { 0, 250, 500, 520, 500, 600, 320 }; // Pos A Lift (Claw clamped @ 250)
const int POSE_TRANSIT_MID[7]    = { 0, 250, 500, 500, 320, 680, 570 }; // High Clearance Midpoint Swing
const int POSE_PLACE_APPROACH[7] = { 0, 250, 500, 500, 380, 620, 820 }; // Pos B Hover (Left Drop @ 820)
const int POSE_PLACE_DROP[7]     = { 0, 250, 500, 460, 460, 560, 820 }; // Pos B Drop (Lowered into bin)
const int POSE_PLACE_RETRACT[7]  = { 0, 700, 500, 500, 380, 620, 820 }; // Pos B Retract (Claw open)

// Diagnostic reference poses
const int POSE_NEAR_PICK[7]      = { 0, 650, 500, 350, 480, 580, 500 }; // R = 140mm, Steep Pitch
const int POSE_FAR_PICK[7]       = { 0, 650, 500, 620, 680, 480, 500 }; // R = 300mm, Horizontal Pitch
const int POSE_DROP_SWING[7]     = { 0, 700, 500, 500, 350, 650, 820 }; // 205° Drop Swing (Base @ 820)

int activeMotor = 6; // Default to Base Yaw (Motor 6)
volatile bool isEStopped = false;

// Command constants
const byte CMD_SERVO_MOVE        = 0x03;
const byte CMD_MULT_SERVO_UNLOAD = 0x14;

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
 * Toggle Baud Rate between 9600 and 115200
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
 * Emergency Stop Handler: Cuts torque and halts motion
 */
void triggerEmergencyStop() {
  isEStopped = true;

  // Send multi-servo unload command (CMD 0x14) to cut torque on all 6 servos
  byte unloadFrame[11] = {
    0x55, 0x55,
    0x09,       // Length = 9
    CMD_MULT_SERVO_UNLOAD, // 0x14
    0x06,       // 6 servos
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
 * Resume from Emergency Stop
 */
void clearEmergencyStop() {
  isEStopped = false;
  Serial.println(F("\n[SAFETY] Clearing E-Stop latch. Servos re-enabled."));
}

/**
 * Send command to move a single servo with deviation compensation
 * Format: 0x55 0x55 [Length=8] [CMD 0x03] [Count=1] [TimeL] [TimeH] [ID] [PosL] [PosH]
 */
void sendServoMove(byte id, uint16_t theoreticalPosition, uint16_t timeMs) {
  if (isEStopped) return;

  // Apply calibrated deviation to theoretical position
  int hardwarePos = theoreticalPosition;
  if (id >= 1 && id <= 6) {
    hardwarePos += SERVO_DEV[id];
    if (hardwarePos < MIN_TICK[id]) hardwarePos = MIN_TICK[id];
    if (hardwarePos > MAX_TICK[id]) hardwarePos = MAX_TICK[id];
  }

  byte len = 8;
  byte posL = hardwarePos & 0xFF;
  byte posH = (hardwarePos >> 8) & 0xFF;
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
 * Move all 6 servos gently with 25ms inter-joint stagger to prevent voltage brownout
 */
void executePoseGentle(const int targetPose[7], uint16_t timeMs) {
  if (isEStopped) return;

  Serial.print(F("[MOTION] Moving arm in "));
  Serial.print(timeMs);
  Serial.println(F(" ms..."));

  // Move in logical order: Base -> Shoulder -> Elbow -> Wrist Pitch -> Roll -> Claw
  for (int i = 6; i >= 1; i--) {
    if (isEStopped) return;
    currentTicks[i] = targetPose[i];
    sendServoMove(i, targetPose[i], timeMs);
    delay(25); // Gentle 25ms staggering prevents sudden current spikes / low voltage buzzer!
  }
}

/**
 * Non-blocking delay that continuously monitors Serial for emergency stop or abort commands.
 * Returns true if delay completed normally, false if interrupted by user or E-stop.
 */
bool abortableWait(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (isEStopped) return false;

    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.length() > 0) {
        if (cmd.charAt(0) == '!' || cmd.equalsIgnoreCase("ESTOP")) {
          triggerEmergencyStop();
          return false;
        } else if (cmd.equalsIgnoreCase("STOP") || cmd.equalsIgnoreCase("ABORT") || 
                   cmd.equalsIgnoreCase("Q") || cmd.equalsIgnoreCase("QUIT") || 
                   cmd.equalsIgnoreCase("S")) {
          Serial.println(F("\n>>> [ABORT] Routine halted by user command. <<<"));
          return false;
        }
      }
    }
    delay(15);
  }
  return true;
}

/**
 * Executes a complete, multi-step Pick-and-Place reach test between two widely separated positions:
 *   - Position A (Pick Zone) : Forward-Right Reach (Base 320, Shoulder 500, Elbow 640)
 *   - Position B (Place Zone): Left NW Drop Bin (Base 820, Shoulder 560, Elbow 460)
 * Demonstrates ~120 degrees of Base yaw rotation and extended elbow reach.
 */
void runPickAndPlaceSequence(bool continuousLoop) {
  int cycle = 0;
  Serial.println(F("\n========================================================"));
  Serial.println(F("   STARTING MULTI-STEP PICK-AND-PLACE REACH TEST        "));
  Serial.println(F("========================================================"));
  Serial.println(F("WAYPOINTS:"));
  Serial.println(F("  Position A (Pick) : Forward-Right Reach (Base=320, Elbow=640)"));
  Serial.println(F("  Position B (Place): Left Drop Bin (Base=820, Elbow=460)"));
  Serial.print(F("MODE: "));
  if (continuousLoop) {
    Serial.println(F("CONTINUOUS LOOP (Type 'stop' or '!' anytime to halt)"));
  } else {
    Serial.println(F("SINGLE CYCLE (Type 'loop' for continuous execution)"));
  }
  Serial.println(F("--------------------------------------------------------"));

  while (!isEStopped) {
    cycle++;
    if (continuousLoop) {
      Serial.print(F("\n>>> --- CYCLE "));
      Serial.print(cycle);
      Serial.println(F(" START --- <<<"));
    }

    // Step 1: Transit Home
    Serial.println(F("[Step 1/10] Moving to Safe Transit Home (Elbow tucked, claw open)..."));
    executePoseGentle(POSE_HOME, 1500);
    if (!abortableWait(1650)) break;

    // Step 2: Approach Pick Zone (Hover Standoff)
    Serial.println(F("[Step 2/10] Approaching Pick Zone (Base -> 320, Hover Standoff)..."));
    executePoseGentle(POSE_PICK_APPROACH, 1600);
    if (!abortableWait(1750)) break;

    // Step 3: Descend & Reach for Object (Test Forward Extension)
    Serial.println(F("[Step 3/10] Extending reach downward to grasp object (Elbow -> 640)..."));
    executePoseGentle(POSE_PICK_GRASP, 1400);
    if (!abortableWait(1550)) break;

    // Step 4: Grasp Object (Claw Motor 1)
    Serial.println(F("[Step 4/10] Closing Claw to grip object (Motor 1 -> 250 ticks)..."));
    sendServoMove(1, 250, 600);
    currentTicks[1] = 250;
    if (!abortableWait(800)) break;

    // Step 5: Lift object vertically
    Serial.println(F("[Step 5/10] Lifting object vertically to clearance altitude..."));
    executePoseGentle(POSE_PICK_LIFT, 1300);
    if (!abortableWait(1450)) break;

    // Step 6: Transit Midpoint Swing
    Serial.println(F("[Step 6/10] Elevated transit swing across arena midline..."));
    executePoseGentle(POSE_TRANSIT_MID, 1500);
    if (!abortableWait(1650)) break;

    // Step 7: Approach Place Zone (Position B)
    Serial.println(F("[Step 7/10] Swiveling to NW Drop Zone (Base -> 820 ticks / 197 deg)..."));
    executePoseGentle(POSE_PLACE_APPROACH, 1800);
    if (!abortableWait(1950)) break;

    // Step 8: Lower over Drop Bin
    Serial.println(F("[Step 8/10] Lowering over Drop Bin..."));
    executePoseGentle(POSE_PLACE_DROP, 1300);
    if (!abortableWait(1450)) break;

    // Step 9: Open Claw to Drop Object
    Serial.println(F("[Step 9/10] Opening Claw to release object (Motor 1 -> 700 ticks)..."));
    sendServoMove(1, 700, 600);
    currentTicks[1] = 700;
    if (!abortableWait(800)) break;

    // Step 10: Retract and return to Transit Home
    Serial.println(F("[Step 10/10] Retracting upward and returning to Transit Home..."));
    executePoseGentle(POSE_PLACE_RETRACT, 1200);
    if (!abortableWait(1350)) break;

    executePoseGentle(POSE_HOME, 1800);
    if (!abortableWait(1950)) break;

    if (!continuousLoop) {
      break; // Completed single cycle!
    }

    Serial.print(F(">>> Cycle "));
    Serial.print(cycle);
    Serial.println(F(" Complete. Pausing 1.5s before next cycle..."));
    if (!abortableWait(1500)) break;
  }

  Serial.println(F("\n========================================================"));
  Serial.println(F("     PICK-AND-PLACE ACTION FINISHED / STOPPED           "));
  Serial.println(F("========================================================\n"));
}

/**
 * Diagnostic Claw Grip Verification Routine (Claw is Motor 1!)
 */
void runClawGripTest() {
  Serial.println(F("\n========================================================"));
  Serial.println(F("     CLAW GRIP VERIFICATION TEST (MOTOR 1 = CLAW)       "));
  Serial.println(F("========================================================"));
  Serial.println(F("1. Opening Motor 1 (Claw) to 700 ticks..."));
  sendServoMove(1, 700, 800);
  currentTicks[1] = 700;
  delay(1000);

  Serial.println(F("2. Insert a 25mm test cube (or pen/marker) between fingers."));
  Serial.println(F("   Clamping in 3 seconds..."));
  delay(3000);

  Serial.println(F("3. Clamping Motor 1 with moderate torque (Tick 250)..."));
  sendServoMove(1, 250, 800);
  currentTicks[1] = 250;
  delay(1200);

  Serial.println(F("4. Grip Evaluation:"));
  Serial.println(F("   - Fully closed (~150-180 ticks): EMPTY CLAW (MISSED)."));
  Serial.println(F("   - Stopped on cube (~380-460 ticks): SUCCESSFUL GRASP!"));
  Serial.println(F("   - Wider than 600 ticks: JAMMED."));
  Serial.println(F("========================================================\n"));
}

void printStatus() {
  Serial.println(F("\n--- Current Servo Positions ---"));
  for (int i = 1; i <= 6; i++) {
    float deg = currentTicks[i] * 0.24f;
    int hw = currentTicks[i] + SERVO_DEV[i];
    
    if (i == activeMotor) Serial.print(F(" -> "));
    else Serial.print(F("    "));

    Serial.print(F("[M"));
    Serial.print(i);
    Serial.print(F(" - "));
    Serial.print(getMotorName(i));
    Serial.print(F("] Theory: "));
    Serial.print(currentTicks[i]);
    Serial.print(F(" ticks ("));
    Serial.print(deg, 1);
    Serial.print(F(" deg) | Hardware: "));
    Serial.print(hw);
    Serial.print(F(" | Dev: "));
    if (SERVO_DEV[i] >= 0) Serial.print(F("+"));
    Serial.println(SERVO_DEV[i]);
  }
  Serial.print(F("Active Selection: Motor "));
  Serial.print(activeMotor);
  Serial.print(F(" ("));
  Serial.print(getMotorName(activeMotor));
  Serial.println(F(")"));
  Serial.print(F("xArm Serial Baud Rate: "));
  Serial.println(xArmBaud);
}

void printHeader() {
  Serial.println(F("\n========================================================"));
  Serial.println(F("   xArm 1s REACH TEST & PICK-AND-PLACE BENCH            "));
  Serial.println(F("========================================================"));
  Serial.println(F("MOTOR MAPPING:"));
  Serial.println(F("  Motor 1 = Claw Gripper"));
  Serial.println(F("  Motor 2 = Wrist Roll"));
  Serial.println(F("  Motor 3 = Wrist Pitch"));
  Serial.println(F("  Motor 4 = Elbow Pitch"));
  Serial.println(F("  Motor 5 = Shoulder Pitch"));
  Serial.println(F("  Motor 6 = Base Yaw"));
  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F("AUTOMATED ACTION COMMANDS:"));
  Serial.println(F("  action     : Run full 10-step Pick-and-Place reach cycle"));
  Serial.println(F("  loop       : Run Pick-and-Place continuously in a loop"));
  Serial.println(F("  stop       : Gracefully halt active routine / loop"));
  Serial.println(F("  pick       : Inspect Position A (Forward-Right Reach)"));
  Serial.println(F("  place      : Inspect Position B (Left NW Drop Bin)"));
  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F("MANUAL TELEOPERATION (Serial Monitor @ 115200):"));
  Serial.println(F("  1 - 6      : Select active Motor"));
  Serial.println(F("  a / d      : Jog active motor -10 / +10 ticks"));
  Serial.println(F("  < / >      : Fine jog -2 / +2 ticks"));
  Serial.println(F("  h          : Move to HOME Pose (Transit Tucked)"));
  Serial.println(F("  p1, p2, p3 : Predefined Poses (Near / Far / Drop)"));
  Serial.println(F("  o / c      : Open / Close Claw (Motor 1)"));
  Serial.println(F("  g          : Run Grip Verification Test (Motor 1)"));
  Serial.println(F("  b          : Toggle Baud Rate (9600 <-> 115200)"));
  Serial.println(F("  s          : Print live STATUS"));
  Serial.println(F("  !          : EMERGENCY STOP (Cut torque immediately)"));
  Serial.println(F("  r          : RESUME from E-stop"));
  Serial.println(F("  ?          : Show this menu"));
  Serial.println(F("========================================================"));
  Serial.println(F("Type 'action' to run full pick-and-place, or 'loop' to cycle continuously."));
}

void setup() {
  // PC USB Serial Monitor
  Serial.begin(PC_BAUD);
  while (!Serial && millis() < 3000);

  // xArm Serial1 (9600 baud)
  xArmSerial.begin(xArmBaud);

  printHeader();
}

void loop() {
  // Check Emergency Stop
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

  // Process Keyboard / Serial CLI Commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    // --- Automated Multi-Step Sequences ---
    if (input.equalsIgnoreCase("ACTION") || input.equalsIgnoreCase("ACT")) {
      runPickAndPlaceSequence(false); // 1 complete cycle
    }
    else if (input.equalsIgnoreCase("LOOP") || input.equalsIgnoreCase("ACTION LOOP")) {
      runPickAndPlaceSequence(true); // Continuous loop
    }
    else if (input.equalsIgnoreCase("PICK") || input.equalsIgnoreCase("POSA")) {
      Serial.println(F("Moving to Position A (Forward-Right Reach)..."));
      executePoseGentle(POSE_PICK_GRASP, 1800);
    }
    else if (input.equalsIgnoreCase("PLACE") || input.equalsIgnoreCase("POSB")) {
      Serial.println(F("Moving to Position B (Left NW Drop Bin)..."));
      executePoseGentle(POSE_PLACE_DROP, 1800);
    }
    else if (input.equalsIgnoreCase("STOP")) {
      Serial.println(F("[SYSTEM] Arm is idle. All servos holding position."));
    }
    // --- Manual Motor Selection: '1' - '6' ---
    else if (input.length() == 1 && input.charAt(0) >= '1' && input.charAt(0) <= '6') {
      activeMotor = input.charAt(0) - '0';
      Serial.print(F(">> Selected Motor "));
      Serial.print(activeMotor);
      Serial.print(F(" ("));
      Serial.print(getMotorName(activeMotor));
      Serial.print(F(") | Current: "));
      Serial.print(currentTicks[activeMotor]);
      Serial.println(F(" ticks"));
    }
    // Jog forward: 'd' or '+'
    else if (input.equalsIgnoreCase("d") || input.equals("+")) {
      currentTicks[activeMotor] += 10;
      if (currentTicks[activeMotor] > MAX_TICK[activeMotor]) currentTicks[activeMotor] = MAX_TICK[activeMotor];
      sendServoMove(activeMotor, currentTicks[activeMotor], 80);
      Serial.print(F("M")); Serial.print(activeMotor); 
      Serial.print(F(" (")); Serial.print(getMotorName(activeMotor)); Serial.print(F(") -> ")); 
      Serial.println(currentTicks[activeMotor]);
    }
    // Jog backward: 'a' or '-'
    else if (input.equalsIgnoreCase("a") || input.equals("-")) {
      currentTicks[activeMotor] -= 10;
      if (currentTicks[activeMotor] < MIN_TICK[activeMotor]) currentTicks[activeMotor] = MIN_TICK[activeMotor];
      sendServoMove(activeMotor, currentTicks[activeMotor], 80);
      Serial.print(F("M")); Serial.print(activeMotor); 
      Serial.print(F(" (")); Serial.print(getMotorName(activeMotor)); Serial.print(F(") -> ")); 
      Serial.println(currentTicks[activeMotor]);
    }
    // Fine jog: '>' or '<'
    else if (input.equals(">")) {
      currentTicks[activeMotor] += 2;
      sendServoMove(activeMotor, currentTicks[activeMotor], 50);
      Serial.print(F("M")); Serial.print(activeMotor); Serial.print(F(" -> ")); Serial.println(currentTicks[activeMotor]);
    }
    else if (input.equals("<")) {
      currentTicks[activeMotor] -= 2;
      sendServoMove(activeMotor, currentTicks[activeMotor], 50);
      Serial.print(F("M")); Serial.print(activeMotor); Serial.print(F(" -> ")); Serial.println(currentTicks[activeMotor]);
    }
    // Home Pose
    else if (input.equalsIgnoreCase("h") || input.equalsIgnoreCase("HOME")) {
      executePoseGentle(POSE_HOME, 1500);
    }
    // Diagnostic Poses
    else if (input.equalsIgnoreCase("P1") || input.equalsIgnoreCase("NEAR")) {
      executePoseGentle(POSE_NEAR_PICK, 1500);
    }
    else if (input.equalsIgnoreCase("P2") || input.equalsIgnoreCase("FAR")) {
      executePoseGentle(POSE_FAR_PICK, 1800);
    }
    else if (input.equalsIgnoreCase("P3") || input.equalsIgnoreCase("DROP")) {
      executePoseGentle(POSE_DROP_SWING, 2000);
    }
    // Claw controls (Motor 1 = Claw!)
    else if (input.equalsIgnoreCase("o")) {
      Serial.println(F("Opening Claw (Motor 1)..."));
      sendServoMove(1, 750, 500);
      currentTicks[1] = 750;
    }
    else if (input.equalsIgnoreCase("c")) {
      Serial.println(F("Closing Claw (Motor 1)..."));
      sendServoMove(1, 250, 500);
      currentTicks[1] = 250;
    }
    // Grip test
    else if (input.equalsIgnoreCase("g")) {
      runClawGripTest();
    }
    // Toggle baud rate
    else if (input.equalsIgnoreCase("b")) {
      toggleBaudRate();
    }
    // Status
    else if (input.equalsIgnoreCase("s")) {
      printStatus();
    }
    // Emergency stop
    else if (input.equals("!") || input.equalsIgnoreCase("ESTOP")) {
      triggerEmergencyStop();
    }
    // Help
    else if (input.equals("?")) {
      printHeader();
    }
  }
}
