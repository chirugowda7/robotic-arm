void setup() {
  // put your setup code here, to run once:

}
#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

// =====================================================
// ROBOTIC ARM - ESP32
// 5 PHYSICAL SERVOS + 1 GRIPPER RELAY
// TRUE SPEED CONTROL
// =====================================================

#define NUM_SERVOS 5

// =====================================================
// SERVO PINS
// =====================================================

const int SERVO_PINS[NUM_SERVOS] = {
  25,   // S1 Base
  13,   // S2 Shoulder
  14,   // S3 Elbow
  27,   // S4 Wrist Pitch
  26    // S5 Wrist Roll
};

// =====================================================
// GRIPPER
// =====================================================

#define GRIPPER_PIN 33

// =====================================================
// SERVO OBJECTS
// =====================================================

Servo servos[NUM_SERVOS];

// =====================================================
// SERVO LIMITS
// =====================================================

const float SERVO_MIN[NUM_SERVOS] = {
  0.0,     // S1
  30.0,    // S2
  60.0,    // S3
  30.0,    // S4
  0.0      // S5
};

const float SERVO_MAX[NUM_SERVOS] = {
  180.0,   // S1
  150.0,   // S2
  180.0,   // S3
  150.0,   // S4
  180.0    // S5
};

// =====================================================
// POWER-ON / HOME POSITION
// =====================================================

const float POWER_ON_ANGLES[NUM_SERVOS] = {
  0.0,      // S1 Base
  110.0,    // S2 Shoulder
  125.0,    // S3 Elbow
  90.0,     // S4 Wrist Pitch
  80.0      // S5 Wrist Roll
};

// =====================================================
// CURRENT AND TARGET POSITIONS
// =====================================================

float currentAngles[NUM_SERVOS] = {
  0.0,
  110.0,
  125.0,
  90.0,
  80.0
};

float targetAngles[NUM_SERVOS] = {
  0.0,
  110.0,
  125.0,
  90.0,
  80.0
};

// =====================================================
// MOVEMENT
// =====================================================

bool moving = false;
bool emergencyStop = false;

// TRUE SPEED = DEGREES PER SECOND
float speedValue = 40.0;

// Movement update interval
const unsigned long MOVE_INTERVAL = 20;

// Last movement update
unsigned long lastMoveTime = 0;

// =====================================================
// FUNCTION DECLARATIONS
// =====================================================

void sendPosition();
void updateMovement();
void processCommand(String command);

// =====================================================
// WRITE SERVO
// =====================================================

void writeServo(int index, float angle)
{
  if (index < 0 || index >= NUM_SERVOS)
    return;

  angle = constrain(
    angle,
    SERVO_MIN[index],
    SERVO_MAX[index]
  );

  int pulseWidth = map(
    (int)angle,
    0,
    180,
    500,
    2400
  );

  pulseWidth = constrain(
    pulseWidth,
    500,
    2400
  );

  servos[index].writeMicroseconds(pulseWidth);
}

// =====================================================
// INITIALIZE ONE SERVO
// =====================================================

void initializeServo(int index)
{
  Serial.print("Initializing Servo ");
  Serial.println(index + 1);

  servos[index].setPeriodHertz(50);

  servos[index].attach(
    SERVO_PINS[index],
    500,
    2400
  );

  delay(300);

  // Move this servo to power-on position
  writeServo(
    index,
    POWER_ON_ANGLES[index]
  );

  delay(700);

  currentAngles[index] =
    POWER_ON_ANGLES[index];

  targetAngles[index] =
    POWER_ON_ANGLES[index];

  Serial.print("Servo ");
  Serial.print(index + 1);
  Serial.print(" READY AT ");

  Serial.print(
    POWER_ON_ANGLES[index],
    1
  );

  Serial.println(" degrees");
}

// =====================================================
// INITIALIZE ALL SERVOS
// ONE AT A TIME
// =====================================================

void initializeAllServos()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("SEQUENTIAL SERVO INITIALIZATION");
  Serial.println("================================");

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    Serial.print("STARTING SERVO ");
    Serial.println(i + 1);

    initializeServo(i);

    // Gap before next servo
    delay(800);
  }

  Serial.println();
  Serial.println("================================");
  Serial.println("ALL SERVOS INITIALIZED");
  Serial.println("================================");
}

// =====================================================
// START MOVEMENT
// =====================================================

void startMovement()
{
  if (emergencyStop)
    return;

  bool different = false;

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    if (
      fabs(
        targetAngles[i] -
        currentAngles[i]
      ) > 0.05
    )
    {
      different = true;
      break;
    }
  }

  if (different)
  {
    moving = true;
    lastMoveTime = millis();
  }
  else
  {
    moving = false;
  }
}

// =====================================================
// UPDATE MOVEMENT
//
// TRUE SPEED CONTROL
//
// speedValue = degrees per second
//
// At 40 deg/sec:
//
// 40 × 0.020 sec = 0.8 degrees/update
//
// The servo with the largest movement travels at
// approximately 40 degrees/second.
// Other servos move proportionally so they arrive
// together.
// =====================================================

void updateMovement()
{
  if (!moving)
    return;

  if (emergencyStop)
  {
    moving = false;
    return;
  }

  unsigned long now = millis();

  unsigned long elapsed =
    now - lastMoveTime;

  if (elapsed < MOVE_INTERVAL)
    return;

  lastMoveTime = now;

  // ===================================================
  // FIND LARGEST REMAINING MOVEMENT
  // ===================================================

  float largestDifference = 0.0;

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    float difference =
      fabs(
        targetAngles[i] -
        currentAngles[i]
      );

    if (difference > largestDifference)
    {
      largestDifference = difference;
    }
  }

  // ===================================================
  // CHECK FINISHED
  // ===================================================

  if (largestDifference <= 0.05)
  {
    for (int i = 0; i < NUM_SERVOS; i++)
    {
      currentAngles[i] =
        targetAngles[i];

      writeServo(
        i,
        currentAngles[i]
      );
    }

    moving = false;

    sendPosition();

    Serial.println("STATUS|IDLE");

    return;
  }

  // ===================================================
  // TIME SINCE LAST UPDATE
  // ===================================================

  float deltaTime =
    elapsed / 1000.0;

  // ===================================================
  // MAXIMUM MOVEMENT THIS UPDATE
  //
  // Example:
  //
  // speedValue = 40 deg/sec
  // deltaTime  = 0.020 sec
  //
  // maxStep = 40 × 0.020
  //         = 0.8 degrees
  // ===================================================

  float maxStep =
    speedValue *
    deltaTime;

  if (maxStep < 0.01)
    maxStep = 0.01;

  // ===================================================
  // MOVE EACH SERVO
  // ===================================================

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    float difference =
      targetAngles[i] -
      currentAngles[i];

    float absoluteDifference =
      fabs(difference);

    if (absoluteDifference > 0.05)
    {
      // Proportional movement
      float movement =
        maxStep *
        (
          absoluteDifference /
          largestDifference
        );

      // Do not overshoot target
      if (movement > absoluteDifference)
      {
        movement =
          absoluteDifference;
      }

      if (difference > 0)
      {
        currentAngles[i] += movement;
      }
      else
      {
        currentAngles[i] -= movement;
      }

      // Apply limits
      currentAngles[i] =
        constrain(
          currentAngles[i],
          SERVO_MIN[i],
          SERVO_MAX[i]
        );

      writeServo(
        i,
        currentAngles[i]
      );
    }
  }

  // ===================================================
  // FINAL POSITION CHECK
  // ===================================================

  bool finished = true;

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    if (
      fabs(
        targetAngles[i] -
        currentAngles[i]
      ) > 0.05
    )
    {
      finished = false;
      break;
    }
  }

  if (finished)
  {
    for (int i = 0; i < NUM_SERVOS; i++)
    {
      currentAngles[i] =
        targetAngles[i];

      writeServo(
        i,
        currentAngles[i]
      );
    }

    moving = false;

    sendPosition();

    Serial.println("STATUS|IDLE");
  }
  else
  {
    Serial.println("STATUS|MOVING");
  }
}

// =====================================================
// MOVE COMMAND
//
// MOVE|S1|S2|S3|S4|S5
//
// Example:
// MOVE|90|100|120|90|80
// =====================================================

void processMoveCommand(String command)
{
  if (emergencyStop)
  {
    Serial.println("STATUS|ESTOP");
    return;
  }

  int values[NUM_SERVOS];

  int startIndex = 5;

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    int separator =
      command.indexOf(
        '|',
        startIndex
      );

    String value;

    if (separator == -1)
    {
      value =
        command.substring(
          startIndex
        );
    }
    else
    {
      value =
        command.substring(
          startIndex,
          separator
        );
    }

    values[i] =
      value.toInt();

    values[i] =
      constrain(
        values[i],
        (int)SERVO_MIN[i],
        (int)SERVO_MAX[i]
      );

    if (separator == -1)
      break;

    startIndex =
      separator + 1;
  }

  // ===================================================
  // SET TARGETS
  // ===================================================

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    targetAngles[i] =
      values[i];
  }

  startMovement();

  if (moving)
  {
    Serial.println("STATUS|MOVING");
  }
  else
  {
    Serial.println("STATUS|IDLE");
  }
}

// =====================================================
// SEND POSITION
// =====================================================

void sendPosition()
{
  Serial.print("POSITION");

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    Serial.print("|");

    Serial.print(
      currentAngles[i],
      1
    );
  }

  Serial.println();
}

// =====================================================
// GRIPPER OPEN
// =====================================================

void openGripper()
{
  digitalWrite(
    GRIPPER_PIN,
    LOW
  );

  Serial.println("GRIPPER|OPEN");
}

// =====================================================
// GRIPPER CLOSE
// =====================================================

void closeGripper()
{
  digitalWrite(
    GRIPPER_PIN,
    HIGH
  );

  Serial.println("GRIPPER|CLOSE");
}

// =====================================================
// HOME
// =====================================================

void goHome()
{
  if (emergencyStop)
    return;

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    targetAngles[i] =
      POWER_ON_ANGLES[i];
  }

  startMovement();

  if (moving)
  {
    Serial.println("STATUS|MOVING");
  }
  else
  {
    Serial.println("STATUS|IDLE");
  }
}

// =====================================================
// EMERGENCY STOP
// =====================================================

void emergencyStopRobot()
{
  emergencyStop = true;
  moving = false;

  // Hold current positions
  for (int i = 0; i < NUM_SERVOS; i++)
  {
    writeServo(
      i,
      currentAngles[i]
    );
  }

  Serial.println(
    "EMERGENCY_STOPPED"
  );

  Serial.println(
    "STATUS|ESTOP"
  );
}

// =====================================================
// RESET EMERGENCY STOP
// =====================================================

void resetEmergencyStop()
{
  emergencyStop = false;

  Serial.println(
    "ESTOP_RESET"
  );

  Serial.println(
    "STATUS|IDLE"
  );
}

// =====================================================
// SET SPEED
//
// SET_SPEED|40
//
// 40 = 40 degrees/second
//
// Allowed:
// 1 - 500 degrees/second
// =====================================================

void setSpeed(String command)
{
  int separator =
    command.indexOf('|');

  if (separator == -1)
    return;

  String value =
    command.substring(
      separator + 1
    );

  speedValue =
    value.toFloat();

  speedValue =
    constrain(
      speedValue,
      1.0,
      500.0
    );

  Serial.print("SPEED|");

  Serial.println(
    speedValue,
    1
  );
}

// =====================================================
// SERIAL COMMAND PROCESSING
// =====================================================

void processCommand(String command)
{
  command.trim();

  if (command.length() == 0)
    return;

  // ===================================================
  // PING
  // ===================================================

  if (command == "PING")
  {
    Serial.println("PONG");
  }

  // ===================================================
  // POSITION
  // ===================================================

  else if (command == "GET_POSITION")
  {
    sendPosition();
  }

  // ===================================================
  // STATUS
  // ===================================================

  else if (command == "GET_STATUS")
  {
    if (emergencyStop)
    {
      Serial.println(
        "STATUS|ESTOP"
      );
    }
    else if (moving)
    {
      Serial.println(
        "STATUS|MOVING"
      );
    }
    else
    {
      Serial.println(
        "STATUS|IDLE"
      );
    }
  }

  // ===================================================
  // HOME
  // ===================================================

  else if (command == "HOME")
  {
    goHome();
  }

  // ===================================================
  // EMERGENCY STOP
  // ===================================================

  else if (
    command ==
    "EMERGENCY_STOP"
  )
  {
    emergencyStopRobot();
  }

  // ===================================================
  // RESET E-STOP
  // ===================================================

  else if (
    command ==
    "RESET_ESTOP"
  )
  {
    resetEmergencyStop();
  }

  // ===================================================
  // SPEED
  // ===================================================

  else if (
    command.startsWith(
      "SET_SPEED|"
    )
  )
  {
    setSpeed(command);
  }

  // ===================================================
  // GRIPPER OPEN
  // ===================================================

  else if (
    command ==
    "GRIPPER|OPEN"
  )
  {
    openGripper();
  }

  // ===================================================
  // GRIPPER CLOSE
  // ===================================================

  else if (
    command ==
    "GRIPPER|CLOSE"
  )
  {
    closeGripper();
  }

  // ===================================================
  // MOVE
  // ===================================================

  else if (
    command.startsWith(
      "MOVE|"
    )
  )
  {
    processMoveCommand(
      command
    );
  }

  // ===================================================
  // SET MODE
  // ===================================================

  else if (
    command.startsWith(
      "SET_MODE|"
    )
  )
  {
    Serial.println(
      "MODE|SMOOTH"
    );
  }

  // ===================================================
  // DEBUG ON
  // ===================================================

  else if (
    command ==
    "DEBUG_ON"
  )
  {
    Serial.println(
      "DEBUG|ON"
    );
  }

  // ===================================================
  // DEBUG OFF
  // ===================================================

  else if (
    command ==
    "DEBUG_OFF"
  )
  {
    Serial.println(
      "DEBUG|OFF"
    );
  }

  // ===================================================
  // UNKNOWN COMMAND
  // ===================================================

  else
  {
    Serial.print(
      "ERROR|UNKNOWN_COMMAND|"
    );

    Serial.println(
      command
    );
  }
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // Allow USB serial to initialize
  delay(2000);

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "ROBOTIC ARM ESP32 STARTING"
  );

  Serial.println(
    "================================"
  );

  Serial.println(
    "Motion speed: 40 deg/sec"
  );

  // ===================================================
  // GRIPPER
  // ===================================================

  pinMode(
    GRIPPER_PIN,
    OUTPUT
  );

  // Gripper OPEN initially
  digitalWrite(
    GRIPPER_PIN,
    LOW
  );

  Serial.println(
    "Gripper initialized"
  );

  delay(500);

  // ===================================================
  // SERVOS
  // ONE AT A TIME
  // ===================================================

  initializeAllServos();

  // ===================================================
  // INITIAL STATE
  // ===================================================

  moving = false;
  emergencyStop = false;

  // ===================================================
  // SEND POWER-ON POSITION
  // ===================================================

  Serial.println();

  Serial.println(
    "POSITION|0|110|125|90|80"
  );

  Serial.println(
    "STATUS|IDLE"
  );

  Serial.println();

  Serial.println(
    "================================"
  );

  Serial.println(
    "ROBOT READY"
  );

  Serial.println(
    "Speed = 40 degrees/sec"
  );

  Serial.println(
    "================================"
  );
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  // ===================================================
  // SERIAL
  // ===================================================

  if (Serial.available())
  {
    String command =
      Serial.readStringUntil('\n');

    processCommand(
      command
    );
  }

  // ===================================================
  // MOVEMENT
  // ===================================================

  updateMovement();
}
void loop() {
  // put your main code here, to run repeatedly:

}
