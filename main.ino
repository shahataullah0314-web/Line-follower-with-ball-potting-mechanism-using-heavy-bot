#include <Arduino.h>
#include <Servo.h>

// --- Pin Definitions ---
#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4

#define ENA  6
#define IN1  9
#define IN2  7
#define ENB  11
#define IN3  8
#define IN4  4

#define ESP32_SIGNAL_PIN 12

#define SERVO1_PIN 2
#define SERVO2_PIN 3
#define SERVO3_PIN 5

// --- Ultrasonic Sensor Pins ---
#define TRIG_PIN 13
#define ECHO_PIN A5

// --- PID Constants ---
float Kp = 9.0;
float Ki = 0.1;
float Kd = 20.0;

// --- Speed Settings ---
int baseSpeed   = 175;
int maxSpeed    = 175;
int minSpeed    = -190;
int pivotSpeed  = 180;

// --- Ultrasonic Settings ---
const int OBSTACLE_DISTANCE = 15;  // cm threshold for obstacle detection
unsigned long lastUltrasonicTime = 0;
const unsigned long ULTRASONIC_INTERVAL = 50;
bool ultrasonicTriggered = false;
unsigned long ultrasonicDebounceTime = 0;
const int ULTRASONIC_DEBOUNCE = 100;

// --- Global Variables ---
float prevError = 0;
float lastError = 0;
bool debugMode  = true;

// --- Obstacle and Servo Settings ---
int obstacleCount = 0;
unsigned long lastObstacleTime = 0;
unsigned long lastPrintTime = 0;

const int COOLDOWN_TIME = 3000;
const int PRINT_INTERVAL = 200;

bool servo1Opened = false;
bool servo2Opened = false;
bool servo3Opened = false;

bool allBallsPotted = false;

// --- Ball Processing State Machine ---
enum BallState {
  BALL_IDLE,
  BALL_STOPPING,
  BALL_OPENING_SERVO,
  BALL_WAITING_RESUME
};
BallState ballState = BALL_IDLE;
unsigned long ballStateStart = 0;
int pendingServo = 0;

// --- Closing Sequence ---
enum CloseState {
  CLOSE_IDLE,
  CLOSE_WAIT_S1,
  CLOSE_WAIT_S2,
  CLOSE_WAIT_S3,
  CLOSE_DONE
};
CloseState closeState = CLOSE_IDLE;
unsigned long closeTimer = 0;

Servo servo1, servo2, servo3;

int s1_open = 0,   s1_closed = 90;
int s2_open = 110, s2_closed = 170;
int s3_open = 0,   s3_closed = 40;

void driveMotors(int leftSpeed, int rightSpeed);
void pivotRight();
void pivotLeft();
void runPID(int s1, int s2, int s3, int s4, int s5);
bool lineDetected(int s1, int s2, int s3, int s4, int s5);
void updateBallState();
void updateClosingSequence();
void openServo(int num);
void closeServo(int num);
float getUltrasonicDistance();
void checkUltrasonicForObstacle();

void setup() {
  Serial.begin(115200);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ESP32_SIGNAL_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);

  servo1.write(s1_closed);
  servo2.write(s2_closed);
  servo3.write(s3_closed);

  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  Serial.println("=================================");
  Serial.println(" Line Follower + Ultrasonic & ESP32");
  Serial.println(" Obstacle detection opens servo");
  Serial.println(" Then resumes line following");
  Serial.println("=================================");
  delay(1000);
}

void loop() {
  int s1 = digitalRead(IR1);
  int s2 = digitalRead(IR2);
  int s3 = digitalRead(IR3);
  int s4 = digitalRead(IR4);
  int s5 = digitalRead(IR5);
  int esp32Signal = digitalRead(ESP32_SIGNAL_PIN);

  checkUltrasonicForObstacle();

  if (closeState != CLOSE_IDLE) {
    updateClosingSequence();
  }

  if (ballState != BALL_IDLE) {
    updateBallState();
    return;
  }

  if (!lineDetected(s1, s2, s3, s4, s5)) {
    if (lastError < 0) pivotLeft();
    else               pivotRight();
  } else {
    runPID(s1, s2, s3, s4, s5);
  }

  if (esp32Signal == LOW && obstacleCount < 3 &&
      millis() - lastObstacleTime > COOLDOWN_TIME &&
      ballState == BALL_IDLE && !allBallsPotted) {

    obstacleCount++;
    pendingServo = obstacleCount;
    Serial.print("\n*** BALL "); Serial.print(pendingServo); Serial.println(" DETECTED by ESP32! ***");

    ballState = BALL_STOPPING;
    ballStateStart = millis();
    driveMotors(0, 0);
  }

  if (debugMode && millis() - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = millis();
    float ultrasonicDist = getUltrasonicDistance();

    Serial.print("S: ");
    Serial.print(s1); Serial.print(s2); Serial.print(s3); Serial.print(s4); Serial.print(s5);
    Serial.print(" | ESP: "); Serial.print(esp32Signal == LOW ? "LOW" : "HIGH");
    Serial.print(" | US: ");
    if (ultrasonicDist > 0 && ultrasonicDist < 400) {
      Serial.print(ultrasonicDist);
      Serial.print("cm");
    } else {
      Serial.print("---");
    }
    Serial.print(" | Balls: "); Serial.print(obstacleCount); Serial.print("/3");
    Serial.print(" | Line: "); Serial.print(lineDetected(s1,s2,s3,s4,s5) ? "YES" : "NO");
    Serial.print(" | Srv: [");
    Serial.print(servo1Opened ? "1O" : "1C"); Serial.print("][");
    Serial.print(servo2Opened ? "2O" : "2C"); Serial.print("][");
    Serial.print(servo3Opened ? "3O" : "3C"); Serial.print("]");
    if (closeState != CLOSE_IDLE) {
      Serial.print(" | CloseState: ");
      switch(closeState) {
        case CLOSE_WAIT_S1: Serial.print("WAIT_S1(30s)"); break;
        case CLOSE_WAIT_S2: Serial.print("WAIT_S2(10s)"); break;
        case CLOSE_WAIT_S3: Serial.print("WAIT_S3(10s)"); break;
        case CLOSE_DONE: Serial.print("DONE"); break;
        default: Serial.print(closeState); break;
      }
    }
    Serial.println();
  }
}

void checkUltrasonicForObstacle() {
  if (ballState != BALL_IDLE || allBallsPotted) return;
  if (obstacleCount >= 3) return;
  if (millis() - lastUltrasonicTime < ULTRASONIC_INTERVAL) return;

  lastUltrasonicTime = millis();
  float distance = getUltrasonicDistance();

  if (distance > 0 && distance < OBSTACLE_DISTANCE) {
    if (!ultrasonicTriggered) {
      ultrasonicTriggered = true;
      ultrasonicDebounceTime = millis();
    } else if (millis() - ultrasonicDebounceTime >= ULTRASONIC_DEBOUNCE) {
      ultrasonicTriggered = false;
      obstacleCount++;
      pendingServo = obstacleCount;
      Serial.print("\n*** OBSTACLE DETECTED by ULTRASONIC! Distance: ");
      Serial.print(distance);
      Serial.print("cm - Opening Servo ");
      Serial.println(pendingServo);
      ballState = BALL_STOPPING;
      ballStateStart = millis();
      driveMotors(0, 0);
    }
  } else {
    ultrasonicTriggered = false;
  }
}

float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.034 / 2;
  return distance;
}

void updateBallState() {
  unsigned long elapsed = millis() - ballStateStart;

  switch (ballState) {
    case BALL_STOPPING:
      driveMotors(0, 0);
      if (elapsed >= 200) {
        ballState = BALL_OPENING_SERVO;
        ballStateStart = millis();
        openServo(pendingServo);
      }
      break;

    case BALL_OPENING_SERVO:
      driveMotors(0, 0);
      if (elapsed >= 500) {
        ballState = BALL_WAITING_RESUME;
        ballStateStart = millis();
        Serial.print("Servo "); Serial.print(pendingServo); Serial.println(" opened!");
      }
      break;

    case BALL_WAITING_RESUME:
      driveMotors(0, 0);
      if (elapsed >= 300) {
        ballState = BALL_IDLE;
        lastObstacleTime = millis();
        if (obstacleCount >= 3 && closeState == CLOSE_IDLE) {
          Serial.println("\n*** ALL 3 OBSTACLES/BALLS COLLECTED! ***");
          Serial.println("Robot will CONTINUE line following");
          Serial.println("Servos will close in 30 seconds...");
          closeState = CLOSE_WAIT_S1;
          closeTimer = millis();
          allBallsPotted = true;
        }
        Serial.println("Resuming line following...\n");
      }
      break;

    default:
      ballState = BALL_IDLE;
      break;
  }
}

void updateClosingSequence() {
  unsigned long elapsed = millis() - closeTimer;

  switch (closeState) {
    case CLOSE_WAIT_S1:
      if (elapsed >= 30000) {
        closeServo(1);
        closeState = CLOSE_WAIT_S2;
        closeTimer = millis();
        Serial.println(">>> Servo 1 CLOSED (after 30 seconds)");
      }
      break;
    case CLOSE_WAIT_S2:
      if (elapsed >= 10000) {
        closeServo(2);
        closeState = CLOSE_WAIT_S3;
        closeTimer = millis();
        Serial.println(">>> Servo 2 CLOSED (after 10 seconds)");
      }
      break;
    case CLOSE_WAIT_S3:
      if (elapsed >= 10000) {
        closeServo(3);
        closeState = CLOSE_DONE;
        Serial.println(">>> Servo 3 CLOSED (after 10 seconds)");
        Serial.println("*** ALL SERVOS CLOSED - MISSION COMPLETE ***");
      }
      break;
    case CLOSE_DONE:
      break;
    default:
      closeState = CLOSE_IDLE;
      break;
  }
}

void openServo(int num) {
  if (num == 1 && !servo1Opened) {
    Serial.println("Opening Servo 1...");
    servo1.write(s1_open);
    servo1Opened = true;
  } else if (num == 2 && !servo2Opened) {
    Serial.println("Opening Servo 2...");
    servo2.write(s2_open);
    servo2Opened = true;
  } else if (num == 3 && !servo3Opened) {
    Serial.println("Opening Servo 3...");
    servo3.write(s3_open);
    servo3Opened = true;
  }
}

void closeServo(int num) {
  if (num == 1 && servo1Opened) {
    Serial.println("Closing Servo 1...");
    servo1.write(s1_closed);
    servo1Opened = false;
  } else if (num == 2 && servo2Opened) {
    Serial.println("Closing Servo 2...");
    servo2.write(s2_closed);
    servo2Opened = false;
  } else if (num == 3 && servo3Opened) {
    Serial.println("Closing Servo 3...");
    servo3.write(s3_closed);
    servo3Opened = false;
  }
}

bool lineDetected(int s1, int s2, int s3, int s4, int s5) {
  return (s1 == 0 || s2 == 0 || s3 == 0 || s4 == 0 || s5 == 0);
}

void runPID(int s1, int s2, int s3, int s4, int s5) {
  int v1 = 1 - s1;
  int v2 = 1 - s2;
  int v3 = 1 - s3;
  int v4 = 1 - s4;
  int v5 = 1 - s5;

  float weightedSum = (-5 * v1) + (-2 * v2) + (0 * v3) + (2 * v4) + (5 * v5);
  int sensorsOnLine = v1 + v2 + v3 + v4 + v5;

  float error = 0;
  if (sensorsOnLine > 0) {
    error = weightedSum / sensorsOnLine;
    lastError = error;
  }

  float derivative = error - prevError;
  float correction = (Kp * error) + (Kd * derivative);
  prevError = error;

  int leftSpeed  = constrain(baseSpeed + (int)correction, minSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed - (int)correction, minSpeed, maxSpeed);

  driveMotors(leftSpeed, rightSpeed);
}

void driveMotors(int left, int right) {
  if (left >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    left = -left;
  }

  if (right >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    right = -right;
  }

  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  analogWrite(ENA, left);
  analogWrite(ENB, right);
}

void pivotRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, pivotSpeed);
  analogWrite(ENB, pivotSpeed);
}

void pivotLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, pivotSpeed);
  analogWrite(ENB, pivotSpeed);
}
