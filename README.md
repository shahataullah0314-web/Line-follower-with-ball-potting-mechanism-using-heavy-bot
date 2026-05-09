# Arduino Line Follower Robot with Ultrasonic & ESP32 Detection

This project is a complete Arduino robot system designed for line following, obstacle detection, and automatic ball collection. It uses PID control and five IR sensors to follow a black line, while an HC-SR04 ultrasonic sensor and an ESP32 signal input independently detect obstacles or balls. When an obstacle is confirmed, the robot stops, opens a servo to collect the object, and then resumes line following.

## Key Features
- Smooth PID line tracking using 5 IR sensors
- Ultrasonic obstacle detection with fast debounce and confirmation
- ESP32 signal input for external ball detection
- Sequential servo opening for up to 3 objects
- Automatic closing sequence after all objects are collected
- Non-blocking state machine for reliable robot behavior
- Serial debug output at 115200 baud

## Hardware Overview
- Arduino Uno/Nano
- 5x IR sensors for line detection (A0 to A4)
- L298N motor driver with 2 DC motors
- 3x servo motors for collection mechanism
- ESP32 trigger input on pin 12
- HC-SR04 ultrasonic sensor on pins 13 and A5
- Standard 5V component power and shared ground

## Operation Summary
The robot normally follows a line at a base speed of 175. When the ultrasonic sensor sees an obstacle closer than 15 cm or the ESP32 input goes LOW, it stops immediately, opens the next servo in sequence, waits for the action to complete, and then resumes line following. After three detections, the robot continues following the track while the servos close one after another on a timed schedule.

## Watch Live here
https://drive.google.com/file/d/1iC59G38f7Q4IUBOwAT9tK1-CKyfLHbj2/view?usp=sharing

## Pin Configuration
- IR sensors: A0, A1, A2, A3, A4
- Motor ENA: 6
- Motor IN1: 9
- Motor IN2: 7
- Motor ENB: 11
- Motor IN3: 8
- Motor IN4: 4
- ESP32 signal: 12
- Servo 1: 2
- Servo 2: 3
- Servo 3: 5
- Ultrasonic TRIG: 13
- Ultrasonic ECHO: A5

## Installation
### PlatformIO
1. Open the project in VS Code with PlatformIO
2. Build with `platformio run`
3. Upload with `platformio run --target upload`

### Arduino IDE
1. Open `main.ino`
2. Select the correct board and port
3. Upload the sketch

## Notes
- Adjust PID values if the robot oscillates or drifts
- Tune `OBSTACLE_DISTANCE` and ultrasonic debounce if the robot is too sensitive
- Verify servo angles and power supply before testing

## License
This project is released under the MIT License.

## 350-Word Description
A full 350-word project description is available in `DESCRIPTION.md` for direct use in GitHub or project documentation.
