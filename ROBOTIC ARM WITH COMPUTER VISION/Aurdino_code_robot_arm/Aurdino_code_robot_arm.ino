#include <Servo.h>

// Servo objects
Servo shoulder1, shoulder2, elbow1, elbow2, wrist;
Servo thumb, indexFinger, middleFinger, ringFinger, pinkyFinger;

// Pins for DC motor
const int enablePin = 12;
const int input1 = 13;
const int input2 = A5;

void setup() {
  Serial.begin(9600);  // Start serial communication

  // Attach servo motors
  shoulder1.attach(9);
  shoulder2.attach(10);
  elbow1.attach(7);
  elbow2.attach(8);
  wrist.attach(11);
  thumb.attach(2);
  indexFinger.attach(3);
  middleFinger.attach(4);
  ringFinger.attach(5);
  pinkyFinger.attach(6);

  // Set up DC motor pins
  pinMode(enablePin, OUTPUT);
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);

  // Initial position
  moveToInitialPosition();
}

void moveToInitialPosition() {
  shoulder1.write(130);
  shoulder2.write(130);
  elbow1.write(130);
  elbow2.write(130);
  wrist.write(130);
  thumb.write(0);
  indexFinger.write(0);
  middleFinger.write(0);
  ringFinger.write(0);
  pinkyFinger.write(0);
}

void gripObject() {
  thumb.write(180);
  indexFinger.write(180);
  middleFinger.write(180);
  ringFinger.write(180);
  pinkyFinger.write(180);
}

void releaseObject() {
  thumb.write(0);
  indexFinger.write(0);
  middleFinger.write(0);
  ringFinger.write(0);
  pinkyFinger.write(0);
}

void rotateBaseClockwise(int duration) {
  digitalWrite(input1, HIGH);
  digitalWrite(input2, LOW);
  delay(duration);
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readString();

    if (command.startsWith("GRIP")) {
      gripObject();
    } else if (command.startsWith("RESET")) {
      moveToInitialPosition();
    } else if (command.startsWith("ROTATE")) {
      rotateBaseClockwise(1000); // Rotate for 1 second
      releaseObject();
    }
  }
}
