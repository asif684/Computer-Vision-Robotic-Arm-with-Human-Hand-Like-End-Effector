# Object Detection and Robotic Arm Control with YOLO and Arduino

This project integrates computer vision with a robotic arm controlled by an Arduino. It uses the YOLOv8 object detection model to identify objects in the camera feed, and based on the detected objects, it sends commands to the Arduino to control a robotic arm. The arm can grip and move objects using servo motors and a DC motor for rotation.

## Requirements

- Python 3.x
- OpenCV (`cv2` library)
- `ultralytics` YOLOv8
- `pyserial` library for serial communication with Arduino
- Arduino IDE
- Arduino Uno (or any compatible Arduino board)
- Servo motors for the robotic arm
- DC motor for rotation
- Webcam for object detection

## Setup

### 1. Python Environment
To run the Python script, you need to install the following libraries:

```bash
pip install opencv-python pyserial ultralytics
```

### 2. Arduino Code
Upload the provided Arduino code to your Arduino Uno using the Arduino IDE. This code controls the servo motors for the robotic arm and interprets commands received via serial communication from the Python script.

### 3. Hardware Setup
- **Servo Motors**: Connect the servo motors to the appropriate pins on the Arduino as specified in the code (pins 2-11 for fingers and joints).
- **DC Motor**: Connect the DC motor to the motor driver (L293D) as specified in the Arduino code.
- **Webcam**: Ensure your webcam is working and connected to the computer.

### 4. Python Script Configuration
Modify the Python script to ensure it points to the correct paths and settings:

- **YOLOv8 Model Path**: Update the path of the YOLO model file (`best (2).pt`) in the Python code to point to the actual location of your trained model.
- **Serial Port**: Adjust the `COM` port in the Python script to match the port where your Arduino is connected (e.g., `COM3` for Windows, `/dev/ttyACM0` for Linux).

### 5. Running the Program
1. **Start the Arduino**: Ensure that the Arduino is powered and the code is uploaded.
2. **Run the Python Script**: Execute the Python script. It will start the webcam and use YOLOv8 for object detection.
3. **Object Detection**: When an object is detected with a confidence greater than the set threshold, the robotic arm will grip the object.
4. **Commands to Arduino**: The Python script sends commands to the Arduino to control the robotic arm based on the detected object, such as gripping and rotating the base.

### 6. Control Flow
- **Detection**: The Python script captures frames from the webcam and passes them through the YOLOv8 model to detect objects.
- **Gripping**: If an object is detected with a confidence greater than the threshold, the Python script sends a "GRIP" command to the Arduino.
- **Reset and Rotate**: After gripping, the Python script sends a "RESET" command to return the arm to its initial position and a "ROTATE" command to rotate the base.

### Python Script (Detection and Control)
```python
import cv2
import serial
from ultralytics import YOLO
import time

# Arduino Serial Port (Adjust COM port as needed)
arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)
time.sleep(2)  # Wait for the connection to establish

# Load the YOLOv8 model
model = YOLO(r"C:\path\to\your\best_model.pt")

# Open the webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Detection threshold
CONFIDENCE_THRESHOLD = 0.5

# Function to send commands to Arduino
def send_command(command):
    try:
        arduino.write(command.encode())
        print(f"Sent to Arduino: {command}")
    except Exception as e:
        print(f"Error sending command: {e}")

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to read from webcam.")
        break

    # Run YOLO object detection
    results = model(frame)

    # Parse detection results
    detected = False
    for result in results:
        for box in result.boxes:
            confidence = box.conf.item()  # Confidence score
            if confidence > CONFIDENCE_THRESHOLD:
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0].tolist())
                detected = True

                # Draw bounding box on the frame
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                label = f"Object Detected: {confidence:.2f}"
                cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Send command to Arduino to grip the object
                send_command("GRIP")  # Custom command for gripping

    if detected:
        # Wait for a moment to ensure the Arduino executes the commands
        time.sleep(5)

        # Send the command to move back to the initial position
        send_command("RESET")

        # Wait and send the command to rotate the base
        time.sleep(2)
        send_command("ROTATE")

        # Break the loop after one complete cycle
        break

    # Display the frame with detections
    cv2.imshow("Object Detection", frame)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
arduino.close()
```

### Arduino Code (Motor Control)
```cpp
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
```

## Conclusion

This project provides a hands-on implementation of integrating object detection with robotic arm control. The Python script enables the arm to interact with objects in real-time, sending commands based on YOLOv8 detections, while the Arduino code governs the movement of the robotic arm.
