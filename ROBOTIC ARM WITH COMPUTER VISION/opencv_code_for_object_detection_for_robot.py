import cv2
import serial
from ultralytics import YOLO
import time

# Arduino Serial Port (Adjust COM port as needed)
arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)
time.sleep(2)  # Wait for the connection to establish

# Load the YOLOv8 model
model = YOLO(r"C:\Users\91630\PycharmProjects\PythonProject1\best (2).pt")

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
