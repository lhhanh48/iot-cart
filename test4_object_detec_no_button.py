from paho.mqtt import client as mqtt_client
import sys
import cv2
import imutils
from yoloDet import YoloTRT
import time
from collections import deque
import Jetson.GPIO as GPIO

# MQTT Configurations
MQTT_ADDRESS = "jetson-nano.local"
MQTT_USER = "ssc"
MQTT_PASSWORD = "12345678"
MQTT_TOPIC_BARCODE = "xe1/barcode"
MQTT_TOPIC_REMOVE = "xe1/remove"

# Dictionary mapping classes to barcodes
class_to_barcode = {
    "brush": "8934839132903",
    "butter": "8936035100397",
    "candy": "8934564100314",
    "chocopie": "8936036025156",
    "milk": "8936127794206",
    "noodle": "8934563651138",
    "panda": "8888077140001",
    "sprite": "8935049501718",
    "tissue": "8934964120608",
    "tuna-can": "8934572034021"
}

# MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print("Failed to connect, return code", rc)

def connect_mqtt():
    client = mqtt_client.Client()
    client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.connect(MQTT_ADDRESS, 1883)
    return client


# GPIO Configuration
led_pin = 18
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.HIGH)

def blink_led():
    GPIO.output(led_pin, GPIO.LOW)
    time.sleep(0.05)
    GPIO.output(led_pin, GPIO.HIGH)
    time.sleep(0.05)

def object_detection(client):  # Add client as a parameter
    # YOLO Model Configurations
    model = YoloTRT(library="yolov5/build/libmyplugins.so", engine="yolov5/build/yolov5s.engine", conf=0.5, yolo_ver="v5")

    # Setup Video Capture
    cap = cv2.VideoCapture(0)

    previous_positions = {}
    movement_states = {}  # Track movement state of each class (down, up, stationary)
    position_history = {obj_class: deque(maxlen=5) for obj_class in class_to_barcode}  # Track position history

    while True:
        ret, frame = cap.read()
        frame = imutils.resize(frame, width=600)
        detections, t = model.Inference(frame)

        current_positions = {}

        # Check each detected object
        for obj in detections:
            obj_class = obj["class"]
            obj_conf = obj["conf"]
            if 'box' in obj and obj_conf >= 0.9:
                bbox = obj['box']
                x, y, w, h = bbox
                center_x = int((x + w) / 2)
                center_y = int((y + h) / 2)

                current_positions[obj_class] = (center_x, center_y)
                position_history[obj_class].append(center_y)
                print(f"current_positions: {current_positions[obj_class]}")
                print(f"position_history: {position_history[obj_class]}")

                if obj_class in class_to_barcode and len(position_history[obj_class]) == position_history[obj_class].maxlen:
                    prev_positions = list(position_history[obj_class])
                    if all(prev_positions[i] < prev_positions[i+1] for i in range(len(prev_positions) - 1)):  # Moving down
                        if movement_states.get(obj_class) != 'down':
                            barcode = class_to_barcode[obj_class]
                            client.publish(MQTT_TOPIC_BARCODE, barcode)
                            movement_states[obj_class] = 'down'
                            blink_led()  # Blink LED when barcode is sent
                    elif all(prev_positions[i] > prev_positions[i+1] for i in range(len(prev_positions) - 1)):  # Moving up
                        if movement_states.get(obj_class) != 'up':
                            barcode = class_to_barcode[obj_class]
                            client.publish(MQTT_TOPIC_REMOVE, barcode)
                            movement_states[obj_class] = 'up'
                            blink_led()  # Blink LED when remove signal is sent
                    else:
                        movement_states[obj_class] = 'stationary'

                    # Draw a crosshair at the center of the bounding box
                    cv2.line(frame, (center_x - 5, center_y), (center_x + 5, center_y), (0, 0, 255), 2)
                    cv2.line(frame, (center_x, center_y - 5), (center_x, center_y + 5), (0, 0, 255), 2)
            #else:
                #print(f"box not found in object: {obj}")  # Debugging: Print warning if 'box' not found


        # Update previous_positions
        previous_positions = current_positions.copy()

        # Display the output frame
        cv2.imshow("Output", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()  # Clean up GPIO pins

def main():
    # Setup MQTT Client
    client = connect_mqtt()
    client.loop_start()
    object_detection(client)  # Pass client to object_detection

if __name__ == "__main__":
    print("MQTT to InfluxDB bridge")
    main()
