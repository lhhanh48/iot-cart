from paho.mqtt import client as mqtt_client
import sys
import cv2
import imutils
from yoloDet import YoloTRT
import time
from collections import deque
import Jetson.GPIO as GPIO
import serial
import serial.tools.list_ports

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
buzz_pin = 18  # Board pin 12
but_pin = 7  # Board pin 18
led_status = 12
GPIO.setmode(GPIO.BOARD)
GPIO.setup(buzz_pin, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(led_status, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(but_pin, GPIO.IN)  # button pin set as input

def buzz():
    GPIO.output(buzz_pin, GPIO.LOW)
    time.sleep(0.05)
    GPIO.output(buzz_pin, GPIO.HIGH)
    time.sleep(0.05)

# Barcode Scanner Function
def barcode_scanner(client):
    GPIO.output(led_status, GPIO.LOW)
    print("Barcode Scanner Function")
    ports = serial.tools.list_ports.comports()
    serialInst = serial.Serial()
    serialInst.baudrate = 9600
    serialInst.port = "/dev/ttyACM0"
    serialInst.open()
    #root = tk.Tk()
    #root.title("Log Viewer")

    while True:
        if serialInst.in_waiting:
            packet = serialInst.readline()
            decoded_packet = packet.decode('utf-8').rstrip('\n')
            print(decoded_packet)
            #display_logs(decoded_packet, root, old_log)
            # Send packet to MQTT broker
            client.publish(MQTT_TOPIC_BARCODE, decoded_packet)
         # Check for button press to exit
        if GPIO.input(but_pin) == GPIO.LOW:
            break
            
# def update_logs(text_widget, logs):
#     # Example: Update logs every second with the current time
#     while True:
#         text_widget.config(state="normal")
#         text_widget.insert(tk.END, current_log + "\n")
#         text_widget.config(state="disabled")
#         text_widget.see(tk.END)  # Scroll to the end
#         old_log = current_log  
            
# def display_logs(logs, root, old_log):

#     text = tk.Text(root)
#     text.pack(expand=True, fill="both")
#     text.configure(state="disabled")
    
#     # Start a thread to update the logs
#     update_thread = threading.Thread(target=update_logs, args=(text,logs))
#     update_thread.daemon = True  # Daemonize the thread
#     update_thread.start()

#     root.mainloop()



def object_detection(client):
    # YOLO Model Configurations
    print("Object Detection Function")
    model = YoloTRT(library="yolov5/build/libmyplugins.so", engine="yolov5/build/yolov5s.engine", conf=0.5, yolo_ver="v5")

    # Setup Video Capture
    cap = cv2.VideoCapture(0)

    previous_positions = {}
    movement_states = {}
    position_history = {obj_class: deque(maxlen=5) for obj_class in class_to_barcode}

    while True:
        ret, frame = cap.read()
        frame = imutils.resize(frame, width=600)
        detections, t = model.Inference(frame)

        current_positions = {}

        for obj in detections:
            obj_class = obj["class"]
            if 'box' in obj:
                bbox = obj['box']
                x, y, w, h = bbox
                center_x = int((x + w) / 2)
                center_y = int((y + h) / 2)

                current_positions[obj_class] = (center_x, center_y)
                position_history[obj_class].append((center_x, center_y))

                if obj_class in class_to_barcode:
                    if len(position_history[obj_class]) == position_history[obj_class].maxlen:
                        prev_positions = list(position_history[obj_class])
                        prev_x_positions = [pos[0] for pos in prev_positions]
                        prev_y_positions = [pos[1] for pos in prev_positions]

                        # Check for vertical movements only (ignoring horizontal movements)
                        if all(prev_y_positions[i] < prev_y_positions[i+1] for i in range(len(prev_y_positions) - 1)):  # Moving down
                            if movement_states.get(obj_class) != 'down':
                                barcode = class_to_barcode[obj_class]
                                client.publish(MQTT_TOPIC_BARCODE, barcode)
                                movement_states[obj_class] = 'down'
                                buzz()
                        elif all(prev_y_positions[i] > prev_y_positions[i+1] for i in range(len(prev_y_positions) - 1)):  # Moving up
                            if movement_states.get(obj_class) != 'up':
                                barcode = class_to_barcode[obj_class]
                                client.publish(MQTT_TOPIC_REMOVE, barcode)
                                movement_states[obj_class] = 'up'
                                buzz()
                        else:
                            movement_states[obj_class] = 'stationary'

                        # Reset history if horizontal movement detected to avoid false positives
                        if any(prev_x_positions[i] != prev_x_positions[i+1] for i in range(len(prev_x_positions) - 1)):
                            position_history[obj_class].clear()

                    # Draw a crosshair at the center of the bounding box
                    cv2.line(frame, (center_x - 5, center_y), (center_x + 5, center_y), (0, 0, 255), 2)
                    cv2.line(frame, (center_x, center_y - 5), (center_x, center_y + 5), (0, 0, 255), 2)

            else:
                print(f"box not found in object: {obj}")

        previous_positions = current_positions.copy()

        # Display the output frame
        cv2.imshow("Output", frame)
        key = cv2.waitKey(1)
        if GPIO.input(but_pin) == GPIO.LOW or key == ord('q'):  # Add button press to exit condition
            break
        time.sleep(0.01)
    cap.release()
    cv2.destroyAllWindows()
    #GPIO.cleanup()  # Clean up GPIO pins

def main():
    # Setup MQTT Client
    client = connect_mqtt()
    client.loop_start()

    print("Starting demo now! Press CTRL+C to exit")
    mode = "barcode_scanner"  # Initial mode

    try:
        while True:
            print("Waiting for button event")
            GPIO.wait_for_edge(but_pin, GPIO.FALLING)
            # event received when button pressed
            buzz()
            if mode == "barcode_scanner":
               barcode_scanner(client)  # Pass client to barcode_scanner
               mode = "object_detection"

            elif mode == "object_detection":
               object_detection(client)  # Pass client to barcode_scanner
               mode = "barcode_scanner"
    finally:
        GPIO.cleanup()  # cleanup all GPIOs

if __name__ == "__main__":
    print("MQTT to InfluxDB bridge")
    main()
