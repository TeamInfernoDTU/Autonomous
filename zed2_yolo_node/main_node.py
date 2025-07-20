import rclpy
from rclpy.node import Node
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge
from .arrowFunc import arrow_detect
import serial
import time
import serial.tools.list_ports

class SerialPortChecker:
    def __init__(self, baud_rate, timeout):
        self.baud_rate = baud_rate
        self.timeout = timeout

    def list_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports if "USB" in port.description]

    def find_port(self, keyword):
        for port in self.list_serial_ports():
            try:
                with serial.Serial(port, self.baud_rate, timeout=self.timeout) as ser:
                    print(f"Checking {port}...")
                    ser.flushInput()
                    ser.flushOutput()
                    for _ in range(10):
                        data = ser.readline().decode('utf-8').strip()
                        if data.lower().startswith(keyword):
                            print(f"SENSOR detected on {port}!")
                            return port
            except Exception as e:
                print(f"Error checking port {port}: {e}")
        print("No port found.")
        exit()

def crop_yolo_bbox(image, bbox):
    if bbox:
        x1, y1, x2, y2 = map(int, bbox.xyxy[0])
        cropped_image = image[y1:y2, x1:x2]
        return cropped_image
    return image

def read_gps_data(diff):
    try:
        with serial.Serial('/dev/ttyUSB1', 9600, timeout=1) as ser:
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').split(',')
                    gps_data = line[1:3]
                    gps_line = f"Latitude: {gps_data[0]} , Longitude: {gps_data[1]}"
                    print(gps_line)
                    with open('gps.txt', 'a') as file:
                        dir = "LEFT" if diff < 0 else "RIGHT"
                        file.write(f"\n################# ARROW {read_gps_data.arrow_number} : {dir} #################\n")
                        file.write(f"{gps_line}\n")
                    read_gps_data.arrow_number += 1
                    return
                time.sleep(0.01)
    except serial.SerialException as e:
        print(f"Serial connection error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Unexpected error: {e}")

read_gps_data.arrow_number = 1

def read_imu_data():
    try:
        with serial.Serial('/dev/ttyUSB1', 9600, timeout=1) as ser:
            print("Connected to MPU6050. Reading IMU data...")
            current_angle = 0
            last_time = time.time()
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore')
                    line = line.split(',')
                    imu_data = line[-1]
                    gz = float(imu_data)
                    if gz:
                        current_time = time.time()
                        delta_time = current_time - last_time
                        last_time = current_time
                        if abs(gz) > 1:
                            current_angle += gz * delta_time
                        print(f"Current Angle: {current_angle:.2f}°")
                        if abs(current_angle) >= 89:
                            print("90-degree turn detected!")
                            return "90-degree turn detected!"
                time.sleep(0.01)
    except serial.SerialException as e:
        print(f"Serial connection error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Unexpected error: {e}")

def depth_calculation(color_frame, depth_frame, x1, y1, x2, y2): 
    color_to_depth_x = depth_frame.shape[1] / color_frame.shape[1]
    color_to_depth_y = depth_frame.shape[0] / color_frame.shape[0]

    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    depth_x = int(center_x * color_to_depth_x)
    depth_y = int(center_y * color_to_depth_y)

    if 0 <= depth_x < depth_frame.shape[1] and 0 <= depth_y < depth_frame.shape[0]:
        depth_value = depth_frame[depth_y, depth_x]
        if depth_value > 0.0:
            return int(depth_value * 1000)  # Convert meters to mm
    return None

class Zed2YoloNode(Node):
    def __init__(self):
        super().__init__('zed2_yolo_node')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Joy, '/joy', 10)

        self.subscription_color = self.create_subscription(
            Image,
            '/zed2/zed_node/rgb/image_rect_color',
            self.color_callback,
            10)
        self.subscription_depth = self.create_subscription(
            Image,
            '/zed2/zed_node/depth/depth_registered',
            self.depth_callback,
            10)

        self.speed = 1.0
        self.model = YOLO("9s15epoch.pt")
        self.cone_model = YOLO("cone2.pt")

        self.cached_results = None
        self.update_interval = 5
        self.frame_count = 0
        self.depth_frame = None
        self.color_frame = None
        self.msg = Joy()
        self.msg.axes = [0.0, 0.0]
        self.window_center = 0
        self.x_center = 0
        self.turnin = False
        self.arrow_number = 1

        with open("gps.txt", "w") as file:
            pass

        self.LR_counter = [0, 0]
        self.outer_offset = 400
        self.inner_offset = 100
        self.isCone = False
        self.robot = FSM(self)
        
        # Add timer for processing frames
        self.timer = self.create_timer(0.1, self.process_frame)

    def align(self, left_bound, right_bound):
        frame_center = (left_bound + right_bound) // 2
        error = self.x_center - frame_center
        steering = error / float(right_bound - left_bound)
        steering = max(min(steering, 1.0), -1.0)
        self.msg.axes = [-steering, self.speed]
        self.pub.publish(self.msg)

    def detect_arrow_direction(self, image):
        direction = arrow_detect(image, far=True)
        if direction == 0:
            print("Arrow direction: LEFT")
            return "LEFT"
        elif direction == 1:
            print("Arrow direction: RIGHT")
            return "RIGHT"
        return None

    def color_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model.predict(self.color_frame, conf=0.5, verbose=False)
        self.cached_results = results
        frame_center = self.color_frame.shape[1] // 2
        self.window_center = frame_center
        if results and results[0].boxes.xyxy.numel() > 0:
            max_box = max_box_calculation(results)
            if max_box is not None:
                x1, y1, x2, y2 = map(int, max_box.xyxy[0])
                self.x_center = (x1 + x2) // 2
        direction = self.detect_arrow_direction(self.color_frame)
        if direction == "LEFT":
            self.LR_counter[0] += 1
        elif direction == "RIGHT":
            self.LR_counter[1] += 1

    def depth_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.depth_frame = np.nan_to_num(self.depth_frame)

    def process_frame(self):
        """Process the current frame with FSM logic"""
        if self.color_frame is not None and self.cached_results is not None:
            found = False
            depth_value = None
            area = 0
            max_box = None
            
            if self.cached_results and self.cached_results[0].boxes.xyxy.numel() > 0:
                max_box = max_box_calculation(self.cached_results)
                if max_box is not None:
                    found = True
                    x1, y1, x2, y2 = map(int, max_box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)
                    if self.depth_frame is not None:
                        depth_value = depth_calculation(self.color_frame, self.depth_frame, x1, y1, x2, y2)
            
            self.robot.state_selector(found, depth_value, area, max_box)

class FSM:
    def __init__(self, node):
        self.node = node
        self.state = "search"
        self.turned = False
        self.search_frames = 0
        self.node.msg.axes = [0.0, 0.0]
        self.node.pub.publish(self.node.msg)

    def search(self):
        self.state = "search"
        self.turned = False
        self.search_frames += 1
        print(self.search_frames)
        if self.search_frames > 20:
            cached_results = self.node.cone_model.predict(self.node.color_frame, conf=0.3, verbose=False)
            max_box = max_box_calculation(cached_results)
            if max_box:
                print("manik isCone ko true kr raha hu. Ange ka dekh lio")
                self.node.isCone = True
            else:
                if not self.node.isCone:
                    self.search_frames = 0    

        self.node.msg.axes = [0.0, 0.0]
        self.node.pub.publish(self.node.msg)

    def approach(self):
        self.state = "approach"
        outer_box_left = self.node.window_center - self.node.outer_offset
        outer_box_right = self.node.window_center + self.node.outer_offset
        if (self.node.x_center > outer_box_right or self.node.x_center < outer_box_left) or self.node.turnin:
            self.node.align(outer_box_left, outer_box_right)  
        else:
            self.node.msg.axes = [0.0, self.node.speed]
            self.node.pub.publish(self.node.msg)
        self.turned = False
        if not self.node.isCone:
            self.search_frames = 0

    def stop(self):
        self.state = "stop"
        self.node.msg.axes = [0.0, 0.0]
        self.node.pub.publish(self.node.msg)

        should_rotate = True
        if not self.turned:
            print("STOPPING FOR 10s")
            read_gps_data(self.node.LR_counter[0] - self.node.LR_counter[1])
            time.sleep(10)
            print(self.node.LR_counter)

            if self.node.LR_counter[0] - self.node.LR_counter[1] > 0:
                print("LEFT")
                self.node.msg.axes = [self.node.speed, 0.0]
            elif self.node.LR_counter[1] - self.node.LR_counter[0] > 0:
                print("RIGHT")
                self.node.msg.axes = [-self.node.speed, 0.0]
            elif self.node.LR_counter[1] == self.node.LR_counter[0]:
                print('''Sab moh maya hai, kcuh bhi ho skta h, 
                      humne apne hath khade krdie h, 
                      ROVER AB BHAGWAN BHAROSE CHLEGA!
                      MOVE BASE krlena chahie tha....
                      Sorry VIGNESH''')
                self.node.msg.axes = [0.0, -self.node.speed]
                should_rotate = False    

            self.node.pub.publish(self.node.msg)
            if should_rotate:
                read_imu_data()
                self.turned = True
                self.node.msg.axes = [0.0, 0.0]
                self.node.pub.publish(self.node.msg)
            else:
                time.sleep(3)  # Let rover go back a bit to get clear view of arrow for inference
            self.node.LR_counter = [0, 0]

    def get_state(self):
        return self.state

    def state_selector(self, found, depth_value, area, max_box):
        distance = 1500

        if (depth_value == None) and found:
            self.approach()
        elif depth_value == None and not found:
            self.search()
        elif depth_value > distance:
            self.approach()
        elif area and depth_value <= distance:
            if area >= self.node.color_frame.shape[1] * self.node.color_frame.shape[0] * 0.03:
                print("turn wala stoping....")
                self.stop()

def max_box_calculation(results):
    if not results:
        return None
    max_area = 0
    max_box = None
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            area = (x2 - x1) * (y2 - y1)
            if area > max_area:
                max_area = area
                max_box = box
    return max_box

def main(args=None):
    rclpy.init(args=args)
    node = Zed2YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()