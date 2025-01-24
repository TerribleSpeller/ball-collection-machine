#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import torch
from ultralytics import YOLO
import cv2
import time
import math
import serial
from ament_index_python.packages import get_package_share_directory
import os
import random
from datetime import datetime

class InferenceNode(Node):
    def __init__(self, model_path, serial_port):
        super().__init__('robot_control')
        package_share_directory = get_package_share_directory('cameranode')
        self.model_path = os.path.join(package_share_directory, 'models', model_path)
        self.serial_port_path = serial_port
        try:
            self.model = YOLO(self.model_path)  
            self.get_logger().info(f"Model loaded from {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Error loading model: {e}")
            rclpy.shutdown()
            return
        try:
            #No, not erorruehahheh
            self.ser = serial.Serial(self.serial_port_path, 9600, timeout=1)  
            self.get_logger().info(f"Serial port opened: {self.serial_port_path}")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            rclpy.shutdown()
            return
        
        self.latest_points = []
        self.create_subscription(PointCloud2, 'tof_point_cloud', self.point_cloud_callback, 10)
        self.timer = self.create_timer(0.1, self.run_code)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            #I HATE SENSITIVE EQUIOPMENT I HATE SENSITIVE EQUIPMENT
            self.get_logger().error("Error: could not open camera")
            rclpy.shutdown()
            return
        if self.cap.isOpened():
            self.get_logger().info("Camera opened")
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        
        self.start_time = time.time()
        
    def point_cloud_callback(self, msg):
        try:
            #Me when I read the pointcloud. 
            self.latest_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")
        
    def run_code(self):
        try:
            if self.ser.in_waiting > 0:
                self.get_logger().info(f"Bytes waiting in serial buffer: {self.ser.in_waiting}")
                data = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Data received: {data}")
                if data == "READY":
                    self.get_logger().info("FIRING!")
                    self.run_inference_from_camera()
                else:
                    self.get_logger().info("Not Ready")
                
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
        
        except Exception as e:
            self.get_logger().error(f"Error in run_code: {e}")
            
    def run_inference_from_camera(self):
        try:
            for _ in range(30):  
                self.cap.read()
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Error: No frame received from camera")
                if self.cap.isOpened():
                    self.get_logger().info("Camera is opened however.")
                return
            
            results = self.model(frame)
            boxes = results[0].boxes
            # angleperms = 0.05667 #F*CKING BULLSH*T
            angleperms = 0.089693 ##Higher Bullsh*t
            distance_threshold = 500 #in mm
            point_count_threshold = 30
            random_angle_threshold = 30 #FOr the dumb ass search algorithim
            angle_threshold = 7
            #0.05667 from 56.67 in 1000 miliseconds. In which the PWM is at max and at 7 volts. 
            
            if boxes:
                sorted_detections = sorted(boxes, key=lambda x: x.conf[0], reverse=True)
                most_confident_detection = sorted_detections[0]
                xywh = most_confident_detection.xywh[0].tolist()
                x_min = int(xywh[0] - (xywh[2] / 2))
                y_min = int(xywh[1] - (xywh[1] / 2))
                x_max = int(xywh[0] + (xywh[2] / 2))  
                y_max = int(xywh[1] + (xywh[3] / 2))
                x_target = int((x_min + x_max) / 2)
                y_target = int((y_min + y_max) / 2)
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                targeted_action = self.determineaction(x_target, y_target, self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT), frame, x_min, y_min, x_max, y_max)
                
                if time.time() - self.start_time >= 1:
                    data = f"{targeted_action}\n"
                    self.get_logger().info(f"Sending to Arduino: {data}")
                    self.ser.write(data.encode())
                    self.ser.flush()
                    self.start_time = time.time()
            else:
                self.get_logger().info("No object detected")
                left_side_count = 0  
                right_side_count = 0
                points_in_angle_range = []

                for point in self.latest_points:
                    x, y, z = point
                    
                    if x == 0 and y == 0 and z == 0:
                        continue
                    
                    point_angle_rad = math.atan2(y, x)
                    point_angle_deg = math.degrees(point_angle_rad)
                    
                    if abs(point_angle_deg - 10) < random_angle_threshold:
                        points_in_angle_range.append(point)
                        distance = math.sqrt(x*2 + y*2 + z*2)
                        
                        if distance < distance_threshold:
                            if point_angle_deg < 0:
                                left_side_count += 1
                            else:
                                right_side_count += 1
                
                if left_side_count > right_side_count:
                    if left_side_count > point_count_threshold:
                        targeted_action = 0 + 15
                        self.get_logger().info(f"Turning with angle {targeted_action} due lack of detected target")    
                    else:
                        targeted_action = random.randint(-30, 30)
                        if targeted_action < angle_threshold:
                            targeted_action = 0 
                        self.get_logger().info(f"No Concern: Maintaining the current direction random search: {targeted_action}")    
                elif right_side_count > left_side_count:
                    if right_side_count > point_count_threshold:
                        targeted_action = 0 - 15 #Probably overkill, but it works. 
                        self.get_logger().info(f"Turning with angle {targeted_action} due lack of detected target")    
                    else:
                        targeted_action = random.randint(-30, 30)
                        if targeted_action < angle_threshold:
                            targeted_action = 0 
                        self.get_logger().info(f"No Concern: Maintaining the current direction random search: {targeted_action}")   
                elif left_side_count > point_count_threshold and right_side_count > point_count_threshold:
                    targeted_action = 180        
                    self.get_logger().info(f"High Concern, stuff in front turning: {targeted_action}")   
                else:
                    targeted_action = random.randint(-30, 30)
                    if targeted_action < angle_threshold:
                        targeted_action = 0 
                    self.get_logger().info(f"Turning with angle {targeted_action} due lack of detected target")    
                
                if time.time() - self.start_time >= 1:
                    toSend = round(targeted_action/angleperms, 1)
                    data = f"{toSend}\n"
                    self.get_logger().info(f"Sending to Arduino as: {data}")
                    self.ser.write(data.encode())
                    self.ser.flush()
                    self.start_time = time.time()
                    
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                image_path = os.path.abspath(f"/home/chipper/imageFiles/frame_{timestamp}.jpg")
                image_center_x = frame.shape[1] // 2
                image_center_y = frame.shape[0] // 2
                cv2.line(frame, (image_center_x, 0), (image_center_x, frame.shape[0]), (255, 0, 0), 2) # x-axis
                cv2.line(frame, (0, image_center_y), (frame.shape[1], image_center_y), (255, 0, 0), 2) # y-axis
                cv2.putText(frame, f"Center: ({image_center_x}, {image_center_y})", (image_center_x + 10, image_center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                cv2.imwrite(image_path, frame)
                self.get_logger().info(f"Saved frame with no object detected to {image_path}")
        
        except Exception as e:
            self.get_logger().error(f"Error in run_inference_from_camera: {e}")
        
    def determineaction(self, coordX, coordY, frameWidth, frameHeight, frame, x_min, y_min, x_max, y_max):
        try:
            #angleperms = 0.05667
            angleperms = 0.089693 #FUCK!
            fov_horizontal_rad = math.radians(120)
            angle_per_pixel = fov_horizontal_rad / frameWidth
            image_center_x = frameWidth / 2
            angle_rad = (coordX - image_center_x) * angle_per_pixel
            angle = math.degrees(angle_rad)
            
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            
            image_center_x = frame.shape[1] // 2
            image_center_y = frame.shape[0] // 2
            cv2.line(frame, (image_center_x, 0), (image_center_x, frame.shape[0]), (255, 0, 0), 2) #x-axis
            cv2.line(frame, (0, image_center_y), (frame.shape[1], image_center_y), (255, 0, 0), 2) #y-axis
            cv2.putText(frame, f"Center: ({image_center_x}, {image_center_y})", (image_center_x + 10, image_center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.putText(frame, f"Angle: {angle:.2f}", (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, f"x_target: {coordX}", (x_min, y_min - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, f"y_target: {coordY}", (x_min, y_min - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            box_width = x_max - x_min
            box_height = y_max - y_min
            cv2.putText(frame, f"Box: {box_width}x{box_height}", (x_min, y_min - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            image_path = os.path.abspath(f"/home/chipper/imageFiles/frame_{timestamp}.jpg")
            cv2.imwrite(image_path, frame)
            self.get_logger().info(f"Saved image with bounding box to {image_path}")
        
            ##DETERMINE ACTION CONFIG
            angle_to_turn = 0
            left_side_count = 0
            right_side_count = 0
            distance_threshold = 500 #in mm
            point_count_threshold = 30
            angle_theshold = 7
            angular_resolution = angle_per_pixel
            ball_size = 0.04 #in meters
            angular_size = box_width * angular_resolution
            correciton_coefficient = 0.875/2
            distance = (ball_size / 2) / math.tan(angular_size / 2)
            distance_actual = distance/correciton_coefficient
            
            if abs(angle) < angle_theshold:
                self.get_logger().info(f"No need to turn; angle is {angle}")
                angle_to_turn = 0
            else:
                angle_to_turn = round(angle / angleperms, 1)
                self.get_logger().info(f"Turning with angle {angle} with time of {angle_to_turn}")
            
            points_in_angle_range = []
        
            for point in self.latest_points:
                x, y, z = point
                
                if x == 0 and y == 0 and z == 0:
                    continue
                
                point_angle_rad = math.atan2(y, x)
                point_angle_deg = math.degrees(point_angle_rad)
                
                if abs(point_angle_deg - angle_to_turn) < angle_theshold:
                    points_in_angle_range.append(point)
                    distance = math.sqrt(x**2 + y**2)
                    
                    if distance < distance_threshold:
                        if point_angle_deg < angle_to_turn:
                            left_side_count += 1
                        else:
                            right_side_count += 1
         
            if left_side_count > right_side_count:
                if left_side_count > point_count_threshold:
                    angle_to_turn = angle_to_turn + round(15/angleperms,1)
                    self.get_logger().info(f"Concern, too many objects near targeted angle, sending as {angle_to_turn} to arduino. Actual Angle {angle_to_turn*angleperms} ")    
                # else:
                #     angle_to_turn = angle_to_turn + round(2/angleperms,1)
                #     self.get_logger().info(f"Concern, several objects near targeted angle, sending as {angle_to_turn} to arduino. Actual Angle {angle_to_turn*angleperms} ")    

            elif right_side_count > left_side_count:
                if right_side_count > point_count_threshold:
                    angle_to_turn = angle_to_turn - round(15/angleperms,1)
                    self.get_logger().info(f"Concern, too many objects near targeted angle, sending as {angle_to_turn} to arduino. Actual Angle {angle_to_turn*angleperms} ")    
                # else:
                #     angle_to_turn = angle_to_turn - round(2/angleperms,1)
                #     self.get_logger().info(f"Concern, several objects near targeted angle, sending as {angle_to_turn} to arduino. Actual Angle {angle_to_turn*angleperms} ")    

            else:
                if left_side_count > point_count_threshold and right_side_count > point_count_threshold:
                    angle_to_turn = round(180/angleperms,1)
                    self.get_logger().info(f"Concern,  object i nfront, sending as {angle_to_turn} to arduino. Actual Angle {angle_to_turn*angleperms} ")    
                else:
                    angle_to_turn = angle_to_turn
        
            if points_in_angle_range:
                self.get_logger().info(f"Found {len(points_in_angle_range)} points near the angle {angle_to_turn}")
                    
                    
            self.get_logger().info(f"Ball Detected - Final Angle turn {angle_to_turn*angleperms}, to Arduino: {angle_to_turn}")
            
            if angle_to_turn == 0:
                angle_to_turn = str(angle_to_turn) + f"|{distance_actual}"

            return angle_to_turn
        
        except Exception as e:
            self.get_logger().error(f"Error in determineaction: {e}")
            return 0  
        
        
def main(args=None):
    rclpy.init(args=args)
    model_path = "golfmodel.pt"
    serial_path = "/dev/ttyUSB0"
    
    #I hope I never had to do this again I hate ROS. 
    node = InferenceNode(model_path, serial_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting Down Node due to Keyboard Interrupt")
    except Exception as e:
        node.get_logger().error(f"Exception in main: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()