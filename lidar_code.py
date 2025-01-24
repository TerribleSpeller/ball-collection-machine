#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import math
import serial
import sensor_msgs_py.point_cloud2 as pc2 

class ToFSensorNode(Node):
    def __init__(self):
        super().__init__('tof_sensor_node')
        self.get_logger().info("Node Initialized")
        self.pc_pub = self.create_publisher(PointCloud2, 'tof_point_cloud', 10)
        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)  #Why is is 1? Because it is. Don't ask. 
        self.points = []
        self.timer = self.create_timer(0.1, self.read_tof_sensor) #Does it in a loop

    def read_tof_sensor(self):
        self.get_logger().info("Attempting to read from the serial port...")
        try:
            if self.ser.in_waiting > 0:
                self.get_logger().info("Data available on serial port.")
                data = self.ser.readline().decode('utf-8').strip() #I'm just assuming lmao
                if not data:
                    self.get_logger().warn("No data received from sensor. Something's Broken Go fix it...")
                    return

                self.get_logger().info(f"Raw data: {data}")

                try:
                    distance_mm, angle_deg = map(float, data.split(','))
                    self.get_logger().info(f"Distance: {distance_mm} mm, Angle: {angle_deg} degrees")

                    angle_rad = math.radians(angle_deg)
                    x = distance_mm * math.cos(angle_rad)
                    y = distance_mm * math.sin(angle_rad)
                    z = 0.0
                    self.points.append([x, y, z])

                    if len(self.points) > 1000:
                        self.points.pop(0)

                    # Create header #If I don't do it it breaks. 
                    header = std_msgs.msg.Header()
                    header.stamp = self.get_clock().now().to_msg()
                    header.frame_id = "base_link"

                    pc_data = pc2.create_cloud_xyz32(header, self.points)  # Use the correct function
                    self.get_logger().info("Publishing PointCloud2 data...")
                    self.pc_pub.publish(pc_data)
                except ValueError as e:
                    self.get_logger().warn(f"Error parsing data: {data}")
                    self.get_logger().warn(f"ValueError: {e}")
        except Exception as e:
            self.get_logger().error(f"Unhandled error in read_tof_sensor: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ToFSensorNode()
    try:
        node.get_logger().info("Node spinning...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unhandled Exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
