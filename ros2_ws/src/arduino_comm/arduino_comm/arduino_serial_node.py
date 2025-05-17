import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
import serial

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        self.publisher_distance = self.create_publisher(Int32, 'distance', 10)
        self.publisher_angle = self.create_publisher(Int32, 'servo_angle', 10)
        self.marker_pub = self.create_publisher(Marker, 'obstacle_marker', 10)

        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.serial_port.in_waiting:
            line = self.serial_port.readline().decode().strip()
            if "distance:" in line and "angle:" in line:
                try:
                    parts = line.split(",")
                    dist = float(parts[0].split(":")[1])
                    angle = int(parts[1].split(":")[1])

                    # Publish distance and angle
                    dist_msg = Int32()
                    dist_msg.data = int(dist)
                    self.publisher_distance.publish(dist_msg)

                    angle_msg = Int32()
                    angle_msg.data = angle
                    self.publisher_angle.publish(angle_msg)

                    # Publish marker
                    self.publish_marker(dist)
                except Exception as e:
                    self.get_logger().warn(f"Parse error: {e}")

    def publish_marker(self, distance):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = distance / 100.0  # Convert cm to m
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

