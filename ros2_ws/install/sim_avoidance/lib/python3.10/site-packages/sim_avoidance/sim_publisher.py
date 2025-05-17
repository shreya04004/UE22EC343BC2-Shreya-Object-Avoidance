import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker

class SimAvoid(Node):
    def __init__(self):
        super().__init__('sim_avoid')
        # Publisher for Range messages
        self.rng_pub = self.create_publisher(Range, 'ultrasonic', 10)
        # Publisher for servo orientation
        self.mkr_pub = self.create_publisher(Marker, 'servo_marker', 10)
        self.timer = self.create_timer(0.1, self.cb)
        self.angle = 0.0

    def cb(self):
        # Simulate distance: oscillate between 5cm and 30cm
        dist = 17.5 + 12.5 * math.sin(self.get_clock().now().nanoseconds / 1e9)
        # If obstacle < 15cm, servo turns away (angle 150°), else 90°
        target_angle = 150.0 if dist < 15.0 else 90.0
        # Slowly move servo angle toward target
        self.angle += (target_angle - self.angle) * 0.2

        # Publish Range message
        m = Range()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'ultrasonic_frame'
        m.radiation_type = Range.ULTRASOUND
        m.field_of_view = 0.5
        m.min_range = 0.02
        m.max_range = 4.0
        m.range = float(dist / 100.0)  # convert cm→m
        self.rng_pub.publish(m)

        # Publish Marker arrow to visualize servo direction
        mk = Marker()
        mk.header = m.header
        mk.ns = 'servo'
        mk.id = 0
        mk.type = Marker.ARROW
        mk.action = Marker.ADD
        mk.scale.x = 0.2  # length of the arrow
        mk.scale.y = 0.05
        mk.scale.z = 0.05
        mk.color.r = 1.0
        mk.color.g = 0.0
        mk.color.b = 0.0
        mk.color.a = 1.0
        # Arrow origin at (0,0,0.2), direction by angle
        theta = math.radians(self.angle - 90)
        mk.pose.position.x = 0.0
        mk.pose.position.y = 0.0
        mk.pose.position.z = 0.2
        mk.pose.orientation.z = math.sin(theta / 2)
        mk.pose.orientation.w = math.cos(theta / 2)
        self.mkr_pub.publish(mk)

def main(args=None):
    rclpy.init(args=args)
    node = SimAvoid()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
