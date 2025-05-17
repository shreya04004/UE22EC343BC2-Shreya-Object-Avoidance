import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')
        self.pub = self.create_publisher(Float32, 'distance', 10)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        line = self.ser.readline().decode('utf-8').strip()
        try:
            dist = float(line)
            msg = Float32(data=dist)
            self.pub.publish(msg)
        except ValueError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DistancePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
