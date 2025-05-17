import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        self.publisher_ = self.create_publisher(Float32, 'object_distance', 10)
        self.serial_port = '/dev/ttyUSB0'  # Adjust this to match your Arduino's port
        self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            try:
                # Read the data from Arduino and convert to float
                distance = float(self.ser.readline().strip())
                msg = Float32()
                msg.data = distance
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published Distance: {distance}')
            except ValueError:
                self.get_logger().error('Failed to read distance value.')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    rclpy.spin(node)
    node.ser.close()

if __name__ == '__main__':
    main()
