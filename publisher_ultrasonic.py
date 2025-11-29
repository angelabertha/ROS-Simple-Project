import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic_distance', 10)

        # GANTI PORT sesuai ESP32 kamu
        self.serial_port = serial.Serial('COM9', 115200, timeout=1)

        self.timer = self.create_timer(0.5, self.publish_data)

    def publish_data(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                distance = float(line)
                msg = Float32()
                msg.data = distance
                self.publisher_.publish(msg)
                self.get_logger().info(f'Distance: {distance} cm')
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
