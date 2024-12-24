import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from . import morse


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "button_event", 10)
        self.serial_port = "/dev/ttyACM0"  # Replace with your Arduino's serial port
        self.baud_rate = 9600
        self.arduino = None

        # Initialize serial connection
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None

        # Timer to check serial data periodically
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.arduino and self.arduino.in_waiting > 0:
            try:
                line = self.arduino.readline().decode("utf-8").strip()
                if any(c in line for c in [".", "-"]):
                    msg = String()
                    msg.data = morse.decrypt(line)
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published: "{msg.data}"')
            except Exception as e:
                self.get_logger().error(f"Error reading from Arduino: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.arduino:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
