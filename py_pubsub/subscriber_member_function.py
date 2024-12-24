import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from . import morse


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            String, "button_event", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        self.serial_port = "/dev/ttyACM2"  # Replace with your Arduino's serial port
        self.baud_rate = 9600
        self.arduino = None

        # Initialize the serial connection to the Arduino
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        # Send the received message to the Arduino
        if self.arduino:
            try:
                self.arduino.write((morse.decrypt(msg.data) + "\n").encode("utf-8"))
                self.get_logger().info(f'Sent to Arduino: "{msg.data}"')
            except Exception as e:
                self.get_logger().error(f"Error sending to Arduino: {e}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        if minimal_subscriber.arduino:
            minimal_subscriber.arduino.close()
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
