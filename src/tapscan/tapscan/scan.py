import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import serial

class PWMMonitor(Node):
    def __init__(self):
        super().__init__('pwm_monitor')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            '/pwm_output',
            self.pwm_callback,
            10
        )
        self.get_logger().info("PWM Monitor Node has started.")
        self.serialc = serial.Serial("COM5", 115200, timeout=1)      

    def pwm_callback(self, msg):
        self.get_logger().info(f"Received PWM values: Left={msg.data[0]}, Right={msg.data[1]}")
        message = f"{msg.data[0]} {msg.data[1]}\n"
        self.serialc.write(message.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = PWMMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()