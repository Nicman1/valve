import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ValveNode(Node):
    def __init__(self, pwm=2000):
        super().__init__('valve_node')
        self.OPEN_VALVE = pwm
        self.CLOSE_VALVE = 0

        self.publisher = self.create_publisher(String, '/valve_status', 10)

        self.subscription = self.create_subscription(
            String,
            '/valve_control',
            self.valve_callback,
            10
        )

        self.get_logger().info('Valve Node Initialized. Waiting for commands...')

    def valve_callback(self, msg):
        """Handles incoming messages on the /valve_control topic"""
        command = msg.data.lower()
        
        if command == "open":
            self.open_valve()
        elif command == "close":
            self.close_valve()
        else:
            self.get_logger().warn(f"Unknown command received: {command}")

    def open_valve(self):
        """Simulates opening the valve and sends a success message"""
        self.get_logger().info(f'Valve opened at {self.OPEN_VALVE} PWM')
        self.send_status("Valve successfully opened")

    def close_valve(self):
        """Simulates closing the valve and sends a success message"""
        self.get_logger().info(f'Valve closed')
        self.send_status("Valve successfully closed")

    def send_status(self, status_message):
        """Publishes status message to /valve_status topic"""
        msg = String()
        msg.data = status_message
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent status: {status_message}')

def main():
    rclpy.init()
    node = ValveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import RPi.GPIO as GPIO

# class ValveNode(Node):
#     def __init__(self, pwm_pin=18, pwm_freq=50, open_duty_cycle=10, close_duty_cycle=5):
#         super().__init__('valve_node')
#         GPIO.setmode(GPIO.BCM)
#         self.pwm_pin = pwm_pin
#         self.pwm_freq = pwm_freq
#         self.OPEN_VALVE = open_duty_cycle
#         self.CLOSE_VALVE = close_duty_cycle

#         GPIO.setup(self.pwm_pin, GPIO.OUT)
#         self.pwm = GPIO.PWM(self.pwm_pin, self.pwm_freq)
#         self.pwm.start(self.CLOSE_VALVE)

#         self.publisher = self.create_publisher(String, '/valve_status', 10)

#         self.subscription = self.create_subscription(
#             String,
#             '/valve_control',
#             self.valve_callback,
#             10
#         )

#         self.get_logger().info('Valve Node Initialized. Waiting for commands...')

#     def valve_callback(self, msg):
#         """Handles incoming messages on the /valve_control topic"""
#         command = msg.data.lower()
        
#         if command == "open":
#             self.open_valve()
#         elif command == "close":
#             self.close_valve()
#         else:
#             self.get_logger().warn(f"Unknown command received: {command}")

#     def open_valve(self):
#         """Simulates opening the valve and sends a success message"""
#         self.get_logger().info(f'Valve opened at {self.OPEN_VALVE}% duty cycle')
#         self.pwm.ChangeDutyCycle(self.OPEN_VALVE)
#         self.send_status("Valve successfully opened")

#     def close_valve(self):
#         """Simulates closing the valve and sends a success message"""
#         self.get_logger().info(f'Valve closed')
#         self.pwm.ChangeDutyCycle(self.CLOSE_VALVE)
#         self.send_status("Valve successfully closed")

#     def send_status(self, status_message):
#         """Publishes status message to /valve_status topic"""
#         msg = String()
#         msg.data = status_message
#         self.publisher.publish(msg)
#         self.get_logger().info(f'Sent status: {status_message}')

# def main():
#     rclpy.init()
#     node = ValveNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     GPIO.cleanup()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
