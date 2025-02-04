import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ValveController(Node):
    def __init__(self, seconds):
        super().__init__('valve_controller')
        self.publisher = self.create_publisher(String, '/valve_control', 10)
        
        self.status_subscription = self.create_subscription(
            String,
            '/valve_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info("Valve Controller Initialized. Press Enter to send 'open'.")
        
        self.timer = None
        self.timer_interval = seconds  
        self.open_sent = False

    def send_command(self, command):
        """Publishes a command to the /valve_control topic"""
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent command: {command}')

    def status_callback(self, msg):
        """Handles incoming status messages from /valve_status"""
        status_message = msg.data
        self.get_logger().info(f"Received valve status: {status_message}")

    def timer_callback(self):
        """Callback function for the timer to send 'close' after a specified interval"""
        self.send_command("close")
        self.timer.cancel()
        self.await_input()

    def start_timer(self):
        """Starts a timer to send 'close' after a specified interval"""
        self.timer = self.create_timer(self.timer_interval, self.timer_callback)

    def await_input(self):
        """Awaits user input to start the cycle again"""
        self.get_logger().info("Press Enter to send 'open' again.")
        input()
        self.send_command("open")
        self.get_logger().info(f"Starting timer for {self.timer_interval} seconds")
        self.start_timer()

def main():
    rclpy.init()
    node = ValveController(10)

    try:
        node.await_input()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down valve controller...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
