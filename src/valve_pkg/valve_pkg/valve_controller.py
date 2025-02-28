import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ValveController(Node):
    def __init__(self, buckets):
        super().__init__('valve_controller')
        self.buckets = buckets
        self.mlWater = 4000
        self.publisher = self.create_publisher(String, '/valve_control', 10)
        
        self.status_subscription = self.create_subscription(
            String,
            '/valve_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info("Valve Controller Initialized")
        
        self.timer = None
        self.timer_interval = None
        self.open_sent = False
        self.calculate_open_time()

    def calculate_open_time(self):
        """Calculates the time the valve should stay open based on the amount of water left"""
        base_time = 5
        max_water = 4000 

        "faire la fonction de temps ici!!"
        open_time = (base_time) * (max_water/ self.mlWater) * (8/self.buckets)
        self.timer_interval = open_time
        self.mlWater = self.mlWater - max_water / self.buckets

    def send_command(self, command):
        """Publishes a command to the /valve_control topic"""
        msg = String()
        msg.data = command
        self.publisher.publish(msg)

    def status_callback(self, msg):
        """Handles incoming status messages from /valve_status"""
        status_message = msg.data
        self.get_logger().info(f"Received valve status: {status_message}")

    def timer_callback(self):
        """Callback function for the timer to send 'close' after a specified interval"""
        if self.mlWater <= 0:
            self.mlWater = 4000
        self.send_command("close")
        self.timer.cancel()
        self.await_input()

    def start_timer(self):
        """Starts a timer to send 'close' after a specified interval"""
        self.calculate_open_time()
        self.timer = self.create_timer(self.timer_interval, self.timer_callback)

    def await_input(self):
        """Awaits user input to start the cycle again"""
        self.get_logger().info("Press Enter to send 'open'")
        input()
        self.send_command("open")
        self.get_logger().info(f"Starting timer for {self.timer_interval} seconds")
        self.start_timer()

def main():
    rclpy.init()
    node = ValveController(2)

    try:
        node.await_input()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down valve controller...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
