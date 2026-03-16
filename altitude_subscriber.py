import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class AltitudeSubscriber(Node):
    def __init__(self):
        super().__init__('altitude_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            '/target_altitude',
            self.listener_callback,
            10
        )
        self.get_logger().info('Altitude Subscriber started! Waiting for target altitude...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received target altitude: {msg.data} m')

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
