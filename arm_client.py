import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ArmClient(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.client = self.create_client(SetBool, '/arm_drone')

        # Wait until service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /arm_drone service...')

        self.send_request(True)  # True = arm, False = disarm

    def send_request(self, arm: bool):
        request = SetBool.Request()
        request.data = arm

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response = future.result()
        self.get_logger().info(f'Response → success: {response.success}, message: {response.message}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
