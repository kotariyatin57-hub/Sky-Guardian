import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ArmService(Node):
    def __init__(self):
        super().__init__('arm_service')
        self.srv = self.create_service(SetBool, '/arm_drone', self.arm_callback)
        self.is_armed = False
        self.get_logger().info('Arm Service ready and waiting...')

    def arm_callback(self, request, response):
        self.is_armed = request.data
        response.success = True

        if self.is_armed:
            response.message = 'Drone ARMED successfully!'
        else:
            response.message = 'Drone DISARMED successfully!'

        self.get_logger().info(f'Request: arm={request.data} → {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
