import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus
)

class TakeoffLandNode(Node):

    def __init__(self, target_altitude: float):
        super().__init__('takeoff_land_node')

        # target altitude (positive = up in NED frame means negative Z)
        self.target_altitude = -abs(target_altitude)
        self.get_logger().info(f'Target altitude: {abs(target_altitude)}m')

        # QoS profile required by PX4
        # With this:
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )    

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_pub)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_sub)

        # Subscriber
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v2', self.status_cb, qos_sub)

        self.vehicle_status = VehicleStatus()
        self.offboard_counter = 0
        self.state = 'INIT'

        # 10Hz timer — offboard needs minimum 2Hz, we use 10Hz
        self.timer = self.create_timer(0.1, self.timer_cb)

        def status_cb(self, msg):
            self.vehicle_status = msg
            self.get_logger().info(
                f'nav_state: {msg.nav_state}, arming_state: {msg.arming_state}',
                throttle_duration_sec=2.0)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def arm(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def land(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')

    def timer_cb(self):
        self.publish_offboard_mode()
        self.publish_setpoint(z=self.target_altitude)

        if self.state == 'INIT':
            # Stream setpoints for 1 second before switching to offboard
            if self.offboard_counter == 20:
                self.send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0, param2=6.0)   # offboard mode
                self.arm()
                self.state = 'TAKEOFF'
                self.get_logger().info('Switching to offboard + arming')
            self.offboard_counter += 1

        elif self.state == 'TAKEOFF':
            current_z = self.vehicle_status.nav_state  # just a liveness check
            self.get_logger().info(
                f'Climbing... target: {abs(self.target_altitude)}m', throttle_duration_sec=2.0)

            # Hover for 5 seconds at altitude then land
            if self.offboard_counter >= 60:   # ~5 seconds at 10Hz after arming
                self.state = 'HOVER'
                self.hover_counter = 0
                self.get_logger().info('Hovering')

            self.offboard_counter += 1

        elif self.state == 'HOVER':
            self.hover_counter += 1
            if self.hover_counter >= 50:   # hover for 5 seconds
                self.land()
                self.state = 'LANDING'
                self.get_logger().info('Landing...')

        elif self.state == 'LANDING':
            pass   # keep publishing offboard mode while landing


def main(args=None):
    rclpy.init(args=args)

    # Change this value to test 2.0, 5.0, 10.0
    TARGET_ALTITUDE = 2.0

    node = TakeoffLandNode(target_altitude=TARGET_ALTITUDE)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
