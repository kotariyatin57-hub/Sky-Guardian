import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)
import math

class WaypointNavNode(Node):

    TRAJECTORIES = {
        'square': [
            ( 5.0,  0.0, -5.0),
            ( 5.0,  5.0, -5.0),
            ( 0.0,  5.0, -5.0),
            ( 0.0,  0.0, -5.0),
        ],
        'triangle': [
            ( 5.0,  0.0, -5.0),
            ( 2.5,  5.0, -5.0),
            ( 0.0,  0.0, -5.0),
        ],
        'figure8': [
            ( 4.0,  0.0, -5.0),
            ( 4.0,  4.0, -5.0),
            ( 0.0,  0.0, -5.0),
            (-4.0,  4.0, -5.0),
            (-4.0,  0.0, -5.0),
            (-4.0, -4.0, -5.0),
            ( 0.0,  0.0, -5.0),
            ( 4.0, -4.0, -5.0),
            ( 4.0,  0.0, -5.0),
        ],
    }

    def __init__(self, trajectory: str):
        super().__init__('waypoint_nav_node')

        if trajectory not in self.TRAJECTORIES:
            self.get_logger().error(f'Unknown trajectory: {trajectory}')
            raise ValueError

        self.waypoints = self.TRAJECTORIES[trajectory]
        self.current_wp = 0
        self.get_logger().info(f'Trajectory: {trajectory} | {len(self.waypoints)} waypoints')

        # QoS
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
            VehicleCommand, '/fmu/in/vehicle_command', qos_pub)

        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_pos_cb, qos_sub)
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v2',
            self.status_cb, qos_sub)

        self.local_pos = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_counter = 0
        self.state = 'INIT'

        self.timer = self.create_timer(0.1, self.timer_cb)

    def local_pos_cb(self, msg):
        self.local_pos = msg

    def status_cb(self, msg):
        self.vehicle_status = msg

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw=0.0):
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

    def distance_to_waypoint(self, wp):
        dx = self.local_pos.x - wp[0]
        dy = self.local_pos.y - wp[1]
        dz = self.local_pos.z - wp[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def timer_cb(self):
        self.publish_offboard_mode()

        if self.state == 'INIT':
            # Stream setpoints at home position before arming
            self.publish_setpoint(0.0, 0.0, -5.0)
            if self.offboard_counter == 20:
                self.send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0, param2=6.0)
                self.arm()
                self.state = 'TAKEOFF'
                self.get_logger().info('Arming + switching to offboard')
            self.offboard_counter += 1

        elif self.state == 'TAKEOFF':
            self.publish_setpoint(0.0, 0.0, -5.0)
            # Wait until we reach takeoff altitude
            if abs(self.local_pos.z - (-5.0)) < 0.5:
                self.state = 'NAVIGATE'
                self.get_logger().info('Reached takeoff altitude, starting navigation')

        elif self.state == 'NAVIGATE':
            wp = self.waypoints[self.current_wp]
            self.publish_setpoint(wp[0], wp[1], wp[2])

            dist = self.distance_to_waypoint(wp)
            self.get_logger().info(
                f'WP {self.current_wp+1}/{len(self.waypoints)} | '
                f'Target: {wp} | Dist: {dist:.2f}m',
                throttle_duration_sec=1.0)

            # Move to next waypoint when within 0.5m
            if dist < 0.5:
                self.current_wp += 1
                if self.current_wp >= len(self.waypoints):
                    self.state = 'RETURN'
                    self.get_logger().info('All waypoints reached! Returning home')
                else:
                    self.get_logger().info(f'Waypoint {self.current_wp} reached, moving to next')

        elif self.state == 'RETURN':
            # Return to home position before landing
            self.publish_setpoint(0.0, 0.0, -5.0)
            if math.sqrt(self.local_pos.x**2 + self.local_pos.y**2) < 0.5:
                self.land()
                self.state = 'LANDING'
                self.get_logger().info('Home reached, landing')

        elif self.state == 'LANDING':
            self.publish_setpoint(0.0, 0.0, -5.0)


def main(args=None):
    rclpy.init(args=args)

    # Change to 'square', 'triangle', or 'figure8'
    TRAJECTORY = 'triangle'

    node = WaypointNavNode(trajectory=TRAJECTORY)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
