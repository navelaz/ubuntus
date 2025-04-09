import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import math
import os
from ament_index_python.packages import get_package_share_directory


def load_params(path):
    with open(path, 'r') as file:
        data = yaml.safe_load(file)
    robot_params = data['robot_params']
    waypoints = data['waypoints']
    return robot_params, waypoints


def is_reachable(p1, p2, max_linear, max_angular):
    dx = p2['x'] - p1['x']
    dy = p2['y'] - p1['y']
    distance = math.hypot(dx, dy)
    angle = abs(p2['theta'] - p1['theta'])
    required_linear = distance
    required_angular = angle
    return required_linear <= max_linear and required_angular <= max_angular


def compute_speeds(p1, p2, time):
    dx = p2['x'] - p1['x']
    dy = p2['y'] - p1['y']
    dtheta = p2['theta'] - p1['theta']
    linear_speed = math.hypot(dx, dy) / time
    angular_speed = dtheta / time
    return linear_speed, angular_speed


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(PoseStamped, 'path_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_next_pose)
        self.current_index = 0

        # Load params from config
        pkg_path = get_package_share_directory('puzzlebot_control')
        params_path = os.path.join(pkg_path, 'config', 'path_params.yaml')
        robot_params, self.waypoints = load_params(params_path)

        self.max_linear = robot_params['max_linear_speed']
        self.max_angular = robot_params['max_angular_speed']

        self.get_logger().info('PathPublisher initialized with {} waypoints.'.format(len(self.waypoints)))

        # Check reachability
        for i in range(len(self.waypoints)-1):
            if not is_reachable(self.waypoints[i], self.waypoints[i+1], self.max_linear, self.max_angular):
                self.get_logger().warn(f"Waypoint {i+1} is not reachable from {i} under given dynamics.")

    def publish_next_pose(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('Finished publishing all waypoints.')
            self.timer.cancel()
            return

        wp = self.waypoints[self.current_index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(wp['x'])
        msg.pose.position.y = float(wp['y'])
        msg.pose.position.z = 0.0

        # Convert yaw to quaternion (simplified)
        yaw = float(wp['theta'])
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        # Tiempo como campo extra en el header (usando stamp.sec como workaround)
        # Esto puede mejorarse con un mensaje custom
        msg.header.stamp.sec = int(wp['time'])

        self.publisher.publish(msg)
        self.get_logger().info(f"Published waypoint {self.current_index} with time {wp['time']}s")

        self.current_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
