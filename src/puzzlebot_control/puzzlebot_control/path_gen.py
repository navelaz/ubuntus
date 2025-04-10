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


def normalize_angle(angle):
    """Normaliza un ángulo al rango [-π, π]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def is_reachable(p1, p2, max_linear_speed, max_angular_speed):
    dx = p2['x'] - p1['x']
    dy = p2['y'] - p1['y']
    distance = math.hypot(dx, dy)

    dtheta = normalize_angle(p2['theta'] - p1['theta'])
    dt = p2['time'] - p1['time']  # Tiempo disponible

    if dt <= 0:
        return False

    required_linear_speed = distance / dt
    required_angular_speed = abs(dtheta) / dt

    # Introducimos un margen de error en las comparaciones
    margin = 0.05  # Puedes ajustar este valor como lo desees

    if required_linear_speed <= (max_linear_speed + margin) and required_angular_speed <= (max_angular_speed + margin):
        return True
    else:
        return False



def compute_speeds(p1, p2, time):
    dx = p2['x'] - p1['x']
    dy = p2['y'] - p1['y']
    dtheta = normalize_angle(p2['theta'] - p1['theta'])

    linear_speed = math.hypot(dx, dy) / time if time > 0 else 0.0
    angular_speed = dtheta / time if time > 0 else 0.0
    return linear_speed, angular_speed


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(PoseStamped, 'path_pose', 10)
        self.timer = None
        self.current_index = 0

        # Load params from config
        pkg_path = get_package_share_directory('puzzlebot_control')
        params_path = os.path.join(pkg_path, 'config', 'path_params.yaml')
        robot_params, self.waypoints = load_params(params_path)

        self.max_linear = robot_params['max_linear_speed']
        self.max_angular = robot_params['max_angular_speed']

        # 👉 Solo calculamos theta si no está definida en el YAML
        for i in range(len(self.waypoints) - 1):
            if 'theta' not in self.waypoints[i] or self.waypoints[i]['theta'] is None:
                dx = self.waypoints[i+1]['x'] - self.waypoints[i]['x']
                dy = self.waypoints[i+1]['y'] - self.waypoints[i]['y']
                self.waypoints[i]['theta'] = math.atan2(dy, dx)

        # También para el último punto
        if 'theta' not in self.waypoints[-1] or self.waypoints[-1]['theta'] is None:
            self.waypoints[-1]['theta'] = self.waypoints[-2]['theta']

        self.get_logger().info(f'🟢 PathPublisher initialized with {len(self.waypoints)} waypoints.')
        self.publish_next_pose()  # inicializa la primera publicación

    def publish_next_pose(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('🥳 Finished publishing all waypoints.')
            return

        wp = self.waypoints[self.current_index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(wp['x'])
        msg.pose.position.y = float(wp['y'])
        msg.pose.position.z = 0.0

        yaw = float(wp['theta'])
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        # Verificamos si el waypoint es alcanzable
        if self.current_index < len(self.waypoints) - 1:
            next_wp = self.waypoints[self.current_index + 1]
            dt = next_wp['time'] - wp['time']
            lin_speed, ang_speed = compute_speeds(wp, next_wp, dt)
            reachable = is_reachable(wp, next_wp, self.max_linear, self.max_angular)
            if reachable:
                self.get_logger().info(f"🧩 Waypoint {self.current_index} is reachable! (linear: {lin_speed:.2f} m/s, angular: {ang_speed:.2f} rad/s)")
            else:
                self.get_logger().warning(f"⚠️ Waypoint {self.current_index} is NOT reachable! (linear: {lin_speed:.2f} m/s, angular: {ang_speed:.2f} rad/s)")
        else:
            dt = 0.0
            lin_speed, ang_speed = 0.0, 0.0

        # Codificamos velocidades y duración en los campos restantes de orientación
        msg.pose.orientation.x = lin_speed
        msg.pose.orientation.y = ang_speed
        msg.pose.orientation.z = dt

        self.publisher.publish(msg)
        self.get_logger().info(f"📤 Published waypoint {self.current_index} with duration {dt:.2f}s")

        self.current_index += 1

        if dt > 0.0:
            if self.timer:
                self.timer.cancel()  # 💥 importante: cancelamos el anterior
            self.timer = self.create_timer(dt, self.publish_next_pose)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
