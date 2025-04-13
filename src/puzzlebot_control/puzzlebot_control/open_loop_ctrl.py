
# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')

        self.wait_for_ros_time()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.state = 0
        self.state_start_time = self.get_clock().now()

        # Constantes de movimiento
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s

        # Cuánto se tarda en recorrer 2m o girar 90°
        self.forward_time = 2.0 / self.linear_speed
        self.rotation_angle = np.deg2rad(90)
        self.rotate_time = self.rotation_angle / self.angular_speed

        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Open loop square trajectory controller initialized!')

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
        rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'ROS time is active!')

    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9

        cmd = Twist()

        # Estados 0, 2, 4, 6 → avanzar
        # Estados 1, 3, 5, 7 → girar 90 grados
        if self.state in [0, 2, 4, 6]:
            cmd.linear.x = self.linear_speed
            self.get_logger().info(f"State {self.state}: Moving forward...")

            if elapsed_time >= self.forward_time:
                self.state += 1
                self.state_start_time = now
                self.get_logger().info(f"Finished forward. State {self.state}: Preparing to rotate.")

        elif self.state in [1, 3, 5, 7]:
            cmd.angular.z = self.angular_speed
            self.get_logger().info(f"State {self.state}: Rotating 90 degrees...")

            if elapsed_time >= self.rotate_time:
                self.state += 1
                self.state_start_time = now
                self.get_logger().info(f"Finished rotation. State {self.state}: Preparing to move forward.")

        elif self.state == 8:
            # Última rotación para cerrar el cuadrado
            cmd.angular.z = self.angular_speed
            self.get_logger().info("State 8: Final rotation to close the square...")

            if elapsed_time >= self.rotate_time:
                self.state += 1
                self.state_start_time = now
                self.get_logger().info("Finished final rotation. Stopping.")

        elif self.state == 9:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("State 9: Trajectory complete. Robot stopped.")
            self.timer.cancel()

        self.cmd_vel_pub.publish(cmd)

#Main
def main(args=None):
    rclpy.init(args=args)

    node = OpenLoopCtrl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()

#Execute Node
if __name__ == '__main__':
    main()