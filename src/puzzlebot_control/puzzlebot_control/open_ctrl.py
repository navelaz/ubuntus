import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math


class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'path_pose', self.pose_callback, 10)

        self.commands = []
        self.current_cmd = None
        self.state = 'STOP'
        self.count = 0
        self.state_start_time = None
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('🟢 Open loop controller initialized!')

    def pose_callback(self, msg):
        cmd = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'theta': 0.0,
            'linear_speed': msg.pose.orientation.x,
            'angular_speed': msg.pose.orientation.y,
            'duration': msg.pose.orientation.z
        }
        self.get_logger().info(f"📥 Received command: {cmd}")
        self.commands.append(cmd)

    def control_loop(self):
        now = self.get_clock().now()
        if self.state_start_time:
            elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        else:
            elapsed = 0.0

        cmd_msg = Twist()

        if self.state == 'STOP':
            if self.commands:
                self.current_cmd = self.commands.pop(0)
                self.state_start_time = now

                lin = self.current_cmd['linear_speed']
                ang = self.current_cmd['angular_speed']

                if abs(lin) < 1e-3 and abs(ang) > 1e-3:
                    self.state = 'TURNING'
                else:
                    self.state = 'RUNNING'

                self.get_logger().info(f"💻 Starting new command: {self.current_cmd} in state {self.state}")
            else:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0

        elif self.state == 'RUNNING':
            if elapsed < self.current_cmd['duration']:
                cmd_msg.linear.x = self.current_cmd['linear_speed']
                cmd_msg.angular.z = self.current_cmd['angular_speed']
            else:
                self.state = 'STOP'
                self.count += 1
                self.get_logger().info(f"↗️ RUNNING command {self.count} complete.")

        elif self.state == 'TURNING':
            if elapsed < self.current_cmd['duration']:
                cmd_msg.angular.z = self.current_cmd['angular_speed']
                cmd_msg.linear.x = 0.08  
            else:
                self.state = 'STOP'
                self.count += 1
                self.get_logger().info(f"🔄 TURNING command {self.count} complete.")

        self.cmd_vel_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
