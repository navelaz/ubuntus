# Imports
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Class Definition
class differential(Node):
    def __init__(self):
        super().__init__('differential')
        # Define a QoS profile with BEST_EFFORT reliability
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        # Subscribers for additional topics
        self.create_subscription(Float32, 'VelocityEncL', self.wL_callback, qos_profile)
        self.create_subscription(Float32, 'VelocityEncR', self.wR_callback, qos_profile)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.get_logger().info("Differential Started 🚀")
        self.v = 0.0
        self.w = 0.0
        self.phiR = 0.0
        self.phiL = 0.0
        self.x = 0.0
        self.y = 0.0
        self.psi = 0.0
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.pub_v = self.create_publisher(Float32, 'Velocity', 10)
        self.pub_w = self.create_publisher(Float32, 'AngularVelocity', 10)
        self.pub_x = self.create_publisher(Float32, 'X', 10)
        self.pub_y = self.create_publisher(Float32, 'Y', 10)
        self.pub_psi = self.create_publisher(Float32, 'Psi', 10)

        self.create_timer(0.1, self.timer_callback)

    def print_all(self):
        self.get_logger().info(
            f"v: {self.v:.2f} | w: {self.w:.2f} | phiR: {self.phiR:.2f} | "
            f"phiL: {self.phiL:.2f} | x: {self.x:.2f} | y: {self.y:.2f} | psi: {self.psi:.2f}"
        )
    def wL_callback(self, msg):
        self.phiL = msg.data
        self.update_position()

    def wR_callback(self, msg):
        self.phiR = msg.data
        self.update_position()
    def update_position(self):
        # Update the position based on the velocity and angular velocity
        dt = 0.1
        self.x += self.v * dt * math.cos(self.psi)
        self.y += self.v * dt * math.sin(self.psi)
        self.psi += wrap_to_pi((self.phiR - self.phiL) * dt)
        # Update the publishers
        self.pub_x.publish(Float32(data=self.x))
        self.pub_y.publish(Float32(data=self.y))
        self.pub_psi.publish(Float32(data=self.psi))
    def timer_callback(self):
        # Publish the current velocity and angular velocity
        self.pub_v.publish(Float32(data=self.v))
        self.pub_w.publish(Float32(data=self.w))
        self.print_all()
    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

# Main
def main(args=None):
    rclpy.init(args=args)

    motor_node = differential()

    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        traceback.print_exc()
    finally:
        motor_node.destroy_node()
        rclpy.try_shutdown()

# Execute Node
if __name__ == '__main__':
    main()