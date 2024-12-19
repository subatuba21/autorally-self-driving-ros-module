import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class ControllerNode(Node):
    def __init__(self):
        super().__init__('team1_pure_pursuit_controller')
        self.odom = self.create_subscription(
            Odometry,  # message type
            'odom',
            self.odom_callback,
            10)

        self.imu = self.create_subscription(
            Imu,  # message type
            'sensors/imu/raw',
            self.imu_callback,
            10)

    def odom_callback(self, msg):
        self.get_logger().info(f"Odometry Pose: {msg.pose.pose.position.x} {msg.pose.pose.position.y} {msg.pose.pose.position.z}")

    def imu_callback(self, msg):
        self.get_logger().info(f"IMU")

def main(args=None):
    rclpy.init(args=args)
    c_node = ControllerNode()

    rclpy.spin(c_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # safety_node.get_logger().info("Hello, this is an informational message.")
    c_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()