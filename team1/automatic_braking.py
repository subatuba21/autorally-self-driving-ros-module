#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool

class SafetyNode(Node):
    def __init__(self):
        super().__init__('team1_automatic_braking')

        # Subscribe to LaserScan and Odometry topics
        self.scan_sub = self.create_subscription(
            LaserScan,  # message type
            'scan',
            self.scan_callback,
            10)
        self.odom = self.create_subscription(
            Odometry,  # message type
            'odom',
            self.odom_callback,
            10)

        # Publish AckermannDriveStamped and Bool messages
        # self.brake_pub = self.create_publisher(AckermannDriveStamped, 'brake', 1)
        self.bool_pub = self.create_publisher(Bool, 'brake', 1)

    def scan_callback(self, scan_msg):
        # Analyze LaserScan data and determine if braking is necessary
        sentVal = False

        def isBad(index, threshold):
            # self.get_logger().info(f"Checking index: {index}, Range: {scan_msg.ranges[index]}, Threshold: {threshold}")
            
            if (scan_msg.ranges[index] < threshold):
                bool_msg = Bool()
                bool_msg.data = True
                self.bool_pub.publish(bool_msg)
                sentVal = True

        # self.get_logger().info(f"{len(scan_msg.ranges)}, {scan_msg.ranges}")
        for i in range(len(scan_msg.ranges)//30):
            start = i * 30
            end = 30 * (i+1)
            arr = scan_msg.ranges[start:end]
            if len(arr) > 0:
                self.get_logger().info(f"Range {start} - {end}. Max: {max(arr)}")

        for i in range(len(scan_msg.ranges)):
            # if i < 15*4 or i > 165*4:
            #     isBad(i, 0.1)
            # elif i < 30*4 or i > 150*4:
            #     isBad(i, 0.1)
            # elif i < 45*4 or i > 145*4:
            #     isBad(i, 0.2)
            # elif i < 60*4 or i > 120*4:
            #     isBad(i, 0.25)
            # else:
            #     isBad(i, 0.35)
            if i > 240 and i < 480:
                isBad(i, 0.31)
            elif (i > 180 and i < 360):
                isBad(i, 0.2)
            if sentVal:
                break
        
        if (not sentVal):
            bool_msg = Bool()
            bool_msg.data = False
            self.bool_pub.publish(bool_msg)


        # if any(range < threshold_distance for range in scan_msg.ranges[]):
        #     bool_msg = Bool()
        #     bool_msg.data = True
        #     self.bool_pub.publish(bool_msg)

        #     # Publish AckermannDriveStamped with velocity set to 0.0 m/s
        #     brake_msg = AckermannDriveStamped()
        #     brake_msg.drive.speed = 0.0
        #     self.brake_pub.publish(brake_msg)

        #     # Publish Bool message with value True
        #     bool_msg = Bool()
        #     bool_msg.data = True
        #     self.bool_pub.publish(bool_msg)
        # else:
        # brake_msg = AckermannDriveStamped()
        # brake_msg.drive.speed = 5.0
        # self.brake_pub.publish(brake_msg)

        #     # Publish Bool message with value True

    def odom_callback(self, odom_msg):
        # Process Odometry data if needed
        self.get_logger().info(f"ODOM")     

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()

    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # safety_node.get_logger().info("Hello, this is an informational message.")
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
