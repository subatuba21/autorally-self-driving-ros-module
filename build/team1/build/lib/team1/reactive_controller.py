class ReactiveController():

    def __init__(self):
        self.ranges = []

    # len(lidar_data) = 720
    # represents lidar data in 180 field of view but angle incremented at 0.25 degrees per element
    def calc_speed_and_angle(self, lidar_data: list):
        speed = 3.0
        angle = 0.0
        # your code goes here


        # end code

        return speed, angle

    # def disparity(self, lidar_data):

    #     return

    
    # def preprocess_lidar(self, lidar_info):
    #     ranges = list(lidar_info.ranges)
    #     angle_min = -1.57
    #     angle_increment = lidar_info.angle_increment

    #     min_angle = -70 / 180.0 * 3.14
    #     min_index = int((min_angle - angle_min) / angle_increment)
    #     max_angle = 70 / 180.0 * 3.14
    #     max_index = int((max_angle - angle_min) / angle_increment)

    #     self.ranges = [0.0] * len(ranges)
    #     for i in range(min_index, max_index + 1):
    #         if not math.isinf(ranges[i]) and not math.isnan(ranges[i]) and ranges[i] <= lidar_info.range_max:
    #             self.ranges[i] = ranges[i]
    
    # def find_closest_point(self):
    #     closest_index = self.ranges.index(min(self.ranges))
    #     return closest_index
    
    # def reactive_control(self, closest_index):
    #     ackermann_drive_result = AckermannDriveStamped()
    #     ackermann_drive_result.header.stamp = rospy.Time.now()
    #     ackermann_drive_result.drive.steering_angle = self.angle_min + closest_index * self.angle_increment

    #     if abs(ackermann_drive_result.drive.steering_angle) > 20.0 / 180.0 * PI:
    #         ackermann_drive_result.drive.speed = 0.5
    #     elif abs(ackermann_drive_result.drive.steering_angle) > 10.0 / 180.0 * PI:
    #         ackermann_drive_result.drive.speed = 1.0
    #     else:
    #         ackermann_drive_result.drive.speed = 1.5

    #     self.pub.publish(ackermann_drive_result)



    