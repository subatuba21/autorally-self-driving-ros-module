import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import copy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO
import torch
import cv2
from threading import Timer


PI = math.pi
MIN_ANGLE = math.pi/2
MAX_ANGLE = -math.pi/2
DISPARITY_THRESHOLD = 0.2
C_W = 0.3
C_L = 0.5

class DisparityExtender:
    CAR_WIDTH = 0.4
    # the min difference between adjacent LiDAR points for us to call them disparate
    DIFFERENCE_THRESHOLD = 0.05
    MAX_SPEED = 6.0
    LINEAR_DISTANCE_THRESHOLD = 5.0
    ANGLE_CHANGE_SPEED = 0.5
    MAX_ANGLE = 0.8
    SLOW_SPEED = 1.0
    MAX_DISTANCE_C = 0.90
    WHEELBASE_WIDTH=0.328#0.328
    coefficient_of_friction=0.2
    gravity=9.81998
    REVERSAL_THRESHHOLD = 0.85
    SLOWDOWN_SLOPE = 0.9

    prev_angle = 0.0
    prev_index = None
    is_reversing = False

    # stop sign state machine
    detected = False
    stopped = False
    started = False

    stop_sign_intervals = 0

        # Function to detect red blob and return boolean
    def detect_red_blob(self, frame):
        # Use only the bottom 50% of the frame
        height = frame.shape[0]
        bottom_half = frame[int(height // 2):, :, :]  # Slice the bottom 50%

        # Split the bottom half into Red, Green, Blue channels
        red_channel = bottom_half[:, :, 0]  # Red channel
        green_channel = bottom_half[:, :, 1]  # Green channel
        blue_channel = bottom_half[:, :, 2]  # Blue channel

        # Create a mask where red is significantly greater than green and blue
        red_mask = (red_channel > 120) & (red_channel > green_channel * 1.4) & (red_channel > blue_channel * 1.4)

        # Convert the mask to binary format
        red_mask = red_mask.astype(np.uint8) * 255

        # Apply morphological operations to clean up the mask
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_DILATE, np.ones((5, 5), np.uint8))

        # Find contours in the red mask
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check for significant blobs
        for contour in contours:
            if cv2.contourArea(contour) > 2000:  # Minimum blob size
                return True  # Red blob detected

        return False  # No red blob detected

    def __init__(self, logger):
        self.logger = logger
        self.logger.info("HEREEEEEEEEEE")
        self.zed = sl.Camera()
        zed = self.zed

        self.model = YOLO("yolo/yolov8n.pt")
        self.model.classes = [11]
        self.stop_sign_detected = False
        

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            # Quit if an error occurred
            exit()

        # detection_parameters = sl.ObjectDetectionParameters()
        # detection_parameters.enable_tracking = True

        # if detection_parameters.enable_tracking:
        #     zed.enable_positional_tracking()

        # print("Object Detection: Loading Module...")
        # err = zed.enable_object_detection(detection_parameters)

        if err != sl.ERROR_CODE.SUCCESS:
            print("Error {}, exit program".format(err))
            zed.close()
            exit()

        # objects = sl.Objects()

        # detection_parameters_runtime = sl.ObjectDetectionRuntimeParameters()
        # detection_parameters_runtime.detection_confidence_threshold = 40

        self.image_zed = sl.Mat()
    
    def detect_stop_sign(self):
        zed = self.zed
        image_zed = self.image_zed
        model = self.model
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            frame = image_zed.get_data()
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
            self.stop_sign_detected = self.detect_red_blob(frame_rgb)
            



    def preprocess_lidar(self, ranges):
        """ Any preprocessing of the LiDAR data can be done in this function.
            Possible Improvements: smoothing of outliers in the data and placing
            a cap on the maximum distance a point can be.
        """
        # remove quadrant of LiDAR directly behind us
        eighth = int(len(ranges) / 6)
        return np.array(ranges[eighth:-eighth])

    def get_differences(self, ranges):
        """ Gets the absolute difference between adjacent elements in
            in the LiDAR data and returns them in an array.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        differences = [0.]  # set first element to 0
        for i in range(1, len(ranges)):
            differences.append(abs(ranges[i] - ranges[i - 1]))
        return differences

    def get_disparities(self, differences, threshold):
        """ Gets the indexes of the LiDAR points that were greatly
            different to their adjacent point.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        disparities = []
        for index, difference in enumerate(differences):
            if difference > threshold:
                disparities.append(index)
        return disparities

    def get_num_points_to_cover(self, dist, width):
        """ Returns the number of LiDAR points that correspond to a width at
            a given distance.
            We calculate the angle that would span the width at this distance,
            then convert this angle to the number of LiDAR points that
            span this angle.
            Current math for angle:
                sin(angle/2) = (w/2)/d) = w/2d
                angle/2 = sininv(w/2d)
                angle = 2sininv(w/2d)
                where w is the width to cover, and d is the distance to the close
                point.
            Possible Improvements: use a different method to calculate the angle
        """
        # angle = 2 * np.arcsin(width / (2 * dist))
        # num_points = int(np.ceil(angle / self.radians_per_point))
        # return num_points
        # angle = 2 * np.arcsin(width / (2 * dist))
        # num_points = int(np.ceil(angle / self.radians_per_point))
        # return num_points
        angle_step=0.25*(math.pi/180)
        arc_length=angle_step*dist
        # self.logger.info(f"Number of points: {int(math.ceil(self.CAR_WIDTH / arc_length))}")
        return int(math.ceil(self.CAR_WIDTH / arc_length))
        # return 10

    def cover_points(self, num_points, start_idx, cover_right, ranges):
        """ 'covers' a number of LiDAR points with the distance of a closer
            LiDAR point, to avoid us crashing with the corner of the car.
            num_points: the number of points to cover
            start_idx: the LiDAR point we are using as our distance
            cover_right: True/False, decides whether we cover the points to
                         right or to the left of start_idx
            ranges: the LiDAR points

            Possible improvements: reduce this function to fewer lines
        """
        new_dist = ranges[start_idx]
        if cover_right:
            for i in range(num_points):
                next_idx = start_idx + 1 + i
                if next_idx >= len(ranges): break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        else:
            for i in range(num_points):
                next_idx = start_idx - 1 - i
                if next_idx < 0: break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        return ranges

    def extend_disparities(self, disparities, ranges, car_width):
        """ For each pair of points we have decided have a large difference
            between them, we choose which side to cover (the opposite to
            the closer point), call the cover function, and return the
            resultant covered array.
            Possible Improvements: reduce to fewer lines
        """
        width_to_cover = car_width / 2
        for index in disparities:
            first_idx = index - 1
            points = ranges[first_idx:first_idx + 2]
            close_idx = first_idx + np.argmin(points)
            far_idx = first_idx + np.argmax(points)
            close_dist = ranges[close_idx]
            num_points_to_cover = self.get_num_points_to_cover(close_dist, width_to_cover)
            cover_right = close_idx < far_idx
            ranges = self.cover_points(num_points_to_cover, close_idx, cover_right, ranges)
        return ranges

    def get_steering_angle(self, range_index, angle_increment, range_len):
        """ Calculate the angle that corresponds to a given LiDAR point and
            process it into a steering angle.
            Possible improvements: smoothing of aggressive steering angles
        """
        angle = -1.57 + (range_index * angle_increment)

        if angle < -1.57:
            angle = -1.57
        elif angle > 1.57:
            angle = 1.57
        return angle

        # degrees = range_index / 3
        # angle = math.radians(degrees)
        

        # if angle < -1.57:
        #     return -1.57
        # elif angle > 1.57:
        #     return 1.57
        # return angle

        # lidar_angle = (range_index - (range_len / 2)) * self.radians_per_point
        # steering_angle = np.clip(lidar_angle, np.radians(-90), np.radians(90))
        # return steering_angle
    
    def calculate_min_turning_radius(self,angle,forward_distance):
        angle = abs(angle)
        μ = self.coefficient_of_friction
        g = self.gravity
        W = self.WHEELBASE_WIDTH

        if angle < 0.0872665:  # If the angle is less than 5 degrees, go at max speed
            maximum_velocity = self.MAX_SPEED
        else:
            # Correct the formula for turning radius
            turning_radius = W / math.tan(angle)

            # Calculate the maximum velocity based on the turning radius
            maximum_velocity = math.sqrt(μ * g * turning_radius)

            # Ensure the velocity does not exceed MAX_SPEED
            maximum_velocity = min(maximum_velocity, self.MAX_SPEED)

            # Calculate the stopping distance required at this speed
            stopping_distance = (maximum_velocity ** 2) / (2 * μ * g)

            if stopping_distance > forward_distance:
                # Reduce maximum_velocity to ensure we can stop within the forward distance
                maximum_velocity = math.sqrt(2 * μ * g * forward_distance)

        # Ensure the vehicle maintains a minimum speed to keep moving
        maximum_velocity = max(maximum_velocity, self.SLOW_SPEED)

        return maximum_velocity
    def after_initial_timer(self):
        self.started = True
        timer = Timer(3.0, self.reset_timer)
        timer.start()

    def reset_timer(self):
        self.started = False
        self.stopped = False

    def _process_lidar(self, lidar_data):
        """ Run the disparity extender algorithm!
            Possible improvements: varying the speed based on the
            steering angle or the distance to the farthest point.
        """

        if self.stop_sign_intervals % 10 == 0:
            self.detect_stop_sign()

            if self.stop_sign_detected and self.stopped == False and self.started == False:
                self.stopped = True
                timer = Timer(2.0, self.after_initial_timer)
                timer.start()
                return 0.0, 0.0, 0.0, None, None, None, None
            elif self.stopped == True and self.started == False:
                return 0.0, 0.0, 0.0, None, None, None, None

        self.stop_sign_intervals += 1

        ranges = lidar_data.ranges
        self.radians_per_point = (2 * np.pi) / len(ranges)
        ranges = self.preprocess_lidar(ranges)
        differences = self.get_differences(ranges)
        disparities = self.get_disparities(differences, self.DIFFERENCE_THRESHOLD)
        proc_ranges = self.extend_disparities(disparities, ranges,
                                              self.CAR_WIDTH)
        # self.logger.info(f"differences: {differences}")
        # self.logger.info(f"disparities: {disparities}")
        # self.logger.info(f"proc_ranges: {proc_ranges}")
        # max_value=max(proc_ranges)

        # if self.prev_index != None and proc_ranges[self.prev_index] > 2 and abs(curr_steering_angle - self.prev_angle) < 0.1:
        #     steering_angle = self.prev_angle
        #     max_value = proc_ranges[self.prev_index]
        #     max_index = self.prev_index
        # else:
        max_value = max(proc_ranges)
        max_index = np.argmax(proc_ranges)

        # np_ranges = np.array(proc_ranges)
        # greater_indices = np.where(np_ranges >= max_value)[0]

        # if(len(greater_indices)==1):
        #     max_index = greater_indices[0]
        # else:
        #     mid=int(len(greater_indices)/2)
        #     max_index = greater_indices[mid]

        np_ranges = np.array(proc_ranges)
        # greater_indices = np_ranges >= self.MAX_DISTANCE_C_THRESHOLD
        # greater_indices = np.where(np_ranges >= min(self.MAX_DISTANCE_C_THRESHOLD, max_value*self.MAX_DISTANCE_C))[0]
        greater_indices = np.where(np_ranges >= max_value*self.MAX_DISTANCE_C)[0]
        differences = np.abs(greater_indices - 360)
        max_index = greater_indices[np.argmin(differences)]
        max_value = proc_ranges[max_index]
        # self.logger.info(f"greater indices: {greater_indices}, max index: {max_index}, max_value: {max_value}")

        steering_angle = self.get_steering_angle(max_index, lidar_data.angle_increment, len(proc_ranges))
        d_theta = abs(steering_angle)

        self.prev_angle = steering_angle
        self.prev_index = max_index

        # self.logger.info(f"Checking max_value: {max_value}, Max_index: {max_index}, Angle: {steering_angle}, Disparity: {disparities}, Ranges: {len(proc_ranges)}")
        self.logger.info(f"angle increment: {lidar_data.angle_increment} \n")
        
        if max_value < (0.6 if not self.is_reversing else 0.75):
            speed = -0.5
            steering_angle = -steering_angle
            self.is_reversing = True
        # elif max_value < self.LINEAR_DISTANCE_THRESHOLD:
        #     speed = max(0.5, self.MAX_SPEED - 0.9 * self.MAX_SPEED * ((self.LINEAR_DISTANCE_THRESHOLD - max_value) / self.LINEAR_DISTANCE_THRESHOLD))
        # elif d_theta > self.ANGLE_CHANGE_THRESHOLD:
        #     speed = calculate_min_turning_radius(steering_angle, max_value)
        else:
            self.is_reversing = False
            speed_d = max(0.5, self.MAX_SPEED -  self.MAX_SPEED * (self.SLOWDOWN_SLOPE * (self.LINEAR_DISTANCE_THRESHOLD - max_value) / self.LINEAR_DISTANCE_THRESHOLD))
            speed_a = self.calculate_min_turning_radius(steering_angle, max_value)
            speed = min(speed_d, speed_a)

            # if max_value < 0.5:
            #     min_speed = 0.3
            # elif max_value < 1.3:
            #     min_speed = 0.5
            # elif max_value < 1.7:
            #     min_speed = 0.75
            # elif max_value < 2.0:
            #     min_speed = 1.0
            # elif max_value < 2.5:
            #     min_speed = 1.3
            # elif max_value < 3:
            #     min_speed = 1.7
            # else:
            #     min_speed = 2.0

            # # min_speed = min(1.5, (max_value / 3))
            # speed = max(0.5, min_speed, speed)

        if speed > self.MAX_SPEED:
            speed = self.MAX_SPEED
        
        # speed = max(0.5, self.MAX_SPEED - 1.2 * self.MAX_SPEED * (d_theta / self.MAX_ANGLE))

        
        # self.logger.info(f"speed: {speed}, max_value: {max_value}")

        # if abs(steering_angle) > 20.0 / 180.0 * 3.14:
        #     speed = 1.5
        # elif abs(steering_angle) > 10.0 / 180.0 * 3.14:
        #     speed = 2.0
        # else:
        #     speed = 2.3

        # if max_value < 1.5:
        #     speed = 0.0

        return speed, steering_angle, max_value, max_index, differences, disparities, proc_ranges

    def process_observation(self, lidar_data, ego_odom):
        return self._process_lidar(lidar_data)


class ackermann_publisher(Node):

    def __init__(self):
        super().__init__('team_1_publisher')
        self.disparity = DisparityExtender(self.get_logger())
        self.laser_subscription = self.create_subscription(
            LaserScan,  # message type
            'scan',
            self.lidar_callback,
            10)

        self.count = 0

        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)

    def lidar_callback(self, msg: LaserScan):
            # Get the speed and angle from the disparity extender
        self.speed, self.angle, self.max_value, self.max_index, self.differences, self.disparities, self.proc_ranges = self.disparity._process_lidar(msg)

        # Steering angle smoothing parameters
        MAX_STEERING_DIFF = 0.05  # Maximum change in steering angle per iteration (in radians)

        # Initialize previous angle if not set
        if not hasattr(self, 'prev_angle'):
            self.prev_angle = self.angle

        # Calculate the difference between current and previous steering angles
        steering_diff = self.angle - self.prev_angle

        # Limit the steering angle change to prevent sudden swerves
        steering_diff = np.clip(steering_diff, -MAX_STEERING_DIFF, MAX_STEERING_DIFF)

        # Update the steering angle with the smoothed value
        smoothed_angle = self.prev_angle + steering_diff
        self.prev_angle = smoothed_angle  # Update the previous angle for the next iteration

        # Create and publish the Ackermann drive message
        stamped_msg = AckermannDriveStamped()
        stamped_msg.drive = AckermannDrive()
        stamped_msg.drive.steering_angle = smoothed_angle
        stamped_msg.drive.speed = self.speed

        self.publisher.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    ackermann_publisher_i = ackermann_publisher()
    rclpy.spin(ackermann_publisher_i)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ackermann_publisher_i.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()