import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import copy

class LidarProcessor(Node):

    def __init__(self):
        super().__init__('team1_lidar_processing')
        self.subscription = self.create_subscription(
            LaserScan,  # message type
            'scan',
            self.listener_callback,
            10)
        self.subscription = self.create_subscription(
            Bool,  # message type
            'brake',
            self.brake_callback,
            10)

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )


        self.subscription  # prevent unused variable warning
        self.last_gap = -300
        self.last_msg = AckermannDriveStamped()
        self.should_brake = False
        self.data_at_angle = []
        self.all_data = LaserScan()
        self.publisher = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

    def brake_callback(self, msg):
        self.should_brake = msg.data

    def listener_callback(self, msg):
        self.all_data = msg
        self.data_at_angle = copy.deepcopy(msg.ranges)

        min_index = minimum_element_index(self.data_at_angle)
        modified_data_at_angle = zero_out_safety_bubble(self.data_at_angle, min_index, 0.2)
        low, high = find_largest_nonzero_sequence(modified_data_at_angle)
        index_to_follow = int((low + high) / 2)

        steering_angle, speed = get_steering_angle(self.all_data, index_to_follow, self.all_data.ranges[min_index])

        drive_msg_all = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        drive_msg.steering_angle = steering_angle
        drive_msg.steering_angle_velocity = 2.0
        drive_msg.speed = speed
        drive_msg_all.drive = drive_msg

        brake_cmd_all = AckermannDriveStamped()
        brake_msg = AckermannDrive()
        brake_msg.steering_angle = - steering_angle
        brake_msg.steering_angle_velocity = 2.0
        brake_msg.speed = -1.5
        # brake_msg.acceleration = 0.5
        # brake_msg.jerk = 0.01
        # brake_msg.steering_angle = 0.0
        # brake_msg.steering_angle_velocity = 0.0
        # brake_msg.speed = 0.0
        brake_msg.acceleration = 0.0
        brake_msg.jerk = 0.0
        brake_cmd_all.drive = brake_msg
        
        if self.should_brake:
            self.last_msg = brake_cmd_all
            self.publisher.publish(brake_cmd_all)
        elif (abs(index_to_follow - self.last_gap) < 3):
            self.publisher.publish(self.last_msg)
        else:
            self.last_msg = drive_msg_all
            self.publisher.publish(drive_msg_all)
        self.last_gap = index_to_follow

    def map_callback(self, msg):
        self.get_logger().info(f"map callback: {msg.info} \n {msg.data}")
        with open('map_data.txt', 'w') as m_file:
            # width
            # height
            # origin
            # values (csv)
            strs = [str(item) for item in msg.data]
            csv = ",".join(strs)
            m_file.write(f"{msg.info.width}\n{msg.info.height}\n{msg.info.origin.position.x},{msg.info.origin.position.y}\n{csv}")


def preprocess_lidar(self, lidar_info):
    ranges = list(lidar_info.ranges)
    angle_min = lidar_info.angle_min
    angle_increment = lidar_info.angle_increment

    min_angle = -70 / 180.0 * PI
    min_index = int((min_angle - angle_min) / angle_increment)
    max_angle = 70 / 180.0 * PI
    max_index = int((max_angle - angle_min) / angle_increment)

    self.ranges = [0.0] * len(ranges)
    for i in range(min_index, max_index + 1):
        if not math.isinf(ranges[i]) and not math.isnan(ranges[i]) and ranges[i] <= lidar_info.range_max:
            self.ranges[i] = ranges[i]

def get_steering_angle(data, index_to_follow, min_distance):
    best_point_steering_angle = 0.0
    speed = 0.5
    best_point_steering_angle = -1.57 + index_to_follow * data.angle_increment
    # if index_to_follow < len(data.ranges)/2:
    #     best_point_steering_angle = - data.angle_increment * float(len(data.ranges)/2.0) - index_to_follow
    # else:
    #     best_point_steering_angle = data.angle_increment * float(index_to_follow - len(data.ranges)/2.0)
    
    if abs(best_point_steering_angle) > 20.0 / 180.0 * 3.14:
        speed = 0.5
    elif abs(best_point_steering_angle) > 10.0 / 180.0 * 3.14:
        speed = 1.0
    else:
        speed = 1.3

    if best_point_steering_angle < -1.57:
        return -1.57
    elif best_point_steering_angle > 1.57:
        return 1.57
    
    return best_point_steering_angle, speed
    
    
    # return best_point_steering_angle / min_distance

def find_largest_nonzero_sequence(data):
    current_start = 0
    current_size = 0
    max_start = 0
    max_size = 0

    current_index = 0

    while current_index < len(data):
        current_start = current_index
        current_size = 0

        while current_index < len(data) and data[current_index] > 0.1:
            current_size = current_size + 1
            current_index = current_index + 1

        if current_size > max_size:
            max_start = current_start
            max_size = current_size
            current_size = 0

        current_index = current_index + 1

    return max_start, max_start + max_size - 1

def minimum_element_index(data):
    count = 0
    index = 0
    min = data[index]
    for val in data:
        if val < min:
            min = val
            index = count
        count = count + 1
    return index

def zero_out_safety_bubble(data, min_index, safety_radius):
    min_point_distance =  data[min_index]
    data[min_index] = 0.0

    curr_index = min_index + 1
    while (curr_index < len(data)) and (data[curr_index] < min_point_distance + safety_radius):
        data[curr_index] = 0.0
        curr_index = curr_index + 1

    curr_index = min_index - 1
    while (curr_index > - 1) and (data[curr_index] < min_point_distance + safety_radius):
        data[curr_index] = 0.0
        curr_index = curr_index - 1

    return data

def main(args=None):
    rclpy.init(args=args)

    lidar_processor = LidarProcessor()

    rclpy.spin(lidar_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
