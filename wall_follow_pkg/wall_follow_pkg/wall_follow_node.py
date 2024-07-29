import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = 'scan'
        drive_topic = 'drive'

        # TODO: create subscribers and publishers
        self.lidarscan_subscriber = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10
        )

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: set PID gains
        # 실험값
        self.kp = 7.0
        self.ki = 0.025
        self.kd = 0.15

        # classic PID
        # self.kp = 4.2
        # self.ki = 14
        # self.kd = 0.315

        # pessen integral rule
        # self.kp = 4.9
        # self.ki = 20.42
        # self.kd = 0.441

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_angle = 0.0

        # TODO: store any necessary values you think you'll need
        self.desired_distance_left = 1.0
        self.integral_limit = 5.0   # Anti-windup limit
        self.velocity = 0.55
        self.max_steering_angle = np.radians(30)
        self.min_steering_angle = np.radians(-30)

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        index = int((angle - range_data.angle_min) / range_data.angle_increment)
        range = range_data.ranges[index]

        # inf, nan 처리
        if np.isinf(range) or np.isnan(range):
            return float('inf')
        
        return range

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        theta = np.radians(45)
        # print(range_data.angle_min)

        a = self.get_range(range_data, range_data.angle_max)
        b = self.get_range(range_data, np.radians(90))

        if a == float('inf') or b == float('inf'):
            return 0.0

        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        dist_from_wall = b * np.cos(alpha)

        error = dist - dist_from_wall  # update the error

        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """

        self.integral += error
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivate = error - self.prev_error

        angle = self.kp * error + self.ki * self.integral + self.kd * derivate
        self.prev_error = error
        # TODO: Use kp, ki & kd to implement a PID controller

        # steering_angle = max(self.min_steering_angle, min(self.max_steering_angle, angle))

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = max(self.min_steering_angle, min(self.prev_angle - angle, self.max_steering_angle))

        print(drive_msg.drive.steering_angle)
        
        drive_msg.drive.speed = velocity
        # TODO: fill in drive message and publish
        self.prev_angle = drive_msg.drive.steering_angle

        self.drive_publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg, self.desired_distance_left) # TODO: replace with error calculated by get_error()
        velocity = self.velocity # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
