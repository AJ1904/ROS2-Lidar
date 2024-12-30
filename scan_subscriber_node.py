import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud, LaserScan
import math
import sys


class ScanSubscriber(Node):
    # Subscribes to the /scan topic, processes the laser data, and publishes centroids

    def __init__(self):
        # Initialize the Node
        super().__init__('scan_subscriber')

        # Create a subscription to receive LaserScan messages from the '/scan' topic
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10 
        )
        # Store the subscription instance
        self.laser_subscription

        # Create a publisher to send PointCloud messages to the '/current_laser_points' topic
        self.publisher_ = self.create_publisher(PointCloud, '/current_laser_points', 10)

        # Initialize an empty list to store the current laser points as Point32 objects
        self.current_laser_points = []

        # Initialize the time stamp to zero
        self.time_stamp = 0

    # Sends the current_laser_points array as a PointCloud message
    def send_current_laser_point(self):
        msg = PointCloud()

        header = Header()
        header.stamp.sec = self.time_stamp  # Set the seconds part of the timestamp
        header.stamp.nanosec = 0  # Set the nanoseconds part of the timestamp
        header.frame_id = "laser" # Set the frame_id
        msg.header = header

        # sets the value sent in the message to be the array of current laser points
        msg.points = self.current_laser_points

        # Publish the PointCloud message
        self.publisher_.publish(msg)

        # Increment the time stamp
        self.time_stamp += 1

    # Callback function to process LaserScan messages    
    def laser_scan_callback(self, msg):
        # Extract relevant information from the LaserScan message
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
        ranges = msg.ranges
        current_points = []

        # Process the LaserScan data and calculate the points
        for i in range(len(angles)):
            angle = angles[i]
            distance = ranges[i]

            if not math.isinf(distance) and not math.isnan(distance):
                # Calculate the coordinates of the point in the laser's frame
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)

                curr_point = Point32()
                curr_point.x = x
                curr_point.y = y
                curr_point.z = 0.0

                # self.get_logger().info(f"Point {i}: x = {x}, y = {y}")
                # Append the calculated Point32 object to the list of current points
                current_points.append(curr_point)

        # Update the current_laser_points list and send the PointCloud message
        self.current_laser_points = current_points
        self.send_current_laser_point()
        
def main(args=None):
    rclpy.init(args=args) # Initialize the ROS client library
    scan_subscriber = ScanSubscriber() # Create an instance of the ScanSubscriber class
    rclpy.spin(scan_subscriber) # Start spinning the node
    scan_subscriber.destroy_node() # Destroy the node when finished
    rclpy.shutdown() # Shutdown the ROS client library

# Entry point for the program
if __name__ == '__main__':
    main()
