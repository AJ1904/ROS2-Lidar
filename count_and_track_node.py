import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
from example_interfaces.msg import Int64
from std_msgs.msg import Bool
import sys
from sensor_msgs.msg import LaserScan
import math
from sklearn.cluster import DBSCAN
import numpy as np

# Define a class named CountAndTrack that inherits from Node
class CountAndTrack(Node):
    # Subscribes to '/current_laser_points', processes the data to track and count people, and publishes the results
    def __init__(self):
        # Initialize the Node
        super().__init__('count_and_track')

        # Create a publisher to send PointCloud messages to the '/person_locations' topic
        self.publisher_ = self.create_publisher(PointCloud, '/person_locations', 10)

        # Create a publisher to send Int64 messages to the '/person_count' topic
        self.person_count_publisher = self.create_publisher(Int64, '/person_count', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.people_points = []
        self.first_call = True

        # Keep track of various variables related to processing laser data
        self.current_laser_points = []
        self.clustered_points = []
        self.moving_objects = []
        self.tracked_people = {}
        self.next_person_id = 0  # Initialize the person ID counter

        # this is a list of integers that corresponds to the list of previous centroids
        # if the int == 0 it is not moving
        # if the int > 0 and int < self.frames_bef_person, it was moving for previous number of frames equal to its value
        # if the int == self.frames_bef_person + 1, it was registered as a person last time
        self.prev_cen_was_moving = []
        self.frames_bef_person = 7 # the number of frames in a row a centroid has to be moving in before they are considered a person

        # Count to keep track of how long a person is not seen before considering them disappeared
        self.disappearance_count_threshold = 100  # You can adjust this value
        self.disappearance_count = {}
        self.previous_centroids = []  # List to store centroids from the previous frame
        self.min_motion_threshold = 0.053  # Adjust the threshold as needed
        self.max_motion_threshold = 1.0 # Adjust the threshold as needed
        self.moving_objects_count = 0

        # Create a subscription to receive PointCloud messages from the '/current_laser_points' topic
        self.current_laser_points_subscription = self.create_subscription(
            PointCloud,
            '/current_laser_points',
            self.current_laser_points_callback,
            10  # Adjust the QoS profile if needed
        )
        self.current_laser_points_subscription
    
    # Callback function to receive current laser points
    def current_laser_points_callback(self,msg):
        self.current_laser_points = msg.points
        self.clustered_points = self.cluster_points()
        
        # Convert clustered_points to a list of (x, y) tuples
        # current_centroids = [(point.x, point.y) for point in self.clustered_points]
        current_centroids = [Point32(x=point.x, y=point.y, z=0.0) for point in self.clustered_points]

        # Detect moving objects
        self.moving_objects = self.detect_motion(current_centroids)
        # self.moving_objects_count = max(self.moving_objects_count, current_objects_count)

        # Update previous_centroids for the next frame
        self.previous_centroids = current_centroids

    # Timer callback function to publish current person locations and counts
    def timer_callback(self):
        msg = PointCloud()
        header = Header()
        header.stamp.sec = self.i  # Set the seconds part of the timestamp
        header.stamp.nanosec = 0  # Set the nanoseconds part of the timestamp
        header.frame_id = "laser" # Set the frame_id
        msg.header = header

        # Set the message's points to the detected moving objects
        msg.points = self.moving_objects

        # Publish the PointCloud message
        self.publisher_.publish(msg)
        self.i += 1
        
        # Publish the moving objects count on the "person_count" topic
        person_count_msg = Int64()
        person_count_msg.data = self.next_person_id
        self.person_count_publisher.publish(person_count_msg)

    # Function to cluster laser points using DBSCAN
    def cluster_points(self):
    	if len(self.current_laser_points) > 0:
            # Convert the laser points to a numpy array
            laser_points = np.array([(point.x, point.y) for point in self.current_laser_points])

            # Apply DBSCAN clustering
            dbscan = DBSCAN(eps=0.5, min_samples=5)  # Adjust `eps` and `min_samples` as needed
            labels = dbscan.fit_predict(laser_points)

            # Process the clusters
            unique_labels = set(labels)
            clustered_points = []

            for label in unique_labels:
                if label == -1:
                    # Points that were considered as noise
                    continue

                # Extract points in the cluster
                cluster_points = laser_points[labels == label]

                # Calculate the centroid of the cluster
                centroid_x = np.mean(cluster_points[:, 0])
                centroid_y = np.mean(cluster_points[:, 1])

                # Create a Point32 object for the cluster's centroid
                cluster_center = Point32()
                cluster_center.x = centroid_x
                cluster_center.y = centroid_y
                cluster_center.z = 0.0
                clustered_points.append(cluster_center)
            
            return clustered_points
    	return []

    # Function to detect moving objects
    def detect_motion(self, current_centroids):
        moving_objects = []

        #this is what self.prev_cen_was_moving will be updated with for the next runthrough
        current_cen_moving = []

        for current_centroid in current_centroids:
            if not self.previous_centroids:
                break

            # if self.prev_cen_was_moving is empty populate with 0's
            if self.prev_cen_was_moving == []:
                self.prev_cen_was_moving = [0 for point in self.previous_centroids]

            min_distance = float('inf')
            closest_previous_centroid = None

            #this stores the index of the element of the previous centroid we are on
            prev_cen_index = 0

            #this stores the int value that the closest centroid had in self.prev_cen_was_moving
            prev_cen_value = 0

            for previous_centroid in self.previous_centroids:
                # Calculate the Euclidean distance between centroids
                distance = np.linalg.norm(
                    np.array([current_centroid.x, current_centroid.y]) - np.array([previous_centroid.x, previous_centroid.y]))

                if distance < min_distance:
                    min_distance = distance
                    closest_previous_centroid = previous_centroid

                    #updates prev_cen_value
                    prev_cen_value = self.prev_cen_was_moving[prev_cen_index]

                # increments this index
                prev_cen_index = prev_cen_index + 1

            #recalculates distance based off the closest centroid
            distance = np.linalg.norm(np.array([current_centroid.x, current_centroid.y]) - np.array([closest_previous_centroid.x, closest_previous_centroid.y]))
            
            # If the closest previous centroid is within the motion threshold, consider it as moving
            if self.min_motion_threshold <= distance <= self.max_motion_threshold:
                

                # Check if this centroid is close to any tracked person
                matched_person_id = self.match_to_tracked_person(current_centroid)

                if matched_person_id is not None:
                    # Update the tracked person's position
                    self.tracked_people[matched_person_id] = current_centroid
                    
                    # increments the number of frames the centroid has been moving
                    current_cen_moving.append(self.frames_bef_person + 1)

                    moving_objects.append(current_centroid)
                else:
                    
                    #sets the number of frames moving to be one that always passed the condition
                    #I don't think this exact number matters but it works
                    current_cen_moving.append(prev_cen_value + 1)

                    if prev_cen_value >= self.frames_bef_person:
                        # Create a new tracked person entry if the centroid has been moving for long enough

                        self.tracked_people[self.next_person_id] = current_centroid
                        self.next_person_id += 1
                        moving_objects.append(current_centroid)
            else:
                # sets the frames moved to 0
                current_cen_moving.append(0)

        # updates self.prev_cen_was_moving for next frame
        self.prev_cen_was_moving = current_cen_moving
        
        # Remove disappeared people
        self.remove_disappeared_people()

        return moving_objects

    # Function to match current centroids to tracked people
    def match_to_tracked_person(self, centroid):
        # Match the current centroid to a tracked person based on proximity
        matched_person_id = None

        for person_id, last_position in self.tracked_people.items():
            # Calculate the distance between the current centroid and the last-known position of the person
            distance = np.linalg.norm(np.array([centroid.x, centroid.y]) - np.array([last_position.x, last_position.y]))

            if distance <= 1.0:  # Adjust the threshold as needed
                matched_person_id = person_id
                break

        return matched_person_id

    # Function to remove disappeared people
    def remove_disappeared_people(self):
        # Remove people who have disappeared for too long
        disappeared_people = []

        for person_id, last_position in self.tracked_people.items():
            if person_id not in self.moving_objects:
                # This person has not been detected in the current frame
                # Increment their disappearance count
                # If the count exceeds the threshold, consider them disappeared
                if person_id in self.disappearance_count:
                    self.disappearance_count[person_id] += 1
                else:
                    self.disappearance_count[person_id] = 1

                if self.disappearance_count[person_id] >= self.disappearance_count_threshold:
                    disappeared_people.append(person_id)

        # Remove disappeared people from the dictionary
        for person_id in disappeared_people:
            del self.tracked_people[person_id]

# Main function
def main(args=None):
    rclpy.init(args=args) # Initialize the ROS client library
    count_and_track = CountAndTrack() # Create an instance of the CountAndTrack class
    rclpy.spin(count_and_track) # Start spinning the node
    count_and_track.destroy_node() # Destroy the node when finished
    rclpy.shutdown() # Shutdown the ROS client library

# Entry point for the program
if __name__ == '__main__':
    main()
