# ROS2 - Lidar sensor

## Team Members
- **Ayushri Jain**
- **Jonas Land**

This assignment required creating software that processes data from a lidar sensor to detect and track the movement of people around a robot. This gave some experience designing and implementing algorithms that cope with the complexity of real sensor data, along with exposure to some additional ROS features.

---

[Video of example1 running.webm](https://github.com/user-attachments/assets/0f0a8c1c-cf45-4322-923a-5419a9d5b94a)

## How the Program Works

The program is designed to process laser scan data to detect and track moving objects, using ROS2 for node management and data handling. It utilizes a launch file to initiate nodes and handle ROS bag files for input and output.

### **1. `start.launch.py`**
The launch file configures and launches nodes and processes, primarily focusing on playing and recording ROS bag files. It also manages the execution of `scan_subscriber_node` and `count_and_track_node`.

#### Key Features:
- **DeclareLaunchArgument**: Declares arguments `rosbag_in` and `rosbag_out` for specifying input and output bag files.
- **Nodes**:
  - `scan_subscriber_node`
  - `count_and_track_node`
- **Processes**:
  - **`play_bag`**: Executes `ros2 bag play` to play the input bag file.
  - **`record_bag`**: Executes `ros2 bag record` to record data to the output bag file.
- **Event Handling**:
  - Registers an event handler for `play_bag` to shut down the program upon completion.

---

### **2. `scan_subscriber_node.py`**
Subscribes to laser scan data, calculates (x, y) coordinates, and publishes processed data as `PointCloud` messages for use by `count_and_track_node`.

#### Features:
- **Node Initialization**:
  - Inherits from the `Node` class.
  - Initializes as `"scan_subscriber"`.
- **Subscriptions and Publications**:
  - Subscribes to `/scan` to process `LaserScan` messages.
  - Publishes processed data to `/current_laser_points` using `PointCloud` messages.
- **Processing Flow**:
  - **`laser_scan_callback`**:
    - Extracts angles and ranges from `LaserScan` messages.
    - Calculates (x, y) coordinates for laser points.
    - Stores points in `current_laser_points`.
    - Calls `send_current_laser_point` after processing all points.
  - **`send_current_laser_point`**:
    - Prepares and publishes `PointCloud` messages with current laser points.

---

### **3. `count_and_track_node.py`**
Processes laser data to identify moving objects, cluster points, and track their motion. Publishes:
- `/person_locations` (current locations of detected people).
- `/person_count` (total count of detected people).

#### Features:
- **Initialization**:
  - Sets up publishers for `/person_locations` and `/person_count`.
  - Initializes timers and variables for tracking objects.
- **Callbacks**:
  - **`current_laser_points_callback`**:
    - Uses DBSCAN clustering to group laser points.
    - Tracks movement of objects using centroids.
    - Updates tracked objects and clusters.
  - **`timer_callback`**:
    - Periodically publishes locations and counts.
- **Methods**:
  - **`cluster_points`**:
    - Clusters laser points using DBSCAN.
    - Identifies potential objects or people based on centroids.
  - **`detect_motion`**:
    - Compares current centroids with previous ones to detect motion.
    - Updates tracked objects based on motion thresholds.
  - **`match_to_tracked_person`**:
    - Matches centroids to previously tracked objects based on proximity.
  - **`remove_disappeared_people`**:
    - Removes objects that have not been detected for a specified duration.

---

### **4. Parameters**

#### **DBSCAN Clustering**:
- **`eps` (0.5)**: Maximum distance between two points to be considered neighbors.
- **`min_samples` (5)**: Minimum number of points required to form a cluster.

#### **Motion Detection**:
- **`min_motion_threshold` and `max_motion_threshold`**:
  - Define the range for detecting object movement.
- **`frames_bef_person` (7)**:
  - Number of consecutive frames an object must move to be classified as a person.

#### **Tracking**:
- **`disappearance_count_threshold` (100)**:
  - Frames after which a person is considered disappeared.
- **Matching Threshold**:
  - Maximum distance (1.0) to match current centroids to tracked objects.

---

## Results and Observations

The results partially met our expectations. The system successfully tracks and counts moving objects in the provided examples. However, differentiating noise from actual moving objects proved challenging, resulting in less clean results than anticipated.

---
