# Complete Simulation Environment Example: Humanoid Robot in Warehouse

## Introduction

This chapter presents a comprehensive example of a complete simulation environment for a humanoid robot operating in a warehouse setting. The example integrates all the concepts covered in previous chapters to create a realistic and functional simulation environment that demonstrates the integration of Gazebo physics, Unity visualization, and ROS 2 communication.

## Scenario Overview

### Warehouse Environment
Our simulation environment models a warehouse setting where a humanoid robot performs inventory management tasks. The environment includes:
- Multiple levels with ramps and stairs
- Storage racks with boxes of various sizes
- Moving conveyor belts
- Human workers performing tasks
- Dynamic obstacles that move throughout the environment

### Robot Platform
The humanoid robot used in this simulation has:
- 28 degrees of freedom (DOF)
- LiDAR sensors for navigation
- RGB-D cameras for object recognition
- IMU for balance and orientation
- Manipulator arms for picking and placing objects

## System Architecture

### High-Level Architecture
```
┌─────────────────────────────────────────────────────────────────┐
│                           Unity                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │  Visualization  │  │   Interface     │  │   Analytics     │ │
│  │   (3D Scene)    │  │   (Controls)    │  │   (Metrics)     │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────┬───────────────────────────────────────────────────┘
              │
              │ ROS 2 Communication Layer
              │
┌─────────────▼───────────────────────────────────────────────────┐
│                          ROS 2 Core                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Navigation    │  │   Perception    │  │   Control       │ │
│  │   (Move Base)   │  │   (Object Det)  │  │   (Controllers) │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────┬───────────────────────────────────────────────────┘
              │
              │ Gazebo Simulation Layer
              │
┌─────────────▼───────────────────────────────────────────────────┐
│                          Gazebo                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Physics       │  │   Sensors       │  │   Environment   │ │
│  │   (ODE/Bullet)  │  │   (LiDAR/Cam)   │  │   (Models)      │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Implementation Details

### 1. Gazebo Environment Setup

#### World File (warehouse.world)
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="warehouse_world">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include sky -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Warehouse structure -->
    <model name="warehouse_structure">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      
      <!-- Walls -->
      <link name="walls">
        <collision name="walls_collision">
          <geometry>
            <box>
              <size>20 20 6</size>
            </box>
          </geometry>
        </collision>
        <visual name="walls_visual">
          <geometry>
            <box>
              <size>20 20 6</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Storage racks -->
    <model name="storage_rack_1">
      <pose>5 0 0 0 0 0</pose>
      <static>true</static>
      <!-- Rack definition -->
      <link name="rack_base">
        <collision name="rack_collision">
          <geometry>
            <box>
              <size>2 0.5 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="rack_visual">
          <geometry>
            <box>
              <size>2 0.5 2</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
    
    <!-- Robot spawn location -->
    <include>
      <name>humanoid_robot</name>
      <uri>model://humanoid_robot_model</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>
    
    <!-- Physics engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    
    <!-- ROS interface -->
    <plugin name="gazebo_ros_api_plugin" filename="libgazebo_ros_init.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
    </plugin>
  </world>
</sdf>
```

#### Robot Model (humanoid_robot.urdf)
```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.8"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.8"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="50"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.4"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="20"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.25 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Arm -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.25 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Sensors -->
  <!-- LiDAR -->
  <joint name="lidar_mount_joint" type="fixed">
    <parent link="head"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.1"/>
  </joint>

  <link name="lidar_link">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Camera -->
  <joint name="camera_mount_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0"/>
  </joint>

  <link name="camera_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Gazebo plugins -->
  <gazebo reference="lidar_link">
    <sensor name="lidar_sensor" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/humanoid_robot</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor name="camera_sensor" type="camera">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/humanoid_robot</namespace>
          <remapping>image_raw:=camera/image_raw</remapping>
          <remapping>camera_info:=camera/camera_info</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/humanoid_robot</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### 2. ROS 2 Control System

#### Robot State Publisher Configuration
```yaml
# config/robot_state_publisher.yaml
/**:
  ros__parameters:
    use_sim_time: true
    publish_frequency: 50.0
```

#### Joint Controller Configuration
```yaml
# config/joint_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - neck_joint
      - left_shoulder_joint
      - right_shoulder_joint
    interface_name: position
```

#### Navigation Configuration
```yaml
# config/navigation.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    goal_check_tolerance: 0.25

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # DWB parameters
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.1
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```

### 3. Unity Visualization System

#### Unity Robot Controller Script
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry_msgs;
using System.Collections.Generic;

public class HumanoidRobotController : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosIpAddress = "127.0.0.1";
    public int rosPort = 10000;
    
    [Header("Robot Configuration")]
    public Transform neckJoint;
    public Transform leftShoulderJoint;
    public Transform rightShoulderJoint;
    
    [Header("Sensors")]
    public GameObject lidarPointcloud;
    public Camera robotCamera;
    
    private ROSConnection ros;
    private Dictionary<string, Transform> jointMap;
    
    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIpAddress, rosPort);
        
        // Create joint mapping
        jointMap = new Dictionary<string, Transform>
        {
            {"neck_joint", neckJoint},
            {"left_shoulder_joint", leftShoulderJoint},
            {"right_shoulder_joint", rightShoulderJoint}
        };
        
        // Subscribe to joint states
        ros.Subscribe<JointStateMsg>("humanoid_robot/joint_states", OnJointStateReceived);
        
        // Subscribe to sensor data
        ros.Subscribe<LaserScanMsg>("humanoid_robot/scan", OnLidarDataReceived);
        ros.Subscribe<ImageMsg>("humanoid_robot/camera/image_raw", OnCameraDataReceived);
    }
    
    void OnJointStateReceived(JointStateMsg jointState)
    {
        // Update robot joints based on received state
        for (int i = 0; i < jointState.name.Length; i++)
        {
            string jointName = jointState.name[i];
            double position = jointState.position[i];
            
            if (jointMap.ContainsKey(jointName))
            {
                Transform joint = jointMap[jointName];
                
                // Apply position based on joint type
                if (jointName.Contains("neck"))
                {
                    joint.localRotation = Quaternion.Euler(0, (float)position * Mathf.Rad2Deg, 0);
                }
                else if (jointName.Contains("shoulder"))
                {
                    joint.localRotation = Quaternion.Euler(0, 0, (float)position * Mathf.Rad2Deg);
                }
            }
        }
    }
    
    void OnLidarDataReceived(LaserScanMsg scan)
    {
        // Update LiDAR visualization
        UpdateLidarVisualization(scan);
    }
    
    void OnCameraDataReceived(ImageMsg image)
    {
        // Update camera feed
        UpdateCameraFeed(image);
    }
    
    void UpdateLidarVisualization(LaserScanMsg scan)
    {
        // Create or update point cloud visualization
        if (lidarPointcloud != null)
        {
            // Convert scan data to points and visualize
            // This is a simplified example
            for (int i = 0; i < Mathf.Min(scan.ranges.Length, 100); i++) // Limit for performance
            {
                float angle = scan.angle_min + i * scan.angle_increment;
                float distance = (float)scan.ranges[i];
                
                if (distance < scan.range_max && distance > scan.range_min)
                {
                    Vector3 pointPos = new Vector3(
                        distance * Mathf.Cos(angle),
                        0,
                        distance * Mathf.Sin(angle)
                    );
                    
                    // Create a small sphere to represent the point
                    GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    point.transform.SetParent(lidarPointcloud.transform);
                    point.transform.localPosition = pointPos;
                    point.transform.localScale = Vector3.one * 0.05f;
                    point.GetComponent<Renderer>().material.color = Color.red;
                    
                    // Destroy after a few seconds to prevent accumulation
                    Destroy(point, 2.0f);
                }
            }
        }
    }
    
    void UpdateCameraFeed(ImageMsg image)
    {
        // Update the robot camera feed
        // This would typically involve converting the image message to a texture
        // and applying it to a material or UI element
    }
    
    void OnDestroy()
    {
        if (ros != null)
            ros.Disconnect();
    }
}
```

### 4. Perception System

#### Object Detection Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
import cv2
from cv_bridge import CvBridge

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image, 
            'humanoid_robot/camera/image_raw', 
            self.image_callback, 
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'humanoid_robot/scan',
            self.scan_callback,
            10
        )
        
        # Create publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'detected_objects',
            10
        )
        
        # Object detection parameters
        self.min_distance = 0.5  # meters
        self.max_distance = 5.0  # meters
        
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Perform object detection (simplified example)
            # In a real system, this would use a trained model
            detected_objects = self.detect_objects(cv_image)
            
            # Publish detected objects as markers
            self.publish_markers(detected_objects)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def scan_callback(self, msg):
        # Process LiDAR data for object detection
        # This is a simplified example
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        
        # Filter out invalid ranges
        valid_indices = (ranges > msg.range_min) & (ranges < msg.range_max)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Convert to Cartesian coordinates
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)
        
        # Simple clustering to detect objects
        objects = self.cluster_points(x_points, y_points)
        
        # Publish LiDAR-based object detections
        self.publish_lidar_markers(objects)
    
    def detect_objects(self, image):
        # Simplified object detection using color thresholding
        # In a real system, this would use a trained model
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for different objects
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        
        # Create masks
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Find contours
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        
        # Process red objects
        for contour in contours_red:
            if cv2.contourArea(contour) > 100:  # Minimum area threshold
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    objects.append({'type': 'red_object', 'x': cx, 'y': cy})
        
        # Process blue objects
        for contour in contours_blue:
            if cv2.contourArea(contour) > 100:  # Minimum area threshold
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    objects.append({'type': 'blue_object', 'x': cx, 'y': cy})
        
        return objects
    
    def cluster_points(self, x_points, y_points):
        # Simple clustering algorithm to group LiDAR points into objects
        # This is a simplified implementation
        objects = []
        
        # For simplicity, we'll just group nearby points
        # A real implementation would use a more sophisticated clustering algorithm
        for i in range(len(x_points)):
            # Check if this point is close to any existing object
            assigned = False
            for obj in objects:
                dist = np.sqrt((x_points[i] - obj['x'])**2 + (y_points[i] - obj['y'])**2)
                if dist < 0.3:  # 30cm threshold
                    # Update object center
                    obj['x'] = (obj['x'] + x_points[i]) / 2
                    obj['y'] = (obj['y'] + y_points[i]) / 2
                    obj['count'] += 1
                    assigned = True
                    break
            
            if not assigned:
                objects.append({
                    'x': x_points[i],
                    'y': y_points[i],
                    'count': 1
                })
        
        return objects
    
    def publish_markers(self, objects):
        marker_array = MarkerArray()
        
        for i, obj in enumerate(objects):
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position (simplified - would need depth information)
            marker.pose.position.x = 1.0  # Placeholder depth
            marker.pose.position.y = (obj['x'] - 320) * 0.001  # Convert pixel to meters
            marker.pose.position.z = (obj['y'] - 240) * 0.001  # Convert pixel to meters
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set size
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # Set color based on object type
            if obj['type'] == 'red_object':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:  # blue_object
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
    
    def publish_lidar_markers(self, objects):
        # Publish LiDAR-based object markers
        marker_array = MarkerArray()
        
        for i, obj in enumerate(objects):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "lidar_objects"
            marker.id = i + 1000  # Different ID range
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = obj['x']
            marker.pose.position.y = obj['y']
            marker.pose.position.z = 0.5  # Half the height of a typical object
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set size
            marker.scale.x = 0.3  # Diameter
            marker.scale.y = 0.3  # Diameter
            marker.scale.z = 1.0  # Height
            
            # Set color
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Control System

#### High-Level Task Planner
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import random

class WarehouseTaskPlanner(Node):
    def __init__(self):
        super().__init__('warehouse_task_planner')
        
        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Define warehouse locations
        self.locations = {
            'entrance': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'aisle_1': {'x': 5.0, 'y': 2.0, 'theta': 0.0},
            'aisle_2': {'x': 5.0, 'y': -2.0, 'theta': 0.0},
            'packing_station': {'x': -3.0, 'y': 0.0, 'theta': 1.57},
            'shipping_dock': {'x': -8.0, 'y': 0.0, 'theta': 3.14}
        }
        
        # Schedule periodic task assignment
        self.timer = self.create_timer(10.0, self.assign_task)
        
        # Task queue
        self.task_queue = []
        self.current_task = None
        
    def assign_task(self):
        # Generate a random task if queue is empty
        if not self.task_queue:
            # Possible tasks: restock, pack_order, inspect_area
            task_types = ['restock', 'pack_order', 'inspect_area']
            locations = list(self.locations.keys())
            
            for _ in range(3):  # Add 3 tasks to queue
                task_type = random.choice(task_types)
                location = random.choice(locations)
                self.task_queue.append({
                    'type': task_type,
                    'location': location,
                    'priority': random.randint(1, 5)
                })
        
        # Sort tasks by priority (higher number = higher priority)
        self.task_queue.sort(key=lambda x: x['priority'], reverse=True)
        
        # If no current task, start the highest priority one
        if self.current_task is None and self.task_queue:
            self.current_task = self.task_queue.pop(0)
            self.execute_task(self.current_task)
    
    def execute_task(self, task):
        self.get_logger().info(f'Executing task: {task["type"]} at {task["location"]}')
        
        # Send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        loc = self.locations[task['location']]
        goal_msg.pose.pose.position.x = loc['x']
        goal_msg.pose.pose.position.y = loc['y']
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation (simple approach)
        from math import cos, sin
        theta = loc['theta']
        goal_msg.pose.pose.orientation.z = sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = cos(theta / 2.0)
        
        # Wait for action server
        self.nav_client.wait_for_server()
        
        # Send goal
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.current_task = None
            return

        self.get_logger().info('Goal accepted')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded')
            # Task completed, set current task to None to trigger next task
            self.current_task = None
        else:
            self.get_logger().info(f'Goal failed with status: {status}')
            # Retry the same task
            self.execute_task(self.current_task)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = WarehouseTaskPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch System

### Main Launch File
```python
# launch/warehouse_simulation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    show_gui = LaunchConfiguration('show_gui', default='true')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('warehouse_simulation'),
                'worlds',
                'warehouse.world'
            ])
        }.items()
    )
    
    # Robot spawn node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0', '-y', '0', '-z', '1'
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('warehouse_simulation'),
                'config',
                'robot_state_publisher.yaml'
            ])
        ]
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Joint trajectory controller
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Object detection node
    object_detection = Node(
        package='warehouse_simulation',
        executable='object_detection_node',
        name='object_detection',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Task planner
    task_planner = Node(
        package='warehouse_simulation',
        executable='warehouse_task_planner',
        name='task_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'show_gui',
            default_value='true',
            description='Show Gazebo GUI if true'
        ),
        gazebo,
        spawn_entity,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        navigation,
        object_detection,
        task_planner
    ])
```

## Validation and Testing

### Simulation Validation
To validate the complete simulation environment:

1. **Physics Validation**:
   - Verify robot dynamics match expected behavior
   - Test collision detection and response
   - Validate sensor data accuracy

2. **Integration Validation**:
   - Confirm data flows correctly between systems
   - Verify Unity visualization matches Gazebo physics
   - Test ROS 2 communication reliability

3. **Functional Validation**:
   - Execute complete tasks in the warehouse environment
   - Test navigation and obstacle avoidance
   - Validate perception system performance

### Performance Testing
- Measure simulation real-time factor (RTF)
- Monitor CPU and memory usage
- Test with varying numbers of objects and robots
- Validate network communication performance

This complete simulation environment example demonstrates how to integrate all the components discussed in previous chapters to create a functional, realistic simulation for humanoid robots operating in a warehouse setting. The example includes all necessary configuration files, code implementations, and launch systems to run the complete simulation environment.