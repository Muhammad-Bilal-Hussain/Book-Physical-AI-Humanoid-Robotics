# Complete Simulation Environment Example

This chapter provides a comprehensive example of implementing a complete digital twin simulation environment that integrates Gazebo physics, Unity visualization, and ROS 2 messaging. The example demonstrates the practical application of all concepts covered in this module.

## Overview

The complete simulation environment combines:
- Gazebo for accurate physics simulation
- Unity for high-fidelity visualization
- ROS 2 for communication and control
- Digital twin principles for real-time synchronization

## System Architecture

### High-Level Architecture
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Physical      │    │   Digital Twin  │    │   Control &     │
│   Robot         │◄──►│   Environment   │◄──►│   Monitoring    │
│                 │    │                 │    │                 │
│ • Real sensors  │    │ • Gazebo        │    │ • ROS 2 nodes   │
│ • Real actuators│    │ • Unity         │    │ • Control algos │
│ • Real physics  │    │ • Bridge system │    │ • Visualization │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Component Interaction Flow
1. Physical robot sends sensor data to digital twin
2. Digital twin updates simulation state
3. Unity visualizes the updated state
4. Control algorithms process simulation data
5. Commands sent back to physical robot

## Implementation Example

### 1. Gazebo Simulation Setup

#### Robot Model (SDF)
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="mobile_robot">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>10</mass>
        <inertia>
          <ixx>0.4</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.4</iyy>
          <iyz>0</iyz>
          <izz>0.4</izz>
        </inertia>
      </inertial>
      <visual name="chassis_visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>
      <collision name="chassis_collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
    </link>

    <!-- Differential drive plugin -->
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/robot1</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <update_rate>30</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.15</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>10</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>chassis</robot_base_frame>
    </plugin>

    <!-- IMU sensor -->
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <pose>0 0 0.05 0 0 0</pose>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/robot1</namespace>
          <remapping>imu:=imu/data</remapping>
        </ros>
        <frame_name>chassis</frame_name>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>

    <!-- LiDAR sensor -->
    <sensor name="lidar_sensor" type="ray">
      <pose>0.15 0 0.1 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_ros2" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/robot1</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </model>
</sdf>
```

### 2. Unity Visualization Setup

#### Unity Robot Controller Script
```csharp
using UnityEngine;
using Ros2Unity;
using Ros2Unity.Messages.GeometryMsgs;
using Ros2Unity.Messages.SensorMsgs;

public class UnityRobotController : MonoBehaviour
{
    [Header("ROS 2 Configuration")]
    [SerializeField] private string rosMasterUri = "http://localhost:11311";
    [SerializeField] private string robotNamespace = "/robot1";
    
    [Header("Robot Components")]
    [SerializeField] private Transform chassis;
    [SerializeField] private Transform leftWheel;
    [SerializeField] private Transform rightWheel;
    [SerializeField] private Transform lidar;
    [SerializeField] private Transform imu;
    
    private Ros2Node rosNode;
    private Subscriber<Odometry> odomSubscriber;
    private Publisher<Twist> cmdVelPublisher;
    private Publisher<LaserScan> lidarPublisher;
    
    private float wheelRadius = 0.075f; // 7.5cm radius
    private float wheelSeparation = 0.3f; // 30cm separation
    
    void Start()
    {
        // Initialize ROS 2
        Ros2CS.Init();
        rosNode = Ros2Node.CreateNode("unity_robot_bridge");
        
        // Create subscribers and publishers
        odomSubscriber = rosNode.CreateSubscriber<Odometry>(
            robotNamespace + "/odom", 
            10, 
            HandleOdometry
        );
        
        cmdVelPublisher = rosNode.CreatePublisher<Twist>(
            robotNamespace + "/cmd_vel", 
            10
        );
        
        lidarPublisher = rosNode.CreatePublisher<LaserScan>(
            robotNamespace + "/unity_scan", 
            10
        );
        
        // Start ROS communication
        StartCoroutine(RosSpin());
    }
    
    IEnumerator RosSpin()
    {
        while (Ros2CS.Ok())
        {
            Ros2CS.SpinOnce();
            yield return new WaitForSeconds(0.01f); // 100Hz
        }
    }
    
    void HandleOdometry(Odometry odom)
    {
        // Update robot position in Unity
        Vector3 position = new Vector3(
            (float)odom.pose.pose.position.x,
            (float)odom.pose.pose.position.z, // Map Z to Unity Y for visualization
            (float)odom.pose.pose.position.y
        );
        
        Quaternion rotation = new Quaternion(
            (float)odom.pose.pose.orientation.x,
            (float)odom.pose.pose.orientation.z,
            (float)odom.pose.pose.orientation.y,
            (float)odom.pose.pose.orientation.w
        );
        
        chassis.position = position;
        chassis.rotation = rotation;
        
        // Update wheel rotations based on velocity
        UpdateWheelRotations(odom.twist.twist);
    }
    
    void UpdateWheelRotations(Twist twist)
    {
        // Calculate wheel velocities from differential drive kinematics
        float linearVel = (float)twist.linear.x;
        float angularVel = (float)twist.angular.z;
        
        float leftVel = (linearVel - angularVel * wheelSeparation / 2.0f) / wheelRadius;
        float rightVel = (linearVel + angularVel * wheelSeparation / 2.0f) / wheelRadius;
        
        // Update wheel rotations
        if (leftWheel != null)
            leftWheel.Rotate(Vector3.right, leftVel * Time.deltaTime * Mathf.Rad2Deg);
        
        if (rightWheel != null)
            rightWheel.Rotate(Vector3.right, rightVel * Time.deltaTime * Mathf.Rad2Deg);
    }
    
    void OnApplicationQuit()
    {
        Ros2CS.Shutdown();
    }
}
```

### 3. ROS 2 Control Node

#### Navigation Controller
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/robot1/odom', self.odom_callback, 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Robot state
        self.current_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.lidar_data = None
        self.target_pose = np.array([5.0, 5.0, 0.0])  # Target position
        
        # PID controller parameters
        self.kp_linear = 1.0
        self.ki_linear = 0.1
        self.kd_linear = 0.05
        self.kp_angular = 2.0
        self.ki_angular = 0.1
        self.kd_angular = 0.1
        
        self.prev_error_linear = 0.0
        self.integral_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.integral_error_angular = 0.0
        
        self.get_logger().info('Navigation controller initialized')
    
    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle avoidance"""
        self.lidar_data = np.array(msg.ranges)
        # Filter out invalid ranges
        self.lidar_data = np.where(
            (self.lidar_data >= msg.range_min) & 
            (self.lidar_data <= msg.range_max), 
            self.lidar_data, 
            np.inf
        )
    
    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y
        
        # Convert quaternion to euler
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        r = R.from_quat(quat)
        euler = r.as_euler('xyz')
        self.current_pose[2] = euler[2]  # yaw
    
    def control_loop(self):
        """Main control loop"""
        if self.lidar_data is None:
            return
            
        # Calculate control commands
        cmd_vel = Twist()
        
        # Simple navigation to target with obstacle avoidance
        distance_to_target = np.linalg.norm(
            self.target_pose[:2] - self.current_pose[:2]
        )
        
        if distance_to_target > 0.5:  # If not at target
            # Calculate desired angle to target
            desired_angle = np.arctan2(
                self.target_pose[1] - self.current_pose[1],
                self.target_pose[0] - self.current_pose[0]
            )
            
            # Calculate angle error
            angle_error = desired_angle - self.current_pose[2]
            # Normalize angle to [-pi, pi]
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
            
            # PID control for angular velocity
            self.integral_error_angular += angle_error
            derivative_error_angular = angle_error - self.prev_error_angular
            
            angular_vel = (
                self.kp_angular * angle_error +
                self.ki_angular * self.integral_error_angular +
                self.kd_angular * derivative_error_angular
            )
            
            # Limit angular velocity
            angular_vel = np.clip(angular_vel, -1.0, 1.0)
            
            # Calculate linear velocity based on angular error and obstacles
            if abs(angle_error) < 0.5:  # Only move forward if roughly aligned
                # Check for obstacles ahead
                front_scan = self.lidar_data[320:400]  # Front 80 degrees
                min_distance = np.min(front_scan)
                
                if min_distance > 1.0:  # No close obstacles
                    linear_vel = min(0.5, distance_to_target * 0.5)
                else:
                    linear_vel = 0.0  # Stop if obstacle too close
                    # Add turning to avoid obstacle
                    angular_vel += 0.5 if np.random.random() > 0.5 else -0.5
            else:
                linear_vel = 0.0  # Don't move forward while turning
            
            # Update previous errors
            self.prev_error_angular = angle_error
            
            cmd_vel.linear.x = float(linear_vel)
            cmd_vel.angular.z = float(angular_vel)
        else:
            # At target, stop the robot
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Log status
        self.get_logger().debug(
            f'Pos: ({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}), '
            f'Dist to target: {distance_to_target:.2f}, '
            f'Cmd: ({cmd_vel.linear.x:.2f}, {cmd_vel.angular.z:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    
    controller = NavigationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Launch Configuration

#### ROS 2 Launch File
```python
# launch/digital_twin.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo server node
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so', 
             '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )
    
    # Gazebo client (optional, for visualization)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Robot spawn node
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mobile_robot',
            '-file', '/path/to/robot/model.sdf',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )
    
    # Navigation controller
    nav_controller = Node(
        package='digital_twin_examples',
        executable='navigation_controller',
        name='navigation_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Unity bridge node (if using ROS 2 for Unity communication)
    unity_bridge = Node(
        package='unity_ros2_bridge',
        executable='unity_bridge',
        name='unity_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_robot,
        nav_controller,
        unity_bridge
    ])
```

## Validation and Testing

### Performance Metrics
- Simulation-to-real time ratio (SSR)
- Communication latency between components
- Sensor data accuracy
- Control response time

### Validation Procedures
1. Compare simulated vs. real sensor data
2. Test control algorithms in both simulation and reality
3. Validate timing synchronization
4. Assess computational performance

## Best Practices

1. **Modular Design**: Keep components loosely coupled for easy maintenance
2. **Configuration Management**: Use parameter files for easy system configuration
3. **Error Handling**: Implement robust error handling and recovery mechanisms
4. **Performance Monitoring**: Continuously monitor system performance
5. **Documentation**: Maintain comprehensive documentation for all interfaces

## Troubleshooting

### Common Issues
- Network connectivity problems between components
- Coordinate system mismatches
- Timing synchronization issues
- Performance bottlenecks

### Solutions
- Verify network configurations and firewall settings
- Use standard ROS coordinate frames (REP-103)
- Implement proper time synchronization mechanisms
- Optimize data processing and communication rates

## Conclusion

This complete simulation environment example demonstrates the integration of Gazebo physics, Unity visualization, and ROS 2 messaging to create a comprehensive digital twin for robotics applications. The example provides a foundation that can be extended and customized for specific use cases and requirements.