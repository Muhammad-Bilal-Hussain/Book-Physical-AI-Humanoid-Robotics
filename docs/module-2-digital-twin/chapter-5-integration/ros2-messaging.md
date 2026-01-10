# ROS 2 Messaging Integration

## Introduction to ROS 2 Messaging

ROS 2 (Robot Operating System 2) provides the communication infrastructure that enables integration between Gazebo physics simulation, Unity visualization, and other robotic components. Understanding ROS 2 messaging is crucial for creating effective integrated simulation environments. This chapter explores the various messaging patterns, topics, services, and actions used in simulation environments.

## ROS 2 Architecture Overview

### Communication Patterns

#### Topics (Publish/Subscribe)
- **Pattern**: One-to-many communication
- **Use Case**: Continuous data streams (sensor data, joint states)
- **Characteristics**: Asynchronous, fire-and-forget
- **Quality of Service**: Configurable reliability and durability

#### Services (Request/Response)
- **Pattern**: One-to-one synchronous communication
- **Use Case**: Actionable requests (spawning models, resetting simulation)
- **Characteristics**: Blocking until response received
- **Quality of Service**: Request-response pattern

#### Actions (Goal/Feedback/Result)
- **Pattern**: Long-running tasks with feedback
- **Use Case**: Navigation, trajectory execution
- **Characteristics**: Goal-oriented with ongoing feedback
- **Quality of Service**: Multi-message interaction pattern

## Core ROS 2 Messages for Simulation

### Standard Message Types

#### Sensor Data Messages
- **`sensor_msgs/JointState`**: Joint positions, velocities, and efforts
- **`sensor_msgs/Image`**: Camera image data
- **`sensor_msgs/LaserScan`**: LiDAR scan data
- **`sensor_msgs/Imu`**: Inertial measurement unit data
- **`sensor_msgs/PointCloud2`**: 3D point cloud data

#### Geometry Messages
- **`geometry_msgs/Twist`**: Linear and angular velocities
- **`geometry_msgs/Pose`**: Position and orientation
- **`geometry_msgs/TransformStamped`**: Coordinate transformations
- **`geometry_msgs/Vector3`**: 3D vector data

#### Standard Messages
- **`std_msgs/String`**: Simple string data
- **`std_msgs/Float64`**: Single floating-point value
- **`std_msgs/Int32`**: Single integer value
- **`builtin_interfaces/Time`**: Timestamp information

## ROS 2 Topics in Simulation

### Essential Simulation Topics

#### Joint State Management
- **`/joint_states`**: Published by Gazebo, subscribed by Unity
- **Message Type**: `sensor_msgs/JointState`
- **Frequency**: Typically 50-100 Hz
- **Purpose**: Synchronize robot joint positions between systems

#### Robot State Information
- **`/tf` and `/tf_static`**: Transform relationships
- **Message Type**: `tf2_msgs/TFMessage`
- **Frequency**: Variable, depending on change rate
- **Purpose**: Maintain coordinate frame relationships

#### Sensor Data Streams
- **`/camera/image_raw`**: Camera image data
- **`/scan`**: LiDAR scan data
- **`/imu/data`**: IMU measurements
- **`/odom`**: Odometry information

#### Control Commands
- **`/cmd_vel`**: Velocity commands for mobile robots
- **`/joint_commands`**: Joint position/velocity/effort commands
- **`/trajectory`**: Trajectory execution commands

### Quality of Service (QoS) Settings

#### Reliability
- **Reliable**: All messages guaranteed to be delivered
- **Best Effort**: Messages may be dropped (suitable for sensor data)

#### Durability
- **Transient Local**: Late-joining subscribers receive last message
- **Volatile**: Only new messages sent to subscribers

#### Lifespan
- **History**: Keep all messages or only last N messages
- **Depth**: Maximum number of messages to store

## ROS 2 Services in Simulation

### Essential Simulation Services

#### Model Management
- **`/spawn_entity`**: Spawn new models in Gazebo
- **`/delete_entity`**: Remove models from Gazebo
- **`/get_entity_state`**: Query model state
- **`/set_entity_state`**: Set model state

#### Simulation Control
- **`/reset_simulation`**: Reset entire simulation
- **`/pause_physics`**: Pause physics simulation
- **`/unpause_physics`**: Resume physics simulation
- **`/set_physics_properties`**: Configure physics parameters

#### Parameter Management
- **`/set_parameters`**: Set node parameters
- **`/get_parameters`**: Query node parameters
- **`/list_parameters`**: List available parameters

## ROS 2 Actions in Simulation

### Navigation Actions
- **`/navigate_to_pose`**: Navigate to specific pose
- **`/follow_waypoints`**: Follow a sequence of waypoints
- **`/compute_path_to_pose`**: Plan path to destination

### Manipulation Actions
- **`/follow_joint_trajectory`**: Execute joint trajectory
- **`/pick_and_place`**: Perform pick and place operation
- **`/move_group`**: Move robot arm to goal

## Implementation Patterns

### Publisher Implementation

#### C++ Publisher Example
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher() : Node("joint_state_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 100 Hz
            std::bind(&JointStatePublisher::publish_joint_states, this));
    }

private:
    void publish_joint_states()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        
        // Set joint names
        msg.name = {"joint1", "joint2", "joint3"};
        
        // Set joint positions
        msg.position = {0.1, 0.2, 0.3};
        
        // Set joint velocities
        msg.velocity = {0.0, 0.0, 0.0};
        
        // Set joint efforts
        msg.effort = {0.0, 0.0, 0.0};
        
        publisher_->publish(msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
```

#### Python Publisher Example
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(timer_period, self.publish_joint_states)
        self.i = 0

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set joint names
        msg.name = ['joint1', 'joint2', 'joint3']
        
        # Set joint positions with some oscillation
        msg.position = [
            math.sin(self.i * 0.1),
            math.cos(self.i * 0.1) * 0.5,
            math.sin(self.i * 0.05) * 0.3
        ]
        
        # Set joint velocities
        msg.velocity = [0.0, 0.0, 0.0]
        
        # Set joint efforts
        msg.effort = [0.0, 0.0, 0.0]
        
        self.publisher.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Implementation

#### C++ Subscriber Example
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStateSubscriber : public rclcpp::Node
{
public:
    JointStateSubscriber() : Node("joint_state_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::QoS(10).reliable(),
            std::bind(&JointStateSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received joint states:");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), 
                "  %s: pos=%.3f, vel=%.3f, eff=%.3f", 
                msg->name[i].c_str(), 
                msg->position[i], 
                msg->velocity[i], 
                msg->effort[i]);
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};
```

#### Unity ROS Subscriber Example
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;

public class JointStateSubscriber : MonoBehaviour
{
    [SerializeField]
    private string topicName = "/joint_states";
    
    [SerializeField]
    private GameObject[] jointObjects;
    
    void Start()
    {
        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(topicName, OnJointStateReceived);
    }
    
    void OnJointStateReceived(JointStateMsg jointState)
    {
        // Update joint objects based on received joint states
        for (int i = 0; i < jointState.name.Length && i < jointObjects.Length; i++)
        {
            int jointIndex = System.Array.IndexOf(jointState.name, jointObjects[i].name);
            if (jointIndex >= 0)
            {
                float angle = (float)jointState.position[jointIndex];
                jointObjects[i].transform.localRotation = 
                    Quaternion.Euler(0, angle * Mathf.Rad2Deg, 0);
            }
        }
    }
}
```

## Advanced Messaging Concepts

### Custom Message Types

#### Creating Custom Messages
```python
# custom_msgs/msg/SimulationState.msg
string simulation_name
bool is_paused
float64 simulation_time
float64 real_time_factor
int32 model_count
```

#### Using Custom Messages
```python
# Publisher
from custom_msgs.msg import SimulationState

publisher = node.create_publisher(SimulationState, '/simulation_state', 10)

msg = SimulationState()
msg.simulation_name = 'test_simulation'
msg.is_paused = False
msg.simulation_time = 10.5
msg.real_time_factor = 1.0
msg.model_count = 5

publisher.publish(msg)
```

### Lifecycle Management

#### Node Lifecycle
- **Unconfigured**: Node created but not configured
- **Inactive**: Configured but not active
- **Active**: Fully operational
- **Finalized**: Node destroyed

#### Managing Node States
```python
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleJointStatePublisher(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_joint_state_publisher')
    
    def on_configure(self, state):
        self.get_logger().info("Configuring...")
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        self.get_logger().info("Activating...")
        self.timer = self.create_timer(0.01, self.publish_joint_states)
        return TransitionCallbackReturn.SUCCESS
```

## Performance Optimization

### Message Efficiency

#### Message Size Reduction
- **Delta Encoding**: Send only changes from previous state
- **Compression**: Compress large messages when possible
- **Throttling**: Limit message frequency to necessary rates
- **Aggregation**: Combine multiple small messages

#### Network Optimization
- **Local Communication**: Use shared memory for local nodes
- **DDS Configuration**: Optimize DDS settings for performance
- **Topic Partitioning**: Separate high-frequency from low-frequency topics
- **Connection Management**: Efficient connection handling

### Quality of Service Tuning

#### High-Frequency Data (e.g., sensor data)
```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

sensor_qos = QoSProfile(
    depth=1,  # Only keep the latest message
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.BEST_EFFORT
)
```

#### Critical Data (e.g., commands)
```python
command_qos = QoSProfile(
    depth=10,  # Keep more messages
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE
)
```

## Troubleshooting Common Issues

### Message Loss
- **Symptoms**: Missing messages or inconsistent data
- **Causes**: Network congestion, insufficient buffer sizes
- **Solutions**: Increase queue sizes, reduce frequency, improve network

### Timing Issues
- **Symptoms**: Outdated or delayed messages
- **Causes**: Slow processing, network latency
- **Solutions**: Optimize processing, use appropriate QoS settings

### Type Mismatches
- **Symptoms**: Failed subscriptions or malformed data
- **Causes**: Incorrect message types or versions
- **Solutions**: Verify message definitions, rebuild workspace

## Integration with Simulation Systems

### Gazebo Integration
- **Plugins**: Use Gazebo ROS 2 plugins for native integration
- **Services**: Leverage Gazebo services for simulation control
- **Topics**: Subscribe to Gazebo-published topics

### Unity Integration
- **ROS-TCP-Connector**: Use for Unity-ROS communication
- **Message Serialization**: Efficient conversion between systems
- **Threading**: Proper thread management for communication

ROS 2 messaging provides the essential communication infrastructure for integrated simulation environments. By understanding and properly implementing these messaging patterns, developers can create robust, efficient, and scalable simulation systems that effectively connect all components of their robotic simulation pipeline.