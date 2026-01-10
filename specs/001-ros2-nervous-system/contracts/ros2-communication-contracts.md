# API Contract: ROS 2 Node Communication

## Overview
This contract defines the conceptual communication patterns between ROS 2 nodes for the educational examples in the ROS 2 Nervous System module.

## Node Communication Patterns

### Publisher-Subscriber Pattern

#### Topic: `/sensor_data`
- **Message Type**: `sensor_msgs/msg/JointState`
- **Purpose**: Broadcasting sensor readings from humanoid robot joints
- **Publisher**: Sensor Interface Node
- **Subscribers**: Perception Node, Control Node
- **QoS Profile**: Reliable delivery, transient local durability

#### Topic: `/robot_state`
- **Message Type**: `nav_msgs/msg/Odometry`
- **Purpose**: Broadcasting current state of the humanoid robot
- **Publisher**: State Estimation Node
- **Subscribers**: Control Node, Visualization Node
- **QoS Profile**: Reliable delivery, transient local durability

### Client-Server Pattern

#### Service: `/move_joint`
- **Request Type**: Custom `MoveJoint.srv`
- **Response Type**: Custom `MoveJoint.srv`
- **Purpose**: Request specific joint movement with position and velocity
- **Server**: Joint Controller Node
- **Clients**: High-level Motion Planning Node
- **Timeout**: 5 seconds

#### Service: `/get_robot_pose`
- **Request Type**: `std_srvs/srv/Empty`
- **Response Type**: `geometry_msgs/msg/PoseStamped`
- **Purpose**: Request current pose of the robot
- **Server**: Pose Estimation Node
- **Clients**: Navigation Node, Planning Node
- **Timeout**: 2 seconds

### Action Pattern

#### Action: `/navigate_to_pose`
- **Goal Type**: `nav2_msgs/action/NavigateToPose`
- **Feedback Type**: `nav2_msgs/action/NavigateToPose`
- **Result Type**: `nav2_msgs/action/NavigateToPose`
- **Purpose**: Navigate humanoid robot to a specified pose
- **Server**: Navigation Server Node
- **Clients**: Task Planning Node
- **Timeout**: 60 seconds for completion

## Node Definitions

### Sensor Interface Node
- **Purpose**: Interface between physical/hardware sensors and ROS 2
- **Publishes**:
  - `/sensor_data` (JointState)
  - `/imu_data` (Imu)
- **Subscribes**: None
- **Services**: None
- **Actions**: None

### Perception Node
- **Purpose**: Process sensor data to extract meaningful information
- **Publishes**:
  - `/environment_map` (OccupancyGrid)
  - `/detected_objects` (ObjectArray)
- **Subscribes**:
  - `/sensor_data` (JointState)
- **Services**: None
- **Actions**: None

### Control Node
- **Purpose**: Generate control commands for robot actuators
- **Publishes**:
  - `/joint_commands` (JointTrajectory)
- **Subscribes**:
  - `/sensor_data` (JointState)
  - `/robot_state` (Odometry)
  - `/motion_plan` (Path)
- **Services**: None
- **Actions**: None

## Quality of Service (QoS) Guidelines

For educational purposes, the following QoS profiles are recommended:

- **Sensor Data**: Reliable delivery with transient local durability for important state information
- **Control Commands**: Reliable delivery with volatile durability for real-time control
- **Planning Information**: Best effort delivery with volatile durability for non-critical information