# Chapter 2: Communication Primitives in ROS 2

## Node Definition and Explanation

### What is a Node?

In ROS 2, a node is a fundamental component that performs computation. Nodes are the basic building blocks of a ROS 2 system. Each node typically handles a specific function, such as sensor processing, path planning, or motor control. Nodes are designed to be modular and independent, allowing for flexible and scalable robot systems.

Technically, a node is an instance of a ROS 2 process that has access to the ROS 2 communication infrastructure. Each node can:

- Publish messages to topics
- Subscribe to messages from topics
- Provide services
- Call services
- Send and receive actions
- Manage parameters

### Node Characteristics

#### Independence
Nodes operate independently of each other. This means that if one node fails, it doesn't necessarily bring down the entire system. This independence is crucial for robot safety and reliability.

#### Single Responsibility
Each node should have a single, well-defined responsibility. For example:
- A camera driver node handles camera data acquisition
- A localization node estimates the robot's position
- A motor controller node sends commands to motors

This approach follows the single responsibility principle, making the system easier to understand, test, and maintain.

#### Communication Interface
Nodes communicate with each other through ROS 2's communication primitives: topics, services, and actions. This allows nodes to be developed separately and then combined to create complex behaviors.

### Creating Nodes

Nodes are created using client libraries like rclpy (Python) or rclcpp (C++). Here's a conceptual example of what happens when creating a node:

1. Initialize the ROS 2 client library
2. Create a node with a unique name
3. Create publishers, subscribers, services, or other communication interfaces
4. Implement the node's functionality
5. Spin the node to process callbacks

### Node Lifecycle

ROS 2 provides a lifecycle system for nodes that allows for more sophisticated management of complex systems. The lifecycle states include:

- Unconfigured: The node is created but not yet configured
- Inactive: The node is configured but not yet activated
- Active: The node is running and performing its function
- Finalized: The node has been shut down

This lifecycle system is particularly useful for humanoid robots where different subsystems may need to be brought up in a specific order or deactivated for safety reasons.

### Naming and Namespaces

Nodes have unique names within the ROS 2 system. To avoid naming conflicts, ROS 2 supports namespaces, which allow nodes to be organized hierarchically. For example, a humanoid robot might have nodes named:

- `/left_arm/controller`
- `/right_arm/controller`
- `/torso/sensors`
- `/head/camera`

This naming convention helps organize complex robot systems and prevents naming conflicts.

### Node Management

ROS 2 provides tools for managing nodes:

- `ros2 node list`: Shows all active nodes
- `ros2 node info <node_name>`: Shows information about a specific node
- Launch files: Allow multiple nodes to be started together with specific configurations

These tools are essential for debugging and monitoring robot systems during development and operation.

## Topic Definition and Explanation

### What is a Topic?

A topic in ROS 2 is a named channel through which nodes exchange messages. Topics implement a publish-subscribe communication pattern, where one or more nodes publish messages to a topic, and one or more nodes subscribe to messages from that topic. This creates an asynchronous, decoupled communication system where publishers and subscribers don't need to know about each other.

Topics are fundamental to ROS 2's distributed architecture. They allow data to flow between nodes without tight coupling, enabling flexible and scalable robot systems.

### Topic Characteristics

#### Asynchronous Communication
Topics provide asynchronous communication between nodes. Publishers send messages without waiting for responses, and subscribers receive messages as they arrive. This allows for efficient data distribution in real-time systems.

#### Named Channels
Topics are identified by unique names (e.g., `/sensor_data`, `/robot_state`). These names follow a hierarchical namespace system, similar to file paths in operating systems.

#### Message Types
Each topic has a specific message type that defines the structure of the data being exchanged. All messages published to a topic must conform to this type. Common message types include:
- `std_msgs`: Basic data types (integers, floats, strings)
- `sensor_msgs`: Sensor data (images, laser scans, joint states)
- `geometry_msgs`: Geometric data (poses, vectors, quaternions)
- `nav_msgs`: Navigation data (paths, occupancy grids)

#### Many-to-Many Communication
Topics support many-to-many communication patterns. Multiple nodes can publish to the same topic, and multiple nodes can subscribe to the same topic. This enables flexible data distribution patterns.

### Quality of Service (QoS) Settings

Topics in ROS 2 support Quality of Service (QoS) settings that allow fine-tuning of communication behavior:

#### Reliability
- **Reliable**: All messages are guaranteed to be delivered (may be slower)
- **Best Effort**: Messages may be lost (faster but not guaranteed)

For example, sensor data might use best effort for real-time performance, while critical safety messages would use reliable delivery.

#### Durability
- **Transient Local**: Late-joining subscribers receive the last message published
- **Volatile**: New subscribers only receive future messages

This is useful for static data like robot configuration that new nodes need to receive immediately.

#### History
- **Keep Last**: Store only the most recent messages
- **Keep All**: Store all messages (use with caution due to memory usage)

#### Rate and Deadline
These settings allow for time-based constraints on message delivery, important for real-time robotic systems.

### Topic Usage in Humanoid Robots

Topics are extensively used in humanoid robots for various purposes:

#### Sensor Data Distribution
- `/joint_states`: Current positions, velocities, and efforts of robot joints
- `/imu/data`: Inertial measurement unit data for orientation and acceleration
- `/camera/color/image_raw`: Raw image data from cameras
- `/scan`: Laser scan data for obstacle detection

#### State Information
- `/robot_state`: Overall robot state information
- `/tf`: Transform data between coordinate frames
- `/battery_status`: Battery level and health information

#### Control Commands
- `/cmd_vel`: Velocity commands for base movement
- `/arm_controller/commands`: Commands for arm movements
- `/head_controller/commands`: Commands for head movements

### Creating and Using Topics

To use topics, nodes must:

1. **Create a Publisher**: Define a publisher for a specific topic name and message type
2. **Publish Messages**: Send messages to the topic at appropriate intervals
3. **Create a Subscriber**: Define a subscriber for a specific topic name and message type
4. **Process Messages**: Implement callback functions to handle incoming messages

### Topic Tools

ROS 2 provides several tools for working with topics:

- `ros2 topic list`: Shows all active topics
- `ros2 topic echo <topic_name>`: Displays messages being published to a topic
- `ros2 topic info <topic_name>`: Shows information about a topic and its publishers/subscribers
- `ros2 topic pub <topic_name> <msg_type> <args>`: Publishes messages to a topic from the command line

These tools are invaluable for debugging and understanding the data flow in robot systems.

## Service Definition and Explanation

### What is a Service?

A service in ROS 2 provides a request-response communication pattern between nodes. Unlike topics, which provide asynchronous, one-way communication, services offer synchronous, two-way communication. A node sends a request to a service and waits for a response. This is similar to a function call in traditional programming, but across the network between different nodes.

Services are essential for operations that require a response or confirmation, such as configuration changes, specific actions, or queries that return results.

### Service Characteristics

#### Synchronous Communication
Services provide synchronous communication where the client node sends a request and waits for a response. This ensures that the client receives a response before continuing, which is important for operations that require confirmation or results.

#### Request-Response Pattern
Each service has two message types:
- **Request**: The message sent by the client to the service server
- **Response**: The message sent by the service server back to the client

For example, a service to move a robot joint might have:
- Request: Joint name, target position, speed
- Response: Success flag, error message if any

#### One-to-One Communication
Unlike topics which support many-to-many communication, services typically involve one server and one client at a time. However, multiple clients can use the same service sequentially.

#### Blocking Operation
When a client calls a service, it typically blocks until it receives a response or a timeout occurs. This ensures that the operation completes before the client continues.

### Service Structure

Services are defined using service definition files with the `.srv` extension. These files contain two parts separated by a line with three dashes (`---`):

```
# Request part (above the ---)
float64 target_position
string joint_name
---
# Response part (below the ---)
bool success
string message
```

### Quality of Service for Services

While services don't use the same QoS settings as topics, they do have timeout configurations that determine how long a client will wait for a response before giving up.

### Service Usage in Humanoid Robots

Services are commonly used in humanoid robots for operations that require immediate responses:

#### Configuration Requests
- `/set_parameters`: Change robot configuration parameters
- `/load_map`: Load a new map for navigation
- `/set_mode`: Change robot operational mode

#### Action Requests
- `/move_joint`: Move a specific joint to a target position
- `/gripper_control`: Open or close a gripper
- `/reset_odometry`: Reset the robot's position estimate

#### Query Operations
- `/get_robot_pose`: Get the current robot position
- `/get_battery_status`: Get battery level and health
- `/get_joint_positions`: Get current positions of all joints

### Creating and Using Services

To use services, nodes must:

1. **Create a Service Server**: Define a service with a specific name and message type, and implement a callback function to handle requests
2. **Create a Service Client**: Define a client for a specific service name and message type
3. **Send Requests**: Create request objects and send them to the service
4. **Process Responses**: Handle the responses returned by the service

### Service Tools

ROS 2 provides tools for working with services:

- `ros2 service list`: Shows all available services
- `ros2 service info <service_name>`: Shows information about a specific service
- `ros2 service call <service_name> <service_type> <request_args>`: Calls a service from the command line

These tools are useful for testing and debugging service-based functionality in robot systems.

## Action Definition and Explanation

### What is an Action?

An action in ROS 2 provides a communication pattern for long-running tasks that require feedback during execution. Actions combine features of both topics and services, offering a goal-oriented communication pattern with continuous feedback and final results. They are ideal for tasks that take a significant amount of time to complete and where the client needs to monitor progress or potentially cancel the operation.

Actions are particularly important in robotics for tasks like navigation, manipulation, and calibration that require ongoing communication between the client and server during execution.

### Action Characteristics

#### Goal-Feedback-Result Pattern
Actions follow a three-part communication pattern:
- **Goal**: The client sends a goal to the action server
- **Feedback**: The server sends periodic feedback updates during execution
- **Result**: The server sends a final result when the action completes

#### Long-Running Operations
Actions are designed for operations that take a significant amount of time to complete, such as:
- Moving a robot to a distant location
- Performing a complex manipulation task
- Calibrating sensors or actuators

#### Cancellation Support
Unlike services, actions support cancellation. A client can request to cancel an action while it's in progress, allowing for responsive and safe robot behavior.

#### Preemption
Actions support preemption, where a new goal can replace a currently executing goal, which is useful for dynamic environments.

### Action Structure

Actions are defined using action definition files with the `.action` extension. These files contain three parts separated by lines with three dashes (`---`):

```
# Goal part (before first ---)
float64 target_x
float64 target_y
float64 target_theta
---
# Result part (between ---)
bool success
string message
---
# Feedback part (after second ---)
float64 current_x
float64 current_y
float64 current_theta
int32 percent_complete
```

### Action States

Actions can be in various states during their lifecycle:
- **Pending**: Goal received but not yet started
- **Active**: Goal is currently being processed
- **Succeeded**: Goal completed successfully
- **Aborted**: Goal execution failed
- **Preempted**: Goal was replaced by a new goal
- **Canceled**: Goal was canceled by the client

### Action Usage in Humanoid Robots

Actions are extensively used in humanoid robots for complex, long-running operations:

#### Navigation Tasks
- `/navigate_to_pose`: Move the robot to a specific location and orientation
- `/follow_waypoints`: Navigate through a sequence of waypoints
- `/explore_area`: Explore an unknown environment

#### Manipulation Tasks
- `/pick_object`: Pick up an object from a specific location
- `/place_object`: Place an object at a specific location
- `/move_arm`: Move the robot's arm to a specific configuration

#### Calibration and Maintenance
- `/calibrate_sensors`: Calibrate various sensors
- `/check_balance`: Perform balance checks and adjustments
- `/perform_self_diagnostic`: Run diagnostic routines

### Creating and Using Actions

To use actions, nodes must:

1. **Create an Action Server**: Define an action with a specific name and message type, and implement callback functions to handle goals, cancellations, and preemptions
2. **Create an Action Client**: Define a client for a specific action name and message type
3. **Send Goals**: Create goal objects and send them to the action server
4. **Monitor Progress**: Receive and process feedback messages during execution
5. **Receive Results**: Handle the final result when the action completes

### Action Tools

ROS 2 provides tools for working with actions:

- `ros2 action list`: Shows all available actions
- `ros2 action info <action_name>`: Shows information about a specific action
- `ros2 action send_goal <action_name> <action_type> <goal_args>`: Sends a goal to an action from the command line

These tools are valuable for testing and debugging action-based functionality in robot systems.

### When to Use Actions vs. Services vs. Topics

Choosing the right communication primitive depends on the specific requirements:

- **Use Topics** for continuous data streams (sensor data, robot state)
- **Use Services** for quick, synchronous operations that return a result immediately
- **Use Actions** for long-running operations that require feedback or the possibility of cancellation

Actions provide the most sophisticated communication pattern in ROS 2, making them essential for complex robotic behaviors.

## Practical Examples with Humanoid Robots

### Communication Patterns in Real Humanoid Systems

Understanding how communication primitives work in practice is essential for developing effective humanoid robot systems. Let's explore how these patterns manifest in real-world humanoid robot applications.

### Example 1: Walking Pattern Generation

Consider a humanoid robot that needs to walk to a specific location. This involves multiple communication patterns:

#### Topic Communication
- `/joint_states`: Published by the robot's joint controllers, providing real-time information about joint angles and velocities
- `/imu/data`: Published by the IMU sensor, providing orientation and acceleration data for balance control
- `/tf`: Published by the transform broadcaster, providing the relationship between different coordinate frames

#### Service Communication
- `/balance_control/set_mode`: A service that allows switching between different balance control modes (e.g., standing, walking, sitting)
- `/gait_planner/load_trajectory`: A service to load predefined walking patterns

#### Action Communication
- `/walking_controller/move_to_pose`: An action that takes a destination pose as a goal, provides feedback about the current step and balance status, and returns a result indicating success or failure

### Example 2: Object Manipulation

When a humanoid robot needs to pick up an object, the communication follows this pattern:

#### Topic Communication
- `/camera/color/image_raw`: Camera feed for object detection
- `/arm_controller/state`: Current state of the arm controller
- `/gripper/joint_states`: State of the gripper joints

#### Service Communication
- `/object_detection/detect_objects`: A service that processes the camera image and returns a list of detected objects with their poses
- `/inverse_kinematics/solve`: A service that calculates the joint angles needed to reach a specific end-effector pose

#### Action Communication
- `/arm_controller/follow_joint_trajectory`: An action that moves the arm along a planned trajectory, providing feedback about progress and returning success or failure
- `/gripper_controller/grasp`: An action that controls the gripper to grasp an object, providing feedback about grip force and returning success when the object is securely held

### Example 3: Human-Robot Interaction

For a humanoid robot engaging in social interaction:

#### Topic Communication
- `/audio/input`: Microphone input for speech recognition
- `/face_detector/face_locations`: Detected face positions for attention control
- `/robot_state`: Current emotional state or mode of the robot

#### Service Communication
- `/speech_recognition/recognize`: A service that processes audio input and returns recognized text
- `/dialog_manager/get_response`: A service that generates appropriate responses based on the conversation context

#### Action Communication
- `/head_controller/look_at`: An action that makes the robot look at a specific person, providing feedback about gaze direction
- `/animation_player/play_animation`: An action that plays a sequence of movements for expressive gestures

### Communication Pattern Selection Guidelines

When designing communication for humanoid robots, consider these guidelines:

#### Use Topics When:
- Data needs to be continuously streamed (sensor data, robot state)
- Multiple nodes need to consume the same data
- Real-time performance is critical
- The data producer doesn't need to know who consumes the data

#### Use Services When:
- A specific task needs to be performed and a result returned
- The operation is relatively quick (less than a few seconds)
- The client needs to wait for completion before proceeding
- A simple request-response pattern is sufficient

#### Use Actions When:
- The operation takes a long time to complete
- The client needs feedback during execution
- The operation might need to be canceled or preempted
- The result depends on conditions that may change during execution

### Quality of Service Considerations

Different communication patterns in humanoid robots require different QoS settings:

#### Safety-Critical Topics
- `/emergency_stop`: Reliable, transient local durability
- `/motor_torque_limits`: Reliable, transient local durability

#### Performance-Critical Topics
- `/camera/image_raw`: Best effort, volatile (some dropped frames acceptable)
- `/imu/data`: Reliable, volatile (critical for balance)

#### Configuration Services
- `/set_parameters`: Reliable, timeout appropriate for operation
- `/load_config`: Reliable, longer timeout acceptable

#### Interactive Actions
- `/speak_text`: Best effort for feedback, reliable for goal/result
- `/navigate_to_pose`: Reliable for goal/result, best effort for feedback

### Integration Example: Navigation and Manipulation

A complex task like "go to the kitchen and pick up a cup" would integrate all communication primitives:

1. **Navigation Phase**:
   - Topics: `/amcl_pose` (localization), `/scan` (obstacle detection)
   - Service: `/move_base_flex/get_path` (path planning)
   - Action: `/move_base_flex/exe_path` (execute navigation)

2. **Manipulation Phase**:
   - Topics: `/camera/color/image_raw` (object detection), `/joint_states` (feedback)
   - Service: `/object_detection/detect_cup` (identify target)
   - Action: `/arm_controller/follow_joint_trajectory` (reach and grasp)

This integrated approach allows humanoid robots to perform complex, multi-step tasks by combining the appropriate communication patterns for each subtask.

## Quality of Service (QoS) Concepts

### Introduction to QoS in ROS 2

Quality of Service (QoS) in ROS 2 refers to a set of policies that define how data is communicated between publishers and subscribers. QoS settings allow developers to fine-tune the behavior of their communication to match the specific requirements of their application. This is particularly important in robotics applications where different data streams have different requirements for reliability, latency, and durability.

QoS policies are negotiated between publishers and subscribers when they discover each other. If the QoS policies are incompatible, the communication may not work as expected.

### Core QoS Policies

#### Reliability Policy
The reliability policy determines whether messages are guaranteed to be delivered:

- **Reliable**: All messages are guaranteed to be delivered. If a message is lost, it will be resent. This is important for critical data like safety messages or control commands.
- **Best Effort**: Messages may be lost. This is suitable for data where occasional loss is acceptable, such as video streams or high-frequency sensor data.

For humanoid robots, reliable communication is essential for safety-critical systems like emergency stops, while best effort may be acceptable for high-frequency sensor data where the latest value is more important than receiving every single message.

#### Durability Policy
The durability policy determines how messages are handled for late-joining subscribers:

- **Transient Local**: The publisher keeps the most recent message for each topic and sends it to new subscribers immediately. This is useful for static data like robot configuration or maps.
- **Volatile**: New subscribers only receive messages published after they join. This is suitable for continuous data streams like sensor readings.

In humanoid robots, transient local durability is often used for topics like `/robot_description` (which contains the URDF), so new nodes immediately receive the robot's structure.

#### History Policy
The history policy determines how many messages are stored for each topic:

- **Keep Last**: Only the most recent messages are kept. The number of messages to keep is configurable.
- **Keep All**: All messages are kept. This should be used carefully as it can consume significant memory.

For humanoid robots, keep last is typically used for sensor data where only the most recent values are relevant.

#### Depth Policy
When using the "Keep Last" history policy, the depth determines how many samples are kept in the queue. For example, a depth of 10 means the last 10 messages are stored.

### Advanced QoS Settings

#### Deadline Policy
The deadline policy tells how often data should be published. This helps detect when a publisher is not sending updates as expected.

#### Lifespan Policy
The lifespan policy sets a time limit for messages. Messages older than this limit are automatically deleted.

#### Liveliness Policy
The liveliness policy lets nodes detect when other nodes or topics become unavailable. This is important for safety systems in humanoid robots.

### QoS in Humanoid Robot Applications

Different aspects of humanoid robot systems require different QoS settings:

#### Sensor Data
- `/joint_states`: Reliable, volatile, keep last (10) - Need all joint data, but only recent values matter
- `/imu/data`: Reliable, volatile, keep last (5) - Critical for balance, but only recent data needed
- `/camera/color/image_raw`: Best effort, volatile, keep last (1) - OK to drop frames, only latest frame matters

#### Control Commands
- `/cmd_vel`: Reliable, volatile, keep last (1) - Need to receive commands reliably, but only latest command matters
- `/joint_group_position_controller/command`: Reliable, volatile, keep last (1) - Critical for safety

#### Static Information
- `/robot_description`: Reliable, transient local, keep last (1) - New nodes need this immediately
- `/map`: Reliable, transient local, keep last (1) - New nodes need the map immediately

### QoS Matching and Compatibility

When a publisher and subscriber connect, their QoS policies are matched. Some policies must be compatible:

- Reliability: A reliable publisher can pair with a best-effort subscriber, but a best-effort publisher cannot pair with a reliable subscriber
- Durability: A transient local publisher can pair with a volatile subscriber, but not vice versa

### Setting QoS in Code

In ROS 2, QoS policies are set when creating publishers and subscribers. ROS 2 provides some predefined QoS profiles for common use cases:

- `rclcpp::QoS(10)`: Default profile with depth of 10
- `rmw_qos_profile_sensor_data`: Optimized for sensor data (best effort, volatile)
- `rmw_qos_profile_services_default`: Default for services (reliable, volatile)
- `rmw_qos_profile_parameter_events`: For parameter events

### Best Practices for QoS

1. **Match QoS to Requirements**: Choose QoS settings based on the specific needs of your data, not just default settings.

2. **Consider Resource Usage**: Transient local durability and keep all history can consume significant memory.

3. **Test with Different Settings**: Experiment with different QoS settings to find the optimal balance between performance and reliability.

4. **Document QoS Choices**: Document why specific QoS settings were chosen for each topic, especially in complex systems like humanoid robots.

5. **Monitor Performance**: Use ROS 2 tools to monitor the performance of your communication and adjust QoS settings as needed.

Understanding and properly configuring QoS settings is crucial for creating robust and efficient communication in humanoid robot systems.

## Comparison of Communication Primitives

### Overview of ROS 2 Communication Primitives

ROS 2 provides three primary communication patterns: topics, services, and actions. Each serves different purposes and has distinct characteristics. Understanding when to use each is crucial for effective robot system design.

### Detailed Comparison Table

| Aspect | Topics | Services | Actions |
|--------|--------|----------|---------|
| **Communication Pattern** | Publish/Subscribe | Request/Response | Goal/Feedback/Result |
| **Synchronicity** | Asynchronous | Synchronous | Synchronous (but with feedback) |
| **Data Flow** | One-way (publisher to subscriber) | Two-way (request and response) | Two-way (goal, feedback, result) |
| **Duration** | Continuous | Immediate | Long-running |
| **Feedback** | No (unless another topic is used) | No (response is final) | Yes (continuous feedback during execution) |
| **Cancellation** | No | No | Yes |
| **Preemption** | No | No | Yes |
| **Use Case** | Continuous data streams | Quick operations with immediate results | Long-running operations requiring monitoring |
| **Examples** | `/joint_states`, `/imu/data`, `/tf` | `/get_map`, `/set_mode`, `/transform_point` | `/move_base`, `/arm_controller`, `/pick_object` |
| **Error Handling** | Publisher/subscriber discovery | Service call failure | Goal rejection, execution failure |
| **Performance** | High throughput | Medium (network round-trip) | Medium to low (extended communication) |
| **Complexity** | Low | Medium | High |

### When to Use Each Primitive

#### Topics
Use topics when you need to:
- Stream continuous data (sensor readings, robot state)
- Broadcast information to multiple subscribers
- Achieve high-frequency communication
- Implement loose coupling between nodes
- Share data that multiple nodes need simultaneously

#### Services
Use services when you need to:
- Perform a specific task and get an immediate result
- Execute operations that should complete quickly (under a few seconds)
- Implement request-response patterns
- Get information from another node (queries)
- Perform configuration changes that return success/failure

#### Actions
Use actions when you need to:
- Execute long-running tasks (navigation, manipulation)
- Monitor progress during execution
- Allow for cancellation or preemption of tasks
- Provide feedback during task execution
- Handle operations that might fail partway through

### Communication Pattern Selection Decision Tree

When deciding which communication pattern to use, consider this decision tree:

1. **Does the operation need to return a result immediately?**
   - Yes → Go to 2
   - No → Likely a Topic

2. **Is the operation expected to complete quickly (under a few seconds)?**
   - Yes → Service
   - No → Go to 3

3. **Do you need to monitor progress during execution?**
   - Yes → Action
   - No → Go to 4

4. **Do you need the ability to cancel the operation?**
   - Yes → Action
   - No → Service (if result needed) or Topic (if no result needed)

### Hybrid Approaches

Sometimes, a combination of communication patterns works best:

#### Topic + Service
- Use a topic for continuous status updates
- Use a service for specific commands or queries
- Example: Publish robot battery status continuously via topic, but use a service to request immediate charging

#### Topic + Action
- Use a topic for general state information
- Use an action for specific long-running tasks
- Example: Publish robot pose continuously, but use an action to navigate to a specific location

#### Service + Action
- Use a service for quick configuration
- Use an action for the actual execution
- Example: Use a service to plan a path, then use an action to follow the path

### Humanoid Robot Examples

#### Walking Control
- **Topic**: `/joint_states` - Continuous feedback on joint positions
- **Service**: `/balance_control/set_mode` - Quick switch between standing/walking modes
- **Action**: `/walking_controller/move_to_pose` - Long-running navigation with feedback

#### Object Manipulation
- **Topic**: `/camera/color/image_raw` - Continuous camera feed
- **Service**: `/object_detection/detect_object` - Quick detection of objects
- **Action**: `/arm_controller/follow_joint_trajectory` - Long-running arm movement with feedback

#### Human-Robot Interaction
- **Topic**: `/audio/input` - Continuous audio stream
- **Service**: `/speech_recognition/recognize` - Quick speech-to-text conversion
- **Action**: `/animation_player/play_animation` - Long-running animation sequence

### Performance Considerations

Each communication primitive has different performance characteristics:

- **Topics** offer the highest throughput and lowest latency, making them ideal for real-time sensor data and control commands.
- **Services** have moderate performance with network round-trip overhead, suitable for operations that don't require real-time performance.
- **Actions** have the highest overhead due to the complexity of the communication pattern, but provide the most functionality for complex operations.

Understanding these differences helps in designing efficient and responsive humanoid robot systems.

## References

1. Macenski, S., Woodall, W., & Faconti, N. (2019). Design and use paradigm of ROS 2. *IEEE Access*, 7, 153148-153154.

2. ROS 2 Documentation. (2023). Quality of Service in ROS 2. https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-settings.html

3. Colomé, A., & Torras, C. (2019). Real-time collision avoidance for robot manipulators: Scalable massively-parallel sphere-sweeping. *IEEE Robotics and Automation Letters*, 4(2), 1836-1843.

4. Quigley, M., et al. (2009). ROS: an open-source Robot Operating System. *ICRA Workshop on Open Source Software*, 3(3.2), 5.

5. Woodall, W., et al. (2018). ROS 2 design overview. *Open Robotics*.

6. ROS 2 Working Groups. (2023). Quality of Service in ROS 2. https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-settings.html

7. Drakopoulos, A., & Manolakos, E. S. (2019). A survey of middleware technologies for real-time communication in robotics. *Journal of Intelligent & Robotic Systems*, 95(2), 369-390.

8. Ferro, F., et al. (2016). RTComp: A real-time component model for ROS. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 1080-1087.

9. Khusainov, R., et al. (2018). Comparison of communication architectures for cooperative robotic systems. *Procedia Computer Science*, 140, 334-343.

10. Object Management Group. (2015). Data Distribution Service (DDS) 1.4 Specification. https://www.omg.org/spec/DDS/1.4/