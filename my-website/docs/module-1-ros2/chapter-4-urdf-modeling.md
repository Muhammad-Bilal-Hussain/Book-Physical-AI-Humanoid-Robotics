# Chapter 4: Modeling the Humanoid Body with URDF

## URDF Fundamentals

### Introduction to URDF

URDF stands for Unified Robot Description Format. It is an XML-based format used to describe robot models in ROS. URDF defines the physical and visual properties of a robot, including its kinematic structure, visual appearance, and collision properties. For humanoid robots in Physical AI systems, URDF is essential for simulation, visualization, and control.

URDF allows developers to represent a robot as a collection of rigid bodies (links) connected by joints. This representation is crucial for:
- Robot simulation in environments like Gazebo
- Kinematic analysis and inverse kinematics calculations
- Visualization in tools like RViz
- Motion planning and collision detection

### URDF Structure

A URDF file consists of several key elements:

#### Robot Element
The root element of every URDF file is the `<robot>` tag, which contains the entire robot description:

```xml
<robot name="humanoid_robot">
  <!-- Robot components go here -->
</robot>
```

#### Link Elements
Links represent rigid bodies in the robot. Each link has physical and visual properties:

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

#### Joint Elements
Joints connect links and define the kinematic relationship between them:

```xml
<joint name="base_to_upper_leg" type="revolute">
  <parent link="base_link"/>
  <child link="upper_leg"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

### Key URDF Concepts

#### Links
Links represent the rigid parts of the robot. Each link can have:
- **Visual properties**: How the link appears in simulation and visualization
- **Collision properties**: How the link interacts with other objects in collision detection
- **Inertial properties**: Mass, center of mass, and inertia tensor for physics simulation

#### Joints
Joints define the connection between links and specify how they can move relative to each other. Joint types include:
- **Revolute**: Rotational joint with one degree of freedom
- **Continuous**: Like revolute but unlimited rotation
- **Prismatic**: Linear sliding joint
- **Fixed**: No movement between links
- **Floating**: Six degrees of freedom
- **Planar**: Movement in a plane

#### Materials
Materials define the visual appearance of links, including color and texture.

#### Transmissions
Transmissions define how actuators (motors) connect to joints, specifying the mechanical relationship between motor and joint.

### URDF in Humanoid Robotics

For humanoid robots, URDF is particularly important because it captures the complex kinematic structure of the human-like body. A humanoid robot typically includes:

- **Torso**: The central body with links for chest, waist, and pelvis
- **Head**: Neck and head links with appropriate joint limits
- **Arms**: Shoulder, elbow, and wrist joints with appropriate degrees of freedom
- **Legs**: Hip, knee, and ankle joints for locomotion
- **Hands/Feet**: For manipulation and stable stance

### Coordinate Frames

URDF uses the right-hand rule for coordinate systems:
- **X-axis**: Forward (anterior) direction
- **Y-axis**: Left (lateral) direction
- **Z-axis**: Up (superior) direction

Joint axes define the direction of motion. For revolute joints, positive rotation follows the right-hand rule around the axis.

### URDF Best Practices

1. **Meaningful Names**: Use descriptive names for links and joints that reflect their function
2. **Proper Root Link**: Define a clear base link that serves as the reference frame
3. **Kinematic Chain**: Ensure all links are connected in a proper kinematic chain
4. **Joint Limits**: Specify realistic joint limits based on physical constraints
5. **Mass Properties**: Accurate mass and inertia values for realistic simulation
6. **Collision vs. Visual**: Use simplified geometries for collision detection to improve performance

### URDF Validation

URDF files should be validated to ensure they are well-formed and represent a physically plausible robot. ROS provides tools like `check_urdf` to validate URDF files.

### Relationship to ROS TF

URDF defines the static transforms between robot links, which ROS's TF (Transform) system uses to maintain the spatial relationship between different parts of the robot over time.

Understanding URDF fundamentals is essential for creating accurate robot models that can be used effectively in simulation, visualization, and control systems for humanoid robots in Physical AI applications.

## Links and Joints in URDF

### Understanding Links

In URDF, a link represents a rigid body part of the robot. Each link is a fundamental building block that has physical properties and can be connected to other links through joints. Links are the static components of a robot model that define its structure.

#### Link Properties

Each link can have several important properties:

##### Visual Properties
The visual element defines how the link appears in simulation and visualization tools:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

- **Origin**: Specifies the position and orientation of the visual geometry relative to the link's frame
- **Geometry**: Defines the shape (box, cylinder, sphere, mesh)
- **Material**: Specifies color and appearance properties

##### Collision Properties
The collision element defines how the link interacts with other objects in collision detection:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

Collision geometry can be simplified compared to visual geometry to improve performance.

##### Inertial Properties
The inertial element defines the physical properties for dynamics simulation:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
</inertial>
```

- **Mass**: The mass of the link in kilograms
- **Inertia**: The 3x3 inertia matrix describing how mass is distributed

### Understanding Joints

Joints connect links and define the kinematic relationship between them. They specify how two links can move relative to each other.

#### Joint Properties

Each joint has several important properties:

##### Joint Type
The type attribute defines the kind of motion allowed:

- **Fixed**: No motion between links (0 DOF)
- **Revolute**: Single rotational axis with limits (1 DOF)
- **Continuous**: Single rotational axis without limits (1 DOF)
- **Prismatic**: Single linear axis with limits (1 DOF)
- **Floating**: Six degrees of freedom (6 DOF)
- **Planar**: Motion in a plane (3 DOF)

##### Parent and Child Links
Joints connect exactly two links:

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
</joint>
```

##### Origin
Specifies the position and orientation of the joint relative to the parent link:

```xml
<origin xyz="1.0 0 0" rpy="0 0 0"/>
```

##### Axis
Defines the axis of motion for revolute and prismatic joints:

```xml
<axis xyz="0 0 1"/>
```

##### Limits
For revolute and prismatic joints, limits define the range of motion:

```xml
<limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
```

- **Lower/Upper**: Range of motion in radians (revolute) or meters (prismatic)
- **Effort**: Maximum torque/force in N-m or N
- **Velocity**: Maximum velocity in rad/s or m/s

### Common Joint Types in Humanoid Robots

#### Revolute Joints
Most common in humanoid robots, representing rotational joints like elbows, knees, and shoulders:

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.356" upper="2.356" effort="50" velocity="2"/>
</joint>
```

#### Fixed Joints
Used for attaching sensors or other components that don't move relative to their parent:

```xml
<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_frame"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
</joint>
```

### Link-Joint Relationships

#### Kinematic Chains
Links and joints form kinematic chains that define the robot's structure. A humanoid robot typically has multiple kinematic chains:

- **Torso chain**: Base to head
- **Arm chains**: Torso to hands (2 chains)
- **Leg chains**: Torso to feet (2 chains)

#### Tree Structure
URDF models form a tree structure with a single root link. Each link (except the root) has exactly one parent, but can have multiple children.

### Practical Example: Simple Arm

Here's a complete example of a simple 2-DOF arm:

```xml
<link name="base_link">
  <inertial>
    <mass value="1"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
  <visual>
    <geometry>
      <cylinder radius="0.1" length="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.1" length="0.2"/>
    </geometry>
  </collision>
</link>

<link name="upper_arm">
  <inertial>
    <mass value="2"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.02"/>
  </inertial>
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.5"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.5"/>
    </geometry>
  </collision>
</link>

<link name="lower_arm">
  <inertial>
    <mass value="1.5"/>
    <inertia ixx="0.08" ixy="0" ixz="0" iyy="0.08" iyz="0" izz="0.01"/>
  </inertial>
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.4"/>
    </geometry>
    <material name="green">
      <color rgba="0 1 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.4"/>
    </geometry>
  </collision>
</link>

<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>

<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 -0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="80" velocity="1"/>
</joint>
```

### Best Practices for Links and Joints

#### Naming Conventions
- Use descriptive names that indicate the function (e.g., `left_elbow_joint`)
- Follow a consistent naming scheme throughout the model
- Use underscores to separate words

#### Joint Placement
- Place joints at the physical pivot point of the connection
- Ensure joint axes align with the intended direction of motion
- Use appropriate joint types for the intended motion

#### Mass and Inertia
- Use realistic mass values based on the actual robot
- Calculate inertia tensors based on the geometry and mass distribution
- Consider the payload when calculating masses for end-effectors

#### Limits
- Set realistic joint limits based on the physical robot
- Include safety margins in the limits
- Consider the effect of joint limits on motion planning

Understanding how to properly define links and joints in URDF is crucial for creating accurate robot models that work well in simulation, visualization, and control systems for humanoid robots.

## Visual and Collision Properties in URDF

### Introduction to Visual and Collision Elements

In URDF, each link can have both visual and collision properties that define how the robot appears and interacts with its environment. These properties are crucial for simulation, visualization, and collision detection in humanoid robots.

The visual element defines how the robot appears in simulation and visualization tools, while the collision element defines how the robot interacts with other objects in collision detection and physics simulation.

### Visual Properties

The visual element specifies how a link appears in visualization tools like RViz and simulation environments like Gazebo.

#### Visual Element Structure

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Geometry definition -->
  </geometry>
  <material name="material_name">
    <!-- Material definition -->
  </material>
</visual>
```

#### Origin in Visual Elements

The origin element specifies the position and orientation of the visual geometry relative to the link's frame:

```xml
<origin xyz="0.1 0 0.05" rpy="0 0 1.57"/>
```

- **xyz**: Position offset (x, y, z) in meters
- **rpy**: Orientation as roll, pitch, yaw in radians

#### Geometry Types

URDF supports several geometry types for visual representation:

##### Box
```xml
<geometry>
  <box size="0.1 0.2 0.3"/>
</geometry>
```
Defines a box with specified dimensions (width, depth, height).

##### Cylinder
```xml
<geometry>
  <cylinder radius="0.05" length="0.3"/>
</geometry>
```
Defines a cylinder with specified radius and length.

##### Sphere
```xml
<geometry>
  <sphere radius="0.05"/>
</geometry>
```
Defines a sphere with specified radius.

##### Mesh
```xml
<geometry>
  <mesh filename="package://robot_description/meshes/link.stl" scale="1 1 1"/>
</geometry>
```
Defines a complex shape using a mesh file. The filename can be a package-relative path or an absolute path.

#### Materials

Materials define the color and appearance of visual elements:

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>
```

- **name**: A unique identifier for the material
- **color**: RGBA values (Red, Green, Blue, Alpha) from 0 to 1

Alternatively, materials can reference textures:

```xml
<material name="texture_material">
  <texture filename="package://robot_description/materials/textures/texture.png"/>
</material>
```

### Collision Properties

The collision element defines how a link interacts with other objects in collision detection and physics simulation.

#### Collision Element Structure

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Geometry definition -->
  </geometry>
</collision>
```

The structure is similar to the visual element, but collision elements are used exclusively for physics calculations.

#### Collision Geometry Considerations

Collision geometry should be:
- **Simple**: Use simple shapes when possible to improve performance
- **Conservative**: Ensure collision geometry encompasses the actual physical link
- **Efficient**: Balance accuracy with computational cost

### Differences Between Visual and Collision Properties

While visual and collision elements have similar structures, they serve different purposes:

#### Visual Properties
- Used for rendering and visualization
- Can use complex meshes for realistic appearance
- Performance impact is mainly on graphics rendering
- Focus on appearance rather than accuracy

#### Collision Properties
- Used for collision detection and physics simulation
- Should use simplified geometry for performance
- Performance impact is on physics calculations
- Focus on accurate collision detection rather than appearance

### Practical Example: Visual and Collision Properties

Here's an example showing both visual and collision properties for a humanoid robot link:

```xml
<link name="upper_arm">
  <!-- Inertial properties for physics simulation -->
  <inertial>
    <mass value="2.5"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
  </inertial>

  <!-- Visual properties for appearance -->
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://humanoid_robot/meshes/upper_arm.dae"/>
    </geometry>
    <material name="arm_material">
      <color rgba="0.8 0.8 0.8 1"/>
    </material>
  </visual>

  <!-- Collision properties for physics -->
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.06" length="0.3"/>
    </geometry>
  </collision>
</link>
```

In this example:
- The visual element uses a detailed mesh for realistic appearance
- The collision element uses a simple cylinder for efficient collision detection
- Both elements are positioned similarly but could differ if needed

### Best Practices for Visual and Collision Properties

#### For Visual Properties
- Use high-quality meshes for important visible parts
- Apply consistent materials across the robot
- Consider using package-relative paths for mesh files
- Optimize mesh complexity for visualization performance

#### For Collision Properties
- Use simple geometric shapes when possible (boxes, cylinders, spheres)
- Ensure collision geometry fully encompasses the physical link
- Consider using multiple collision elements for complex shapes
- Balance accuracy with simulation performance

#### Performance Considerations
- Complex visual meshes don't significantly impact physics performance
- Complex collision meshes significantly impact physics performance
- Use convex hulls or simplified meshes for collision detection
- Consider using multiple simple shapes instead of one complex shape

### Advanced Topics

#### Multiple Visual and Collision Elements

A single link can have multiple visual and collision elements:

```xml
<link name="complex_link">
  <visual name="main_visual">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>

  <visual name="attachment_visual">
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.02" length="0.05"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>

  <collision name="main_collision">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
</link>
```

#### Using the Same Geometry for Visual and Collision

When visual and collision geometry are identical, you can define the geometry once and reference it:

```xml
<link name="simple_link">
  <visual>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
  </collision>
</link>
```

Properly defining visual and collision properties is essential for creating realistic simulations and accurate collision detection in humanoid robots. The balance between visual fidelity and computational efficiency is key to achieving good performance in both visualization and physics simulation.

## Practical Examples of Humanoid URDF Files

### Introduction to Humanoid Robot Modeling

Modeling humanoid robots in URDF requires careful consideration of the complex kinematic structure that mimics the human body. A humanoid robot typically includes a torso, head, two arms, and two legs, each with multiple joints that allow for human-like movement.

### Complete Humanoid Robot Example

Here's a simplified but complete URDF example for a humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base/Fixed link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.15" length="1.0"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.15" length="1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="8.0"/>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <inertia ixx="0.8" ixy="0.0" ixz="0.0" iyy="0.8" iyz="0.0" izz="0.2"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin_color">
        <color rgba="0.9 0.8 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="10" velocity="1.5"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_shoulder">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_yaw" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0.1 0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1.0"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="15" velocity="1.0"/>
  </joint>

  <!-- Right Arm (similar to left, mirrored) -->
  <link name="right_shoulder">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_yaw" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="0.15 -0.1 0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1.0"/>
  </joint>

  <link name="right_upper_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1.0"/>
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="15" velocity="1.0"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_hip">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0 0.1 -0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="30" velocity="0.8"/>
  </joint>

  <link name="left_thigh">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_pitch" type="revolute">
    <parent link="left_hip"/>
    <child link="left_thigh"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="0.2" effort="30" velocity="0.8"/>
  </joint>

  <link name="left_shin">
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="30" velocity="0.8"/>
  </joint>

  <link name="left_foot">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_ankle" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="0.8"/>
  </joint>

  <!-- Right Leg (similar to left, mirrored) -->
  <link name="right_hip">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="0 -0.1 -0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="30" velocity="0.8"/>
  </joint>

  <link name="right_thigh">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_pitch" type="revolute">
    <parent link="right_hip"/>
    <child link="right_thigh"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="0.2" effort="30" velocity="0.8"/>
  </joint>

  <link name="right_shin">
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_knee" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="30" velocity="0.8"/>
  </joint>

  <link name="right_foot">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_ankle" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="0.8"/>
  </joint>

</robot>
```

### Analysis of the Humanoid URDF

This example demonstrates several important concepts for humanoid robot modeling:

#### Structure
- The robot has a hierarchical structure with a base link and multiple kinematic chains
- Each limb (arms and legs) forms its own kinematic chain
- The head is connected to the torso via a neck joint

#### Joint Limitations
- Joint limits are set to realistic values based on human anatomy
- Different joints have different types and ranges of motion
- Effort and velocity limits are specified for physics simulation

#### Mass and Inertia
- Reasonable mass values are assigned to each link
- Inertia tensors are estimated based on the geometry and mass

#### Visual and Collision Properties
- Simple geometric shapes are used for both visual and collision elements
- Colors are used to distinguish different parts of the robot

### Real-World Humanoid Examples

#### NAO Robot Structure
The popular NAO humanoid robot has a similar structure with:
- 25 degrees of freedom
- Sensors in the head (cameras, microphones, tactile sensors)
- Actuated joints in the arms and legs
- Feet with pressure sensors

#### ATLAS Robot Structure
More complex humanoid robots like ATLAS have:
- Hydraulic actuation
- More complex kinematic structures
- Additional sensors and computational units

### Common Patterns in Humanoid URDFs

#### Head Structure
```xml
<link name="head">
  <!-- Head link definition -->
</link>
<joint name="neck_joint" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <!-- Joint definition -->
</joint>
```

#### Limb Structure
```xml
<!-- Upper segment -->
<link name="upper_segment">
  <!-- Link definition -->
</link>

<!-- Lower segment -->
<link name="lower_segment">
  <!-- Link definition -->
</link>

<!-- Connecting joint -->
<joint name="connecting_joint" type="revolute">
  <parent link="upper_segment"/>
  <child link="lower_segment"/>
  <!-- Joint definition -->
</joint>
```

### Tips for Creating Humanoid URDFs

1. **Start Simple**: Begin with basic geometric shapes and add complexity gradually
2. **Reference Human Anatomy**: Use human joint ranges as a reference for realistic limits
3. **Balance Detail and Performance**: Use detailed meshes for visualization but simpler shapes for collision detection
4. **Test Regularly**: Validate your URDF with tools like `check_urdf` and test in simulation
5. **Use Xacro**: For complex humanoid robots, use Xacro macros to avoid repetition

These practical examples provide a foundation for creating URDF models of humanoid robots for use in simulation, visualization, and control systems.

## Xacro for Parameterized Descriptions

### Introduction to Xacro

Xacro (XML Macros) is a macro language for XML files that extends URDF with features like constants, properties, mathematical expressions, and reusable macros. Xacro is particularly useful for humanoid robots, which often have symmetrical components (left/right arms and legs) and repeated elements that can be parameterized.

Xacro files use the `.xacro` extension and are processed by the `xacro` command-line tool to generate standard URDF files.

### Basic Xacro Concepts

#### Including Xacro
To use Xacro features, include the xacro namespace in your XML declaration:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">
```

#### Properties and Constants
Xacro allows you to define properties that can be reused throughout the file:

```xml
<xacro:property name="PI" value="3.14159"/>
<xacro:property name="robot_mass" value="50.0"/>
<xacro:property name="arm_length" value="0.4"/>
```

#### Mathematical Expressions
Xacro supports mathematical expressions using the `${ }` syntax:

```xml
<origin xyz="${arm_length} 0 0" rpy="0 ${PI/2} 0"/>
<mass value="${robot_mass * 0.1}"/>
```

### Xacro Macros

Macros are reusable blocks of XML that can accept parameters:

```xml
<xacro:macro name="simple_cylinder" params="name radius length mass xyz *origin">
  <link name="${name}">
    <inertial>
      <mass value="${mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="${mass*(3*radius*radius + length*length)/12}"
               ixy="0" ixz="0"
               iyy="${mass*(3*radius*radius + length*length)/12}"
               iyz="0"
               izz="${mass*radius*radius/2}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${radius}" length="${length}"/>
      </geometry>
    </visual>
    <collision>
      <xacro:insert_block name="origin"/>
      <geometry>
        <cylinder radius="${radius}" length="${length}"/>
      </geometry>
    </collision>
  </link>
</xacro:macro>
```

### Practical Xacro Example for Humanoid Robot

Here's how to use Xacro to simplify the creation of a humanoid robot with symmetrical components:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define constants -->
  <xacro:property name="PI" value="3.14159"/>
  <xacro:property name="mass_shoulder" value="1.0"/>
  <xacro:property name="mass_arm" value="1.5"/>
  <xacro:property name="arm_radius" value="0.04"/>
  <xacro:property name="arm_length" value="0.4"/>
  <xacro:property name="shoulder_radius" value="0.06"/>
  <xacro:property name="shoulder_length" value="0.1"/>

  <!-- Macro for a simple cylinder link with inertial properties -->
  <xacro:macro name="cylinder_link" params="name radius length mass xyz rpy">
    <link name="${name}">
      <inertial>
        <mass value="${mass}"/>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <inertia
          ixx="${mass*(3*radius*radius + length*length)/12}"
          ixy="0" ixz="0"
          iyy="${mass*(3*radius*radius + length*length)/12}"
          iyz="0"
          izz="${mass*radius*radius/2}"/>
      </inertial>
      <visual>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <material name="gray">
          <color rgba="0.5 0.5 0.5 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Macro for a simple box link -->
  <xacro:macro name="box_link" params="name size_x size_y size_z mass xyz rpy">
    <link name="${name}">
      <inertial>
        <mass value="${mass}"/>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <inertia
          ixx="${mass*(size_y*size_y + size_z*size_z)/12}"
          ixy="0" ixz="0"
          iyy="${mass*(size_x*size_x + size_z*size_z)/12}"
          iyz="0"
          izz="${mass*(size_x*size_x + size_y*size_y)/12}"/>
      </inertial>
      <visual>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <box size="${size_x} ${size_y} ${size_z}"/>
        </geometry>
        <material name="white">
          <color rgba="1 1 1 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <box size="${size_x} ${size_y} ${size_z}"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Macro for an arm (either left or right) -->
  <xacro:macro name="arm" params="side position_x position_y">
    <!-- Shoulder joint -->
    <cylinder_link
      name="${side}_shoulder"
      radius="${shoulder_radius}"
      length="${shoulder_length}"
      mass="${mass_shoulder}"
      xyz="0 0 0"
      rpy="0 0 0"/>

    <joint name="${side}_shoulder_yaw" type="revolute">
      <parent link="torso"/>
      <child link="${side}_shoulder"/>
      <origin xyz="${position_x} ${position_y} 0.4" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="${-PI/2}" upper="${PI/2}" effort="20" velocity="1.0"/>
    </joint>

    <!-- Upper arm -->
    <cylinder_link
      name="${side}_upper_arm"
      radius="${arm_radius}"
      length="${arm_length}"
      mass="${mass_arm}"
      xyz="0 0 ${-arm_length/2}"
      rpy="0 0 0"/>

    <joint name="${side}_shoulder_pitch" type="revolute">
      <parent link="${side}_shoulder"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="0 0 ${-shoulder_length/2}" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="${-PI/2}" upper="${PI/2}" effort="20" velocity="1.0"/>
    </joint>

    <!-- Lower arm -->
    <cylinder_link
      name="${side}_lower_arm"
      radius="${arm_radius*0.75}"
      length="${arm_length*0.8}"
      mass="${mass_arm*0.75}"
      xyz="0 0 ${-arm_length*0.8/2}"
      rpy="0 0 0"/>

    <joint name="${side}_elbow" type="revolute">
      <parent link="${side}_upper_arm"/>
      <child link="${side}_lower_arm"/>
      <origin xyz="0 0 ${-arm_length}" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="${-PI/2}" upper="${PI/2}" effort="15" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <cylinder_link
    name="base_link"
    radius="0.15"
    length="1.0"
    mass="10.0"
    xyz="0 0 0.5"
    rpy="0 0 0"/>

  <!-- Torso -->
  <box_link
    name="torso"
    size_x="0.3"
    size_y="0.3"
    size_z="0.6"
    mass="8.0"
    xyz="0 0 0.3"
    rpy="0 0 0"/>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
  </joint>

  <!-- Use the arm macro to create both left and right arms -->
  <xacro:arm side="left" position_x="0.15" position_y="0.1"/>
  <xacro:arm side="right" position_x="0.15" position_y="-0.1"/>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin_color">
        <color rgba="0.9 0.8 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-PI/4}" upper="${PI/4}" effort="10" velocity="1.5"/>
  </joint>

</robot>
```

### Advanced Xacro Features

#### Conditional Statements
Xacro supports conditional statements using the `if` attribute:

```xml
<xacro:macro name="conditional_part" params="include_sensor:=false">
  <xacro:if value="${include_sensor}">
    <link name="sensor_mount">
      <!-- Sensor mount definition -->
    </link>
  </xacro:if>
</xacro:macro>
```

#### Including Other Xacro Files
You can include other xacro files to modularize your robot definition:

```xml
<xacro:include filename="$(find robot_description)/urdf/sensors.urdf.xacro"/>
<xacro:include filename="$(find robot_description)/urdf/arms.urdf.xacro"/>
```

#### Loops
Xacro supports loops for repetitive structures:

```xml
<xacro:macro name="spine_links" params="count spacing">
  <xacro:property name="link_mass" value="1.0"/>
  <xacro:property name="link_radius" value="0.05"/>
  <xacro:property name="link_length" value="0.1"/>

  <xacro:property name="i" value="0"/>
  <xacro:while value="${i < count}">
    <cylinder_link
      name="vertebra_${i}"
      radius="${link_radius}"
      length="${link_length}"
      mass="${link_mass}"
      xyz="0 0 ${i * spacing}"
      rpy="0 0 0"/>

    <xacro:if value="${i > 0}">
      <joint name="connection_${i}" type="fixed">
        <parent link="vertebra_${i - 1}"/>
        <child link="vertebra_${i}"/>
        <origin xyz="0 0 ${spacing}" rpy="0 0 0"/>
      </joint>
    </xacro:if>

    <xacro:property name="i" value="${i + 1}"/>
  </xacro:while>
</xacro:macro>
```

### Benefits of Using Xacro for Humanoid Robots

#### Reduced Repetition
Xacro significantly reduces code duplication, especially for symmetrical components like arms and legs.

#### Parameterization
Changes to dimensions, masses, or other properties can be made in one place and automatically propagated throughout the model.

#### Modularity
Complex robots can be broken down into modular components that can be included as needed.

#### Readability
Xacro makes URDF files more readable by separating constants and logic from the structural definition.

### Converting Xacro to URDF

To convert a Xacro file to URDF, use the xacro command:

```bash
xacro input_file.urdf.xacro > output_file.urdf
```

Or in ROS 2:
```bash
ros2 run xacro xacro input_file.urdf.xacro -o output_file.urdf
```

### Best Practices for Xacro

1. **Use Meaningful Property Names**: Choose descriptive names for your properties to make the code more readable.

2. **Group Related Parameters**: Organize related parameters together in property blocks.

3. **Document Your Macros**: Add comments explaining what each macro does and what parameters it expects.

4. **Validate Your Output**: Always check that the generated URDF is valid and represents your intended robot.

5. **Use Units Consistently**: Stick to consistent units (SI units are standard in ROS).

Xacro is an essential tool for creating complex humanoid robot models efficiently, reducing redundancy, and making the URDF files more maintainable and flexible.

## URDF Integration with Simulation Environments

### Introduction to Simulation Integration

URDF models are essential for simulating humanoid robots in various environments. Simulation allows developers to test robot behaviors, validate control algorithms, and verify kinematic models before deploying to physical hardware. The integration between URDF and simulation environments is critical for accurate and efficient simulation.

### Gazebo Simulation Environment

Gazebo is one of the most widely used simulation environments for robotics, particularly in the ROS ecosystem. URDF models integrate with Gazebo through several mechanisms:

#### Gazebo-Specific Tags in URDF

URDF files can include Gazebo-specific tags that define how the robot behaves in simulation:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>
```

#### Gazebo Plugins

Gazebo plugins can be attached to URDF models to provide specific functionality:

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>
```

#### Joint Transmission Elements

For proper control in simulation, URDF models need transmission elements:

```xml
<transmission name="left_elbow_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_elbow">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_elbow_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Integration Process

#### 1. Model Conversion
When a URDF model is loaded into Gazebo, it undergoes several transformations:
- Links become rigid bodies with mass and inertia properties
- Joints become constraint systems that limit motion
- Visual elements become renderable objects
- Collision elements become physics collision primitives

#### 2. Physics Properties
The inertial properties defined in URDF directly influence the physics simulation:
- Mass values affect how the robot responds to forces
- Inertia tensors affect rotational dynamics
- Friction coefficients (defined in Gazebo tags) affect contact behavior

#### 3. Sensor Integration
Sensors defined in URDF are instantiated in the simulation environment:

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link_optical</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Other Simulation Environments

#### Webots
Webots is another popular robotics simulator that can import URDF models. The integration process involves:
- Converting URDF to Webots PROTO format
- Mapping joint types and limits
- Preserving visual and collision properties

#### Mujoco
Mujoco is a physics engine often used for robotics research. URDF models can be converted to Mujoco format, though this may require some manual adjustments to ensure accurate physics simulation.

#### PyBullet
PyBullet is a Python-based physics simulator that can directly load URDF files. It's commonly used for machine learning research involving robotics.

### Best Practices for Simulation Integration

#### 1. Accurate Physical Properties
Ensure that mass, inertia, and friction values in your URDF are realistic:
- Use CAD software to calculate accurate inertial properties
- Verify that total robot mass matches the physical robot
- Set appropriate friction coefficients for different surfaces

#### 2. Simplified Collision Models
Use simplified geometries for collision detection to improve simulation performance:
- Approximate complex shapes with multiple simple primitives
- Use bounding boxes instead of detailed meshes for collision
- Balance accuracy with computational efficiency

#### 3. Proper Joint Limits
Set realistic joint limits based on the physical robot:
- Include safety margins to prevent damage during simulation
- Consider the effect of joint limits on motion planning
- Verify that limits are consistent with the physical robot

#### 4. Sensor Placement
Carefully consider sensor placement in simulation:
- Position sensors as they are on the physical robot
- Configure sensor parameters to match physical sensors
- Consider the field of view and range of sensors

### Simulation-Specific Considerations for Humanoid Robots

#### Balance and Stability
Humanoid robots require special attention to balance in simulation:
- Accurate center of mass calculation is critical
- Proper foot contact modeling for stable stance
- Realistic actuator dynamics to simulate real-world limitations

#### Complex Kinematics
Humanoid robots have complex kinematic chains that require:
- Accurate joint placement and orientation
- Proper handling of closed-loop kinematic chains
- Consideration of kinematic singularities

#### Computational Performance
Simulating humanoid robots can be computationally intensive:
- Optimize collision geometry for performance
- Use appropriate solver parameters
- Consider simplifying models for faster simulation

### Launching Robots in Simulation

To launch a URDF model in Gazebo simulation, you typically use a launch file:

```xml
<launch>
  <!-- Load the URDF into the parameter server -->
  <param name="robot_description"
         command="$(find xacro)/xacro $(find humanoid_robot_description)/urdf/humanoid_robot.urdf.xacro"/>

  <!-- Spawn the robot in Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
        args="-param robot_description -urdf -model humanoid_robot" />

  <!-- Start Gazebo -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
</launch>
```

### Troubleshooting Simulation Issues

#### Robot Falls Through Ground
- Check that all links have proper mass and inertia values
- Verify that collision elements are defined for ground-contacting links
- Ensure that the physics engine is properly configured

#### Joints Behaving Unexpectedly
- Verify joint limits and types match the physical robot
- Check that joint origins are correctly specified
- Ensure transmission elements are properly defined

#### Poor Simulation Performance
- Simplify collision geometry
- Reduce the number of complex meshes
- Adjust physics engine parameters (step size, solver type)

The integration of URDF models with simulation environments is a critical step in developing and testing humanoid robots. Proper integration ensures that simulation results accurately reflect the behavior of the physical robot, enabling effective development and validation of control algorithms and behaviors.

## Sample URDF Code Snippets

### Basic Robot Structure

Here's a minimal URDF file that demonstrates the basic structure:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### Complete Joint Example

Here's a more complete example showing how to connect two links with a joint:

```xml
<?xml version="1.0"?>
<robot name="two_link_robot">
  <!-- First link -->
  <link name="link1">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Second link -->
  <link name="link2">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>

    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting the two links -->
  <joint name="joint1" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

### Humanoid Torso Example

Here's a more complex example showing a humanoid torso with multiple attachments:

```xml
<?xml version="1.0"?>
<robot name="humanoid_torso">
  <!-- Torso link -->
  <link name="torso">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <inertia ixx="0.8" ixy="0" ixz="0" iyy="0.8" iyz="0" izz="0.2"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
  </link>

  <!-- Head attachment -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.8 0.7 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="5" velocity="2"/>
  </joint>

  <!-- Left shoulder attachment -->
  <link name="left_shoulder">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0.15 0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>
</robot>
```

### Sensor Attachment Example

Here's how to attach a camera sensor to a robot:

```xml
<!-- Camera link -->
<link name="camera_link">
  <inertial>
    <mass value="0.1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>

  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
</link>

<!-- Joint to attach camera to head -->
<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
</joint>

<!-- Gazebo-specific sensor definition -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.01</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>camera1</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

### Transmission Example

Here's how to define transmissions for joint control:

```xml
<!-- Example transmission for a joint -->
<transmission name="left_elbow_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_elbow_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_elbow_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo-Specific Properties

Here's how to add Gazebo-specific properties to a link:

```xml
<gazebo reference="left_foot">
  <material>Gazebo/White</material>
  <mu1>0.8</mu1>
  <mu2>0.8</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <fdir1>1 0 0</fdir1>
</gazebo>
```

### Complete Humanoid Hand Example

Here's a more complex example showing a simple hand model:

```xml
<!-- Palm link -->
<link name="left_palm">
  <inertial>
    <mass value="0.3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>

  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.08 0.02"/>
    </geometry>
    <material name="skin">
      <color rgba="0.9 0.8 0.7 1"/>
    </material>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.08 0.02"/>
    </geometry>
  </collision>
</link>

<!-- Thumb -->
<link name="left_thumb">
  <inertial>
    <mass value="0.05"/>
    <origin xyz="0 0 -0.025" rpy="0 0 0"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.00005"/>
  </inertial>

  <visual>
    <origin xyz="0 0 -0.025" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.01" length="0.05"/>
    </geometry>
    <material name="skin">
      <color rgba="0.9 0.8 0.7 1"/>
    </material>
  </visual>

  <collision>
    <origin xyz="0 0 -0.025" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.01" length="0.05"/>
    </geometry>
  </collision>
</link>

<joint name="left_thumb_joint" type="revolute">
  <parent link="left_palm"/>
  <child link="left_thumb"/>
  <origin xyz="0.04 0.03 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="1.57" effort="2" velocity="1"/>
</joint>

<!-- Index finger -->
<link name="left_index">
  <inertial>
    <mass value="0.04"/>
    <origin xyz="0 0 -0.03" rpy="0 0 0"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.00004"/>
  </inertial>

  <visual>
    <origin xyz="0 0 -0.03" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.008" length="0.06"/>
    </geometry>
    <material name="skin">
      <color rgba="0.9 0.8 0.7 1"/>
    </material>
  </visual>

  <collision>
    <origin xyz="0 0 -0.03" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.008" length="0.06"/>
    </geometry>
  </collision>
</link>

<joint name="left_index_joint" type="revolute">
  <parent link="left_palm"/>
  <child link="left_index"/>
  <origin xyz="0.03 0.01 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="1.57" effort="2" velocity="1"/>
</joint>
```

These code snippets demonstrate various aspects of URDF modeling for humanoid robots, from basic structure to complex multi-link systems with sensors and transmissions. They can serve as templates for building more complex humanoid robot models.