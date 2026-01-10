# Data Model: ROS 2 Nervous System Module

## Overview
This document describes the conceptual data models and structures for Module 1 of the "Physical AI & Humanoid Robotics" book, focusing on ROS 2 as the robotic nervous system.

## Core Entities

### ROS 2 Architecture Components

#### Node
- **Definition**: Independent process that performs computation in the ROS 2 system
- **Attributes**:
  - node_name (string): Unique identifier for the node
  - namespace (string): Optional grouping mechanism
  - lifecycle_state (enum): Active, Inactive, Unknown
- **Relationships**: 
  - Publishes to zero or more Topics
  - Subscribes to zero or more Topics
  - Provides zero or more Services
  - Clients zero or more Services

#### Topic
- **Definition**: Named bus over which nodes exchange messages
- **Attributes**:
  - topic_name (string): Unique identifier for the topic
  - message_type (string): Type definition of messages exchanged
  - qos_profile (struct): Quality of service settings
- **Relationships**:
  - Published by one or more Nodes (publishers)
  - Subscribed to by one or more Nodes (subscribers)

#### Service
- **Definition**: Request-response communication pattern between nodes
- **Attributes**:
  - service_name (string): Unique identifier for the service
  - request_type (string): Type definition of request messages
  - response_type (string): Type definition of response messages
- **Relationships**:
  - Provided by exactly one Node (server)
  - Used by zero or more Nodes (clients)

#### Action
- **Definition**: Goal-oriented communication pattern with feedback
- **Attributes**:
  - action_name (string): Unique identifier for the action
  - goal_type (string): Type definition of goal messages
  - feedback_type (string): Type definition of feedback messages
  - result_type (string): Type definition of result messages
- **Relationships**:
  - Provided by exactly one Node (server)
  - Used by zero or more Nodes (clients)

### Robot Representation

#### URDF Model
- **Definition**: Unified Robot Description Format for representing robot structure
- **Attributes**:
  - robot_name (string): Name of the robot
  - links (list): Collection of rigid bodies
  - joints (list): Connections between links
  - materials (list): Visual properties
- **Relationships**:
  - Associated with one Robot Configuration

#### Link
- **Definition**: Rigid body element in the robot structure
- **Attributes**:
  - link_name (string): Unique identifier for the link
  - visual_mesh (string): Visual representation file
  - collision_mesh (string): Collision detection geometry
  - inertial_properties (struct): Mass, center of mass, inertia tensor
- **Relationships**:
  - Connected via Joints to other Links

#### Joint
- **Definition**: Connection between two links allowing relative motion
- **Attributes**:
  - joint_name (string): Unique identifier for the joint
  - joint_type (enum): Fixed, Revolute, Continuous, Prismatic, etc.
  - parent_link (string): Link that is the parent in the kinematic chain
  - child_link (string): Link that is the child in the kinematic chain
  - limits (struct): Position, velocity, and effort limits
- **Relationships**:
  - Connects exactly one Parent Link to exactly one Child Link

### AI-Agent Integration

#### Python AI Agent
- **Definition**: Software component implementing AI algorithms that interfaces with ROS 2
- **Attributes**:
  - agent_name (string): Identifier for the AI agent
  - decision_logic (function): Core AI algorithm implementation
  - ros_interface (struct): rclpy node wrapper
- **Relationships**:
  - Communicates via rclpy with ROS 2 system
  - Publishes to and subscribes from ROS 2 Topics
  - Calls and provides ROS 2 Services

## State Transitions

### Node Lifecycle
- Unconfigured → Inactive: After successful configuration
- Inactive → Active: After activation
- Active → Inactive: After deactivation
- Inactive → Unconfigured: After cleanup
- Any state → Finalized: After shutdown

### Action Server States
- Pending → Active: When goal is accepted
- Active → Processing: During execution
- Processing → Succeeded: When goal completed successfully
- Processing → Cancelled: When goal cancelled
- Processing → Aborted: When goal execution failed

## Validation Rules

### From Functional Requirements
- FR-001: Architecture components must be clearly defined with their roles
- FR-002: Communication primitives must be distinguished with examples
- FR-003: AI agent integration points must be specified
- FR-004: URDF elements must be explained with their purposes
- FR-005: Control pipeline concepts must be modeled appropriately

### Quality Constraints
- All entity relationships must be consistent with ROS 2 design principles
- URDF elements must conform to XML schema specifications
- Message types must be well-defined and compatible across nodes