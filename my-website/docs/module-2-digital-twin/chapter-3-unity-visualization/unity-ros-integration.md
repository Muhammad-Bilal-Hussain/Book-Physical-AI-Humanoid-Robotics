# Unity-ROS Integration Examples

## Introduction to Unity-ROS Integration

Unity-ROS integration enables communication between Unity's visualization environment and ROS (Robot Operating System) systems. This integration allows Unity to serve as a high-fidelity visualization layer while ROS handles the robotics-specific processing, including physics simulation, sensor data processing, and robot control. This chapter provides practical examples of how to implement and use Unity-ROS integration in robotics applications.

## Unity Robotics Package Overview

### Core Components

The Unity Robotics Package provides essential tools for ROS integration:

#### ROS-TCP-Connector
The primary communication bridge between Unity and ROS:
- **TCP/IP Communication**: Establishes network connections with ROS master
- **Message Serialization**: Converts between ROS messages and Unity data structures
- **Topic Management**: Handles publishing and subscribing to ROS topics
- **Service Calls**: Enables calling ROS services from Unity

#### URDF Importer
Facilitates importing robot models from ROS URDF files:
- **Model Conversion**: Translates URDF to Unity GameObjects
- **Joint Mapping**: Maps ROS joints to Unity components
- **Collision Geometry**: Imports collision properties
- **Visual Geometry**: Imports visual representations

### Installation and Setup

#### Prerequisites
- Unity 2020.3 LTS or later
- ROS Melodic/Noetic (for ROS1) or ROS2 Foxy/Dashing (for ROS2)
- Compatible operating system (currently Linux/Windows for ROS1, Linux/Windows/macOS for ROS2)

#### Installation Process
1. Install Unity Robotics Package via Unity Package Manager
2. Import ROS-TCP-Connector
3. Set up URDF Importer if needed
4. Configure network settings for communication

## Basic Integration Examples

### Simple Publisher Example

A basic example of publishing data from Unity to ROS:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs;

public class SimplePublisher : MonoBehaviour
{
    ROSConnection ros;
    
    // Topic name to publish to
    public string topicName = "unity_message";
    
    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();
    }
    
    void Update()
    {
        // Publish a string message every second
        if (Time.time % 1.0f < Time.deltaTime)
        {
            // Create a message to send
            StringMsg message = new StringMsg("Hello from Unity! Time: " + Time.time);
            
            // Publish the message
            ros.Publish(topicName, message);
        }
    }
}
```

### Simple Subscriber Example

A basic example of subscribing to ROS topics in Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs;

public class SimpleSubscriber : MonoBehaviour
{
    ROSConnection ros;
    
    // Topic name to subscribe to
    public string topicName = "chatter";
    
    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();
        
        // Subscribe to the topic
        ros.Subscribe<StringMsg>(topicName, OnMessageReceived);
    }
    
    void OnMessageReceived(StringMsg message)
    {
        Debug.Log("Received message: " + message.data);
    }
}
```

## Robot Control Integration

### Joint State Publisher

Example of publishing joint states from Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;
using System.Collections.Generic;

public class JointStatePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "joint_states";
    public List<Transform> jointTransforms;
    public List<string> jointNames;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }
    
    void Update()
    {
        // Publish joint states at 30 Hz
        if (Time.time % 0.033f < Time.deltaTime)
        {
            // Create joint state message
            JointStateMsg jointState = new JointStateMsg();
            jointState.name = jointNames.ToArray();
            jointState.position = new double[jointTransforms.Count];
            
            // Get joint positions from Unity transforms
            for (int i = 0; i < jointTransforms.Count; i++)
            {
                // Convert Unity rotation to ROS joint position
                jointState.position[i] = jointTransforms[i].localEulerAngles.y * Mathf.Deg2Rad;
            }
            
            jointState.header.stamp = new TimeStamp(Time.time);
            jointState.header.frame_id = "base_link";
            
            ros.Publish(topicName, jointState);
        }
    }
}
```

### Robot State Subscriber

Example of updating Unity robot model based on ROS messages:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;
using System.Collections.Generic;

public class RobotStateSubscriber : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "joint_states";
    
    public List<Transform> jointTransforms;
    public List<string> jointNames;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(topicName, OnJointStateReceived);
    }
    
    void OnJointStateReceived(JointStateMsg jointState)
    {
        // Update each joint based on received state
        for (int i = 0; i < jointNames.Count; i++)
        {
            int jointIndex = System.Array.IndexOf(jointState.name, jointNames[i]);
            if (jointIndex >= 0)
            {
                float angle = (float)jointState.position[jointIndex];
                jointTransforms[i].localEulerAngles = new Vector3(0, angle * Mathf.Rad2Deg, 0);
            }
        }
    }
}
```

## Sensor Simulation Integration

### Camera Data Publisher

Example of publishing camera data from Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs;
using System.Collections;

public class CameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "camera/image_raw";
    public Camera unityCamera;
    
    RenderTexture renderTexture;
    Texture2D texture2D;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        // Set up camera rendering
        int width = 640;
        int height = 480;
        
        renderTexture = new RenderTexture(width, height, 24);
        unityCamera.targetTexture = renderTexture;
        
        texture2D = new Texture2D(width, height, TextureFormat.RGB24, false);
    }
    
    void Update()
    {
        // Publish camera data at 30 Hz
        if (Time.time % 0.033f < Time.deltaTime)
        {
            // Read pixels from render texture
            RenderTexture.active = renderTexture;
            texture2D.ReadPixels(new Rect(0, 0, texture2D.width, texture2D.height), 0, 0);
            texture2D.Apply();
            
            // Convert to ROS image message
            ImageMsg imageMsg = new ImageMsg();
            imageMsg.header = new HeaderMsg();
            imageMsg.header.stamp = new TimeStamp(Time.time);
            imageMsg.header.frame_id = "camera_frame";
            
            imageMsg.height = (uint)texture2D.height;
            imageMsg.width = (uint)texture2D.width;
            imageMsg.encoding = "rgb8";
            imageMsg.is_bigendian = 0;
            imageMsg.step = (uint)(imageMsg.width * 3); // 3 bytes per pixel for RGB
            
            // Convert texture to byte array
            byte[] imageData = texture2D.GetRawTextureData<byte>();
            imageMsg.data = imageData;
            
            ros.Publish(topicName, imageMsg);
        }
    }
}
```

### LiDAR Simulation

Example of simulating LiDAR data in Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;
using System.Collections.Generic;

public class LidarSimulation : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "scan";
    
    public Transform lidarOrigin;
    public int numberOfRays = 360;
    public float maxRange = 10.0f;
    public float angleMin = -Mathf.PI;
    public float angleMax = Mathf.PI;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }
    
    void Update()
    {
        // Publish LiDAR data at 10 Hz
        if (Time.time % 0.1f < Time.deltaTime)
        {
            // Simulate LiDAR rays
            List<float> ranges = new List<float>();
            
            for (int i = 0; i < numberOfRays; i++)
            {
                float angle = angleMin + (angleMax - angleMin) * i / (numberOfRays - 1);
                Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
                
                RaycastHit hit;
                if (Physics.Raycast(lidarOrigin.position, 
                                   lidarOrigin.TransformDirection(direction), 
                                   out hit, maxRange))
                {
                    ranges.Add(hit.distance);
                }
                else
                {
                    ranges.Add(maxRange);
                }
            }
            
            // Create ROS LaserScan message
            LaserScanMsg scanMsg = new LaserScanMsg();
            scanMsg.header = new HeaderMsg();
            scanMsg.header.stamp = new TimeStamp(Time.time);
            scanMsg.header.frame_id = "laser_frame";
            
            scanMsg.angle_min = angleMin;
            scanMsg.angle_max = angleMax;
            scanMsg.angle_increment = (angleMax - angleMin) / (numberOfRays - 1);
            scanMsg.time_increment = 0.0f; // Not applicable for simulated data
            scanMsg.scan_time = 0.1f; // 10 Hz
            scanMsg.range_min = 0.1f;
            scanMsg.range_max = maxRange;
            
            scanMsg.ranges = ranges.ToArray();
            scanMsg.intensities = new float[ranges.Count]; // Optional intensity data
            
            ros.Publish(topicName, scanMsg);
        }
    }
}
```

## Advanced Integration Patterns

### Service Client Example

Example of calling ROS services from Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_srvs;
using System;

public class ServiceClientExample : MonoBehaviour
{
    ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }
    
    public void CallSetBoolService(string serviceName, bool data)
    {
        // Create request
        SetBoolRequest request = new SetBoolRequest();
        request.data = data ? 1 : 0;
        
        // Call service
        ros.CallService<SetBoolResponse>(serviceName, request, OnServiceResponse);
    }
    
    void OnServiceResponse(SetBoolResponse response)
    {
        Debug.Log("Service response: " + response.success + ", message: " + response.message);
    }
    
    void OnGUI()
    {
        if (GUI.Button(new Rect(10, 10, 150, 30), "Call Service True"))
        {
            CallSetBoolService("/set_bool_service", true);
        }
        
        if (GUI.Button(new Rect(10, 50, 150, 30), "Call Service False"))
        {
            CallSetBoolService("/set_bool_service", false);
        }
    }
}
```

### Action Client Example

Example of using ROS actions from Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using System;

public class ActionClientExample : MonoBehaviour
{
    ROSConnection ros;
    string actionName = "/move_base";
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }
    
    public void SendGoal(float x, float y, float theta)
    {
        // This is a simplified example - actual action implementation
        // would require more complex message handling
        Debug.Log($"Sending navigation goal to ({x}, {y}, {theta})");
        
        // In a real implementation, you would:
        // 1. Create a goal message
        // 2. Send it to the action server
        // 3. Handle feedback and result messages
    }
}
```

## Integration Best Practices

### Performance Optimization

#### Network Communication
- **Message Frequency**: Balance update rate with network capacity
- **Message Size**: Optimize message content to reduce bandwidth
- **Connection Management**: Handle connection failures gracefully
- **Threading**: Use appropriate threading for communication

#### Unity-Specific Optimizations
- **Update Frequency**: Don't update ROS communication every frame
- **Object Pooling**: Reuse message objects to reduce garbage collection
- **Batching**: Combine multiple updates when possible
- **Caching**: Cache frequently used values

### Error Handling

#### Network Issues
```csharp
public class RobustROSConnection : MonoBehaviour
{
    ROSConnection ros;
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;
    
    void Start()
    {
        // Configure ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIPAddress, rosPort);
        
        // Handle connection errors
        ros.OnConnectionFailed += OnConnectionFailed;
    }
    
    void OnConnectionFailed(string errorMessage)
    {
        Debug.LogError("ROS connection failed: " + errorMessage);
        // Implement retry logic or fallback behavior
    }
}
```

#### Message Validation
- **Data Validation**: Verify message content before processing
- **Bounds Checking**: Ensure values are within expected ranges
- **Type Safety**: Use appropriate message types
- **Fallback Behavior**: Handle missing or invalid data gracefully

## Real-World Integration Examples

### Teleoperation Interface

Example of a Unity-based teleoperation interface:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry_msgs;

public class TeleoperationInterface : MonoBehaviour
{
    ROSConnection ros;
    public string cmdVelTopic = "cmd_vel";
    
    public float linearSpeed = 1.0f;
    public float angularSpeed = 1.0f;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }
    
    void Update()
    {
        // Get input from user
        float linear = Input.GetAxis("Vertical") * linearSpeed;
        float angular = Input.GetAxis("Horizontal") * angularSpeed;
        
        // Create and publish velocity command
        if (Mathf.Abs(linear) > 0.01f || Mathf.Abs(angular) > 0.01f)
        {
            TwistMsg cmdVel = new TwistMsg();
            cmdVel.linear = new Vector3Msg(linear, 0, 0);
            cmdVel.angular = new Vector3Msg(0, 0, angular);
            
            ros.Publish(cmdVelTopic, cmdVel);
        }
    }
}
```

### Multi-Robot Coordination

Example of coordinating multiple robots through Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;

public class MultiRobotCoordinator : MonoBehaviour
{
    ROSConnection ros;
    public List<string> robotNames;
    public Dictionary<string, GameObject> robotObjects;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        // Initialize robot objects dictionary
        foreach (string name in robotNames)
        {
            // Find robot object by name
            GameObject robot = GameObject.Find(name);
            if (robot != null)
            {
                robotObjects[name] = robot;
            }
        }
        
        // Subscribe to robot status topics
        foreach (string name in robotNames)
        {
            ros.Subscribe<Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs.StringMsg>(
                name + "/status", 
                (msg) => OnRobotStatusReceived(name, msg)
            );
        }
    }
    
    void OnRobotStatusReceived(string robotName, 
        Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs.StringMsg status)
    {
        Debug.Log($"Robot {robotName} status: {status.data}");
        
        // Update Unity representation based on status
        if (robotObjects.ContainsKey(robotName))
        {
            // Update robot visualization based on status
            UpdateRobotVisualization(robotName, status.data);
        }
    }
    
    void UpdateRobotVisualization(string robotName, string status)
    {
        // Update robot appearance based on status
        GameObject robot = robotObjects[robotName];
        if (robot != null)
        {
            // Example: Change color based on status
            Renderer renderer = robot.GetComponent<Renderer>();
            if (renderer != null)
            {
                if (status == "active")
                    renderer.material.color = Color.green;
                else if (status == "idle")
                    renderer.material.color = Color.yellow;
                else
                    renderer.material.color = Color.red;
            }
        }
    }
}
```

## Troubleshooting Common Issues

### Connection Problems
- **Network Configuration**: Verify IP addresses and ports
- **Firewall Settings**: Ensure ports are open for communication
- **ROS Master**: Confirm ROS master is running and accessible
- **Protocol Compatibility**: Ensure ROS1/ROS2 compatibility

### Performance Issues
- **Message Frequency**: Reduce update rates if experiencing lag
- **Message Size**: Optimize large message content
- **Unity Performance**: Profile and optimize Unity application
- **Network Bandwidth**: Monitor and optimize network usage

### Synchronization Problems
- **Time Synchronization**: Ensure consistent time handling
- **State Consistency**: Implement proper state management
- **Update Order**: Manage update order between systems
- **Latency Compensation**: Account for communication delays

Unity-ROS integration provides a powerful platform for combining Unity's visualization capabilities with ROS's robotics functionality. By following these examples and best practices, developers can create sophisticated robotics applications that leverage the strengths of both systems.