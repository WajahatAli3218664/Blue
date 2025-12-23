# Physics Simulation

## Introduction

Physics simulation is the foundation of realistic robot testing. Gazebo provides accurate physics modeling for robotics applications.

## Gazebo World Creation

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="robot_world">
    <!-- Physics engine -->
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Custom obstacles -->
    <model name="obstacle_box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Robot Physics Properties

```xml
<!-- Add physics properties to robot links -->
<link name="base_link">
  <inertial>
    <mass value="10.0"/>
    <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
  </inertial>
  <collision>
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
  </collision>
</link>

<!-- Gazebo-specific properties -->
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <selfCollide>true</selfCollide>
</gazebo>
```

## Advanced Physics Control

```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist

class PhysicsController(Node):
    def __init__(self):
        super().__init__('physics_controller')
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
    def reset_robot_position(self, x, y, z):
        """Reset robot to specific position"""
        request = SetEntityState.Request()
        state = EntityState()
        state.name = "my_robot"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        request.state = state
        
        future = self.set_state_client.call_async(request)
        return future

    def apply_force(self, force_x, force_y, force_z):
        """Apply external force to robot"""
        # Implementation for force application
        pass
```

## Unity Integration

For photorealistic rendering, integrate with Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class UnityRobotController : MonoBehaviour
{
    ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("/cmd_vel", ReceiveVelocityCommand);
    }
    
    void ReceiveVelocityCommand(TwistMsg velocityMessage)
    {
        // Apply velocity to Unity rigidbody
        Rigidbody rb = GetComponent<Rigidbody>();
        rb.velocity = new Vector3(
            (float)velocityMessage.linear.x,
            0,
            (float)velocityMessage.linear.z
        );
    }
}
```

## Hardware Considerations

- **RTX GPUs**: Enable real-time ray tracing in Unity
- **Jetson Deployment**: Optimize physics calculations for edge devices
- **Cloud Simulation**: Use AWS RoboMaker for large-scale simulations