# URDF & Python Agent Integration

## Introduction

URDF (Unified Robot Description Format) is an XML format for representing a robot model. When combined with Python agents, it creates intelligent robotic systems that understand their physical structure.

## What is URDF?

URDF describes:
- **Links**: Rigid bodies of the robot (chassis, arms, wheels)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Sensors**: Cameras, lidars, IMUs
- **Visual/Collision**: 3D meshes and collision boundaries

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Wheel link -->
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting base to wheel -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.2 0 -0.1" rpy="0 1.57 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Python Agent Integration

Create an intelligent agent that controls the URDF robot:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
import time

class RobotAgent(Node):
    def __init__(self):
        super().__init__('robot_agent')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Robot state
        self.position = [0.0, 0.0, 0.0]  # x, y, theta
        self.joint_positions = {'wheel_joint': 0.0}
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Mission parameters
        self.target_position = [2.0, 2.0]
        self.mission_state = "navigate_to_target"

    def control_loop(self):
        """Main control loop for the robot agent"""
        if self.mission_state == "navigate_to_target":
            self.navigate_to_target()
        elif self.mission_state == "patrol":
            self.patrol_behavior()
        elif self.mission_state == "idle":
            self.stop_robot()

    def navigate_to_target(self):
        """Navigate to target position using simple proportional control"""
        # Calculate distance and angle to target
        dx = self.target_position[0] - self.position[0]
        dy = self.target_position[1] - self.position[1]
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # Create velocity command
        cmd = Twist()
        
        if distance > 0.1:  # Not at target yet
            # Proportional control
            cmd.linear.x = min(0.5, distance * 0.5)
            
            # Angular control
            angle_diff = target_angle - self.position[2]
            cmd.angular.z = angle_diff * 0.8
            
        else:
            # Reached target
            self.mission_state = "patrol"
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
        self.update_joint_states()

    def patrol_behavior(self):
        """Simple patrol behavior"""
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.1  # Slow turn
        
        self.cmd_vel_pub.publish(cmd)
        self.update_joint_states()

    def stop_robot(self):
        """Stop all robot movement"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def update_joint_states(self):
        """Update and publish joint states"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.joint_positions.keys())
        joint_state.position = list(self.joint_positions.values())
        
        # Simulate wheel rotation
        self.joint_positions['wheel_joint'] += 0.1
        
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    robot_agent = RobotAgent()
    rclpy.spin(robot_agent)
    robot_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File Integration

Create a launch file to start everything together:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open('my_robot.urdf').read()}]
        ),
        
        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),
        
        # Our robot agent
        Node(
            package='my_robot_package',
            executable='robot_agent',
            output='screen'
        ),
        
        # RViz for visualization
        ExecuteProcess(
            cmd=['rviz2', '-d', 'robot_config.rviz'],
            output='screen'
        )
    ])
```

## Advanced Agent Behaviors

### State Machine Implementation

```python
class RobotStateMachine:
    def __init__(self):
        self.states = {
            'IDLE': self.idle_state,
            'EXPLORING': self.explore_state,
            'CHARGING': self.charge_state,
            'EMERGENCY': self.emergency_state
        }
        self.current_state = 'IDLE'
        self.battery_level = 100.0

    def update(self):
        # Execute current state
        next_state = self.states[self.current_state]()
        
        # Transition logic
        if next_state:
            self.current_state = next_state

    def idle_state(self):
        if self.battery_level > 50:
            return 'EXPLORING'
        elif self.battery_level < 20:
            return 'CHARGING'
        return None

    def explore_state(self):
        # Exploration logic
        if self.battery_level < 30:
            return 'CHARGING'
        return None
```

## Sensor Integration

Add sensors to your URDF and integrate with the agent:

```xml
<!-- Add camera sensor -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.3 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Gazebo camera plugin -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>robot/camera1</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

## Hardware Deployment Notes

- **RTX Workstations**: Handle complex URDF models with detailed meshes
- **Jetson Orin Nano**: Optimize URDF for lightweight deployment
- **RealSense Integration**: Replace simulated sensors with real camera data

## Practical Exercise

Create a delivery robot:
1. Design URDF with mobile base and cargo compartment
2. Implement Python agent with waypoint navigation
3. Add obstacle avoidance behavior
4. Integrate battery monitoring system

## Next Steps

In Module 2, we'll learn how to simulate these URDF robots in realistic physics environments using Gazebo and Unity.