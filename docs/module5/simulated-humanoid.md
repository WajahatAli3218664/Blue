# Simulated Humanoid

## Introduction

Build a complete humanoid robot simulation with realistic physics, sensors, and AI integration.

## Humanoid URDF Design

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base/Torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="20.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="0.5"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Arms -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.15 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="2.0"/>
  </joint>

  <!-- Legs -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder radius="0.08" length="0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.08" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="0 0.1 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  </joint>

  <!-- Sensors -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head_link"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid</robotNamespace>
    </plugin>
  </gazebo>

  <!-- Camera sensor -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="head_camera">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>1920</width>
          <height>1080</height>
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
        <cameraName>humanoid/camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Humanoid Control System

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        
        # Publishers
        self.joint_pub = self.create_publisher(Float64MultiArray, '/humanoid/joint_group_position_controller/command', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Joint names and current positions
        self.joint_names = [
            'neck_joint',
            'left_shoulder_joint', 'right_shoulder_joint',
            'left_elbow_joint', 'right_elbow_joint',
            'left_hip_joint', 'right_hip_joint',
            'left_knee_joint', 'right_knee_joint',
            'left_ankle_joint', 'right_ankle_joint'
        ]
        
        self.joint_positions = [0.0] * len(self.joint_names)
        self.walking_phase = 0.0
        self.walking_speed = 0.0
        
        # Control timer
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.walking_speed = msg.linear.x
        
    def control_loop(self):
        """Main control loop for humanoid"""
        if abs(self.walking_speed) > 0.01:
            self.generate_walking_gait()
        else:
            self.maintain_balance()
        
        # Publish joint commands
        joint_cmd = Float64MultiArray()
        joint_cmd.data = self.joint_positions
        self.joint_pub.publish(joint_cmd)
    
    def generate_walking_gait(self):
        """Generate walking gait pattern"""
        # Simple sinusoidal walking pattern
        self.walking_phase += 0.1 * self.walking_speed
        
        # Hip joints (alternating)
        left_hip_angle = 0.3 * math.sin(self.walking_phase)
        right_hip_angle = -0.3 * math.sin(self.walking_phase)
        
        # Knee joints (always positive for walking)
        left_knee_angle = max(0, 0.5 * math.sin(self.walking_phase))
        right_knee_angle = max(0, -0.5 * math.sin(self.walking_phase))
        
        # Ankle joints (for balance)
        left_ankle_angle = -0.1 * math.sin(self.walking_phase)
        right_ankle_angle = 0.1 * math.sin(self.walking_phase)
        
        # Arm swing (opposite to legs)
        left_shoulder_angle = -0.2 * math.sin(self.walking_phase)
        right_shoulder_angle = 0.2 * math.sin(self.walking_phase)
        
        # Update joint positions
        self.joint_positions[1] = left_shoulder_angle   # left_shoulder
        self.joint_positions[2] = right_shoulder_angle  # right_shoulder
        self.joint_positions[5] = left_hip_angle        # left_hip
        self.joint_positions[6] = right_hip_angle       # right_hip
        self.joint_positions[7] = left_knee_angle       # left_knee
        self.joint_positions[8] = right_knee_angle      # right_knee
        self.joint_positions[9] = left_ankle_angle      # left_ankle
        self.joint_positions[10] = right_ankle_angle    # right_ankle
    
    def maintain_balance(self):
        """Maintain standing balance"""
        # Reset to neutral standing position
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = 0.0

class HumanoidPerception(Node):
    def __init__(self):
        super().__init__('humanoid_perception')
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, '/humanoid/camera/image_raw', self.image_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # Publishers
        self.object_pub = self.create_publisher(String, '/detected_objects', 10)
        
        # Computer vision
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        """Process camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Object detection
            objects = self.detect_objects(cv_image)
            
            # Publish detected objects
            if objects:
                object_msg = String()
                object_msg.data = ', '.join(objects)
                self.object_pub.publish(object_msg)
                
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
    
    def detect_objects(self, image):
        """Simple object detection"""
        # Implement object detection logic
        # This could use YOLO, detectron2, or other models
        return ['person', 'chair', 'table']  # Placeholder
    
    def joint_callback(self, msg):
        """Monitor joint states"""
        # Check for joint limits, overheating, etc.
        pass

def main(args=None):
    rclpy.init(args=args)
    
    # Start both nodes
    controller = HumanoidController()
    perception = HumanoidPerception()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(perception)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        controller.destroy_node()
        perception.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Behaviors

```python
class HumanoidBehaviors:
    def __init__(self):
        self.behaviors = {
            'wave': self.wave_gesture,
            'bow': self.bow_gesture,
            'point': self.point_gesture,
            'dance': self.dance_sequence
        }
    
    def wave_gesture(self):
        """Perform waving gesture"""
        sequence = [
            {'right_shoulder_joint': 1.5, 'right_elbow_joint': -1.0},
            {'right_shoulder_joint': 1.2, 'right_elbow_joint': -0.5},
            {'right_shoulder_joint': 1.5, 'right_elbow_joint': -1.0},
            {'right_shoulder_joint': 0.0, 'right_elbow_joint': 0.0}
        ]
        return sequence
    
    def bow_gesture(self):
        """Perform bowing gesture"""
        sequence = [
            {'neck_joint': 0.5, 'left_hip_joint': 0.3, 'right_hip_joint': 0.3},
            {'neck_joint': 0.0, 'left_hip_joint': 0.0, 'right_hip_joint': 0.0}
        ]
        return sequence
```

## Hardware Deployment

- **Simulation**: Test in Gazebo before real hardware
- **Real Hardware**: Deploy on actual humanoid platforms
- **Safety**: Implement joint limits and collision detection