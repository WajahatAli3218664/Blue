# Sensors & Environment Rendering

## Introduction

Realistic sensor simulation is crucial for developing robust robotic systems. This topic covers camera, lidar, and IMU simulation.

## Camera Sensor Simulation

```xml
<!-- RGB Camera -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera">
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
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>robot/camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

## Depth Camera (RealSense Simulation)

```xml
<!-- Depth Camera -->
<gazebo reference="depth_camera_link">
  <sensor type="depth" name="depth_camera">
    <update_rate>20</update_rate>
    <camera>
      <horizontal_fov>1.047198</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>3</far>
      </clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <baseline>0.2</baseline>
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>depth_camera_ir</cameraName>
      <imageTopicName>/depth_camera/color/image_raw</imageTopicName>
      <cameraInfoTopicName>/depth_camera/color/camera_info</cameraInfoTopicName>
      <depthImageTopicName>/depth_camera/depth/image_raw</depthImageTopicName>
      <depthImageCameraInfoTopicName>/depth_camera/depth/camera_info</depthImageCameraInfoTopicName>
      <pointCloudTopicName>/depth_camera/depth/points</pointCloudTopicName>
      <frameName>depth_camera_link</frameName>
      <pointCloudCutoff>0.5</pointCloudCutoff>
      <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
      <distortionK1>0</distortionK1>
      <distortionK2>0</distortionK2>
      <distortionK3>0</distortionK3>
      <distortionT1>0</distortionT1>
      <distortionT2>0</distortionT2>
      <CxPrime>0</CxPrime>
      <Cx>0</Cx>
      <Cy>0</Cy>
      <focalLength>0</focalLength>
      <hackBaseline>0</hackBaseline>
    </plugin>
  </sensor>
</gazebo>
```

## LiDAR Sensor

```xml
<!-- 2D LiDAR -->
<gazebo reference="lidar_link">
  <sensor type="ray" name="hls_lfcd_lds">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>5</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>0.0</min_angle>
          <max_angle>6.28319</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.120</min>
        <max>3.5</max>
        <resolution>0.015</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="gazebo_ros_lds_lfcd_controller" filename="libgazebo_ros_laser.so">
      <topicName>scan</topicName>
      <frameName>lidar_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

## IMU Sensor

```xml
<!-- IMU -->
<gazebo reference="imu_link">
  <gravity>true</gravity>
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <topic>__default_topic__</topic>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <topicName>imu</topicName>
      <bodyName>imu_link</bodyName>
      <updateRateHZ>10.0</updateRateHZ>
      <gaussianNoise>0.0</gaussianNoise>
      <xyzOffset>0 0 0</xyzOffset>
      <rpyOffset>0 0 0</rpyOffset>
      <frameName>imu_link</frameName>
    </plugin>
    <pose>0 0 0 0 0 0</pose>
  </sensor>
</gazebo>
```

## Sensor Data Processing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, '/robot/camera/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth_camera/depth/image_raw', self.depth_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
    def image_callback(self, msg):
        """Process RGB camera data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Object detection example
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                                     param1=50, param2=30, minRadius=0, maxRadius=0)
            
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                for (x, y, r) in circles:
                    cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
                    
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
    
    def depth_callback(self, msg):
        """Process depth camera data"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            
            # Obstacle detection from depth
            obstacles = np.where(depth_image < 1.0)  # Objects closer than 1m
            if len(obstacles[0]) > 100:  # Significant obstacle
                self.get_logger().warn('Obstacle detected!')
                
        except Exception as e:
            self.get_logger().error(f'Depth processing error: {e}')
    
    def lidar_callback(self, msg):
        """Process LiDAR data"""
        ranges = np.array(msg.ranges)
        
        # Find closest obstacle
        valid_ranges = ranges[np.isfinite(ranges)]
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            if min_distance < 0.5:  # 50cm threshold
                self.get_logger().warn(f'Close obstacle at {min_distance:.2f}m')
    
    def imu_callback(self, msg):
        """Process IMU data"""
        # Extract orientation and angular velocity
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        
        # Detect if robot is tilted
        if abs(linear_acceleration.x) > 2.0 or abs(linear_acceleration.y) > 2.0:
            self.get_logger().warn('Robot tilted!')

def main(args=None):
    rclpy.init(args=args)
    sensor_processor = SensorProcessor()
    rclpy.spin(sensor_processor)
    sensor_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Environment Rendering

Create realistic environments:

```xml
<!-- Warehouse environment -->
<model name="warehouse">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <mesh>
          <uri>model://warehouse/meshes/warehouse.dae</uri>
        </mesh>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>model://warehouse/meshes/warehouse.dae</uri>
        </mesh>
      </geometry>
    </visual>
  </link>
</model>
```

## Hardware Integration

- **RealSense D435i**: Match simulation parameters with real sensor specs
- **RTX GPUs**: Enable real-time ray tracing for realistic lighting
- **Jetson Orin**: Optimize sensor processing for edge deployment

## Next Steps

In Module 3, we'll use these simulated sensors with NVIDIA Isaac for advanced perception and navigation.