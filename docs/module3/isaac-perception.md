# Isaac ROS Perception

## Introduction

Isaac ROS provides GPU-accelerated perception capabilities for real-time robotics applications.

## Object Detection with Isaac ROS

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from isaac_ros_detectnet import DetectNetNode

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Isaac ROS DetectNet
        self.detectnet = DetectNetNode()
        
        # Subscribers and Publishers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        
    def image_callback(self, msg):
        """Process image with Isaac ROS DetectNet"""
        detections = self.detectnet.process_image(msg)
        self.detection_pub.publish(detections)
        
        # Log detected objects
        for detection in detections.detections:
            self.get_logger().info(f'Detected: {detection.results[0].id} with confidence {detection.results[0].score}')

def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()
    rclpy.spin(perception_node)
    perception_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Depth Estimation

```yaml
# Isaac ROS Stereo Depth
stereo_depth:
  ros__parameters:
    input_left_topic: "/left/image_rect"
    input_right_topic: "/right/image_rect"
    output_topic: "/depth/image"
    queue_size: 10
    disparity_range: 128
    max_disparity: 64.0
```

## Visual SLAM

```python
import rclpy
from rclpy.node import Node
from isaac_ros_visual_slam import VisualSlamNode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        # Isaac ROS Visual SLAM
        self.visual_slam = VisualSlamNode()
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Timer for SLAM updates
        self.timer = self.create_timer(0.1, self.slam_update)
        
    def slam_update(self):
        """Update SLAM and publish pose/map"""
        pose = self.visual_slam.get_current_pose()
        occupancy_map = self.visual_slam.get_occupancy_grid()
        
        if pose:
            self.pose_pub.publish(pose)
        if occupancy_map:
            self.map_pub.publish(occupancy_map)

def main(args=None):
    rclpy.init(args=args)
    slam_node = SLAMNode()
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hardware Optimization

- **RTX GPUs**: Enable TensorRT optimization for inference
- **Jetson Orin**: Deploy optimized models for edge computing
- **Memory Management**: Efficient GPU memory usage for real-time performance