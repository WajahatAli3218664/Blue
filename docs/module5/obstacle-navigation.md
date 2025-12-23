# Obstacle Navigation

## Introduction

Advanced obstacle navigation for humanoid robots using multi-sensor fusion and AI-powered path planning.

## Multi-Sensor Fusion

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import cv2
from cv_bridge import CvBridge

class ObstacleNavigationNode(Node):
    def __init__(self):
        super().__init__('obstacle_navigation')
        
        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/humanoid/camera/image_raw', self.camera_callback, 10)
        self.depth_sub = self.create_subscription(PointCloud2, '/depth_camera/points', self.pointcloud_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/obstacles', 10)
        
        # Navigation state
        self.current_goal = None
        self.obstacles = []
        self.bridge = CvBridge()
        
        # Sensor fusion data
        self.lidar_obstacles = []
        self.vision_obstacles = []
        self.depth_obstacles = []
        
        # Control timer
        self.timer = self.create_timer(0.1, self.navigation_loop)
        
    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter valid ranges
        valid_indices = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Convert to Cartesian coordinates
        obstacles = []
        for r, theta in zip(valid_ranges, valid_angles):
            if r < 2.0:  # Only consider obstacles within 2 meters
                x = r * np.cos(theta)
                y = r * np.sin(theta)
                obstacles.append([x, y])
        
        self.lidar_obstacles = obstacles
    
    def camera_callback(self, msg):
        """Process camera data for visual obstacle detection"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect obstacles using computer vision
            obstacles = self.detect_visual_obstacles(cv_image)
            self.vision_obstacles = obstacles
            
        except Exception as e:
            self.get_logger().error(f'Camera processing error: {e}')
    
    def detect_visual_obstacles(self, image):
        """Detect obstacles using computer vision"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        obstacles = []
        for contour in contours:
            # Filter by area
            area = cv2.contourArea(contour)
            if area > 1000:  # Minimum obstacle size
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Estimate 3D position (simplified)
                # This would normally use depth information
                estimated_distance = 2000 / w  # Rough distance estimation
                angle = (x + w/2 - image.shape[1]/2) * 0.001  # Rough angle
                
                obs_x = estimated_distance * np.cos(angle)
                obs_y = estimated_distance * np.sin(angle)
                obstacles.append([obs_x, obs_y])
        
        return obstacles
    
    def pointcloud_callback(self, msg):
        """Process point cloud data"""
        # Convert point cloud to obstacle list
        # This would use pcl_ros or similar for processing
        pass
    
    def goal_callback(self, msg):
        """Set new navigation goal"""
        self.current_goal = msg
        self.get_logger().info(f'New goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
    
    def navigation_loop(self):
        """Main navigation control loop"""
        if not self.current_goal:
            return
        
        # Fuse sensor data
        all_obstacles = self.fuse_obstacle_data()
        
        # Plan path
        velocity_cmd = self.plan_velocity(all_obstacles)
        
        # Publish velocity command
        self.cmd_vel_pub.publish(velocity_cmd)
    
    def fuse_obstacle_data(self):
        """Fuse obstacles from multiple sensors"""
        all_obstacles = []
        
        # Add LiDAR obstacles (high confidence)
        for obs in self.lidar_obstacles:
            all_obstacles.append({
                'position': obs,
                'confidence': 0.9,
                'source': 'lidar'
            })
        
        # Add vision obstacles (medium confidence)
        for obs in self.vision_obstacles:
            all_obstacles.append({
                'position': obs,
                'confidence': 0.6,
                'source': 'vision'
            })
        
        # Remove duplicates and merge nearby obstacles
        fused_obstacles = self.merge_nearby_obstacles(all_obstacles)
        
        return fused_obstacles
    
    def merge_nearby_obstacles(self, obstacles):
        """Merge obstacles that are close to each other"""
        merged = []
        merge_distance = 0.3  # 30cm threshold
        
        for obs in obstacles:
            merged_with_existing = False
            
            for existing in merged:
                distance = np.linalg.norm(
                    np.array(obs['position']) - np.array(existing['position'])
                )
                
                if distance < merge_distance:
                    # Merge obstacles (weighted average)
                    total_conf = obs['confidence'] + existing['confidence']
                    existing['position'] = [
                        (obs['position'][0] * obs['confidence'] + 
                         existing['position'][0] * existing['confidence']) / total_conf,
                        (obs['position'][1] * obs['confidence'] + 
                         existing['position'][1] * existing['confidence']) / total_conf
                    ]
                    existing['confidence'] = min(1.0, total_conf)
                    merged_with_existing = True
                    break
            
            if not merged_with_existing:
                merged.append(obs)
        
        return merged
    
    def plan_velocity(self, obstacles):
        """Plan velocity using dynamic window approach"""
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        
        # Current robot state (simplified)
        robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0
        
        # Calculate goal direction
        goal_distance = np.sqrt(goal_x**2 + goal_y**2)
        goal_angle = np.arctan2(goal_y, goal_x)
        
        # Dynamic window parameters
        max_linear_vel = 0.5
        max_angular_vel = 1.0
        
        # Simple obstacle avoidance
        cmd = Twist()
        
        if goal_distance < 0.1:
            # Reached goal
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.current_goal = None
            self.get_logger().info('Goal reached!')
        else:
            # Check for obstacles in path
            obstacle_in_path = False
            min_obstacle_distance = float('inf')
            
            for obs in obstacles:
                obs_distance = np.linalg.norm(obs['position'])
                obs_angle = np.arctan2(obs['position'][1], obs['position'][0])
                
                # Check if obstacle is in our path
                if (abs(obs_angle - goal_angle) < 0.5 and 
                    obs_distance < 1.0 and 
                    obs['confidence'] > 0.5):
                    obstacle_in_path = True
                    min_obstacle_distance = min(min_obstacle_distance, obs_distance)
            
            if obstacle_in_path:
                # Obstacle avoidance behavior
                cmd.linear.x = max(0.1, min_obstacle_distance - 0.5)
                
                # Turn away from obstacle
                if goal_angle > 0:
                    cmd.angular.z = max_angular_vel * 0.5
                else:
                    cmd.angular.z = -max_angular_vel * 0.5
            else:
                # Move toward goal
                cmd.linear.x = min(max_linear_vel, goal_distance * 0.5)
                cmd.angular.z = goal_angle * 0.8
        
        return cmd

class AdvancedPathPlanner:
    def __init__(self):
        self.grid_resolution = 0.1
        self.robot_radius = 0.3
        
    def plan_path(self, start, goal, obstacles):
        """Plan path using A* with obstacle inflation"""
        # Create occupancy grid
        grid = self.create_occupancy_grid(obstacles)
        
        # Inflate obstacles by robot radius
        inflated_grid = self.inflate_obstacles(grid)
        
        # Run A* algorithm
        path = self.astar(start, goal, inflated_grid)
        
        return path
    
    def create_occupancy_grid(self, obstacles):
        """Create occupancy grid from obstacle list"""
        # Implementation for grid creation
        pass
    
    def inflate_obstacles(self, grid):
        """Inflate obstacles by robot radius"""
        # Implementation for obstacle inflation
        pass
    
    def astar(self, start, goal, grid):
        """A* path planning algorithm"""
        # Implementation of A* algorithm
        pass

def main(args=None):
    rclpy.init(args=args)
    navigation_node = ObstacleNavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Navigation Strategies

```python
class HumanoidNavigationStrategies:
    def __init__(self):
        self.strategies = {
            'narrow_passage': self.navigate_narrow_passage,
            'stairs': self.navigate_stairs,
            'dynamic_obstacles': self.navigate_dynamic_obstacles,
            'crowded_space': self.navigate_crowded_space
        }
    
    def navigate_narrow_passage(self, passage_width):
        """Navigate through narrow passages"""
        if passage_width < 0.8:  # Tight passage
            return {
                'linear_velocity': 0.2,  # Slow and careful
                'angular_velocity': 0.0,
                'body_posture': 'narrow'  # Adjust arm positions
            }
    
    def navigate_stairs(self, stair_detection):
        """Navigate stairs (up or down)"""
        return {
            'gait_pattern': 'stair_climbing',
            'step_height': stair_detection['step_height'],
            'handrail_detection': True
        }
    
    def navigate_dynamic_obstacles(self, moving_obstacles):
        """Navigate around moving obstacles (people, other robots)"""
        predictions = self.predict_obstacle_motion(moving_obstacles)
        return self.plan_with_predictions(predictions)
    
    def predict_obstacle_motion(self, obstacles):
        """Predict future positions of moving obstacles"""
        predictions = []
        for obs in obstacles:
            # Simple linear prediction
            future_pos = [
                obs['position'][0] + obs['velocity'][0] * 2.0,  # 2 seconds ahead
                obs['position'][1] + obs['velocity'][1] * 2.0
            ]
            predictions.append(future_pos)
        return predictions
```

## Hardware Integration

- **Multi-Sensor Setup**: LiDAR, cameras, depth sensors, IMU
- **Real-time Processing**: Optimize for low-latency navigation
- **Safety Systems**: Emergency stop and collision avoidance
- **Edge Deployment**: Run navigation stack on Jetson Orin Nano