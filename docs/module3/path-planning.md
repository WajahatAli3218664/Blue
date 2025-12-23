# Path Planning & Navigation

## Introduction

Advanced path planning and navigation using Isaac ROS and Nav2 integration.

## Navigation Stack Setup

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from nav2_simple_commander import BasicNavigator
import numpy as np

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Nav2 Basic Navigator
        self.navigator = BasicNavigator()
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Navigation state
        self.current_map = None
        self.navigation_active = False
        
    def goal_callback(self, goal_msg):
        """Handle new navigation goal"""
        self.get_logger().info(f'New goal received: {goal_msg.pose.position.x}, {goal_msg.pose.position.y}')
        
        # Set goal using Nav2
        self.navigator.goToPose(goal_msg)
        self.navigation_active = True
        
        # Monitor navigation progress
        self.create_timer(0.5, self.check_navigation_status)
    
    def check_navigation_status(self):
        """Monitor navigation progress"""
        if not self.navigation_active:
            return
            
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == BasicNavigator.TaskResult.SUCCEEDED:
                self.get_logger().info('Navigation succeeded!')
            else:
                self.get_logger().warn('Navigation failed!')
            self.navigation_active = False
    
    def map_callback(self, map_msg):
        """Update current map for path planning"""
        self.current_map = map_msg
        
        # Extract occupancy grid data
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        
        # Convert to numpy array for processing
        occupancy_data = np.array(map_msg.data).reshape((height, width))
        
        # Identify free space, obstacles, and unknown areas
        free_space = (occupancy_data == 0)
        obstacles = (occupancy_data == 100)
        unknown = (occupancy_data == -1)
        
        self.get_logger().info(f'Map updated: {width}x{height}, resolution: {resolution}')

def main(args=None):
    rclpy.init(args=args)
    nav_controller = NavigationController()
    rclpy.spin(nav_controller)
    nav_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Path Planning

```python
import heapq
import math
from typing import List, Tuple

class AStarPlanner:
    def __init__(self, occupancy_grid, resolution):
        self.grid = occupancy_grid
        self.resolution = resolution
        self.height, self.width = occupancy_grid.shape
        
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                    
                x, y = pos[0] + dx, pos[1] + dy
                if (0 <= x < self.height and 0 <= y < self.width and 
                    self.grid[x, y] == 0):  # Free space
                    neighbors.append((x, y))
        return neighbors
    
    def plan_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """A* path planning algorithm"""
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
            
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return []  # No path found

class DynamicWindowApproach:
    def __init__(self):
        self.max_speed = 1.0
        self.max_yaw_rate = 1.0
        self.max_accel = 0.2
        self.max_delta_yaw_rate = 0.2
        self.v_resolution = 0.01
        self.yaw_rate_resolution = 0.1
        self.dt = 0.1
        self.predict_time = 3.0
        
    def calc_dynamic_window(self, x, config):
        """Calculate dynamic window based on current state"""
        # Current velocity constraints
        vs = [0, self.max_speed, -self.max_yaw_rate, self.max_yaw_rate]
        
        # Dynamic constraints based on acceleration
        vd = [x[3] - self.max_accel * self.dt,
              x[3] + self.max_accel * self.dt,
              x[4] - self.max_delta_yaw_rate * self.dt,
              x[4] + self.max_delta_yaw_rate * self.dt]
        
        # Final dynamic window
        dw = [max(vs[0], vd[0]), min(vs[1], vd[1]),
              max(vs[2], vd[2]), min(vs[3], vd[3])]
        
        return dw
    
    def predict_trajectory(self, x_init, v, y, dt, predict_time):
        """Predict robot trajectory"""
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        
        while time <= predict_time:
            x = self.motion(x, [v, y], dt)
            trajectory = np.vstack((trajectory, x))
            time += dt
            
        return trajectory
    
    def motion(self, x, u, dt):
        """Robot motion model"""
        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]
        
        return x
```

## Hardware Deployment

- **Jetson Orin**: Real-time path planning with GPU acceleration
- **RTX Workstations**: Complex multi-robot path planning
- **Cloud Integration**: AWS RoboMaker for fleet management