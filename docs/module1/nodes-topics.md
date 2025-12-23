# ROS 2 Nodes & Topics

## Introduction

ROS 2 nodes are the fundamental building blocks of any robotic system. Think of them as individual programs that perform specific tasks and communicate with each other through topics.

## What are ROS 2 Nodes?

A **node** is an executable that uses ROS 2 to communicate with other nodes. Nodes can:
- Publish data to topics
- Subscribe to topics to receive data
- Provide or call services
- Execute actions

## What are Topics?

**Topics** are named buses over which nodes exchange messages. They implement a publish/subscribe pattern:
- **Publishers** send data to topics
- **Subscribers** receive data from topics
- Multiple nodes can publish/subscribe to the same topic

## Creating Your First Node

Let's create a simple publisher node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Subscriber Node

Now let's create a subscriber to receive the messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Nodes

1. **Terminal 1** - Start the publisher:
```bash
python3 publisher_node.py
```

2. **Terminal 2** - Start the subscriber:
```bash
python3 subscriber_node.py
```

## Key Concepts

### Quality of Service (QoS)
ROS 2 provides QoS policies to handle different communication requirements:
- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Transient local vs. volatile
- **History**: Keep last vs. keep all

### Node Lifecycle
Nodes go through different states:
1. **Unconfigured** → **Inactive** → **Active** → **Finalized**

## Practical Exercise

Create a robot sensor simulation:
1. **Temperature Sensor Node**: Publishes random temperature readings
2. **Monitor Node**: Subscribes to temperature and logs alerts for high values
3. **Control Node**: Adjusts fan speed based on temperature

## Hardware Integration Notes

- **RTX Workstations**: Handle complex multi-node systems with GPU acceleration
- **Jetson Orin Nano**: Deploy lightweight nodes for edge computing
- **RealSense Sensors**: Integrate with perception nodes for real-world data

## Next Steps

In the next topic, we'll learn how to integrate URDF models with these ROS 2 nodes to create intelligent robotic agents.