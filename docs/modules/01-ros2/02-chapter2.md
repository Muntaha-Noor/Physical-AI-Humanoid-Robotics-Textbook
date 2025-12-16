# Chapter 2: Controlling Humanoids with `rclpy`

## Writing ROS 2 Nodes in Python with `rclpy`

**`rclpy`** is the Python client library for ROS 2. It provides a convenient and Pythonic way to write ROS 2 applications, including nodes, publishers, subscribers, services, and actions. If you're familiar with Python, `rclpy` allows you to quickly develop and test robotic functionalities.

Before you start, make sure you have ROS 2 installed (Module 1) and your Python environment is set up with `rclpy`.

### Basic `rclpy` Node Structure

Every `rclpy` node inherits from `rclpy.node.Node`. Here's the most basic structure:

```python
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_custom_node') # Initialize the node with a unique name
        self.get_logger().info('My Custom Node has been started!')

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    node = MyCustomNode() # Create the node
    try:
        rclpy.spin(node) # Keep the node alive and processing callbacks
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    finally:
        node.destroy_node() # Destroy the node
        rclpy.shutdown() # Shut down rclpy

if __name__ == '__main__':
    main()
```
*   `rclpy.init(args=args)`: Initializes the ROS 2 client library. Must be called before any ROS 2 activity.
*   `super().__init__('my_custom_node')`: Calls the base `Node` class constructor, giving the node a name.
*   `rclpy.spin(node)`: Blocks until the node is shut down, allowing callbacks (e.g., from subscribers, timers) to be processed.
*   `node.destroy_node()` and `rclpy.shutdown()`: Cleanly shuts down the node and the client library.

## Connecting Python AI Agents to ROS Controllers

`rclpy` enables AI agents written in Python to seamlessly connect with ROS 2-based robot controllers. An AI agent might be a reinforcement learning policy, a classical planning algorithm, or an LLM-based cognitive planner (as we'll see in Module 4). By using `rclpy`, these agents can:

*   **Receive Sensor Data**: Subscribe to topics publishing camera images, LiDAR scans, joint states, etc.
*   **Send Commands**: Publish velocity commands, joint position targets, or trigger ROS 2 actions/services to robot controllers.
*   **Integrate with Perception/Planning**: Utilize outputs from other ROS 2 nodes (e.g., object detections, navigation paths).

## Using Publishers, Subscribers, Timers, and Services in `rclpy`

### Python Publisher Example (`motor_command_publisher.py`)

This node will simulate an AI agent publishing motor commands.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # Common message for velocity commands

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1 # publish every 100ms
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.get_logger().info('Motor Command Publisher started.')

    def timer_callback(self):
        msg = Twist()
        self.linear_x = 0.1 # Move forward at 0.1 m/s
        self.angular_z = 0.0 # No rotation
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing velocity: linear_x={self.linear_x:.2f}, angular_z={self.angular_z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Subscriber Example (`joint_state_subscriber.py`)

This node will subscribe to simulated joint states from a robot.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # Common message for joint states

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states', # Assuming 'joint_states' topic
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.get_logger().info('Joint State Subscriber started.')

    def listener_callback(self, msg):
        # self.get_logger().info(f'Received Joint States: {msg.name}, {msg.position}')
        for i in range(len(msg.name)):
            self.get_logger().info(f'  Joint: {msg.name[i]}, Position: {msg.position[i]:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Timer Example (`status_reporter.py`)

Timers allow a node to execute a callback function periodically.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StatusReporter(Node):
    def __init__(self):
        super().__init__('status_reporter')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer_period = 2.0 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Status Reporter started.')

    def timer_callback(self):
        msg = String()
        msg.data = 'Robot is operational and standing by.'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing status: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = StatusReporter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Service Server/Client Example (`add_two_ints_server.py`, `add_two_ints_client.py`)

ROS 2 services require a `.srv` file defining the request and response types. For this example, assume you have a custom package with `AddTwoInts.srv`:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

**Service Server:**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Assuming this SRV is available

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Service started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client:**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Assuming this SRV is available
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()
        self.get_logger().info('Add Two Ints Client started.')

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        node = Node('add_two_ints_client_error')
        node.get_logger().error('Usage: ros2 run <package_name> add_two_ints_client A B')
        rclpy.shutdown()
        return

    node = AddTwoIntsClient()
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    response = node.send_request(a, b)
    if response:
        node.get_logger().info(f'Result of add_two_ints: {a} + {b} = {response.sum}')
    else:
        node.get_logger().error('Service call failed.')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
To run the service examples:
1.  Ensure you have a custom ROS 2 package where `AddTwoInts.srv` is defined and built.
2.  Launch the server: `ros2 run <package_name> add_two_ints_server`
3.  Launch the client: `ros2 run <package_name> add_two_ints_client 5 7`

## Hands-on Example: Sending Mock Motor Commands

Let's create a full example where a Python AI agent (simulated here) periodically publishes a `Twist` message to the `cmd_vel` topic, simulating a robot moving forward.

First, ensure you have a robot in simulation (e.g., Gazebo or Isaac Sim) that subscribes to `/cmd_vel` and translates these into actual robot movement.

**`simple_ai_agent.py`:**

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.5 # Publish commands every 0.5 seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Simple AI Agent started.')
        self.forward_speed = 0.1 # m/s
        self.turn_speed = 0.0 # rad/s

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.forward_speed
        msg.angular.z = self.turn_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Commanding: Linear X={self.forward_speed:.2f}, Angular Z={self.turn_speed:.2f}')

def main(args=None):
    rclpy.init(args=args)
    agent_node = SimpleAIAgent()
    rclpy.spin(agent_node)
    agent_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
To run this:
1.  Launch your simulated robot (e.g., in Gazebo).
2.  Run `python simple_ai_agent.py`.
Your robot should start moving forward in the simulation. You can then modify `forward_speed` or `turn_speed` in the code to experiment with different movements.

This hands-on example demonstrates how simple Python `rclpy` nodes can directly control a humanoid robot's movements through ROS 2 topics.
