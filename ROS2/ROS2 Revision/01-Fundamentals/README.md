# ROS2 Fundamentals

## 📖 Table of Contents

1. [ROS2 Architecture Overview](#ros2-architecture-overview)
2. [Nodes and Node Management](#nodes-and-node-management)
3. [Topics and Publishers/Subscribers](#topics-and-publisherssubscribers)
4. [Services](#services)
5. [Actions](#actions)
6. [Parameters](#parameters)
7. [Launch Systems](#launch-systems)
8. [Quality of Service (QoS)](#quality-of-service-qos)

---

## 🏗️ ROS2 Architecture Overview

### Key Concepts
- **Distributed System**: ROS2 operates as a distributed computing framework
- **Data Distribution Service (DDS)**: Underlying middleware for communication
- **Real-time Capabilities**: Support for real-time and safety-critical applications
- **Cross-platform**: Linux, Windows, macOS support

### Core Components
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Application   │    │   Application   │    │   Application   │
│     Layer       │    │     Layer       │    │     Layer       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│      ROS2       │    │      ROS2       │    │      ROS2       │
│   Client API    │    │   Client API    │    │   Client API    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │       DDS       │
                    │   Middleware    │
                    └─────────────────┘
```

---

## 🤖 Nodes and Node Management

### What is a Node?
A **node** is a fundamental building block in ROS2 - an executable that performs a specific task.

#### Node Characteristics:
- **Single Purpose**: Each node should have one clear responsibility
- **Independent**: Can run in separate processes
- **Communicating**: Interact through topics, services, and actions
- **Discoverable**: Automatically find other nodes in the network

### Node Lifecycle
```python
# Basic Node Structure (Python)
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node has been started')
        
    def cleanup(self):
        self.get_logger().info('Node shutting down')

def main():
    rclpy.init()
    node = MyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
```

### Node Management Commands
```bash
# List all active nodes
ros2 node list

# Get information about a specific node
ros2 node info /node_name

# Run a node
ros2 run package_name executable_name

# Kill a node
ros2 lifecycle set /node_name shutdown
```

---

## 📡 Topics and Publishers/Subscribers

### Publisher-Subscriber Pattern
The most common communication pattern in ROS2.

```
┌─────────────┐    Topic: /sensor_data      ┌─────────────┐
│  Publisher  │ ─────────────────────────▶  │ Subscriber  │
│    Node     │      (sensor_msgs/Image)    │    Node     │
└─────────────┘                             └─────────────┘
```

### Key Concepts:
- **Asynchronous**: Non-blocking communication
- **Many-to-Many**: Multiple publishers and subscribers per topic
- **Type Safety**: Messages must match defined interfaces
- **QoS Configurable**: Reliability, durability, history settings

### Example Implementation

#### Publisher (Python)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

#### Subscriber (Python)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### Topic Management Commands
```bash
# List all topics
ros2 topic list

# Get topic information
ros2 topic info /topic_name

# Echo topic messages
ros2 topic echo /topic_name

# Publish to a topic
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello ROS2'"

# Show message type
ros2 interface show std_msgs/msg/String
```

---

## 🔧 Services

### Request-Response Pattern
Services provide **synchronous** communication for request-response interactions.

```
┌─────────────┐    Service: /add_two_ints    ┌─────────────┐
│   Client    │ ──────────────────────────▶ │   Server    │
│    Node     │    Request: {a: 5, b: 3}     │    Node     │
│             │ ◀────────────────────────── │             │
└─────────────┘    Response: {sum: 8}       └─────────────┘
```

### Service Characteristics:
- **Synchronous**: Client waits for response
- **One-to-One**: Single server per service
- **Reliable**: Built-in error handling
- **Blocking**: Can timeout if server unavailable

### Example Implementation

#### Service Server (Python)
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_service')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: a={request.a} b={request.b}')
        return response
```

#### Service Client (Python)
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
            
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.cli.call_async(request)
        return future
```

### Service Management Commands
```bash
# List all services
ros2 service list

# Get service type
ros2 service type /service_name

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# Find services by type
ros2 service find example_interfaces/srv/AddTwoInts
```

---

## 🎯 Actions

### Long-Running Task Pattern
Actions are for **asynchronous** tasks that take time and provide feedback.

```
┌─────────────┐         Action: /fibonacci         ┌─────────────┐
│   Client    │ ────────────────────────────────▶ │   Server    │
│    Node     │   Goal: {order: 10}                │    Node     │
│             │                                    │             │
│             │ ◀──────────────────────────────── │             │
│             │   Feedback: {partial_sequence}     │             │
│             │                                    │             │
│             │ ◀──────────────────────────────── │             │
└─────────────┘   Result: {sequence: [0,1,1,2...]} └─────────────┘
```

### Action Components:
- **Goal**: What you want to achieve
- **Feedback**: Progress updates during execution
- **Result**: Final outcome when completed
- **Cancellation**: Ability to stop ongoing actions

### Example Implementation

#### Action Server (Python)
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + 
                feedback_msg.partial_sequence[i-1])
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)  # Simulate work
            
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
```

### Action Management Commands
```bash
# List all actions
ros2 action list

# Get action info
ros2 action info /action_name

# Send action goal
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}"

# Show action interface
ros2 interface show example_interfaces/action/Fibonacci
```

---

## ⚙️ Parameters

### Configuration Management
Parameters provide **dynamic configuration** for nodes.

### Parameter Types:
- **bool**: True/False values
- **int64**: Integer numbers
- **float64**: Floating point numbers
- **string**: Text values
- **byte_array**: Binary data
- **bool_array, int64_array, float64_array, string_array**: Arrays

### Example Implementation
```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('update_rate', 10.0)
        
        # Get parameter values
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        self.get_logger().info(f'Parameter value: {my_param}')
        
        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        return SetParametersResult(successful=True)
```

### Parameter Management Commands
```bash
# List node parameters
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name parameter_name

# Set parameter value
ros2 param set /node_name parameter_name value

# Load parameters from file
ros2 param load /node_name parameter_file.yaml

# Save parameters to file
ros2 param dump /node_name > parameters.yaml
```

---

## 🚀 Launch Systems

### Orchestrating Multiple Nodes
Launch files allow you to **start multiple nodes** and configure the system.

#### Basic Launch File (Python)
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='talker'
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener'
        )
    ])
```

#### Advanced Launch File with Parameters
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
            
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'update_rate': 50.0
            }],
            remappings=[
                ('/input_topic', '/sensors/camera/image'),
                ('/output_topic', '/processed/image')
            ]
        )
    ])
```

### Launch Commands
```bash
# Run a launch file
ros2 launch package_name launch_file.py

# Run with arguments
ros2 launch package_name launch_file.py use_sim_time:=true

# List available launch files
ros2 pkg list --packages-up-to package_name
```

---

## 🌐 Quality of Service (QoS)

### Communication Reliability Control
QoS policies control **how messages are delivered** between nodes.

### Key QoS Policies:

#### Reliability
- **RELIABLE**: Guaranteed delivery (TCP-like)
- **BEST_EFFORT**: Best attempt delivery (UDP-like)

#### Durability
- **TRANSIENT_LOCAL**: Keep messages for late subscribers
- **VOLATILE**: Only send to current subscribers

#### History
- **KEEP_LAST**: Keep only N most recent messages
- **KEEP_ALL**: Keep all messages (subject to resource limits)

### QoS Configuration Example
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Custom QoS profile
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# Create publisher with custom QoS
self.publisher_ = self.create_publisher(
    String, 
    'topic', 
    qos_profile)

# Predefined QoS profiles
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

# For sensor data (best effort, volatile)
sensor_pub = self.create_publisher(String, 'sensors', qos_profile_sensor_data)

# For system messages (reliable, transient local)
system_pub = self.create_publisher(String, 'system', qos_profile_system_default)
```

---

## 📋 Quick Reference Commands

### Node Management
```bash
ros2 node list                    # List all nodes
ros2 node info /node_name         # Node details
ros2 run package executable       # Run a node
```

### Topic Operations
```bash
ros2 topic list                   # List topics
ros2 topic echo /topic_name       # Monitor topic
ros2 topic pub /topic type "data" # Publish message
ros2 topic info /topic_name       # Topic details
```

### Service Operations
```bash
ros2 service list                 # List services  
ros2 service call /srv_name type  # Call service
ros2 service type /srv_name       # Service type
```

### Parameter Operations
```bash
ros2 param list /node_name        # List parameters
ros2 param get /node_name param   # Get parameter
ros2 param set /node_name param   # Set parameter
```

### System Information
```bash
ros2 doctor                       # System health check
ros2 wtf                          # Debug information
rqt_graph                         # Visualize node graph
```

---

## 🎓 Next Steps

1. **Practice**: Implement the examples in this guide
2. **Experiment**: Modify QoS settings and observe behavior
3. **Build**: Create your own nodes and communication patterns
4. **Advanced Topics**: Move to complex scenarios and optimization

---

*Continue to `02-Communication/` for detailed communication patterns and best practices!*