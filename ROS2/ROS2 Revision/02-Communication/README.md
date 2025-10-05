# Communication Patterns in ROS2

## 📖 Table of Contents

1. [Communication Overview](#communication-overview)
2. [Advanced Topic Patterns](#advanced-topic-patterns)
3. [Service Design Patterns](#service-design-patterns)
4. [Action Implementation Strategies](#action-implementation-strategies)
5. [Message Interfaces](#message-interfaces)
6. [Communication Best Practices](#communication-best-practices)

---

## 🌐 Communication Overview

### The Three Pillars of ROS2 Communication

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│     TOPICS      │    │    SERVICES     │    │    ACTIONS      │
│                 │    │                 │    │                 │
│ • Asynchronous  │    │ • Synchronous   │    │ • Asynchronous  │
│ • Many-to-Many  │    │ • Request-Reply │    │ • Long-running  │
│ • Streaming     │    │ • One-to-One    │    │ • Cancellable   │
│ • Publish/Sub   │    │ • Blocking      │    │ • Feedback      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### When to Use Each Pattern

| Use Case | Topics | Services | Actions |
|----------|--------|----------|---------|
| Sensor Data | ✅ | ❌ | ❌ |
| Commands | ✅ | ✅ | ❌ |
| Queries | ❌ | ✅ | ❌ |
| Navigation | ❌ | ❌ | ✅ |
| Processing | ✅ | ✅ | ✅ |
| Real-time | ✅ | ❌ | ❌ |

---

## 📡 Advanced Topic Patterns

### 1. Multi-Layer Communication

```python
# Sensor Layer -> Processing Layer -> Control Layer
class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        
        # Input: Raw sensor data
        self.raw_sub = self.create_subscription(
            sensor_msgs.msg.LaserScan,
            '/scan_raw',
            self.raw_callback,
            10)
            
        # Output: Processed data
        self.processed_pub = self.create_publisher(
            geometry_msgs.msg.PointCloud2,
            '/scan_processed',
            10)
            
        # Output: Feature extraction
        self.features_pub = self.create_publisher(
            custom_msgs.msg.Features,
            '/scan_features',
            10)
```

### 2. Topic Remapping Strategies

```python
# Launch file with intelligent remapping
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensors',
            executable='lidar_driver',
            name='front_lidar',
            remappings=[
                ('scan', 'front_lidar/scan'),
                ('diagnostics', 'front_lidar/diagnostics')
            ]
        ),
        Node(
            package='sensors', 
            executable='lidar_driver',
            name='rear_lidar',
            remappings=[
                ('scan', 'rear_lidar/scan'),
                ('diagnostics', 'rear_lidar/diagnostics')
            ]
        )
    ])
```

### 3. Conditional Publishing

```python
class SmartPublisher(Node):
    def __init__(self):
        super().__init__('smart_publisher')
        
        self.publisher_ = self.create_publisher(
            std_msgs.msg.String, 
            'conditional_topic', 
            10)
            
        # Only publish if there are subscribers
        self.timer = self.create_timer(1.0, self.publish_callback)
        
    def publish_callback(self):
        if self.publisher_.get_subscription_count() > 0:
            msg = std_msgs.msg.String()
            msg.data = f'Message at {self.get_clock().now().to_msg()}'
            self.publisher_.publish(msg)
            self.get_logger().info('Published message')
        else:
            self.get_logger().debug('No subscribers, skipping publish')
```

---

## 🔧 Service Design Patterns

### 1. Stateful Service Server

```python
class StatefulService(Node):
    def __init__(self):
        super().__init__('stateful_service')
        
        # Internal state
        self.counter = 0
        self.active_requests = {}
        
        self.service = self.create_service(
            custom_interfaces.srv.ProcessData,
            'process_data',
            self.process_callback)
            
    def process_callback(self, request, response):
        # Generate unique request ID
        request_id = f"req_{self.counter}"
        self.counter += 1
        
        # Store request state
        self.active_requests[request_id] = {
            'timestamp': self.get_clock().now(),
            'data': request.input_data
        }
        
        # Process based on current state
        if len(self.active_requests) > 10:
            response.success = False
            response.message = "Server overloaded"
        else:
            # Actual processing
            response.result = self.process_data(request.input_data)
            response.success = True
            response.message = f"Processed as {request_id}"
            
        # Cleanup old requests
        self.cleanup_old_requests()
        
        return response
```

### 2. Service Client with Timeout and Retry

```python
class RobustServiceClient(Node):
    def __init__(self):
        super().__init__('robust_service_client')
        
        self.client = self.create_client(
            custom_interfaces.srv.ProcessData,
            'process_data')
            
    async def call_service_robust(self, data, max_retries=3, timeout=5.0):
        for attempt in range(max_retries):
            try:
                # Wait for service
                if not self.client.wait_for_service(timeout_sec=timeout):
                    self.get_logger().warn(f'Service not available (attempt {attempt + 1})')
                    continue
                    
                # Make request
                request = custom_interfaces.srv.ProcessData.Request()
                request.input_data = data
                
                future = self.client.call_async(request)
                
                # Wait for response with timeout
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
                
                if future.result() is not None:
                    return future.result()
                else:
                    self.get_logger().warn(f'Service call failed (attempt {attempt + 1})')
                    
            except Exception as e:
                self.get_logger().error(f'Service call exception: {e}')
                
        raise RuntimeError(f'Service call failed after {max_retries} attempts')
```

---

## 🎯 Action Implementation Strategies

### 1. Cancellable Action Server

```python
class CancellableActionServer(Node):
    def __init__(self):
        super().__init__('cancellable_action_server')
        
        self.action_server = ActionServer(
            self,
            custom_interfaces.action.LongTask,
            'long_task',
            self.execute_callback,
            cancel_callback=self.cancel_callback)
            
        self.current_goal_handle = None
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.current_goal_handle = goal_handle
        
        feedback_msg = custom_interfaces.action.LongTask.Feedback()
        
        for i in range(goal_handle.request.iterations):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return custom_interfaces.action.LongTask.Result()
                
            # Check if goal is still active
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return custom_interfaces.action.LongTask.Result()
                
            # Simulate work and provide feedback
            feedback_msg.current_iteration = i
            feedback_msg.progress = (i / goal_handle.request.iterations) * 100
            goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(1)  # Simulate processing time
            
        # Successful completion
        goal_handle.succeed()
        result = custom_interfaces.action.LongTask.Result()
        result.final_result = f'Completed {goal_handle.request.iterations} iterations'
        return result
        
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
```

### 2. Action Client with Progress Tracking

```python
class ProgressTrackingClient(Node):
    def __init__(self):
        super().__init__('progress_tracking_client')
        
        self.action_client = ActionClient(
            self, 
            custom_interfaces.action.LongTask,
            'long_task')
            
    def send_goal(self, iterations):
        goal_msg = custom_interfaces.action.LongTask.Goal()
        goal_msg.iterations = iterations
        
        self.action_client.wait_for_server()
        
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
            
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
            
        self.get_logger().info('Goal accepted')
        
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: {feedback.progress:.1f}% '
            f'(Iteration {feedback.current_iteration})')
            
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.final_result}')
```

---

## 📝 Message Interfaces

### 1. Custom Message Definition

```bash
# custom_interfaces/msg/SensorReading.msg
Header header
string sensor_id
float64 temperature
float64 humidity  
float64 pressure
geometry_msgs/Point location
uint8 STATUS_OK=0
uint8 STATUS_WARNING=1
uint8 STATUS_ERROR=2
uint8 status
string[] error_messages
```

### 2. Custom Service Definition

```bash
# custom_interfaces/srv/ProcessSensorData.srv
# Request
SensorReading[] readings
string processing_mode
bool validate_data
---
# Response  
bool success
string message
ProcessedData result
float64 processing_time
string[] warnings
```

### 3. Custom Action Definition

```bash
# custom_interfaces/action/NavigateToGoal.action
# Goal
geometry_msgs/PoseStamped target_pose
string planner_id
float64 tolerance
---
# Result
bool success
string message
geometry_msgs/PoseStamped final_pose
float64 total_distance
duration total_time
---
# Feedback
geometry_msgs/PoseStamped current_pose
float64 distance_remaining
float64 estimated_time_remaining
string current_status
```

### 4. Message Validation

```python
class MessageValidator(Node):
    def __init__(self):
        super().__init__('message_validator')
        
        self.subscription = self.create_subscription(
            custom_interfaces.msg.SensorReading,
            'sensor_data',
            self.validate_and_process,
            10)
            
    def validate_and_process(self, msg):
        # Validate header
        if not self.validate_header(msg.header):
            self.get_logger().error('Invalid header')
            return
            
        # Validate sensor ID
        if not msg.sensor_id or len(msg.sensor_id) < 3:
            self.get_logger().error('Invalid sensor ID')
            return
            
        # Validate ranges
        if msg.temperature < -50 or msg.temperature > 100:
            self.get_logger().warn('Temperature out of expected range')
            
        # Validate status
        valid_statuses = [
            custom_interfaces.msg.SensorReading.STATUS_OK,
            custom_interfaces.msg.SensorReading.STATUS_WARNING,
            custom_interfaces.msg.SensorReading.STATUS_ERROR
        ]
        
        if msg.status not in valid_statuses:
            self.get_logger().error('Invalid status value')
            return
            
        # Process valid message
        self.process_sensor_reading(msg)
        
    def validate_header(self, header):
        # Check timestamp
        current_time = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(header.stamp)
        
        age = current_time - msg_time
        if age.nanoseconds > 5e9:  # 5 seconds
            self.get_logger().warn('Message too old')
            return False
            
        return True
```

---

## 🎯 Communication Best Practices

### 1. Namespace Organization

```python
# Organized namespace structure
/robot_1/
    /sensors/
        /lidar/scan
        /camera/image
        /imu/data
    /navigation/
        /cmd_vel
        /odom
        /map
    /manipulation/
        /joint_states
        /gripper/command
```

### 2. Message Frequency Management

```python
class AdaptivePublisher(Node):
    def __init__(self):
        super().__init__('adaptive_publisher')
        
        self.publisher_ = self.create_publisher(
            sensor_msgs.msg.PointCloud2,
            'pointcloud',
            10)
            
        # Adaptive frequency based on system load
        self.base_frequency = 10.0  # Hz
        self.current_frequency = self.base_frequency
        
        self.timer = self.create_timer(
            1.0 / self.current_frequency,
            self.publish_callback)
            
        # Monitor system resources
        self.resource_timer = self.create_timer(
            1.0, 
            self.adjust_frequency)
            
    def adjust_frequency(self):
        # Simulate system load check
        cpu_usage = self.get_cpu_usage()  # Custom implementation
        
        if cpu_usage > 80:
            # Reduce frequency under high load
            self.current_frequency = max(1.0, self.current_frequency * 0.8)
        elif cpu_usage < 50:
            # Increase frequency under low load
            self.current_frequency = min(
                self.base_frequency, 
                self.current_frequency * 1.1)
                
        # Update timer
        self.timer.destroy()
        self.timer = self.create_timer(
            1.0 / self.current_frequency,
            self.publish_callback)
```

### 3. Error Handling and Recovery

```python
class ResilientCommunicator(Node):
    def __init__(self):
        super().__init__('resilient_communicator')
        
        # Multiple communication channels
        self.primary_pub = self.create_publisher(
            std_msgs.msg.String, 
            'primary_channel', 
            10)
            
        self.backup_pub = self.create_publisher(
            std_msgs.msg.String,
            'backup_channel',
            10)
            
        # Health monitoring
        self.health_timer = self.create_timer(
            1.0, 
            self.monitor_health)
            
        self.communication_health = {
            'primary': True,
            'backup': True,
            'last_primary_success': self.get_clock().now(),
            'last_backup_success': self.get_clock().now()
        }
        
    def publish_message(self, message):
        success = False
        
        # Try primary channel first
        if self.communication_health['primary']:
            try:
                self.primary_pub.publish(message)
                self.communication_health['last_primary_success'] = self.get_clock().now()
                success = True
            except Exception as e:
                self.get_logger().error(f'Primary channel failed: {e}')
                self.communication_health['primary'] = False
                
        # Fallback to backup channel
        if not success and self.communication_health['backup']:
            try:
                self.backup_pub.publish(message)
                self.communication_health['last_backup_success'] = self.get_clock().now()
                success = True
            except Exception as e:
                self.get_logger().error(f'Backup channel failed: {e}')
                self.communication_health['backup'] = False
                
        if not success:
            self.get_logger().error('All communication channels failed')
            
    def monitor_health(self):
        current_time = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=10)
        
        # Check primary channel health
        if (current_time - self.communication_health['last_primary_success']) > timeout:
            self.communication_health['primary'] = False
            
        # Check backup channel health  
        if (current_time - self.communication_health['last_backup_success']) > timeout:
            self.communication_health['backup'] = False
            
        # Attempt recovery
        if not any(self.communication_health.values()):
            self.attempt_recovery()
            
    def attempt_recovery(self):
        self.get_logger().info('Attempting communication recovery...')
        # Implement recovery logic
        self.communication_health['primary'] = True
        self.communication_health['backup'] = True
```

### 4. Performance Monitoring

```python
class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        # Message timing statistics
        self.message_stats = {
            'count': 0,
            'total_latency': 0.0,
            'max_latency': 0.0,
            'min_latency': float('inf')
        }
        
        self.subscription = self.create_subscription(
            std_msgs.msg.Header,
            'timestamped_topic',
            self.analyze_performance,
            10)
            
        # Performance reporting
        self.report_timer = self.create_timer(
            10.0,  # Report every 10 seconds
            self.report_performance)
            
    def analyze_performance(self, msg):
        # Calculate latency
        current_time = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.stamp)
        latency = (current_time - msg_time).nanoseconds / 1e9  # Convert to seconds
        
        # Update statistics
        self.message_stats['count'] += 1
        self.message_stats['total_latency'] += latency
        self.message_stats['max_latency'] = max(
            self.message_stats['max_latency'], 
            latency)
        self.message_stats['min_latency'] = min(
            self.message_stats['min_latency'], 
            latency)
            
    def report_performance(self):
        if self.message_stats['count'] > 0:
            avg_latency = (
                self.message_stats['total_latency'] / 
                self.message_stats['count'])
                
            self.get_logger().info(
                f"Performance Report:\n"
                f"  Messages: {self.message_stats['count']}\n"
                f"  Avg Latency: {avg_latency:.3f}s\n" 
                f"  Min Latency: {self.message_stats['min_latency']:.3f}s\n"
                f"  Max Latency: {self.message_stats['max_latency']:.3f}s")
                
            # Reset statistics
            self.message_stats = {
                'count': 0,
                'total_latency': 0.0,
                'max_latency': 0.0,
                'min_latency': float('inf')
            }
```

---

## 🚀 Advanced Communication Patterns

### 1. Event-Driven Architecture

```python
class EventDrivenNode(Node):
    def __init__(self):
        super().__init__('event_driven_node')
        
        # Event publishers
        self.event_publishers = {
            'sensor_error': self.create_publisher(
                std_msgs.msg.String, 'events/sensor_error', 10),
            'navigation_complete': self.create_publisher(
                std_msgs.msg.String, 'events/navigation_complete', 10),
            'battery_low': self.create_publisher(
                std_msgs.msg.String, 'events/battery_low', 10)
        }
        
        # Event subscribers
        self.create_subscription(
            std_msgs.msg.String, 'events/sensor_error',
            self.handle_sensor_error, 10)
            
    def emit_event(self, event_type, data):
        if event_type in self.event_publishers:
            msg = std_msgs.msg.String()
            msg.data = data
            self.event_publishers[event_type].publish(msg)
            
    def handle_sensor_error(self, msg):
        self.get_logger().error(f'Sensor error detected: {msg.data}')
        # Trigger recovery actions
        self.emit_event('recovery_started', 'Sensor error recovery initiated')
```

### 2. Message Filtering and Routing

```python
class MessageRouter(Node):
    def __init__(self):
        super().__init__('message_router')
        
        # Input subscription
        self.input_sub = self.create_subscription(
            sensor_msgs.msg.LaserScan,
            'scan_input',
            self.route_message,
            10)
            
        # Output publishers based on criteria
        self.high_res_pub = self.create_publisher(
            sensor_msgs.msg.LaserScan, 'scan_high_res', 10)
        self.low_res_pub = self.create_publisher(
            sensor_msgs.msg.LaserScan, 'scan_low_res', 10)
        self.filtered_pub = self.create_publisher(
            sensor_msgs.msg.LaserScan, 'scan_filtered', 10)
            
    def route_message(self, msg):
        # Route based on data characteristics
        point_count = len(msg.ranges)
        
        if point_count > 1000:
            self.high_res_pub.publish(msg)
        elif point_count > 100:
            self.low_res_pub.publish(msg)
            
        # Apply filtering
        if self.contains_obstacles(msg):
            filtered_msg = self.filter_obstacles(msg)
            self.filtered_pub.publish(filtered_msg)
            
    def contains_obstacles(self, scan_msg):
        # Check for obstacles within range
        min_range = 0.1
        max_range = 5.0
        obstacle_threshold = 2.0
        
        for range_val in scan_msg.ranges:
            if min_range < range_val < obstacle_threshold:
                return True
        return False
        
    def filter_obstacles(self, scan_msg):
        # Create filtered version
        filtered_msg = copy.deepcopy(scan_msg)
        # Apply filtering logic
        return filtered_msg
```

---

## 🎓 Communication Testing

### Unit Testing Communication

```python
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestCommunication(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        
    @classmethod  
    def tearDownClass(cls):
        rclpy.shutdown()
        
    def setUp(self):
        self.node = Node('test_node')
        
    def tearDown(self):
        self.node.destroy_node()
        
    def test_publisher_subscriber(self):
        received_messages = []
        
        def callback(msg):
            received_messages.append(msg.data)
            
        # Create publisher and subscriber
        pub = self.node.create_publisher(String, 'test_topic', 10)
        sub = self.node.create_subscription(
            String, 'test_topic', callback, 10)
            
        # Give time for discovery
        time.sleep(0.5)
        
        # Publish message
        msg = String()
        msg.data = 'test_message'
        pub.publish(msg)
        
        # Spin to process message
        rclpy.spin_once(self.node, timeout_sec=1.0)
        
        # Assert message received
        self.assertEqual(len(received_messages), 1)
        self.assertEqual(received_messages[0], 'test_message')
```

---

*Continue to `03-Practical-Examples/` for hands-on implementations and real-world scenarios!*