---
sidebar_position: 8
---

# Interactive ROS Node Communication Examples

## Overview

This section provides interactive examples and demonstrations of ROS node communication patterns. These examples include practical code implementations, simulation scenarios, and hands-on exercises that illustrate how ROS nodes communicate using different communication patterns.

## Interactive Example 1: Publisher-Subscriber Pattern

### Basic Publisher Node

```python
#!/usr/bin/env python3
"""
Interactive ROS Publisher Example
This example demonstrates a publisher that publishes messages at a configurable rate
and allows user interaction to modify the message content.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
import time
import threading
import sys

class InteractivePublisher(Node):
    def __init__(self):
        super().__init__('interactive_publisher')

        # Create publishers
        self.string_publisher = self.create_publisher(String, 'chatter', 10)
        self.counter_publisher = self.create_publisher(Int32, 'counter', 10)

        # Timer for publishing
        self.timer = self.create_timer(0.5, self.timer_callback)  # Publish every 0.5 seconds

        # Internal counter
        self.counter = 0

        # Interactive control
        self.message_prefix = "Hello"
        self.publish_enabled = True

        # Start interactive input thread
        self.input_thread = threading.Thread(target=self.interactive_input, daemon=True)
        self.input_thread.start()

        self.get_logger().info('Interactive Publisher Node Started')
        self.get_logger().info('Commands: "msg [text]" to change message, "stop/start" to control publishing, "quit" to exit')

    def timer_callback(self):
        """Callback function for the timer that publishes messages"""
        if self.publish_enabled:
            # Create and publish string message
            msg = String()
            msg.data = f'{self.message_prefix} - Counter: {self.counter}'
            self.string_publisher.publish(msg)

            # Create and publish counter message
            counter_msg = Int32()
            counter_msg.data = self.counter
            self.counter_publisher.publish(counter_msg)

            self.get_logger().info(f'Published: "{msg.data}" and counter: {counter_msg.data}')

            self.counter += 1

    def interactive_input(self):
        """Handle interactive user input in a separate thread"""
        while rclpy.ok():
            try:
                user_input = input().strip()

                if user_input.lower() == 'quit':
                    self.get_logger().info('Quit command received. Shutting down...')
                    rclpy.shutdown()
                    sys.exit(0)
                elif user_input.lower() == 'stop':
                    self.publish_enabled = False
                    self.get_logger().info('Publishing stopped')
                elif user_input.lower() == 'start':
                    self.publish_enabled = True
                    self.get_logger().info('Publishing started')
                elif user_input.startswith('msg '):
                    new_message = user_input[4:]  # Remove 'msg ' prefix
                    self.message_prefix = new_message
                    self.get_logger().info(f'Message prefix changed to: "{new_message}"')
                else:
                    self.get_logger().info(f'Unknown command: {user_input}. Use: msg [text], stop, start, or quit')

            except EOFError:
                # Handle case where input is not available (e.g., in some environments)
                time.sleep(0.1)
                continue
            except Exception as e:
                self.get_logger().error(f'Error processing input: {e}')

def main(args=None):
    rclpy.init(args=args)

    publisher = InteractivePublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Interrupted, shutting down...')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Basic Subscriber Node

```python
#!/usr/bin/env python3
"""
Interactive ROS Subscriber Example
This example demonstrates a subscriber that processes messages and provides feedback
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32

class InteractiveSubscriber(Node):
    def __init__(self):
        super().__init__('interactive_subscriber')

        # Create subscribers
        self.string_subscription = self.create_subscription(
            String,
            'chatter',
            self.string_listener_callback,
            10)
        self.string_subscription  # prevent unused variable warning

        self.counter_subscription = self.create_subscription(
            Int32,
            'counter',
            self.counter_listener_callback,
            10)
        self.counter_subscription  # prevent unused variable warning

        self.received_count = 0
        self.last_message = ""

        self.get_logger().info('Interactive Subscriber Node Started')

    def string_listener_callback(self, msg):
        """Callback function for string messages"""
        self.received_count += 1
        self.last_message = msg.data

        self.get_logger().info(f'I heard: "{msg.data}"')
        self.get_logger().info(f'Total messages received: {self.received_count}')

        # Simple message analysis
        words = msg.data.split()
        word_count = len(words)
        self.get_logger().info(f'Word count in message: {word_count}')

    def counter_listener_callback(self, msg):
        """Callback function for counter messages"""
        self.get_logger().info(f'Counter value: {msg.data}')

        # Check for special conditions
        if msg.data % 10 == 0 and msg.data != 0:
            self.get_logger().info(f'Checkpoint reached: {msg.data} messages processed!')

def main(args=None):
    rclpy.init(args=args)

    subscriber = InteractiveSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info('Interrupted, shutting down...')
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Interactive Example 2: Service Client-Server Pattern

### Service Server Node

```python
#!/usr/bin/env python3
"""
Interactive ROS Service Server Example
This example demonstrates a service that performs calculations based on user requests
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class InteractiveServiceServer(Node):
    def __init__(self):
        super().__init__('interactive_service_server')

        # Create service
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        # Track service usage
        self.request_count = 0

        self.get_logger().info('Interactive Service Server Started')

    def add_two_ints_callback(self, request, response):
        """Callback function for the service"""
        self.request_count += 1

        # Perform the calculation
        result = request.a + request.b

        # Log the request
        self.get_logger().info(f'Incoming request: {request.a} + {request.b}')
        self.get_logger().info(f'Sending response: {result}')
        self.get_logger().info(f'Total requests processed: {self.request_count}')

        # Set the response
        response.sum = result

        # Additional processing based on result
        if result > 100:
            self.get_logger().info('Large result detected (>100)')
        elif result < 0:
            self.get_logger().info('Negative result detected')
        else:
            self.get_logger().info('Normal result range')

        return response

def main(args=None):
    rclpy.init(args=args)

    server = InteractiveServiceServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Interrupted, shutting down...')
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Node

```python
#!/usr/bin/env python3
"""
Interactive ROS Service Client Example
This example demonstrates a client that sends requests to the service server
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import threading
import time
import sys

class InteractiveServiceClient(Node):
    def __init__(self):
        super().__init__('interactive_service_client')

        # Create client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request_count = 0
        self.successful_requests = 0

        # Start interactive input thread
        self.input_thread = threading.Thread(target=self.interactive_input, daemon=True)
        self.input_thread.start()

        self.get_logger().info('Interactive Service Client Started')
        self.get_logger().info('Commands: "add [num1] [num2]" to send request, "quit" to exit')

    def send_request(self, a, b):
        """Send a request to the service"""
        self.request_count += 1

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Asynchronous call
        future = self.cli.call_async(request)

        # Wait for response with timeout
        timer = self.create_timer(1.0, lambda: self.check_future(future, request))

    def check_future(self, future, request):
        """Check if the future is complete"""
        if future.done():
            try:
                response = future.result()
                self.successful_requests += 1
                self.get_logger().info(
                    f'Result of {request.a} + {request.b} = {response.sum} '
                    f'(Success rate: {self.successful_requests}/{self.request_count})'
                )
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        else:
            self.get_logger().info('Request still pending...')

    def interactive_input(self):
        """Handle interactive user input"""
        while rclpy.ok():
            try:
                user_input = input().strip()

                if user_input.lower() == 'quit':
                    self.get_logger().info('Quit command received. Shutting down...')
                    rclpy.shutdown()
                    sys.exit(0)
                elif user_input.startswith('add '):
                    try:
                        parts = user_input.split()
                        if len(parts) == 3:
                            a = int(parts[1])
                            b = int(parts[2])
                            self.get_logger().info(f'Sending request: {a} + {b}')
                            self.send_request(a, b)
                        else:
                            self.get_logger().info('Usage: add [num1] [num2]')
                    except ValueError:
                        self.get_logger().info('Invalid numbers. Usage: add [num1] [num2]')
                else:
                    self.get_logger().info(f'Unknown command: {user_input}. Use: add [num1] [num2] or quit')

            except EOFError:
                time.sleep(0.1)
                continue
            except Exception as e:
                self.get_logger().error(f'Error processing input: {e}')

def main(args=None):
    rclpy.init(args=args)

    client = InteractiveServiceClient()

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Interrupted, shutting down...')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Interactive Example 3: Action Client-Server Pattern

### Action Server Node

```python
#!/usr/bin/env python3
"""
Interactive ROS Action Server Example
This example demonstrates a Fibonacci action server with feedback
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class InteractiveActionServer(Node):
    def __init__(self):
        super().__init__('interactive_fibonacci_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=None,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info('Interactive Fibonacci Action Server Started')

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action"""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Handle an accepted goal"""
        self.get_logger().info('Goal accepted, executing...')
        # Start executing the action in a separate thread
        import threading
        thread = threading.Thread(target=self.execute, args=(goal_handle,))
        thread.start()

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        """Execute the goal and provide feedback"""
        self.get_logger().info('Executing goal...')

        # Get the order from the goal
        order = goal_handle.request.order

        # Feedback and result
        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # Start the Fibonacci sequence
        fibonacci_sequence = [0, 1]

        # Provide initial feedback
        feedback_msg.sequence = fibonacci_sequence[:min(2, order)]
        goal_handle.publish_feedback(feedback_msg)

        # Calculate the Fibonacci sequence up to the requested order
        for i in range(1, order):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal was canceled')
                goal_handle.canceled()
                result_msg.sequence = fibonacci_sequence
                return result_msg

            # Check if goal was aborted
            if not goal_handle.is_active:
                self.get_logger().info('Goal was aborted')
                result_msg.sequence = fibonacci_sequence
                return result_msg

            # Calculate next Fibonacci number
            next_fib = fibonacci_sequence[i] + fibonacci_sequence[i-1]
            fibonacci_sequence.append(next_fib)

            # Publish feedback periodically
            if i % 2 == 0:  # Provide feedback every 2 steps
                feedback_msg.sequence = fibonacci_sequence
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'Feedback: {fibonacci_sequence}')

            # Simulate some processing time
            time.sleep(0.1)

        # Check if goal was canceled after completion
        if goal_handle.is_cancel_requested:
            self.get_logger().info('Goal was canceled')
            goal_handle.canceled()
            result_msg.sequence = fibonacci_sequence
            return result_msg

        # Set the result and return
        goal_handle.succeed()
        result_msg.sequence = fibonacci_sequence

        self.get_logger().info(f'Result: {fibonacci_sequence}')
        return result_msg

def main(args=None):
    rclpy.init(args=args)

    action_server = InteractiveActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        action_server.get_logger().info('Interrupted, shutting down...')
    finally:
        action_server.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    import time  # Added for the sleep in execute method
    main()
```

### Action Client Node

```python
#!/usr/bin/env python3
"""
Interactive ROS Action Client Example
This example demonstrates a Fibonacci action client with feedback handling
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci
import threading
import time
import sys

class InteractiveActionClient(Node):
    def __init__(self):
        super().__init__('interactive_fibonacci_action_client')

        # Create action client
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Start interactive input thread
        self.input_thread = threading.Thread(target=self.interactive_input, daemon=True)
        self.input_thread.start()

        self.get_logger().info('Interactive Fibonacci Action Client Started')
        self.get_logger().info('Commands: "fib [order]" to start Fibonacci calculation, "quit" to exit')

    def send_goal(self, order):
        """Send a goal to the action server"""
        self.get_logger().info(f'Sending Fibonacci goal with order: {order}')

        # Create goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Send goal async
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Set result callback
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Get result async
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def interactive_input(self):
        """Handle interactive user input"""
        while rclpy.ok():
            try:
                user_input = input().strip()

                if user_input.lower() == 'quit':
                    self.get_logger().info('Quit command received. Shutting down...')
                    rclpy.shutdown()
                    sys.exit(0)
                elif user_input.startswith('fib '):
                    try:
                        parts = user_input.split()
                        if len(parts) == 2:
                            order = int(parts[1])
                            if order > 0:
                                self.send_goal(order)
                            else:
                                self.get_logger().info('Order must be positive')
                        else:
                            self.get_logger().info('Usage: fib [order]')
                    except ValueError:
                        self.get_logger().info('Invalid number. Usage: fib [order]')
                else:
                    self.get_logger().info(f'Unknown command: {user_input}. Use: fib [order] or quit')

            except EOFError:
                time.sleep(0.1)
                continue
            except Exception as e:
                self.get_logger().error(f'Error processing input: {e}')

def main(args=None):
    rclpy.init(args=args)

    action_client = InteractiveActionClient()

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Interrupted, shutting down...')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Interactive Example 4: Parameter Server Usage

### Parameter Server Node

```python
#!/usr/bin/env python3
"""
Interactive ROS Parameter Server Example
This example demonstrates dynamic parameter configuration
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
import threading
import time
import sys

class InteractiveParameterNode(Node):
    def __init__(self):
        super().__init__('interactive_parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('operating_mode', 'normal')
        self.declare_parameter('debug_level', 1)

        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Internal state
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.operating_mode = self.get_parameter('operating_mode').value
        self.debug_level = self.get_parameter('debug_level').value

        # Timer for periodic parameter updates
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Start interactive input thread
        self.input_thread = threading.Thread(target=self.interactive_input, daemon=True)
        self.input_thread.start()

        self.get_logger().info('Interactive Parameter Node Started')
        self.get_logger().info('Current parameters:')
        self.print_current_parameters()
        self.get_logger().info('Commands: "set [name] [value]", "list", "quit"')

    def parameter_callback(self, params):
        """Callback for parameter changes"""
        result = SetParametersResult()
        result.successful = True
        result.reason = 'Parameters set successfully'

        for param in params:
            if param.name == 'robot_name' and param.type_ == Parameter.Type.STRING:
                self.robot_name = param.value
                self.get_logger().info(f'Robot name updated to: {param.value}')
            elif param.name == 'max_velocity' and param.type_ == Parameter.Type.DOUBLE:
                if param.value > 0 and param.value <= 5.0:  # Reasonable velocity limits
                    self.max_velocity = param.value
                    self.get_logger().info(f'Max velocity updated to: {param.value}')
                else:
                    result.successful = False
                    result.reason = 'Max velocity must be between 0 and 5.0'
            elif param.name == 'safety_distance' and param.type_ == Parameter.Type.DOUBLE:
                if param.value > 0 and param.value <= 2.0:  # Reasonable distance limits
                    self.safety_distance = param.value
                    self.get_logger().info(f'Safety distance updated to: {param.value}')
                else:
                    result.successful = False
                    result.reason = 'Safety distance must be between 0 and 2.0'
            elif param.name == 'operating_mode' and param.type_ == Parameter.Type.STRING:
                if param.value in ['normal', 'debug', 'maintenance', 'emergency']:
                    self.operating_mode = param.value
                    self.get_logger().info(f'Operating mode updated to: {param.value}')
                else:
                    result.successful = False
                    result.reason = 'Operating mode must be normal, debug, maintenance, or emergency'
            elif param.name == 'debug_level' and param.type_ == Parameter.Type.INTEGER:
                if param.value >= 0 and param.value <= 3:
                    self.debug_level = param.value
                    self.get_logger().info(f'Debug level updated to: {param.value}')
                else:
                    result.successful = False
                    result.reason = 'Debug level must be between 0 and 3'
            else:
                result.successful = False
                result.reason = f'Invalid parameter: {param.name} or invalid type'

        return result

    def timer_callback(self):
        """Periodic timer callback to demonstrate parameter usage"""
        self.get_logger().info(
            f'Robot: {self.robot_name}, Mode: {self.operating_mode}, '
            f'Vel: {self.max_velocity}m/s, Safe dist: {self.safety_distance}m'
        )

    def print_current_parameters(self):
        """Print current parameter values"""
        self.get_logger().info(f'  robot_name: {self.robot_name}')
        self.get_logger().info(f'  max_velocity: {self.max_velocity}')
        self.get_logger().info(f'  safety_distance: {self.safety_distance}')
        self.get_logger().info(f'  operating_mode: {self.operating_mode}')
        self.get_logger().info(f'  debug_level: {self.debug_level}')

    def interactive_input(self):
        """Handle interactive user input"""
        while rclpy.ok():
            try:
                user_input = input().strip()

                if user_input.lower() == 'quit':
                    self.get_logger().info('Quit command received. Shutting down...')
                    rclpy.shutdown()
                    sys.exit(0)
                elif user_input.lower() == 'list':
                    self.get_logger().info('Current parameters:')
                    self.print_current_parameters()
                elif user_input.startswith('set '):
                    parts = user_input.split()
                    if len(parts) >= 3:
                        param_name = parts[1]
                        param_value = ' '.join(parts[2:])  # Handle values with spaces

                        # Try to determine parameter type and set it
                        try:
                            # Try as integer first
                            int_val = int(param_value)
                            self.set_parameters([Parameter(param_name, Parameter.Type.INTEGER, int_val)])
                        except ValueError:
                            try:
                                # Try as float
                                float_val = float(param_value)
                                self.set_parameters([Parameter(param_name, Parameter.Type.DOUBLE, float_val)])
                            except ValueError:
                                # Assume string
                                self.set_parameters([Parameter(param_name, Parameter.Type.STRING, param_value)])
                    else:
                        self.get_logger().info('Usage: set [param_name] [value]')
                else:
                    self.get_logger().info(f'Unknown command: {user_input}. Use: set [name] [value], list, or quit')

            except EOFError:
                time.sleep(0.1)
                continue
            except Exception as e:
                self.get_logger().error(f'Error processing input: {e}')

def main(args=None):
    rclpy.init(args=args)

    # Import here to avoid circular dependency issues
    from rclpy.parameter_service import SetParametersResult

    param_node = InteractiveParameterNode()

    try:
        rclpy.spin(param_node)
    except KeyboardInterrupt:
        param_node.get_logger().info('Interrupted, shutting down...')
    finally:
        param_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Interactive Simulation: ROS Node Communication Network

### Network Monitor Node

```python
#!/usr/bin/env python3
"""
ROS Network Monitor
This example demonstrates monitoring and visualizing ROS communication patterns
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import qos_profile_system_default
import threading
import time
from collections import defaultdict, deque
import json

class ROSNetworkMonitor(Node):
    def __init__(self):
        super().__init__('ros_network_monitor')

        # Data structures for monitoring
        self.topic_subscribers = defaultdict(list)
        self.topic_publishers = defaultdict(list)
        self.message_history = defaultdict(lambda: deque(maxlen=100))  # Keep last 100 messages per topic
        self.node_connections = defaultdict(set)

        # Setup monitoring timer
        self.monitor_timer = self.create_timer(1.0, self.monitor_callback)

        # Setup network analysis timer
        self.analysis_timer = self.create_timer(5.0, self.analysis_callback)

        # Start interactive input thread
        self.input_thread = threading.Thread(target=self.interactive_input, daemon=True)
        self.input_thread.start()

        self.get_logger().info('ROS Network Monitor Started')
        self.get_logger().info('Commands: "topics", "nodes", "stats", "quit"')

    def monitor_callback(self):
        """Monitor current network state"""
        # In a real implementation, this would query the ROS graph
        # For this example, we'll just log that monitoring is active
        self.get_logger().info('Monitoring network activity...')

    def analysis_callback(self):
        """Perform network analysis"""
        # This would analyze the network topology and communication patterns
        self.get_logger().info('Network analysis completed')
        self.get_logger().info(f'Tracked topics: {list(self.message_history.keys())}')

    def interactive_input(self):
        """Handle interactive user input"""
        while rclpy.ok():
            try:
                user_input = input().strip()

                if user_input.lower() == 'quit':
                    self.get_logger().info('Quit command received. Shutting down...')
                    rclpy.shutdown()
                    sys.exit(0)
                elif user_input.lower() == 'topics':
                    self.list_topics()
                elif user_input.lower() == 'nodes':
                    self.list_nodes()
                elif user_input.lower() == 'stats':
                    self.show_statistics()
                else:
                    self.get_logger().info(f'Unknown command: {user_input}. Use: topics, nodes, stats, or quit')

            except EOFError:
                time.sleep(0.1)
                continue
            except Exception as e:
                self.get_logger().error(f'Error processing input: {e}')

    def list_topics(self):
        """List all topics with their message counts"""
        self.get_logger().info('Tracked Topics:')
        for topic, messages in self.message_history.items():
            self.get_logger().info(f'  {topic}: {len(messages)} messages')

    def list_nodes(self):
        """List all nodes (in a real system, this would query the graph)"""
        self.get_logger().info('Active Nodes: [This would show real node information in a complete implementation]')

    def show_statistics(self):
        """Show network statistics"""
        total_messages = sum(len(messages) for messages in self.message_history.values())
        self.get_logger().info(f'Total tracked messages: {total_messages}')
        self.get_logger().info(f'Tracked topics: {len(self.message_history)}')

def main(args=None):
    rclpy.init(args=args)

    monitor = ROSNetworkMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Interrupted, shutting down...')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Exercise: Build Your Own Communication System

### Exercise 1: Temperature Monitoring System

Create a ROS system with:
1. A sensor node that publishes temperature readings
2. A monitor node that subscribes to temperature data and alerts if thresholds are exceeded
3. A control node that can adjust system parameters via services

### Exercise 2: Robot Navigation System

Create a navigation system with:
1. A path planner node that provides navigation goals via actions
2. A robot controller node that executes navigation commands
3. A safety monitor node that can cancel dangerous operations

### Exercise 3: Multi-Robot Coordination

Design a system where multiple robots coordinate:
1. Each robot publishes its status and position
2. A central coordinator manages task assignment
3. Robots can request assistance from each other

## Testing and Validation Tools

### Basic Communication Test Script

```python
#!/usr/bin/env python3
"""
ROS Communication Test Script
This script helps validate basic ROS communication patterns
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from std_msgs.msg import Header
import time

class CommunicationTester(Node):
    def __init__(self):
        super().__init__('communication_tester')

        # Test publishers
        self.test_publisher = self.create_publisher(String, 'test_topic', 10)
        self.int_publisher = self.create_publisher(Int32, 'test_int', 10)
        self.float_publisher = self.create_publisher(Float32, 'test_float', 10)

        # Test subscribers
        self.test_subscription = self.create_subscription(
            String, 'test_topic', self.test_callback, 10)
        self.int_subscription = self.create_subscription(
            Int32, 'test_int', self.int_callback, 10)
        self.float_subscription = self.create_subscription(
            Float32, 'test_float', self.float_callback, 10)

        # Test timer
        self.timer = self.create_timer(1.0, self.test_timer)
        self.test_counter = 0

        self.get_logger().info('Communication Tester Started')

    def test_timer(self):
        """Send test messages"""
        # Send string message
        msg = String()
        msg.data = f'Test message {self.test_counter}'
        self.test_publisher.publish(msg)

        # Send int message
        int_msg = Int32()
        int_msg.data = self.test_counter
        self.int_publisher.publish(int_msg)

        # Send float message
        float_msg = Float32()
        float_msg.data = float(self.test_counter) * 1.5
        self.float_publisher.publish(float_msg)

        self.test_counter += 1
        self.get_logger().info(f'Sent test messages #{self.test_counter}')

    def test_callback(self, msg):
        """Handle string test message"""
        self.get_logger().info(f'Received string: {msg.data}')

    def int_callback(self, msg):
        """Handle int test message"""
        self.get_logger().info(f'Received int: {msg.data}')

    def float_callback(self, msg):
        """Handle float test message"""
        self.get_logger().info(f'Received float: {msg.data}')

def run_basic_tests():
    """Run basic communication tests"""
    rclpy.init()

    tester = CommunicationTester()

    # Run for 10 seconds
    start_time = time.time()
    while time.time() - start_time < 10.0:
        rclpy.spin_once(tester, timeout_sec=0.1)

    tester.destroy_node()
    rclpy.shutdown()
    print("Basic communication tests completed!")

if __name__ == '__main__':
    run_basic_tests()
```

These interactive examples provide hands-on experience with ROS node communication patterns, demonstrating publishers/subscribers, services, actions, parameters, and network monitoring. Each example includes practical code that can be run and modified to explore different aspects of ROS communication.