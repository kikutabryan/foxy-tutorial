# ROS2 Foxy Tutorial: Creating a Simple Publisher and Subscriber in Python

This tutorial will walk you through the process of creating a simple ROS2 package with a publisher and a subscriber node in Python. 

## Prerequisites
1. ROS2 Foxy: If not already installed, you can find instructions on the [official ROS2 website](https://index.ros.org/doc/ros2/Installation/Foxy/). 

## Steps

After installing, open a new terminal and source your ROS2 installation:
```bash
source /opt/ros/foxy/setup.bash
```

1. **Create a new ROS2 workspace and a new package**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_pub_sub
```

This creates a new directory 'ros2_ws' in your home directory. Inside 'ros2_ws/src', a new package 'py_pub_sub' is created.

2. **Navigate to the package directory**
```bash
cd py_pub_sub
```

Here, you'll see several files. The most important ones for us are:

- `setup.py`: This is where we define our Python executables.
- `package.xml`: This is where we define package dependencies.

3. **Modify setup.py**

Edit `setup.py` to look like this:

```python
from setuptools import setup

package_name = 'py_pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'py_pub_sub.publisher_node',
        'py_pub_sub.subscriber_node'
    ],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'publisher = py_pub_sub.publisher_node:main',
            'subscriber = py_pub_sub.subscriber_node:main',
        ],
    },
)
```

This tells ROS2 that we have two Python scripts, `publisher_node` and `subscriber_node`, and they can be run with the commands `ros2 run py_pub_sub publisher` and `ros2 run py_pub_sub subscriber` respectively.

4. **Create the Publisher and Subscriber nodes**

In the `py_pub_sub` directory, create two Python files `publisher_node.py` and `subscriber_node.py`. 

**publisher_node.py:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS2 ' + str(self.i)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    my_publisher = MyPublisher()

    rclpy.spin(my_publisher)

    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**subscriber_node.py:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    my_subscriber = MySubscriber()

    rclpy.spin(my_subscriber)

    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

5. **Build and source the package**

Return to the workspace directory and build your package:

```bash
cd ~/ros2_ws
colcon build --packages-select py_pub_sub
```

After building, don't forget to source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Now your nodes should be ready to run.

6. **Running the Publisher and Subscriber nodes**

Open two new terminals. In each, source your ROS2 and workspace installations:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
```

In one terminal, start the publisher:

```bash
ros2 run py_pub_sub publisher
```

In the other, start the subscriber:

```bash
ros2 run py_pub_sub subscriber
```

The subscriber should start printing the messages published by the publisher.

## Conclusion

That's it! You've created a simple ROS2 package with a publisher and subscriber node in Python. Happy coding!
