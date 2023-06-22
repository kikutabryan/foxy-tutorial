# ROS2 Foxy: Video Stream Publisher and Grayscale Converter Subscriber

This tutorial will guide you through the process of creating a ROS2 package with a publisher and subscriber node in Python. The publisher node captures video from the built-in webcam using OpenCV, then publishes it to a topic. The subscriber node receives the video, converts it to grayscale, and displays it.

## Prerequisites

- You need to have ROS2 Foxy installed. If you haven't done this already, follow the official guide [here](https://index.ros.org/doc/ros2/Installation/Foxy/).
- Make sure you have Python3 installed. Most ROS2 distributions require Python3. If you don't have Python3, download it [here](https://www.python.org/downloads/).
- OpenCV needs to be installed for the video capture functionality. Instructions are [here](https://docs.opencv.org/4.5.0/d2/de6/tutorial_py_setup_in_ubuntu.html).

## Step 1: Source ROS2 Foxy

Before creating a new package, you need to source your ROS2 Foxy environment. Open a new terminal window and enter:

```bash
source /opt/ros/foxy/setup.bash
```

## Step 2: Creating Workspace and Package

If you haven't set up a workspace yet, create a new one:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Next, create a new package named `video_tutorial` with dependencies on `rclpy` and `std_msgs`.

```bash
ros2 pkg create --build-type ament_python video_tutorial --dependencies rclpy std_msgs
```

Navigate to the newly created package:

```bash
cd video_tutorial
```

## Step 3: Writing the Python Nodes

We will write two Python scripts, one for the publisher (`video_publisher.py`) and one for the subscriber (`video_subscriber.py`).

### The Publisher Node

Create a new file named `video_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

# Our Node class inherits from the Node base class.
class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        # We create a publisher that publishes to the "VideoTopic" topic.
        # We also set the Quality of Service profile to 10.
        self.publisher_ = self.create_publisher(Image, 'VideoTopic', 10)
        timer_period = 0.1  # seconds (10fps)
        # We create a timer that calls the "timer_callback" function every 0.1 seconds.
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.bridge = CvBridge()
        # We start capturing video from the webcam.
        self.cap = cv2.VideoCapture(0)

    def timer_callback(self):
        # We capture a new frame from the webcam.
        ret, frame = self.cap.read()
        if ret:
            # We convert the frame to an Image message.
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # We publish the Image message.
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing video frame: "%d"' % self.i)
            self.i += 1

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

The `VideoPublisher` class is a custom Node. Upon initialization, it creates a publisher that publishes `Image` messages on the `VideoTopic` topic.

The `timer_callback` function captures a frame from the webcam every time it is called, converts the frame into an `Image` message, and publishes the message.

### The Subscriber Node

Create a new file named `video_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        # We create a subscription to the "VideoTopic" topic.
        # The "listener_callback" function is called whenever a new message is received.
        self.subscription = self.create_subscription(
            Image,
            'VideoTopic',
            self.listener_callback,
            10)
        self.subscription
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # We convert the Image message to a cv2 image.
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # We convert the cv2 image to grayscale.
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # We display the grayscale image.
        cv2.imshow('video', gray_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()
    rclpy.spin(video_subscriber)
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

The `VideoSubscriber` class is also a custom Node. Upon initialization, it creates a subscription to the `VideoTopic` topic. The `listener_callback` function is called whenever a new message is received.

The `listener_callback` function converts the received `Image` message back into a cv2 image, converts the image to grayscale, and displays the image.

## Step 4: Modifying setup.py

You have to add your node's entry points to the `setup.py` file for the nodes to be correctly linked to your package.

Replace the content of the `setup.py` file located in the `video_tutorial` directory with the following:

```python
from setuptools import setup

package_name = 'video_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'video_publisher',
        'video_subscriber',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
    ],
    description='ROS2 video stream publisher and grayscale converter.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = video_publisher:main',
            'video_subscriber = video_subscriber:main',
        ],
    },
)
```

The `entry_points` field contains the mapping between the names of your scripts and their location.

## Step 5: Modifying package.xml

Open the `package.xml` file in your `video_tutorial` directory and make sure to add `sensor_msgs` and `cv_bridge` as dependencies. Your `package.xml` should look like the following:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>video_tutorial</name>
  <version>0.0.0</version>
  <description>My first ROS 2 package</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

The `package.xml` file contains information about the package, including its dependencies. Here we specify that our package depends on `sensor_msgs` and `cv_bridge`.

## Step 6: Build and Source the Package

Navigate to the root of your workspace:

```bash
cd ~/ros2_ws
```

Build the workspace:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Building the workspace generates executable files from your nodes. Sourcing the workspace allows ROS2 to find these executables.

## Step 7: Run the Nodes

You can now run the publisher node in one terminal:

```bash
ros2 run video_tutorial video_publisher
```

And the subscriber node in another terminal:

```bash
ros2 run video_tutorial video_subscriber
```

You should see your video feed displayed in grayscale in a new window.

That's it! You've successfully created a ROS2 package with a video publisher and a grayscale converter subscriber. Happy coding!

## License

This project is licensed under the Apache-2.0 License - see the [LICENSE.md](LICENSE.md) file for details
```
