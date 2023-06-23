# ROS2 Foxy: Video Stream Publisher and Grayscale Converter Subscriber

This tutorial will guide you through the process of creating a ROS2 package with a publisher and subscriber node in Python. The publisher node captures video from the built-in webcam using OpenCV, then publishes it to a topic. The subscriber node receives the video, converts it to grayscale, and displays it.

## ROS2 Foxy Introduction

ROS2 Foxy is the latest major release of the Robot Operating System 2 (ROS2), a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the development of complex robotic systems. This introduction section will cover the advantages of using ROS, as well as explain some of the basic concepts in ROS2, such as nodes, topics, messages, subscribers, and publishers.

### Advantages of Using ROS

ROS offers several advantages for developing robot software:

1. **Modularity:** ROS follows a modular approach, allowing developers to break down their robot software into smaller, reusable components called nodes. This modular structure enables easy collaboration, code reusability, and scalability.

2. **Communication Infrastructure:** ROS provides a flexible communication infrastructure that enables nodes to exchange data and information seamlessly. It uses a publish-subscribe pattern, allowing decoupled communication between nodes.

3. **Cross-Platform Compatibility:** ROS supports multiple operating systems and programming languages, making it versatile and accessible to a wide range of developers. It allows components written in different languages to communicate with each other.

4. **Large and Active Community:** ROS has a vibrant and supportive community of researchers, developers, and roboticists. It offers a vast ecosystem of packages and libraries, making it easier to leverage existing code and solutions for building complex robotic systems.

### Basic Concepts in ROS2

#### Nodes

In ROS2, a node is a standalone process that performs a specific computation or functionality. Nodes communicate with each other by publishing and subscribing to topics.

#### Topics

Topics are named buses that facilitate communication between nodes. A node can publish data, known as messages, to a topic, and other nodes can receive that data by subscribing to the same topic. Topics follow a publish-subscribe mechanism, allowing multiple nodes to communicate asynchronously.

#### Messages

Messages are the data structures used for communication between nodes. They define the format and type of data exchanged on topics. ROS provides a wide range of predefined message types for common data, such as sensor readings, control commands, and robot poses. Additionally, developers can define custom message types to suit their specific needs.

#### Publishers

A publisher is a node that sends messages on a specific topic. It creates and publishes messages to the topic, allowing other nodes to receive and process the data.

#### Subscribers

A subscriber is a node that receives messages from a specific topic. It subscribes to the topic and processes the incoming messages according to its functionality.

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
mkdir -p ros2_ws/src
cd ros2_ws/src
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

Navigate to the folder with the same name as the newly created package `video_tutorial`:

```bash
cd video_tutorial
```

### The Publisher Node

Create a new file named `video_publisher.py`:

```bash
touch video_publisher.py
```

Open the file `video_publisher.py` with a text editor and copy and paste the following code into the python file:

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

Make sure to save the changes to the file!

### Examine the code

The following lines are where we import the necessary libraries:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
```

`rclpy` is the standard Python library for ROS2. `Node` is the base class for all ROS2 nodes in Python. `Image` is the message type we will use for the video frames. `cv2` is the OpenCV library, which we use to interact with the webcam and capture video frames. `CvBridge` is a ROS library that provides an interface between ROS messages and OpenCV data types.

Next, we define our custom node class, `VideoPublisher`:

```python
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
```

The `VideoPublisher` class is a custom Node. Upon initialization, it creates a publisher that publishes `Image` messages on the `VideoTopic` topic. The timer is set to call a function every 0.1 seconds, which corresponds to 10 frames per second. The `CvBridge` object is used to convert between ROS Image messages and OpenCV images. The `cv2.VideoCapture(0)` command starts the video capture on the default camera, which is usually the built-in webcam.

The callback for the timer is defined in the `timer_callback` method:

```python
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
```

This function captures a frame from the webcam every time it is called, converts the frame into an `Image` message using `CvBridge`, and publishes the message. It also logs the number of the frame.

### The Subscriber Node

Create a new file named `video_subscriber.py` in the `video_tutorial` directory:

```bash
touch video_subscriber.py
```

Open the file `video_subscriber.py` with a text editor and copy and paste the following code into the python file:

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

Make sure to save the changes to the file!

### Examine the code

We start again by importing the necessary libraries:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
```

Just like before, `rclpy` is for ROS2, `Node` is for creating our custom ROS2 node, `Image` is the message type, `cv2` is for the OpenCV operations and `CvBridge` is for converting between ROS messages and OpenCV data types.

Next, we define our `VideoSubscriber` node:

```python
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
```

The `VideoSubscriber` class is also a custom Node. Upon initialization, it creates a subscription to the `VideoTopic` topic. The `listener_callback` function is called whenever a new message is received on this topic.

Finally, we define the `listener_callback` function:

```python
def listener_callback(self, msg):
    # We convert the Image message to a cv2 image.
    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    # We convert the cv2 image to grayscale.
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # We display the grayscale image.
    cv2.imshow('video', gray_image)
    cv2.waitKey(1)
```

This function is called when a new `Image` message is received. It converts the received `Image` message back into a cv2 image using `CvBridge`, then it converts the image to grayscale using OpenCV's `cvtColor` function, and displays the image using `cv2.imshow`. The `cv2.waitKey(1)` command is necessary to display the image.

## Step 4: Modifying setup.py

The `setup.py` file is used in Python projects to define metadata, dependencies, and configuration for the installation process.

In summary, it performs the following tasks:

1. Set project metadata: name, version, author, description, license, etc.
2. Specify project dependencies: other Python packages or libraries required by the project.
3. Define entry points: functions or scripts that can be invoked from the command line or used as importable modules.
4. Configure installation options: specifying additional files, package directories, or scripts to include in the distribution.
5. Provide instructions for building, testing, and distributing the project.

The `setup.py` file, used in conjunction with the `setuptools` package, enables seamless packaging, distribution, and installation of Python projects.

Navigate back one directory to locate `setup.py`:

```bash
cd ..
```

Open `setup.py` with a text editor of your choice:

```python
from setuptools import setup

package_name = 'video_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_name@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

Replace all the `TODO` sections of the code with the appropriate metadata. Make sure to update the values for `maintainer`, `maintainer_email`, `description`, and `license`. For the license, we will be using an `Apache License 2.0`.

To be able to access the nodes created by console, the `entry_points` field needs to be updated. The `entry_points` field contains the mapping between the names of your scripts and their location. Modify the `setup.py` file to have the `entry_points` match the following:

```python
entry_points={
    'console_scripts': [
        'video_publisher = video_tutorial.video_publisher:main',
        'video_subscriber = video_tutorial.video_subscriber:main',
    ],
},
```

Where, `video_publisher` is the console name that will be used to call the command, `video_tutorial` is the package name, `video_publisher` or `video_subsciber` is the name of the python script file, and lastly, `main` is a reference to the main function in the python script. Thus, the format for adding a console script for a ROS2 Foxy node should be as follows:

```python
'console_name = package_name.python_script_name:main',
```

The final form of the `setup.py` file should be as follows:

```python
from setuptools import setup

package_name = 'video_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_name@todo.todo',
    description='Example of webcam video publisher and subsciber nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = video_tutorial.video_publisher:main',
            'video_subscriber = video_tutorial.video_subscriber:main',
        ],
    },
)
```

## Step 5: Modifying package.xml

The `package.xml` file is used in ROS (Robot Operating System) projects to define metadata, dependencies, and build configuration for the package.

In summary, it performs the following tasks:

1. Set package metadata: name, version, description, maintainer, and license information.
2. Specify package dependencies: other ROS packages that the current package depends on.
3. Define package build configuration: specifying the build type, build and run dependencies, and any additional build instructions.
4. Declare package exports: specifying the ROS nodes, libraries, and other artifacts that should be exported and made available to other packages.
5. Include additional package-level information: such as package authors, URLs, and any package-specific tags or requirements.

The `package.xml` file, along with the accompanying `CMakeLists.txt` file, enables building, packaging, and managing ROS packages, allowing for easy integration and collaboration within the ROS ecosystem.

Open `package.xml` with a text editor of your choice:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>video_tutorial</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="your_name@todo.todo">your_name</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

You will need to add `sensor_msgs` and `cv_bridge` as dependencies in the file. Place these in the `<depend>` section ensuring that they are wrapped by the statement `<depend>`.

Your `package.xml` should look like the following:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>video_tutorial</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="your_name@todo.todo">your_name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

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
