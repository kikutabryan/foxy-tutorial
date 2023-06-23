# ROS2 Foxy: Video Stream Publisher and Grayscale Converter Subscriber

This tutorial will guide you through the process of creating a ROS2 package with a publisher and subscriber node in Python. The publisher node captures video from the built-in webcam using OpenCV, then publishes it to a topic. The subscriber node receives the video, converts it to grayscale, and displays it.

## Getting Started with ROS2 Foxy

ROS2 Foxy is the most recent significant iteration of the Robot Operating System 2 (ROS2). This robust platform offers a comprehensive suite of tools, libraries, and protocols designed to streamline and facilitate the creation of intricate robotic software. This introductory section will delve into the key advantages of utilizing ROS, along with a brief overview of fundamental ROS2 concepts such as nodes, topics, messages, subscribers, and publishers.

### Why Use ROS?

Several inherent benefits make ROS an ideal choice for the development of robotic software:

1. **Modularity:** Emphasizing a modular design, ROS encourages developers to create their robot software as a collection of reusable components called nodes. This modular approach promotes collaboration, improves scalability, and increases the potential for code reusability.

2. **Advanced Communication Infrastructure:** ROS includes a flexible and efficient communication system that ensures smooth data and information transfer between nodes. Its adoption of the publish-subscribe model enables independent communication amongst nodes.

3. **Cross-Platform Compatibility:** With its support for a wide range of operating systems and programming languages, ROS stands as a versatile choice for developers worldwide. It enables components coded in different languages to communicate seamlessly.

4. **Vibrant Community Support:** Backed by an active community of researchers, developers, and roboticists, ROS offers a rich ecosystem of pre-built packages and libraries. This reduces the need for creating solutions from scratch and enables developers to build complex robotic systems more effectively.

### Unraveling the Basics of ROS2

#### Nodes

In ROS2, nodes are independent processes designed to perform specific computational tasks. Nodes can interact with each other by subscribing to and publishing on topics.

#### Topics

A topic in ROS2 is a named bus that aids in the communication between nodes. Nodes can publish structured data, called messages, to a topic. Other nodes, by subscribing to the same topic, can receive this data. Topics in ROS2 employ a publish-subscribe mechanism, thereby facilitating asynchronous communication between multiple nodes.

#### Messages

Messages in ROS2 are structured data types utilized for node-to-node communication. They establish the format and type of data to be exchanged over topics. ROS2 includes numerous predefined message types catering to common data needs such as sensor data, robot positions, and control commands. It also allows developers to design custom message types for more specific requirements.

#### Publishers

A publisher is a type of node responsible for generating and dispatching messages over a specific topic. This enables other nodes to receive and utilize the published data.

#### Subscribers

A subscriber, on the other hand, is a type of node that accepts messages from a specified topic. Upon subscription, the node can receive incoming messages and process the data as per its functionality.

## Prerequisites

- You need to have ROS2 Foxy installed. If you haven't done this already, follow the official guide [here](https://index.ros.org/doc/ros2/Installation/Foxy/).
- Make sure you have Python3 installed. Most ROS2 distributions require Python3. If you don't have Python3, download it [here](https://www.python.org/downloads/).
- OpenCV needs to be installed for the video capture functionality. Instructions are [here](https://docs.opencv.org/4.5.0/d2/de6/tutorial_py_setup_in_ubuntu.html).

## Step 1: Initialize Your ROS2 Foxy Environment

Before starting package development, it is crucial to properly initialize your ROS2 Foxy environment. This can be done in a new terminal session using the following command:

```bash
# Open a new terminal session (Ctrl + Alt + T in most Linux systems)
# Source your ROS2 Foxy environment with this command:

source /opt/ros/foxy/setup.bash
```

Please note that this command is designed for Linux-based operating systems. For other operating systems, please refer to the relevant sections in the official ROS2 documentation.

## Step 2: Establishing Workspace and Package

To create a new ROS2 Foxy package, you'll first need a workspace. If one doesn't exist, follow the steps below to create a new workspace and navigate into it:

```bash
# Create a new workspace and its 'src' directory
mkdir -p ros2_ws/src

# Navigate into the 'src' directory of the newly created workspace
cd ros2_ws/src
```

With your workspace set up, you can proceed to create a new package named `video_tutorial`. This package will have dependencies on `rclpy` and `std_msgs`:

```bash
# Create a new package 'video_tutorial' with 'rclpy' and 'std_msgs' as dependencies
ros2 pkg create --build-type ament_python video_tutorial --dependencies rclpy std_msgs
```

After successfully creating the package, navigate into the `video_tutorial` package directory:

```bash
# Navigate to the 'video_tutorial' package directory
cd video_tutorial
```


## Step 3: Writing the Python Nodes

In this step, we will create two Python nodes for publishing and subscribing. They are `video_publisher.py` and `video_subscriber.py` respectively.

First, let's navigate to the `video_tutorial` directory which we created in the previous step:

```bash
cd video_tutorial
```

### The Publisher Node

Create a new file named `video_publisher.py` in the `video_tutorial` directory:

```bash
touch video_publisher.py
```

Open the file `video_publisher.py` in a text editor and insert the following code:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

# The VideoPublisher class is a custom node for publishing video frames.
class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        # Create a publisher that publishes Image messages to the "VideoTopic" topic.
        self.publisher_ = self.create_publisher(Image, 'VideoTopic', 10)
        timer_period = 0.1  # 10 frames per second
        # Create a timer that calls the "timer_callback" function every 0.1 seconds.
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Frame counter
        self.i = 0
        # Create a CvBridge object for converting between ROS Image messages and OpenCV images.
        self.bridge = CvBridge()
        # Start capturing video from the webcam.
        self.cap = cv2.VideoCapture(0)

    def timer_callback(self):
        # Capture a new frame from the webcam.
        ret, frame = self.cap.read()
        if ret:
            # Convert the OpenCV image to an Image message.
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the Image message.
            self.publisher_.publish(msg)
            # Log the current frame number.
            self.get_logger().info('Publishing video frame: "%d"' % self.i)
            # Increment the frame counter.
            self.i += 1

def main(args=None):
    # Initialize the rclpy library.
    rclpy.init(args=args)
    # Create an instance of the VideoPublisher.
    video_publisher = VideoPublisher()
    # Spin the ROS2 node until it's shut down.
    rclpy.spin(video_publisher)
    # Destroy the node explicitly. (optional - otherwise it will be done automatically by the garbage collector)
    video_publisher.destroy_node()
    # Shutdown the rclpy library.
    rclpy.shutdown()

# This will ensure the main function gets executed only if the script is executed directly, and not imported as a module.
if __name__ == '__main__':
    main()
```

Don't forget to save the changes!

### Deep Dive into the Code

First, we import the necessary modules for the script:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
```

Here `rclpy` is the standard ROS2 Python library, `Node` is the base class for all ROS2 nodes in Python, `Image` is the ROS message type used for the video frames. `cv2` is the OpenCV library for handling image and video data, and `CvBridge` is a ROS library that provides an interface to convert between ROS messages and OpenCV data types.

Next, we have the `VideoPublisher` class:

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

In the `VideoPublisher` class, the `__init__` method initializes the node with the name 'video_publisher'. It then creates a publisher that publishes Image messages on the `VideoTopic` topic with a Quality of Service (QoS) profile of 10. The QoS profile can be adjusted to match the specific requirements of your system, but in this case, a QoS profile of 10 means that the publisher will store up to 10 messages in a buffer, and if the buffer is full, it will start to drop the oldest messages.

The `timer_period` is set to 0.1 seconds, corresponding to 10 frames per second. The `create_timer` function creates a timer that calls the `timer_callback` method every `timer_period` seconds.

The `i` variable is a frame counter initialized to 0. The `bridge` variable is an instance of the `CvBridge` class, which provides a bridge between ROS Image messages and OpenCV images. The `cap` variable is a `cv2.VideoCapture` object that starts capturing video from the webcam when it's created.

The `timer_callback` method is the callback function for the timer:

```python
def timer_callback(self):
    # We capture a new frame from the webcam.
    ret, frame = self.cap.read()
    if ret:
        # We convert the frame to an Image message.
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        # We publish the Image message.
        self.publisher_.publish(msg)
        # We log the current frame number.
        self.get_logger().info('Publishing video frame: "%d"' % self.i)
        # We increment the frame counter.
        self.i += 1
```

Every time the timer fires, it calls this function, which reads a frame from the webcam, converts the frame to a ROS Image message using the `CvBridge` instance, publishes the Image message using the publisher, logs the frame number, and increments the frame counter.

Finally, we have the `main` function:

```python
def main(args=None):
    # Initialize the rclpy library.
    rclpy.init(args=args)
    # Create an instance of the VideoPublisher.
    video_publisher = VideoPublisher()
    # Spin the ROS2 node until it's shut down.
    rclpy.spin(video_publisher)
    # Destroy the node explicitly. (optional - otherwise it will be done automatically by the garbage collector)
    video_publisher.destroy_node()
    # Shutdown the rclpy library.
    rclpy.shutdown()
```

The `main` function initializes the `rclpy` library, creates an instance of the `VideoPublisher`, and spins the ROS2 node so it can process callbacks. It then destroys the node and shuts down the `rclpy` library. The explicit call to `destroy_node` is optional since the Python garbage collector will do this automatically when the script ends, but it's included here for completeness.

The `if __name__ == '__main__':` check ensures that the `main` function is called only when this script is executed directly, not when it's imported as a module.

### The Subscriber Node

Create a new file named `video_subscriber.py` in the `video_tutorial` directory:

```bash
touch video_subscriber.py
```

Open the file `video_subscriber.py` in a text editor and insert the following code:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

# The VideoSubscriber class is a custom node for subscribing to video frames.
class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        # Create a subscription to the "VideoTopic" topic.
        # The "listener_callback" function is called whenever a new message is received.
        self.subscription = self.create_subscription(
            Image,
            'VideoTopic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert the Image message to a cv2 image.
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Convert the cv2 image to grayscale.
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Display the grayscale image.
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

Make sure to save the changes!

### Deep Dive into the Code

We start by importing the necessary modules:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
```

`rclpy` is the ROS2 Python library, `Node` is the base class for ROS2 nodes in Python, `Image` is the ROS message type for video frames, `cv2` is the OpenCV library for image and video processing, and `CvBridge` is a ROS library for converting between ROS messages and OpenCV data types.

Next, we define the `VideoSubscriber` class:

```python
class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        # Create a subscription to the "VideoTopic" topic.
        # The "listener_callback" function is called whenever a new message is received.
        self.subscription = self.create_subscription(
            Image,
            'VideoTopic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
```

The `VideoSubscriber` class is a custom node for subscribing to video frames. In the `__init__` method, it creates a subscription to the `VideoTopic` topic. Whenever a new message is received on this topic, the `listener_callback` method is called.

Finally, we have the `listener_callback` method:

```python
def listener_callback(self, msg):
    # Convert the Image message to a cv2 image.
    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    # Convert the cv2 image to grayscale.
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Display the grayscale image.
    cv2.imshow('video', gray_image)
    cv2.waitKey(1)
```

This method is called when a new `Image` message is received. It converts the received message to a cv2 image using `CvBridge`, converts the image to grayscale using OpenCV's `cvtColor` function, and displays the grayscale image using `cv2.imshow`. The `cv2.waitKey(1)` command is necessary to display the image and wait for a key press.

The `main` function and the `if __name__ == '__main__':` check are the same as in the publisher node section and ensure that the node is executed when the script is run directly.


## Step 4: Modifying setup.py

The `setup.py` file is an essential part of Python projects as it defines metadata, dependencies, and configuration for the installation process.

The `setup.py` file performs the following tasks:

1. Sets project metadata: name, version, author, description, license, etc.
2. Specifies project dependencies: other Python packages or libraries required by the project.
3. Defines entry points: functions or scripts that can be invoked from the command line or used as importable modules.
4. Configures installation options: specifying additional files, package directories, or scripts to include in the distribution.
5. Provides instructions for building, testing, and distributing the project.

The `setup.py` file, in conjunction with the `setuptools` package, enables seamless packaging, distribution, and installation of Python projects.

Let's navigate back one directory to locate `setup.py`:

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

Replace all the `TODO` sections in the code with the appropriate metadata. Update the values for `maintainer`, `maintainer_email`, `description`, and `license`. In this case, we will use the `Apache License 2.0`.

To enable access to the nodes through the command line, the `entry_points` field needs to be updated. The `entry_points` field contains the mapping between the names of your scripts and their locations. Modify the `setup.py` file to have the `entry_points` match the following:

```python
entry_points={
    'console_scripts': [
        'video_publisher = video_tutorial.video_publisher:main',
        'video_subscriber = video_tutorial.video_subscriber:main',
    ],
},
```

In this example, `video_publisher` is the console name that will be used to call the command, `video_tutorial` is the package name, `video_publisher` or `video_subscriber` is the name of the Python script file, and `main` is a reference to the main function in the Python script. To add a console script for a ROS2 Foxy node, use the following format:

```python
'console_name = package_name.python_script_name:main',
```

The final version of the `setup.py` file should be as follows:

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
    description='Example of webcam video publisher and subscriber nodes',
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

The `package.xml` file in ROS (Robot Operating System) projects defines metadata, dependencies, and build configuration for the package.

The `package.xml` file performs the following tasks:

1. Sets package metadata: name, version, description, maintainer, and license information.
2. Specifies package dependencies: other ROS packages that the current package depends on.
3. Defines package build configuration: specifying the build type, build and run dependencies, and any additional build instructions.
4. Declares package exports: specifying the ROS nodes, libraries, and other artifacts that should be exported and made available to other packages.
5. Includes additional package-level information: such as package authors, URLs, and any package-specific tags or requirements.

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

You will need to add `sensor_msgs` and `cv_bridge` as dependencies in the file. Place these in the `<depend>` section, ensuring that they are wrapped by the `<depend>` statement.

Your updated `package.xml` should look like the following:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>video_tutorial</name>
  <version>0.0.0</version>
  <description>Example of webcam video publisher and subscriber nodes</description>
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

The updated `package.xml` file includes the `sensor_msgs` and `cv_bridge` dependencies, making them part of the `<depend>` section. This ensures that the package can properly build and run with the required dependencies.

## Step 6: Build and Source the Package

Navigate to the root of your workspace:

```bash
cd ../..
```

Build the workspace:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Building the workspace generates executable files from your nodes, compiling them into a usable form. Sourcing the workspace ensures that ROS2 can find these executables and use them.

## Step 7: Run the Nodes

Now, you can run the publisher node in one terminal:

```bash
ros2 run video_tutorial video_publisher
```

And the subscriber node in another terminal:

```bash
ros2 run video_tutorial video_subscriber
```

Running the publisher node starts capturing video frames from the webcam and publishing them to the `VideoTopic` topic. The subscriber node receives these frames from the `VideoTopic` topic, converts them to grayscale, and displays them in a new window.

You should now see your video feed displayed in grayscale in the subscriber terminal window.

Congratulations! You have successfully created a ROS2 package with a video publisher and a grayscale converter subscriber. Feel free to explore and enhance the functionality of your ROS2 project. Happy coding!


## License

This project is licensed under the Apache-2.0 License - see the [LICENSE.md](LICENSE.md) file for details
