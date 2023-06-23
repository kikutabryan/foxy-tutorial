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
