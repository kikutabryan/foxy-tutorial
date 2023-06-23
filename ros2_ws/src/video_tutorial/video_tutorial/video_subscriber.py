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
