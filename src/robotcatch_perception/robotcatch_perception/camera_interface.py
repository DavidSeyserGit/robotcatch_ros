import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher(Node):
    
    # Constants for camera configuration
    CAMERA_INDEX = 0  # Use 0 for default camera
    FPS = 30
    WIDTH = 640
    HEIGHT = 480

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FPS, self.FPS)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
        
        # Create timer for camera capture
        timer_period = 1.0 / self.FPS  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Capture frame from camera
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # Publish the image
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing camera frame')
            
            # Display the image (comment this out if running headless)
            cv2.imshow('Camera Feed', frame)
            cv2.waitKey(1)

    def __del__(self):
        # Release the camera when the node is destroyed
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
