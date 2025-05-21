import rclpy
from rclpy.node import Node
import cv2
import numpy as np
# import torch # You might not need torch directly anymore, depending on later code
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from pathlib import Path

# Import the YOLO class from the ultralytics library
from ultralytics import YOLO

# Also need ament_index_python to find resource files after installation
from ament_index_python.packages import get_package_share_directory


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

        # Check if camera opened successfully
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera with index {self.CAMERA_INDEX}.")
            # You might want to handle this more gracefully, e.g., exit the node
            # rclpy.shutdown() # Not ideal here, maybe raise an exception or set a flag
            # For now, we'll let it continue, but timer_callback won't do anything if cap is not open
            self.is_camera_open = False
        else:
             self.is_camera_open = True


        # Load YOLOv11n model using the ultralytics library
        try:
            # Get the path to the package's share directory
            package_share_directory = get_package_share_directory('robotcatch_perception')
            # Construct the full path to the model file within the resource directory
            self.model_path = os.path.join(package_share_directory, 'resource', 'best.pt')
            self.get_logger().info(f'Attempting to load model from ROS install share: {self.model_path}')
        except IndexError:
             # Fallback if ament_index_python cannot find the package (e.g., running directly from source)
            self.model_path = str(Path(__file__).parent.parent / 'resource' / 'best.pt')
            self.get_logger().warn(f'Could not find package share directory. Loading model from relative path: {self.model_path}')


        # Add a check to ensure the file exists *before* trying to load the model
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model file not found at: {self.model_path}")
             # Consider raising an exception or handling this failure gracefully
            raise FileNotFoundError(f"Model file not found at: {self.model_path}")

        self.get_logger().info(f'Loading model from: {self.model_path}')

        # Use the YOLO class to load the model
        self.model = YOLO(self.model_path)
        # Set confidence threshold
        self.model.conf = 0.5 # Note: this might need to be set when running inference depending on the ultralytics version

        self.get_logger().info("YOLO model loaded successfully.")


        # Create timer for camera capture
        timer_period = 1.0 / self.FPS  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Only attempt to capture and process if the camera is open
        if not self.is_camera_open:
             self.get_logger().warn("Camera not open. Skipping frame processing.")
             return

        # Capture frame from camera
        ret, frame = self.cap.read()
        if ret:
            # Run inference using the predict method
            # Note: results object structure is different in newer ultralytics versions
            # verbose=False suppresses inference output to console
            results = self.model.predict(frame, conf=self.model.conf, verbose=False)

            # Draw detection results - ultralytics results object has a plot() method
            # plot() returns a numpy array (the annotated image)
            annotated_frame = results[0].plot()

            # Convert OpenCV image (numpy array) to ROS message
            # plot() returns BGR format by default, so 'bgr8' is correct
            msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

            # Publish the image
            self.publisher_.publish(msg)
            # self.get_logger().info('Publishing detected frame') # This can be noisy

        else:
             self.get_logger().warn("Failed to read frame from camera.")


    def __del__(self):
        # Release the camera when the node is destroyed
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera released.")


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = None # Initialize to None
    try:
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except FileNotFoundError as e:
        camera_publisher.get_logger().error(f"Node startup failed: {e}")
    except Exception as e:
        if camera_publisher:
             camera_publisher.get_logger().fatal(f"Unhandled exception: {e}", exc_info=True)
        else:
             # Log error if node creation itself failed
             rclpy.logging.get_logger("camera_publisher_main").fatal(f"Failed to create CameraPublisher node: {e}", exc_info=True)
    finally:
        if camera_publisher is not None:
            camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
