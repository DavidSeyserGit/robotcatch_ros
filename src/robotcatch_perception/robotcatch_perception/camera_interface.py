import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from pathlib import Path
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker


class CameraPublisher(Node):
    CAMERA_INDEX = 1
    FPS = 30
    WIDTH = 640
    HEIGHT = 480

    def __init__(self):
        super().__init__('camera_publisher')
        
        # Publishers
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.ball_position_publisher = self.create_publisher(Point, 'ball/position', 10)
        self.ball_velocity_publisher = self.create_publisher(Vector3, 'ball/velocity', 10)
        self.marker_publisher = self.create_publisher(Marker, 'ball/visualization_marker', 10)
        
        self.bridge = CvBridge()
        self.prev_ball_pos = None
        self.prev_ball_time = None

        # Initialize camera
        self.cap = cv2.VideoCapture(self.CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FPS, self.FPS)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
        self.is_camera_open = self.cap.isOpened()
        
        if not self.is_camera_open:
            self.get_logger().error(f"Failed to open camera with index {self.CAMERA_INDEX}.")

        # Load YOLO model
        try:
            package_share_directory = get_package_share_directory('robotcatch_perception')
            self.model_path = os.path.join(package_share_directory, 'resource', 'best.pt')
        except IndexError:
            self.model_path = str(Path(__file__).parent.parent / 'resource' / 'best.pt')
            self.get_logger().warn(f'Using relative model path: {self.model_path}')

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model file not found at: {self.model_path}")
            raise FileNotFoundError(f"Model file not found at: {self.model_path}")

        self.model = YOLO(self.model_path)
        self.model.conf = 0.5
        self.get_logger().info("YOLO model loaded successfully.")

        # Create timer for camera capture
        self.timer = self.create_timer(1.0 / self.FPS, self.timer_callback)

    def timer_callback(self):
        if not self.is_camera_open:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera.")
            return

        # Run inference
        results = self.model.predict(frame, conf=self.model.conf, verbose=False)
        annotated_frame = results[0].plot()

        # Publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.image_publisher.publish(img_msg)
        
        # Process ball detection
        self.process_ball_detection(results[0], frame.shape)

    def process_ball_detection(self, result, frame_shape):
        height, width = frame_shape[:2]
        
        # Default position (no detection)
        point_msg = Point()
        point_msg.x = -1.0
        point_msg.y = -1.0
        point_msg.z = 0.0
        
        # Default velocity
        vel_msg = Vector3()
        vel_msg.x = 0.0
        vel_msg.y = 0.0
        vel_msg.z = 0.0
        
        # Check for ball detections
        if len(result.boxes) > 0:
            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy() 
            cls = result.boxes.cls.cpu().numpy()
            
            ball_indices = np.where(cls == 0)[0]
            
            if len(ball_indices) > 0:
                # Get highest confidence ball detection
                best_idx = ball_indices[np.argmax(confs[ball_indices])] if len(ball_indices) > 1 else ball_indices[0]
                
                # Get bounding box and calculate center
                x1, y1, x2, y2 = boxes[best_idx]
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # Normalize coordinates
                norm_x = center_x / width
                norm_y = center_y / height
                
                # Update position message
                point_msg.x = float(norm_x)
                point_msg.y = float(norm_y)
                
                # Calculate velocity if we have previous position
                current_time = self.get_clock().now()
                if self.prev_ball_pos is not None and self.prev_ball_time is not None:
                    dt = (current_time - self.prev_ball_time).nanoseconds / 1e9
                    if dt > 0:
                        vel_x = (norm_x - self.prev_ball_pos[0]) / dt
                        vel_y = (norm_y - self.prev_ball_pos[1]) / dt
                        
                        # Update velocity message
                        vel_msg.x = float(vel_x)
                        vel_msg.y = float(vel_y)
                        
                        # Visualize velocity
                        self.publish_velocity_arrow(norm_x, norm_y, vel_x, vel_y)
                
                # Update previous position and time
                self.prev_ball_pos = (norm_x, norm_y)
                self.prev_ball_time = current_time
                
                # Visualize ball position
                self.publish_ball_marker(norm_x, norm_y)
        
        # Publish position and velocity
        self.ball_position_publisher.publish(point_msg)
        self.ball_velocity_publisher.publish(vel_msg)
        
    def publish_velocity_arrow(self, x, y, vel_x, vel_y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ball_velocity"
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Arrow start point (ball position)
        start_point = Point()
        start_point.x = (x - 0.5) * 2.0
        start_point.y = (y - 0.5) * -2.0
        start_point.z = 0.0
        marker.points.append(start_point)
        
        # Arrow end point (based on velocity)
        end_point = Point()
        scale_factor = 2
        end_point.x = start_point.x + vel_x * scale_factor
        end_point.y = start_point.y - vel_y * scale_factor
        end_point.z = 0.0
        marker.points.append(end_point)
        
        # Arrow appearance
        marker.scale.x = 0.05  # Shaft diameter
        marker.scale.y = 0.1   # Head diameter
        marker.scale.z = 0.1   # Head length
        
        # Green color
        marker.color.g = 1.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 1
        self.marker_publisher.publish(marker)
        
    def publish_ball_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ball"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position (convert normalized coordinates to visualization space)
        marker.pose.position.x = (x - 0.5) * 2.0
        marker.pose.position.y = (y - 0.5) * -2.0
        marker.pose.position.z = 0.0
        
        # Orientation (identity quaternion)
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Red color
        marker.color.r = 1.0
        marker.color.a = 1.0
        
        self.marker_publisher.publish(marker)

    def __del__(self):
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera released.")


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = None
    
    try:
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except FileNotFoundError as e:
        if camera_publisher:
            camera_publisher.get_logger().error(f"Node startup failed: {e}")
    except Exception as e:
        if camera_publisher:
            camera_publisher.get_logger().fatal(f"Unhandled exception: {e}")
        else:
            rclpy.logging.get_logger("camera_publisher_main").fatal(f"Failed to create node: {e}")
    finally:
        if camera_publisher is not None:
            camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
