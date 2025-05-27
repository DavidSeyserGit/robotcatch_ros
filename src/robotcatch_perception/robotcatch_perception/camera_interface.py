import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import os
from pathlib import Path
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Vector3, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import message_filters


class RealSenseYOLOProcessor(Node):
    def __init__(self):
        super().__init__('realsense_yolo_processor')
        
        # Publishers
        self.ball_position_publisher = self.create_publisher(
            Point, 'ball/position', 10
        )
        self.ball_position_3d_publisher = self.create_publisher(
            PointStamped, 'ball/position_3d', 10
        )
        self.ball_velocity_publisher = self.create_publisher(
            Vector3, 'ball/velocity', 10
        )
        self.ball_velocity_3d_publisher = self.create_publisher(
            Vector3, 'ball/velocity_3d', 10
        )
        self.marker_publisher = self.create_publisher(
            Marker, 'ball/visualization_marker', 10
        )
        self.annotated_image_publisher = self.create_publisher(
            Image, 'camera/annotated_image', 10
        )
        
        self.bridge = CvBridge()
        self.prev_ball_pos = None
        self.prev_ball_pos_3d = None
        self.prev_ball_time = None
        self.camera_info = None

        # Load YOLO model
        try:
            package_share_directory = get_package_share_directory(
                'robotcatch_perception'
            )
            self.model_path = os.path.join(
                package_share_directory, 'resource', 'best.pt'
            )
        except IndexError:
            self.model_path = str(
                Path(__file__).parent.parent / 'resource' / 'best.pt'
            )
            self.get_logger().warn(f'Using relative model path: {self.model_path}')

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model file not found at: {self.model_path}")
            raise FileNotFoundError(f"Model file not found at: {self.model_path}")

        self.model = YOLO(self.model_path)
        self.model.conf = 0.5
        self.get_logger().info("YOLO model loaded successfully.")

        # Subscribers for synchronized RGB and Depth
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/realsense2_camera/color/image_raw'
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/realsense2_camera/aligned_depth_to_color/image_raw'
        )
        
        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/realsense2_camera/color/camera_info', 
            self.camera_info_callback, 10
        )
        
        # Synchronize RGB and Depth messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.1
        )
        self.ts.registerCallback(self.synchronized_callback)
        
        self.get_logger().info("RealSense YOLO processor initialized. Waiting for camera data...")

    def camera_info_callback(self, msg):
        """Store camera intrinsics"""
        self.camera_info = msg

    def pixel_to_3d_point(self, pixel_x, pixel_y, depth_value):
        """Convert pixel coordinates and depth to 3D point"""
        if depth_value == 0 or self.camera_info is None:
            return None
        
        # Get camera intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        # Convert depth from mm to meters (RealSense depth is typically in mm)
        depth_m = depth_value / 1000.0
        
        # Convert pixel to 3D point
        x = (pixel_x - cx) * depth_m / fx
        y = (pixel_y - cy) * depth_m / fy
        z = depth_m
        
        return [x, y, z]

    def synchronized_callback(self, rgb_msg, depth_msg):
        """Process synchronized RGB and depth images"""
        try:
            # Convert ROS images to OpenCV format
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            
            # Run YOLO inference on RGB image
            results = self.model.predict(rgb_image, conf=self.model.conf, verbose=False)
            annotated_frame = results[0].plot()
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            annotated_msg.header = rgb_msg.header
            self.annotated_image_publisher.publish(annotated_msg)
            
            # Process ball detection with depth information
            self.process_ball_detection(results[0], rgb_image.shape, depth_image, rgb_msg.header)
            
        except Exception as e:
            self.get_logger().error(f"Error in synchronized callback: {e}")

    def process_ball_detection(self, result, frame_shape, depth_image, header):
        height, width = frame_shape[:2]
        
        # Default 2D position (no detection)
        point_msg = Point()
        point_msg.x = -1.0
        point_msg.y = -1.0
        point_msg.z = 0.0
        
        # Default 3D position
        point_3d_msg = PointStamped()
        point_3d_msg.header = header
        point_3d_msg.point.x = 0.0
        point_3d_msg.point.y = 0.0
        point_3d_msg.point.z = 0.0
        
        # Default velocities
        vel_msg = Vector3()
        vel_3d_msg = Vector3()
        
        # Check for ball detections
        if len(result.boxes) > 0:
            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy() 
            cls = result.boxes.cls.cpu().numpy()
            
            ball_indices = np.where(cls == 0)[0]
            
            if len(ball_indices) > 0:
                # Get highest confidence ball detection
                best_idx = (ball_indices[np.argmax(confs[ball_indices])] 
                           if len(ball_indices) > 1 else ball_indices[0])
                
                # Get bounding box and calculate center
                x1, y1, x2, y2 = boxes[best_idx]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                # Normalize coordinates for 2D position
                norm_x = center_x / width
                norm_y = center_y / height
                
                # Update 2D position message
                point_msg.x = float(norm_x)
                point_msg.y = float(norm_y)
                
                # Get depth value at ball center
                if (0 <= center_y < depth_image.shape[0] and 
                    0 <= center_x < depth_image.shape[1]):
                    depth_value = depth_image[center_y, center_x]
                    
                    if depth_value > 0:
                        # Convert to 3D coordinates
                        point_3d = self.pixel_to_3d_point(center_x, center_y, depth_value)
                        
                        if point_3d is not None:
                            point_3d_msg.point.x = float(point_3d[0])
                            point_3d_msg.point.y = float(point_3d[1])
                            point_3d_msg.point.z = float(point_3d[2])
                            
                            # Calculate velocities
                            current_time = self.get_clock().now()
                            if (self.prev_ball_pos is not None and 
                                self.prev_ball_pos_3d is not None and 
                                self.prev_ball_time is not None):
                                
                                dt = (current_time - self.prev_ball_time).nanoseconds / 1e9
                                if dt > 0:
                                    # 2D velocity
                                    vel_x = (norm_x - self.prev_ball_pos[0]) / dt
                                    vel_y = (norm_y - self.prev_ball_pos[1]) / dt
                                    vel_msg.x = float(vel_x)
                                    vel_msg.y = float(vel_y)
                                    
                                    # 3D velocity
                                    vel_3d_x = (point_3d[0] - self.prev_ball_pos_3d[0]) / dt
                                    vel_3d_y = (point_3d[1] - self.prev_ball_pos_3d[1]) / dt
                                    vel_3d_z = (point_3d[2] - self.prev_ball_pos_3d[2]) / dt
                                    vel_3d_msg.x = float(vel_3d_x)
                                    vel_3d_msg.y = float(vel_3d_y)
                                    vel_3d_msg.z = float(vel_3d_z)
                                    
                                    # Visualize 3D position and velocity
                                    self.publish_3d_markers(point_3d, [vel_3d_x, vel_3d_y, vel_3d_z], header)
                            
                            # Update previous positions and time
                            self.prev_ball_pos = (norm_x, norm_y)
                            self.prev_ball_pos_3d = point_3d
                            self.prev_ball_time = current_time
        
        # Publish all messages
        self.ball_position_publisher.publish(point_msg)
        self.ball_position_3d_publisher.publish(point_3d_msg)
        self.ball_velocity_publisher.publish(vel_msg)
        self.ball_velocity_3d_publisher.publish(vel_3d_msg)

    def publish_3d_markers(self, position, velocity, header):
        """Publish 3D visualization markers for ball position and velocity"""
        # Ball position marker
        ball_marker = Marker()
        ball_marker.header = header
        ball_marker.ns = "ball_3d"
        ball_marker.id = 0
        ball_marker.type = Marker.SPHERE
        ball_marker.action = Marker.ADD
        
        ball_marker.pose.position.x = position[0]
        ball_marker.pose.position.y = position[1]
        ball_marker.pose.position.z = position[2]
        ball_marker.pose.orientation.w = 1.0
        
        ball_marker.scale.x = 0.05
        ball_marker.scale.y = 0.05
        ball_marker.scale.z = 0.05
        
        ball_marker.color.r = 1.0
        ball_marker.color.a = 1.0
        
        self.marker_publisher.publish(ball_marker)
        
        # Velocity arrow marker
        velocity_magnitude = np.sqrt(sum(v**2 for v in velocity))
        if velocity_magnitude > 0.01:  # Only show if velocity is significant
            arrow_marker = Marker()
            arrow_marker.header = header
            arrow_marker.ns = "ball_velocity_3d"
            arrow_marker.id = 1
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            # Start point
            start_point = Point()
            start_point.x = position[0]
            start_point.y = position[1]
            start_point.z = position[2]
            arrow_marker.points.append(start_point)
            
            # End point (scaled velocity)
            scale_factor = 0.5
            end_point = Point()
            end_point.x = position[0] + velocity[0] * scale_factor
            end_point.y = position[1] + velocity[1] * scale_factor
            end_point.z = position[2] + velocity[2] * scale_factor
            arrow_marker.points.append(end_point)
            
            arrow_marker.scale.x = 0.01  # Shaft diameter
            arrow_marker.scale.y = 0.02  # Head diameter
            arrow_marker.scale.z = 0.02  # Head length
            
            arrow_marker.color.g = 1.0
            arrow_marker.color.a = 1.0
            
            arrow_marker.lifetime.sec = 1
            self.marker_publisher.publish(arrow_marker)


def main(args=None):
    rclpy.init(args=args)
    processor = None
    
    try:
        processor = RealSenseYOLOProcessor()
        rclpy.spin(processor)
    except Exception as e:
        if processor:
            processor.get_logger().fatal(f"Unhandled exception: {e}")
        else:
            rclpy.logging.get_logger("realsense_yolo_main").fatal(
                f"Failed to create node: {e}"
            )
    finally:
        if processor is not None:
            processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()