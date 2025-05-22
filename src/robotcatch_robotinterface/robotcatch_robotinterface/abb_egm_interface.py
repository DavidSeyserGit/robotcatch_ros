import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ABBRobotEGM import EGM


class ABBRobotEGMNode(Node):
    def __init__(self):
        super().__init__('abb_robot_egm_node')
        
        # Create a publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'abb_robot/joint_states',
            10
        )
        
        # Initialize EGM connection
        self.egm = EGM()
        
        # Create a timer to poll the robot state
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        
        self.get_logger().info('ABB Robot EGM node has been started')

    def timer_callback(self):
        # Get current joint states
        success, state = self.egm.receive_from_robot()

        
        if not success:
            self.get_logger().error('Failed to receive data from robot')
            return
            
        # Create and publish joint state message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'joint_{i+1}' for i in range(6)]
        joint_state_msg.position = state.joint_angles
        
        self.joint_state_publisher.publish(joint_state_msg)


    def destroy_node(self):
        # Clean up EGM connection
        if hasattr(self, 'egm') and self.egm is not None:
            self.egm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ABBRobotEGMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
