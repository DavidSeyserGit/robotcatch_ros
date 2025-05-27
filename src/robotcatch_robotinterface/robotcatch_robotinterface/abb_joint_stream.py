import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ABBRobotEGM import EGM
import math

class ABBRobotEGMNode(Node):
    def __init__(self):
        super().__init__('abb_joint_publisher')
        # Publish into the 'joint_states' topic directly
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',  # publish where robot_state_publisher listens
            10
        )
        self.egm = EGM()
        # 250 Hz timer
        self.timer = self.create_timer(0.004, self.timer_callback)
        self.get_logger().info('ABB Robot EGM node has been started')

    def timer_callback(self):
        success, state = self.egm.receive_from_robot()
        if not success:
            self.get_logger().error('Failed to receive data from robot')
            return

        # convert degree→radian
        radians = [math.radians(deg) for deg in state.joint_angles]

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'joint_{i+1}' for i in range(len(radians))]
        joint_state_msg.position = radians

        self.joint_state_publisher.publish(joint_state_msg)

    def destroy_node(self):
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
