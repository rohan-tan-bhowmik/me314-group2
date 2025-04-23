import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from me314_msgs.msg import CommandQueue, CommandWrapper

class moveToPoint(Node):
    def __init__(self):
        super().__init__('move_to_point')
        self.moved = 0
        
        # Replace the direct publishers with the command queue publisher
        self.command_queue_pub = self.create_publisher(CommandQueue, '/me314_xarm_command_queue', 10)
        
        # Subscribe to current arm pose and gripper position for status tracking (optional)
        self.current_arm_pose = None
        self.pose_status_sub = self.create_subscription(Pose, '/me314_xarm_current_pose', self.arm_pose_callback, 10)
        
        self.current_gripper_position = None
        self.gripper_status_sub = self.create_subscription(Float64, '/me314_xarm_gripper_position', self.gripper_position_callback, 10)
        
        self.move_sub = self.create_subscription(Point, '/cube_position', self.cube_pos_callback, 10)

    def arm_pose_callback(self, msg: Pose):
        self.current_arm_pose = msg

    def cube_pos_callback(self, point: Point):
        # point.x = 0.2
        # point.y = -0.2
        # point.z = 0.02
        
        # point.x = 0.16
        # point.y = 0.0
        # point.z = 0.5
        if self.moved > 1: return
        # point.y *= -?1.0
        point.z = -point.z if point.z < 0 else point.z 
        self.get_logger().info(f"Cube detected at: ({point.x:.3f}, {point.y:.3f}, {point.z:.3f})")
        # return
        self.publish_pose(point)
        self.moved +=1 

    def gripper_position_callback(self, msg: Float64):
        self.current_gripper_position = msg.data

    def publish_pose(self, point):
        """
        Publishes a pose command to the command queue using an array format.
        pose_array format: [x, y, z, qx, qy, qz, qw]
        """
        # Create a CommandQueue message containing a single pose command
        queue_msg = CommandQueue()
        queue_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Create a CommandWrapper for the pose command
        wrapper = CommandWrapper()
        wrapper.command_type = "pose"
        
        # Populate the pose_command with the values from the pose_array
        wrapper.pose_command.x = point.x
        wrapper.pose_command.y = point.y
        wrapper.pose_command.z = point.z
        wrapper.pose_command.qx = 1.0
        wrapper.pose_command.qy = 0.0
        wrapper.pose_command.qz = 0.0
        wrapper.pose_command.qw = 0.0
        
        # Add the command to the queue and publish
        queue_msg.commands.append(wrapper)
        self.command_queue_pub.publish(queue_msg)
        
        self.get_logger().info(f"Published Pose to command queue:\n"
                               f"  position=({wrapper.pose_command.x}, {wrapper.pose_command.y}, {wrapper.pose_command.z})\n"
                               f"  orientation=({wrapper.pose_command.qx}, {wrapper.pose_command.qy}, "
                               f"{wrapper.pose_command.qz}, {wrapper.pose_command.qw})")

    def publish_gripper_position(self, gripper_pos: float):
        """
        Publishes a gripper command to the command queue.
        For example:
          0.0 is "fully open"
          1.0 is "closed"
        """
        # Create a CommandQueue message containing a single gripper command
        queue_msg = CommandQueue()
        queue_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Create a CommandWrapper for the gripper command
        wrapper = CommandWrapper()
        wrapper.command_type = "gripper"
        wrapper.gripper_command.gripper_position = gripper_pos
        
        # Add the command to the queue and publish
        queue_msg.commands.append(wrapper)
        self.command_queue_pub.publish(queue_msg)
        
        self.get_logger().info(f"Published gripper command to queue: {gripper_pos:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = moveToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()