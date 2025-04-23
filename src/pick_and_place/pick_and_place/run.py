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

import time

class run(Node):
    def __init__(self):
        super().__init__('run')

        self.cube_position = None
        self.stage = 0

        # Stage 0 = locate red + green
        # Stage 1 = grip red
        # Stage 2 = go to green + release
        # Stage 3 = done! return home

        self.cube_sub = self.create_subscription(Point, '/cube_position', self.cube_pos_callback, 10)

        self.command_queue_pub = self.create_publisher(CommandQueue, '/me314_xarm_command_queue', 10)
        # self.gripper_status_sub = self.create_subscription(Float64, '/me314_xarm_gripper_position', self.gripper_position_callback, 10)

        self.create_timer(3.0, self.stage_machine)

    def stage_machine(self):
        if self.stage == 0:
            self.publish_pose(self.cube_position)
            # time.sleep(3)
            self.stage += 1

    
    def cube_pos_callback(self, point: Point):
        self.cube_position = point

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


    # def point_reached():
    #     return
    
def main(args=None):
    rclpy.init(args=args)
    node = run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()