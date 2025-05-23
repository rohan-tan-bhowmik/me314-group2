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
from sensor_msgs.msg import Image, CameraInfo
import matplotlib.pyplot as plt
import time

class run(Node):
    def __init__(self):
        super().__init__('run')

        self.current_arm_pose = None
        self.current_gripper_position = None
        self.cube_position = None
        self.goal_position = None
        self.stage = 0
        self.cube_initialized = False
        self.goal_initialized = False

        self.hole_position = None
        self.peg_position = None
        self.hole_initialized = False
        self.peg_initialized = False

        self.save_pt = Point()

        # Stage 0 = locate red + green
        # Stage 1 = grip red
        # Stage 2 = go to green + release
        # Stage 3 = done! return home

        self.pose_status_sub = self.create_subscription(Pose, '/me314_xarm_current_pose', self.arm_pose_callback, 10)
        self.gripper_status_sub = self.create_subscription(Float64, '/me314_xarm_gripper_position', self.gripper_position_callback, 10)
        self.cube_sub = self.create_subscription(Point, '/cube_position', self.cube_pos_callback, 10)
        self.goal_sub = self.create_subscription(Point, '/goal_position', self.goal_pos_callback, 10)
        
        self.hole_sub = self.create_subscription(Point, '/hole_position', self.hole_pos_callback, 10)
        self.peg_sub = self.create_subscription(Point, '/peg_position', self.peg_pos_callback, 10)

        self.command_queue_pub = self.create_publisher(CommandQueue, '/me314_xarm_command_queue', 10)

        # self.image_sub = self.create_subscription(
        #     Image,
        #     '/color/image_raw', 
        #     self.show_camfeed,
        #     10
        # )
        self.task = "peg" #"block"
        self.create_timer(3.0, self.stage_machine)

    # def show_camfeed(self, msg):
    #     self.get_logger().info("Received RGB image")
    #     cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #     plt.imshow(cv_image)
    #     plt.show()

    def stage_machine(self):
        self.get_logger().info("Stage: " + str(self.stage))
        if self.stage == 0:
            
            # TO DO: Go to home position
            if self.task == "peg":
                if self.hole_position == None or self.cube_position == None:
                    if self.current_arm_pose.position.z > 0.5:
                        self.get_logger().info("Max height of arm reached - cube or goal not found")
                        return
                    point = Point()
                    point.x = self.current_arm_pose.position.x
                    point.y = self.current_arm_pose.position.y
                    point.z = self.current_arm_pose.position.z + 0.05
                    self.publish_pose(point)
                    return
                else:
                    self.get_logger().info("Both peg and hole found")
                    self.stage += 1
            # lift arm until both cube and goal are visible
            if self.task == "block":
                if self.cube_position == None or self.goal_position == None:
                    if self.current_arm_pose.position.z > 0.5:
                        self.get_logger().info("Max height of arm reached - cube or goal not found")
                        return
                    point = Point()
                    point.x = self.current_arm_pose.position.x
                    point.y = self.current_arm_pose.position.y
                    point.z = self.current_arm_pose.position.z + 0.03
                    self.publish_pose(point)
                    return
                else:
                    self.get_logger().info("Both cube and goal found")
                    self.stage += 1

        elif self.stage == 1:
            # self.cube_position.z = self.cube_position.z - 0.0233
            pos = self.cube_position
            pos.z = 0.06
            if self.task == "peg":
                pos = self.cube_position
                pos.x += 0.015
                pos.z = 0.1
            self.publish_pose(pos)
            self.get_logger().info("In position to grab")
            self.stage += 1

        elif self.stage == 2:
            # TO DO: Grab Cube
            self.publish_gripper_position(0.49 if self.task == "peg" else 0.75)
            self.get_logger().info("Grabbing cube")
            self.stage += 1

        elif self.stage == 3:
            # self.goal_position.z = self.goal_position.z + 0.03
            point = Point()
            point.x = self.current_arm_pose.position.x
            point.y = self.current_arm_pose.position.y
            point.z = self.current_arm_pose.position.z + 0.11
            self.publish_pose(point)

            pos = ""
            if self.task == "peg":


                pos = self.hole_position
                pos.y -= 0.005
                pos.z = self.current_arm_pose.position.z
                self.publish_pose(pos)
                time.sleep(0.1)

                pos = self.hole_position
                pos.z = 0.12

                self.publish_pose(pos)
            else:
                pos = self.goal_position
                pos.z = self.current_arm_pose.position.z
                self.publish_pose(pos)
            self.get_logger().info("In position to release")
            self.stage += 1

        elif self.stage == 4:
            # TO DO: Release Cube
            self.publish_gripper_position(0.0)
            self.get_logger().info("Releasing cube")
            self.stage += 1

        # time.sleep(3)

    def arm_pose_callback(self, msg: Pose):
        self.current_arm_pose = msg

    def cube_pos_callback(self, point: Point):
        if (self.cube_initialized):
            return
        self.cube_position = point
        if (point != None):
            self.cube_initialized = True

    def goal_pos_callback(self, point: Point):
        if (self.goal_initialized):
            return
        self.goal_position = point
        if (point != None):
            self.goal_initialized = True
    
    def hole_pos_callback(self, point: Point):
        if (self.hole_initialized):
            return
        self.hole_position = point
        if (point != None):
            self.hole_initialized = True

    def peg_pos_callback(self, point: Point):
        if (self.peg_initialized):
            return
        self.peg_position = point
        if (point != None):
            self.peg_initialized = True
            

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

    # def point_reached():
    #     return
    
def main(args=None):
    rclpy.init(args=args)
    node = run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main()