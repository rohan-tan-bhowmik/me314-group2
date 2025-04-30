#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float64
from me314_msgs.msg import CommandQueue, CommandWrapper
import time
import numpy as np
from torque import FTMonitor  # <-- import your force/torque class

class Run(Node):
    def __init__(self, ft_monitor):
        super().__init__('peg_in_hole_runner')
        self.stage = 0
        self.cube_position = None
        self.ft_monitor = ft_monitor  # Pass reference to FT monitor

        self.cube_sub = self.create_subscription(Point, '/cube_position', self.cube_pos_callback, 10)
        self.command_queue_pub = self.create_publisher(CommandQueue, '/me314_xarm_command_queue', 10)

        self.create_timer(0.5, self.stage_machine)

    def cube_pos_callback(self, point: Point):
        self.cube_position = point

    def publish_pose(self, point: Point):
        queue_msg = CommandQueue()
        queue_msg.header.stamp = self.get_clock().now().to_msg()

        wrapper = CommandWrapper()
        wrapper.command_type = "pose"
        wrapper.pose_command.x = point.x
        wrapper.pose_command.y = point.y
        wrapper.pose_command.z = point.z
        wrapper.pose_command.qx = 1.0
        wrapper.pose_command.qy = 0.0
        wrapper.pose_command.qz = 0.0
        wrapper.pose_command.qw = 0.0

        queue_msg.commands.append(wrapper)
        self.command_queue_pub.publish(queue_msg)
        self.get_logger().info(f"Published Pose to: {point}")

    def stage_machine(self):
        # TEMPORARY
        if self.stage == 0 and self.cube_position:
            self.get_logger().info("Stage 0: Move to block")
            self.publish_pose(self.cube_position)
            self.stage += 1

        elif self.stage == 1:
            force = np.linalg.norm(self.ft_monitor.get_force_vector())
            if force > 5.0:
                self.get_logger().info("Stage 1: Detected contact force, proceeding")
                self.stage += 1

        elif self.stage == 2:
            self.get_logger().info("Stage 2: Lowering slowly to find hole")
            # Example insertion logic: small z steps
            pose = self.cube_position
            pose.z -= 0.005
            self.publish_pose(pose)

            torque = np.linalg.norm(self.ft_monitor.get_torque_vector())
            if torque < 0.5:
                self.get_logger().info("Stage 2: Aligned, inserted!")
                self.stage += 1


def main(args=None):
    rclpy.init(args=args)

    ft_monitor = FTMonitor()
    peg_runner = Run(ft_monitor)

    # Add both nodes to the executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ft_monitor)
    executor.add_node(peg_runner)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down.")
    finally:
        ft_monitor.destroy_node()
        peg_runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
