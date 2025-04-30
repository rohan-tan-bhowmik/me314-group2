#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
from rclpy.duration import Duration
import cv2
import matplotlib.pyplot as plt

# --- HSV bounds for object detection ---
BLOCK_HSV_LOWER = (0, 100, 100)     # Red-like (can adjust)
BLOCK_HSV_UPPER = (10, 255, 255)

TARGET_HSV_LOWER = (50, 100, 100)   # Green-like (can adjust)
TARGET_HSV_UPPER = (90, 255, 255)


class PixelToCoordNode(Node):
    def __init__(self):
        super().__init__("pixel_to_coord")

        # Frames
        self.target_frame = 'link_base'
        self.source_frame = 'camera_color_optical_frame'

        # Topics for camera data
        self.CameraIntrinsicsTopic = "/color/camera_info"
        self.DepthCameraIntrinsicsTopic = "/aligned_depth_to_color/camera_info"
        self.CameraTopic = "/color/image_raw"
        self.DepthCameraTopic = "/aligned_depth_to_color/image_raw"
        self.BlockObjectCenter = "/image_block_center"
        self.TargetObjectCenter = "/image_target_center"

        # CV Bridge Initialization
        self.bridge = CvBridge()

        # TF Listener Initialization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera Subscribers
        self.create_subscription(CameraInfo, self.CameraIntrinsicsTopic, self.GetCameraIntrinsics, 1)
        self.create_subscription(CameraInfo, self.DepthCameraIntrinsicsTopic, self.GetDepthCameraIntrinsics, 1)
        self.create_subscription(Image, self.CameraTopic, self.GetCV2Image, 10)
        self.create_subscription(Image, self.DepthCameraTopic, self.GetDepthCV2Image, 10)
        self.create_subscription(Point, self.BlockObjectCenter, lambda msg: self.getPointInBaseFrame(msg, "block"), 10)
        self.create_subscription(Point, self.TargetObjectCenter, lambda msg: self.getPointInBaseFrame(msg, "target"), 10)

        # Variables for camera data and transforms
        self.cv_Image = None
        self.cv_DepthImage = None
        self.alpha = None
        self.beta = None
        self.u0 = None
        self.v0 = None
        self.alpha_depth = None
        self.beta_depth = None
        self.u0_depth = None
        self.v0_depth = None
        self.baseTransform = None

        # Timer to get TF transform
        self.timer = self.create_timer(1.0, self.lookup_base_transform)

        # Publishers for 3D coordinates
        self.publisher_cube = self.create_publisher(Point, 'cube_position', 10)
        self.publisher_goal = self.create_publisher(Point, 'goal_position', 10)

    def GetTransform(self, target_frame, source_frame, timeout=30.0, debug=False):
        if debug:
            all_frames = self.tf_buffer.all_frames_as_string()
            self.get_logger().info(f"Frames in buffer:\n{all_frames}")

        if self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=10.0)):
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )
                translation = transform.transform.translation
                rotation_quat = transform.transform.rotation

                rotation_matrix = R.from_quat([
                    rotation_quat.x,
                    rotation_quat.y,
                    rotation_quat.z,
                    rotation_quat.w
                ]).as_matrix()

                base_transform = np.eye(4)
                base_transform[:3, :3] = rotation_matrix
                base_transform[:3, 3] = [translation.x, translation.y, translation.z]
                
                return base_transform

            except TransformException as ex:
                self.get_logger().warn(f"Could not transform {source_frame} to {target_frame}: {ex}")
                time.sleep(1)

        raise RuntimeError(f'Timed out waiting for transform from {source_frame} to {target_frame}')
    
    def lookup_base_transform(self):
        try:
            transform_matrix = self.GetTransform(self.target_frame, self.source_frame)
            self.baseTransform = transform_matrix
            self.get_logger().info("Successfully got base transform.")
            self.timer.cancel()
        except RuntimeError as e:
            self.get_logger().warn(str(e))

    def GetCameraIntrinsics(self, CameraMsg):
        self.alpha = CameraMsg.k[0]
        self.beta = CameraMsg.k[4]
        self.u0 = CameraMsg.k[2]
        self.v0 = CameraMsg.k[5]
        self.get_logger().info("Successfully got camera intrinsics.")

    def GetDepthCameraIntrinsics(self, DepthCameraMsg):
        self.alpha_depth = DepthCameraMsg.k[0]
        self.beta_depth = DepthCameraMsg.k[4]
        self.u0_depth = DepthCameraMsg.k[2]
        self.v0_depth = DepthCameraMsg.k[5]
        self.get_logger().info("Successfully got depth camera intrinsics.")

    def GetCV2Image(self, ImageMsg):
        try:
            self.cv_Image = self.bridge.imgmsg_to_cv2(ImageMsg, desired_encoding="passthrough")
            self.get_logger().info("Successfully got CV2 image.")
        except Exception as e:
            self.get_logger().error(f"Failed to convert RGB image: {e}")

    def GetDepthCV2Image(self, DepthImageMsg):
        try:
            self.cv_DepthImage = self.bridge.imgmsg_to_cv2(DepthImageMsg, desired_encoding="passthrough")
            self.get_logger().info("Successfully got depth CV2 image.")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def getPointInBaseFrame(self, point, objectType):
        pixel_x = int(point.x)
        pixel_y = int(point.y)

        self.baseTransform = self.GetTransform(self.target_frame, self.source_frame)
        if not (self.cv_DepthImage is not None and 
                all(v is not None for v in [self.alpha_depth, self.beta_depth, self.u0_depth, self.v0_depth])):
            self.get_logger().warn("Skipping getPointInBaseFrame — data not ready yet.")
            return

        # Get depth value in meters
        Z_c = float(self.cv_DepthImage[pixel_y, pixel_x]) / 1000.0
        X_c = (pixel_x - self.u0_depth) * Z_c / self.alpha_depth
        Y_c = (pixel_y - self.v0_depth) * Z_c / self.beta_depth

        point_camera_frame_homogeneous = np.array([X_c, Y_c, Z_c, 1.0])

        if self.baseTransform is None:
            raise ValueError("Missing base transform")

        point_base_frame = np.dot(self.baseTransform, point_camera_frame_homogeneous)

        coord = Point()
        coord.x = point_base_frame[0]
        coord.y = point_base_frame[1]
        coord.z = point_base_frame[2]

        if objectType == "block":
            self.publisher_cube.publish(coord)
        elif objectType == "target":
            self.publisher_goal.publish(coord)
        else:
            self.get_logger().info("Invalid object type passed to getPointInBaseFrame")

        self.get_logger().info(f"Published {objectType} coordinates: ({coord.x:.3f}, {coord.y:.3f}, {coord.z:.3f})")
        return coord


def main():
    rclpy.init()
    node = PixelToCoordNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
