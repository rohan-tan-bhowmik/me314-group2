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
from std_msgs.msg import Float32MultiArray
import time
from rclpy.duration import Duration
#from builtin_interfaces.msg import Time

class PixelToCoordNode(Node):
    def __init__(self):
        super().__init__("pixel_to_coord")

        #Frames
        self.target_frame = 'link_base'
        self.source_frame = 'camera_link'
        
        # Topics for camera data
        self.CameraIntrinsicsTopic = "/color/camera_info"
        self.DepthCameraIntrinsicsTopic = "/aligned_depth_to_color/camera_info"
        self.CameraTopic = "/gazebo/default/UF_ROBOT/link7/cameracolor/image"
        self.DepthCameraTopic = "/gazebo/default/UF_ROBOT/link7/cameradepth/image"

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

        #use timer to find base transform after init occurs
        self.timer = self.create_timer(1.0, self.lookup_base_transform)

        #publish object locaftion
        self.publisher_ = self.create_publisher(Float32MultiArray, 'object_position', 10)

    def GetTransform(self, target_frame, source_frame, timeout=30.0, debug=False):
        if debug:
            all_frames = self.tf_buffer.all_frames_as_string()
            self.get_logger().info(f"Frames in buffer:\n{all_frames}")

        #check if transform is possible first with timeout    
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
            self.timer.cancel()  # Stop the timer once we succeed
        except RuntimeError as e:
            self.get_logger().warn(str(e))

    def GetCameraIntrinsics(self, CameraMsg):
        # Extract RGB camera intrinsics from CameraInfo message
        self.alpha = CameraMsg.k[0]
        self.beta = CameraMsg.k[4]
        self.u0 = CameraMsg.k[2]
        self.v0 = CameraMsg.k[5]

    def GetDepthCameraIntrinsics(self, DepthCameraMsg):
        # Extract depth camera intrinsics from CameraInfo message
        self.alpha_depth = DepthCameraMsg.k[0]
        self.beta_depth = DepthCameraMsg.k[4]
        self.u0_depth = DepthCameraMsg.k[2]
        self.v0_depth = DepthCameraMsg.k[5]

    def GetCV2Image(self, ImageMsg):
        # Convert ROS Image message to OpenCV format for RGB image
        try:
            self.cv_Image = self.bridge.imgmsg_to_cv2(ImageMsg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Failed to convert RGB image: {e}")

    def GetDepthCV2Image(self, DepthImageMsg):
        # Convert ROS Image message to OpenCV format for depth image
        try:
            self.cv_DepthImage = self.bridge.imgmsg_to_cv2(DepthImageMsg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def getPointInBaseFrame(self, pixel_x, pixel_y):
        self.baseTransform = self.GetTransform(self.target_frame, self.source_frame)
        if not (self.cv_DepthImage is not None and 
                all(v is not None for v in [self.alpha_depth, self.beta_depth, 
                                            self.u0_depth, self.v0_depth])):
            raise ValueError("Missing depth image or intrinsics")

        # Compute camera coordinates from pixel coordinates and depth image
        Z_c = float(self.cv_DepthImage[int(pixel_y), int(pixel_x)]) / 1000.0  # Convert mm to meters
        X_c = (pixel_x - self.u0_depth) * Z_c / self.alpha_depth
        Y_c = (pixel_y - self.v0_depth) * Z_c / self.beta_depth

        point_camera_frame_homogeneous = np.array([X_c, Y_c, Z_c, 1.0])

        if not (self.baseTransform is not None):
            raise ValueError("Missing base transform")

        point_base_frame = np.dot(self.baseTransform, point_camera_frame_homogeneous)
        
        #publish xyz coords
        msg = Float32MultiArray()
        msg.data = point_base_frame[:3]
        self.publisher_.publish(msg)

        return msg  # Return only the x, y, z coordinates

def main():
    rclpy.init()
    node = PixelToCoordNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()