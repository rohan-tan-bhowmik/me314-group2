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

class ImageToPixel(Node):
    def __init__(self):
        super().__init__("image_to_pixel")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/realsense2_camera_node/color/image_raw', 
            self.findPixel,
            10
        )
        self.pixel_coin_pub = self.create_publisher(Point, '/image_coin_center', 10)
        self.pixel_green_pub = self.create_publisher(Point, '/image_green_center', 10)

    def findPixel(self, msg):
        # self.get_logger().info("AAAAAAAAAAAAAAAAAAAAAAAAA")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            circles = cv2.HoughCircles(
                image_gray,
                cv2.HOUGH_GRADIENT,
                dp=1.2,
                minDist=5,
                param1=100,   # Canny edge detector threshold
                param2=30,    # Accumulator threshold (smaller -> more false positives)
                minRadius=2,
                maxRadius=100
            )

            centers = []
            if circles is not None:
                for x, y, r in circles[0]:
                    centers.append((x, y, r))
                pt = Point()
                pt.x = float(centers[0][0])
                pt.y = float(centers[0][1])
                self.pixel_coin_pub.publish(pt)


        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageToPixel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()