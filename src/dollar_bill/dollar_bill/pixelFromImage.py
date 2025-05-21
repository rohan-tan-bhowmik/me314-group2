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
        self.pixel_dollar_pub = self.create_publisher(Point, '/image_dollar_center', 10)
        self.pixel_green_pub = self.create_publisher(Point, '/image_green_center', 10)

    def findPixel(self, msg):
        # self.get_logger().info("AAAAAAAAAAAAAAAAAAAAAAAAA")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            

            lower_green = np.array([35, 100, 100])
            upper_green = np.array([85, 255, 255])

            mask_green = cv2.inRange (image_hsv, lower_green, upper_green)

            #find green square
            contours_green, _ = cv2.findContours(mask_green,
                                cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours_green) > 0:
                green_area = max(contours_green, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(green_area)
                cv2.rectangle(cv_image,(x, y),(x+w, y+h),(0, 255, 255), 2)
                pixel = Point()
                pixel.x = float(x + w / 2.0)
                pixel.y = float(y + h / 2.0)

                self.pixel_green_pub.publish(pixel)

                # plt.plot(pixel.x, pixel.y, marker='o', color='magenta', markersize=2)
            
            #find dollar bill
            mask_dollar = cv2.bitwise_not(mask_green)
            img_dollar = cv2.bitwise_and(cv_image, cv_image, mask=mask_dollar)
            gray_dollar = cv2.cvtColor(img_dollar, cv2.COLOR_BGR2GRAY)
            edges_dollar = cv2.Canny(gray_dollar, 50, 150)

            contours_dollar, _ = cv2.findContours(edges_dollar,
                                                cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_dollar) > 0:
                dollar_area = max(contours_dollar, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(dollar_area)
                cv2.rectangle(cv_image,(x, y),(x+w, y+h),(0, 255, 255), 2)
                pixel = Point()
                pixel.x = float(x + w / 2.0)
                pixel.y = float(y + h / 2.0)
                self.pixel_dollar_pub.publish(pixel)

            
                # self.get_logger().info(f"yippee: {centers}m {cv_image.shape[0]}")
                # self.get_logger().info(f"yoohoo: {circles}")
            
            
            # plt.title("Color Camera View")
            # plt.axis("off")
            # plt.pause(0.001)  # Small pause to update the frame
            # plt.clf()
                

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