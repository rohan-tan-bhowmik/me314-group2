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
            '/color/image_raw', 
            self.findPixel,
            10
        )
        self.pixel_pub = self.create_publisher(Point, '/image_with_box', 10)

    def findPixel(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([179, 255, 255])
            mask = cv2.inRange (image_hsv, lower_red, upper_red) | cv2.inRange(image_hsv, lower_red2, upper_red2)
            #mask1 = cv2.inRange(hsv_img, lower_red, upper_red)
            #mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
            #mask = cv2.bitwise_or(mask1, mask2)
            masked_img = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            contours, _ = cv2.findContours(mask,
                           cv2.RETR_TREE,
                           cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                red_area = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(red_area)
                cv2.rectangle(cv_image,(x, y),(x+w, y+h),(0, 0, 255), 2)
                pixel = Point()
                pixel.x = float(x + w / 2.0)
                pixel.y = float(y + h / 2.0)
            #boxed_image_msg = self.bridge.cv2_to_imgmsg(msg, encoding='bgr8')
                self.pixel_pub.publish(pixel)

                plt.imshow(cv_image)
                plt.plot(pixel.x, pixel.y, marker='o', color='yellow', markersize=2)
                plt.title("Color Camera View")
                plt.axis("off")
                plt.pause(0.001)  # Small pause to update the frame
                plt.clf()
            
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageToPixel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()