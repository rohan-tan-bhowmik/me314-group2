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
        self.pixel_red_pub = self.create_publisher(Point, '/image_red_center', 10)
        self.pixel_green_pub = self.create_publisher(Point, '/image_green_center', 10)
        self.pixel_hole_pub = self.create_publisher(Point, '/image_hole_center', 10)
        self.pixel_peg_pub = self.create_publisher(Point, '/image_peg_center', 10)
        self.task = "peg" #"block" 

    def findPixel(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2("test_blue.jpg", desired_encoding='bgr8')
                #msg, desired_encoding='bgr8')
            image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            if self.task == "block" or self.task == "peg":     
                lower_red = np.array([0, 130, 130])
                upper_red = np.array([8, 255, 255])
                lower_red2 = np.array([160, 130, 130])
                upper_red2 = np.array([179, 255, 255])
                
                mask_red = cv2.inRange (image_hsv, lower_red, upper_red) | cv2.inRange(image_hsv, lower_red2, upper_red2)
                
                #mask1 = cv2.inRange(hsv_img, lower_red, upper_red)
                #mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
                #mask = cv2.bitwise_or(mask1, mask2)
                #masked_img = cv2.bitwise_and(cv_image, cv_image, mask=mask)
                contours_red, _ = cv2.findContours(mask_red,
                                cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)

                # plt.imshow(cv_image)

                if len(contours_red) > 0:

                    red_area = max(contours_red, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(red_area)
                    cv2.rectangle(cv_image,(x, y),(x+w, y+h),(0, 0, 255), 2)
                    pixel = Point()
                    pixel.x = float(x + w / 2.0)
                    pixel.y = float(y + h / 2.0)
                    self.pixel_red_pub.publish(pixel)


                # plt.plot(pixel.x, pixel.y, marker='o', color='yellow', markersize=2)
                if self.task == "block":
                    lower_green = np.array([36, 130, 130])
                    upper_green = np.array([86, 255, 255])
                    mask_green = cv2.inRange (image_hsv, lower_green, upper_green)
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
                    
                elif self.task == "peg":
                    lower_blue = np.array([100, 150, 50])
                    upper_blue = np.array([130, 255, 255])
                    blue_mask = cv2.inRange(image_hsv, lower_blue, upper_blue)
                    contours_blue = cv2.bitwise_and(cv_image, cv_image, mask=blue_mask)
                    gray = cv2.cvtColor(contours_blue, cv2.COLOR_BG2GRAY)
                    gray_blur = cv2.medianBlur(gray, 5)
                    circles = cv2.HoughCircles(
                        gray_blur,
                        cv2.HOUGH_GRADIENT,
                        dp=1.2,
                        minDist=cv_image.shape[0]
                    )
                    #self.get_logger().info(circles)
                    centers = []
                    if circles is not None:
                        for x, y, r in circles[0]:
                            centers.append((x, y, r))
                        self.pixel_hole_pub.publish(Point(centers[0][0], centers[0][1]))
                
                
                
                
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