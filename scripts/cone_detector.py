#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        if self.LineFollower == False:
            box = cd_color_segmentation(image, None)
        else:
            x_min = 0.8 #if desired_distance is 0.5m
            x_max = 1.0
            mask_min = self.xToV(x_max)
            mask_max = self.xToV(x_min)

            x = 0
            y = mask_min
            h = mask_max - mask_min
            w = image.shape[:2][1]

            mask = np.zeros(image.shape[:2], np.uint8)
            mask[y:y+h,x:x+w] = 255

            line_img = cv2.bitwise_and(image,image,mask = mask)

            box = cd_color_segmentation(line_image,None)

        v = box[1][1]
        u = (box[1][0]+box[0][0])/2

        self.cone_pub.publish(u,v)

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

    def xToV(self,x):
        v = 254.6*x^4 -1075.8*x^3 + 1727*x^2 - 1309.8*x +649.5
        return(int(round(v)))



if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
