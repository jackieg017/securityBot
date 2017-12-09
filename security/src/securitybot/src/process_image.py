#!/usr/bin/env python
# Using this CvBridge Tutorial for converting ROS images to OpenCV2 images
# Citation: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rospy
import cv2
import sys
import numpy as np
# ROS Image message
from std_msgs.msg import String
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        #retrieve a BGR image from the kinect
        self.image_sub = rospy.Subscriber('camera/rgb/image_color' , Image , self.process_img_callback)
        self.image_pub = rospy.Publisher('surveillance_system', Image)

    # Converts a ROS Image message to openCV images
    def ros_to_np_img(self, ros_img_msg):
        return self.bridge.imgmsg_to_cv2(ros_img_msg,'bgr8')

    def process_img_callback(self,data):
        #a 10Hz publishing rate
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                cv_image = ros_to_openCV_img(data)
                #Publish our image to the 'surveillance_system' topic
                self.image_pub.publish(cv_image)
                r.sleep()
            except CvBridgeError as exception:
                print(exception)


if __name__ == '__main__':
    rospy.init_node('process_image')
    #When a image_converter object instance is initialized it will automatically run the subsriber and publisher
    img_conv = image_converter()
    try:
        #Wait for messages to arrive on the subscribed topics, and exit the node when it is killed with Ctrl+C
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    #Destroy all windows that are connected to a camera.
    cv2.destroyAllWindows()

