#!/usr/bin/env python
# Using this CvBridge Tutorial for converting ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

 #rospy for the subscriber
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
    # Converts a ROS Image message to a NumPy array to be displayed by OpenCV
    def ros_to_np_img(self, ros_img_msg):
        return np.array(self.bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))

    def process_img_callback(self,data):
        #a 10Hz publishing rate
        # r = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                #previously self.bridge.imgmsg_to_cv2(data, "bgr8")
                # self.current_image = self.ros_to_np_img(data)
                self.current_image = data

                # previously commented out (Dec. 1st --date of change)
                # cv2.imshow("CV Image", self.current_image)
                #cv2.waitKey(3)
            except CvBridgeError as exception:
                print(exception)
     
            try:
                #Publish our image to the 'intruder_detection' topic
                self.image_pub.publish(self.current_image)

                #Does our object decttion algorihtm expect the image to be a openCV image or 2d np array?
                #self.image_pub.publish(ros_to_np_img(self.bridge.cv2_to_imgmsg(cv_image, "bgr8")))
            except CvBridgeError as exception:
                print(exception)

            #Use our rate object to sleep until it is time to publish again
            #r.sleep()
    def __init__(self):
        self.bridge = CvBridge()
        self.current_image = None;
        #retrieve a BGR image from the kinect
        self.image_sub = rospy.Subscriber('camera/rgb/image_color' , Image , self.process_img_callback)
        self.image_pub = rospy.Publisher('intruder_detection', Image)

#Python syntax for main method
if __name__ == '__main__':
    #Run this program as a new node in the ROS computation graph called 'image_converter'
    #anonymous is set to true when we aare going to run the node multiple times..
    #rospy.init_node('image_converter', anonymous=True)
    rospy.init_node('img_converter')
    #When a image_converter object instance is initialized it will automatically run the subsriber and publisher
    img_conv = image_converter()
    try:
        #Wait for messages to arrive on the subscribed topics, and exit the node when it is killed with Ctrl+C
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    #Destroy all windows that are connected to a camera.
    # cv2.destroyAllWindows()



