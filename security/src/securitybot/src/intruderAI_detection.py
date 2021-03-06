#!/usr/bin/env python
#Citation: https://www.pyimagesearch.com/2017/09/18/real-time-object-detection-with-deep-learning-and-opencv/

import numpy as np
import imutils
#rospy for the subscriber
import rospy
import cv2
# ROS Image and boolean message
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
#from imutils.video import VideoStream

#get Caffe model definition (architecture of the model that was used during training)
protext = "/home/ee106/ros_ws/security/src/securitybot/src/MobileNetSSD_deploy.prototxt.txt"
#get trained Caffe model 
model = "/home/ee106/ros_ws/security/src/securitybot/src/MobileNetSSD_deploy.caffemodel"

#bounding box color (blue)
box_color = [ 175.36398378,   16.50476277,    2.24319739]

path = "/home/ee106/ros_ws/security/src/securitybot/src/MobileNetSSD_deploy.caffemodel/images"
# load our serialized model from disk
print("loading model...")
net = cv2.dnn.readNetFromCaffe(protext, model)

# initialize the video stream and allow the cammera sensor to warmup
#print("starting video stream...")
#video_stream = VideoStream(src=0).start()
#time.sleep(2.0)

class intruderAI:
    def __init__(self):
        #retrieve a BGR image from the kinect
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('surveillance_system' , Image , self.detect_intruder_callback)
        self.image_pub_boolean = rospy.Publisher('intruder_found_bool', Bool)
        self.image_pub_img = rospy.Publisher('intruder_found_img', Image)

    def ros_to_cv_img(self, ros_img_msg):
        # return np.array(self.bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))
        return self.bridge.imgmsg_to_cv2(ros_img_msg, 'bgr8')

    def cv_img_to_ros(self, cv_img):
        return self.bridge.cv2_to_imgmsg(cv_img, "bgr8")

    #Our call back is being passed in image data that the kinect is capturing
    def detect_intruder_callback(self,img_data):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            ros_to_cvimg = self.ros_to_cv_img(img_data)
            # load the current frame(img_data) and grab its dimension and convert it to a blob
            frame = imutils.resize(ros_to_cvimg, width=400)
 
            #Grab the frame dimensions and convert it to a blob
            #Each frame/img dimension consist of (length, width, color-channel)
            h = frame.shape[0]
            w = frame.shape[1]

            #perform mean subtraction(normalize current frame) which results in a known blob shape
            blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

            # pass the blob through the network to obtain the detections and predictions after the forward pass
            net.setInput(blob)
            detections = net.forward()

            # loop over the detections/predictions
            for i in np.arange(0, detections.shape[2]):
                # extract the confidence (i.e., probability) associated with the prediction
                confidence = detections[0, 0, i, 2]
        
                # extract the index of the class label from the detections/predictions
                # Compute the (x, y)-coordinates of the bounding box for the object
                idx = int(detections[0, 0, i, 1])

                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])

                #minX, minY, maxX, maxY correspond to the points of each corner of the current bounding box of the image
                (minX, minY, maxX, maxY) = box.astype("int")
                #index 15 corresponds to the label of "intruder"
                if idx == 15:
                    #Let the homeowner know that an intruder is present
                    self.image_pub_boolean.publish(True)

                    #Creating bounding box over intruder. Preparing new image that will be attached to the email
                    label = "INTRUDER"
                    prediction_accuracy = confidence * 100
                    label_info = "{}: {:.2f}%".format(label, prediction_accuracy)
                    
                    cv2.rectangle(img = frame, pt1 = (minX, minY), pt2 = (maxX, maxY), color = box_color, thickness =2)
                    #Pad where the label will show up in the box, in this case 15 cm below the upper left border
                    minY = minY + 15
            
                    cv2.putText(frame, text = label_info, org = (minX, minY),fontFace = cv2.FONT_HERSHEY_SIMPLEX, 
                        fontScale = 0.5, color = box_color, thickness = 2)


                    self.image_pub_img.publih(frame)

                    r.sleep()

                #cv2.imshow("frame", frame)
                #r.sleep()


            key = cv2.waitKey(1) & 0xFF


#Python syntax for main method
if __name__ == '__main__':
    rospy.init_node('surveillance_system', anonymous=True)
    #When a intruderAI object instance is initialized it will automatically run the subsriber and publisher
    intruderAI = intruderAI()
    try:
    #Wait for messages to arrive on the subscribed topics, and exit the node when it is killed with Ctrl+C
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    #Destroy all windows that are connected to a camera.
    cv2.destroyAllWindows()


