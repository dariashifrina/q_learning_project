#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class CatRecognizer(object):

    def __init__(self):

        self.initalized = False

        rospy.init_node('cat_recognizer')

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)

        self.data_stuff = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        # load the opencv2 XML classifier for cat faces
        # obtained from https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalcatface_extended.xml
        self.catface_cascade = cv2.CascadeClassifier('catface_detector.xml') 

        self.seen_first_image = False

        self.initalized = True
        self.navigator = rospy.Publisher('/cmd_vel', Twist, queue_size =10)


    def image_callback(self, data):

        if (not self.initalized):
            return

        if (not self.seen_first_image):

            # we have now seen the first image
            #self.seen_first_image = True

            # take the ROS message with the image and turn it into a format cv2 can use
            img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # turn the image into a grayscale
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # using the XML classifier, we now detect cat faces in the image
            cat_faces = self.catface_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

            for (x,y,w,h) in cat_faces:
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                catx = x
                caty = y
                width = w
            prop_control = 0.3
            center = width/2
            error = (center - catx)
            twister = Twist()
            twister.linear.x = 0.3
            twister.angular.z = prop_control * error * 3.1415/180
            print(twister.angular.z)
            self.navigator.publish(twister)
            # visualize the cat face location in the image
            #cv2.imshow('img',img)
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
    def process_scan(self, data):
        twister = Twist()
        if(data.ranges[0]< 0.2):
            twister.linear.x = 0
        self.navigator.publish(twister)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = CatRecognizer()
    node.run()