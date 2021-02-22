#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

                # set up ROS / OpenCV bridge
                self.bridge = cv_bridge.CvBridge()

                # initalize the debugging window
                cv2.namedWindow("window", 1)

                # subscribe to the robot's RGB camera data stream
                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)

                self.navigator = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        def image_callback(self, msg):

                # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                # # TODO: define the upper and lower bounds for what should be considered 'yellow'
                # lower_yellow = numpy.array([30, 64, 127]) #TODO
                # upper_yellow = numpy.array([30, 255, 255]) #TODO
                # mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                #
                # # this erases all pixels that aren't yellow
                # h, w, d = image.shape
                # search_top = int(3*h/4)
                # search_bot = int(3*h/4 + 20)
                # mask[0:search_top, 0:w] = 0
                # mask[search_bot:h, 0:w] = 0

                # using moments() function, the center of the yellow pixels is determined
                M = cv2.moments(mask)
                # if there are any yellow pixels found
                if M['m00'] > 0:
                        # center of the yellow pixels in the image
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])

                        # a red circle is visualized in the debugging window to indicate
                        # the center point of the yellow pixels
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                        # TODO: based on the location of the line (approximated
                        #       by the center of the yellow pixels), implement
                        #       proportional control to have the robot follow
                        #       the yellow line
                        prop_control = 0.3
                        center = w/2
                        error = (center - cx)

                        twister = Twist()
                        twister.linear.x = 0.3
                        twister.angular.z = prop_control * error*3.1415/180
                        print(twister.angular.z)

                        self.navigator.publish(twister)

                # shows the debugging window
                cv2.imshow("window", image)
                cv2.waitKey(3)


        def run(self):
                rospy.spin()

if __name__ == '__main__':

        rospy.init_node('line_follower')
        follower = Follower()
        follower.run()
