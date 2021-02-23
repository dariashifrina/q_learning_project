#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from q_learning_project.msg import QMatrix, QMatrixRow, RobotMoveDBToBlock, QLearningReward

class QLearner:

        def __init__(self):
            #Once everything is initialized will set to true
            self.initialized = False

            #set up Subscriber and Publishers
            #TODO link correct function for Sub
            self.q_reward = rospy.Subscriber('/q_learning/reward', QLearningReward, self.image_callback)
            self.q_matrix = rospy.Publisher('/q_learning/q_matrix', QMatrix, queue_size=10)
            self.bot_action = rospy.Publisher('/q_learning/robot_action', RobotMoveDBToBlock, queue_size=10)
            print('testing')

            self.initialize_state_matrix()
            print(self.state_matrix)
            #self.initialize_action_matrix()

            self.initialized = True

                # # set up ROS / OpenCV bridge
                # self.bridge = cv_bridge.CvBridge()
                #
                # # subscribe to the robot's RGB camera data stream
                # self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                #         Image, self.image_callback)
                #
                # self.navigator = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # The state matrix is a dict where the key is the state number and the value is an array
        # contain the red, green, and blue dumbell locations in that order.
        # For reference about locations: 0 = Origin,1 = Block 1, 2 = Block 2, 3 = Block 3
        def initialize_state_matrix(self):
            self.state_matrix = {}

            state_num = 0
            for blue_loc in range(0, 4):
                for green_loc in range(0, 4):
                    for red_loc in range(0, 4):
                        self.state_matrix[state_num] = [red_loc, green_loc, blue_loc]
                        state_num = state_num + 1


        def image_callback(self, msg):

                # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                print('here')

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

                # # using moments() function, the center of the yellow pixels is determined
                # M = cv2.moments(mask)
                # # if there are any yellow pixels found
                # if M['m00'] > 0:
                #         # center of the yellow pixels in the image
                #         cx = int(M['m10']/M['m00'])
                #         cy = int(M['m01']/M['m00'])
                #
                #         # a red circle is visualized in the debugging window to indicate
                #         # the center point of the yellow pixels
                #         cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                #
                #         # TODO: based on the location of the line (approximated
                #         #       by the center of the yellow pixels), implement
                #         #       proportional control to have the robot follow
                #         #       the yellow line
                #         prop_control = 0.3
                #         center = w/2
                #         error = (center - cx)
                #
                #         twister = Twist()
                #         twister.linear.x = 0.3
                #         twister.angular.z = prop_control * error*3.1415/180
                #         print(twister.angular.z)
                #
                #         self.navigator.publish(twister)



        def run(self):
                rospy.spin()

if __name__ == '__main__':

    #TODO Change this init_node
        rospy.init_node('line_QLearner')
        q_learner = QLearner()
        q_learner.run()
