#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from q_learning_project.msg import QMatrix, QMatrixRow, RobotMoveDBToBlock, QLearningReward
import moveit_commander
import keras_ocr
from math import pi


class QLearner:

        def __init__(self):
            #Once everything is initialized will set to true
            self.initialized = False

            #set up Subscriber and Publishers
            #TODO link correct function for Sub
            self.q_reward = rospy.Subscriber('/q_learning/reward', QLearningReward, self.image_callback)
            self.q_matrix = rospy.Publisher('/q_learning/q_matrix', QMatrix, queue_size=10)
            self.bot_action = rospy.Publisher('/q_learning/robot_action', RobotMoveDBToBlock, queue_size=10)

            self.initialize_state_matrix()
            self.initialize_action_matrix()

            print('Initiliazation Complete')

            # # set up ROS / OpenCV bridge
            self.bridge = cv_bridge.CvBridge()
            #self.movement = False
            self.movement = True
            self.block = False
    
            # # subscribe to the robot's RGB camera data stream
            #self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
            #self.navigator = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            # ROS subscribe to the Scan topic to monitor items in the robot's va
            #rospy.Subscriber("/scan", LaserScan, self.process_scan)

            # the interface to the group of joints making up the turtlebot3
            # openmanipulator arm
            self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

            # the interface to the group of joints making up the turtlebot3
            # openmanipulator gripper
            self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
            self.initialized = True
            #self.normal_grasp()
            self.dumbbell_grasp_position()
            self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
            self.navigator = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            rospy.Subscriber("/scan", LaserScan, self.process_scan)
            self.turn = False

        # The state matrix is a dict where the key is the state number and the value is an array
        # contain the red, green, and blue dumbell locations in that order.
        # For reference about locations: 0 = Origin,1 = Block 1, 2 = Block 2, 3 = Block 3

        #call this to make robot do neutral grasp n position
        def normal_grasp(self):
             if(self.initialized):
                print("ready to scoop dumbbell")
                arm_joint_goal = [0.0,0,0,0]

                self.move_group_arm.go(arm_joint_goal, wait=True)
                # Calling ``stop()`` ensures that there is no residual movement
                self.move_group_arm.stop()
                gripper_joint_goal = [0.016,0.016]
                #self.move_group_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
                self.move_group_gripper.go(gripper_joint_goal, wait=True)
                self.move_group_gripper.stop()           

        #call this to make robot stoop and open grasp. ready to grip dumbbell
        def dumbbell_grasp_position(self):
            if(self.initialized):
                print("ready to scoop dumbbell")
                arm_joint_goal = [0.0,0.99,-0.942478,0.309]

                self.move_group_arm.go(arm_joint_goal, wait=True)
                # Calling ``stop()`` ensures that there is no residual movement
                self.move_group_arm.stop()
                gripper_joint_goal = [0.016,0.016]
                #self.move_group_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
                self.move_group_gripper.go(gripper_joint_goal, wait=True)
                self.move_group_gripper.stop()

        #call this to make robot close grasp and lift joints to carry dumbbell
        def dumbbell_pickup_position(self):
            if(self.initialized):
                print("pickup dumbbell")
                gripper_joint_goal = [0.002,0.002]
                self.move_group_gripper.go(gripper_joint_goal, wait=True)
                self.move_group_gripper.stop()      
                arm_joint_goal = [0.05,0,0,-1.2]
                self.move_group_arm.go(arm_joint_goal, wait=True)
                # Calling ``stop()`` ensures that there is no residual movement
                self.move_group_arm.stop() 
                self.turn = True


        def initialize_state_matrix(self):
            self.state_matrix = {}

            # Iterate through the locations of all 3 dumbells in a way that a) creates
            # 64 states, and b) enables each state to correspond to the state matrix given
            # in the project description (ie. state 0 here has all dumbells at the origin)
            state_num = 0
            for blue_loc in range(0, 4):
                for green_loc in range(0, 4):
                    for red_loc in range(0, 4):
                        self.state_matrix[state_num] = [red_loc, green_loc, blue_loc]
                        state_num = state_num + 1

        # Create the action matrix. This leverages the state matrix created in
        # initialize_state_matrix(). Key features are: inability for a state transition to move 2
        # dumbells simultaneously and the prevention of two dumbells at the same Block
        def initialize_action_matrix(self):
            # Create a 64x64 numpy matrix where the default value is -1
            self.action_matrix = numpy.full((64, 64), -1)

            for x in range(0, 64):
                current_state = self.state_matrix[x]
                possible_act = [-1] * 64
                for y in range(0, 64):
                    next_state = self.state_matrix[y]
                    valid = True
                    if x != y:
                        # Check if multiple dumbells have been moved
                        moved_dumbells = 0
                        for i in range(0, 3):
                            if current_state[i] != next_state[i]:
                                # Since dumbells start at the origin (0) and only
                                # ever move once, if the current state is greater than
                                # the next state we know this is not a valid move
                                if current_state[i] > next_state[i]:
                                    valid = False
                                    break
                                # Dumbells can only move from origin to a block (not between blocks)
                                elif current_state[i] > 0 and next_state[i] > 0:
                                    valid = False
                                    break
                                else:
                                    moved_dumbells = moved_dumbells + 1

                            if moved_dumbells > 1:
                                valid = False
                                break

                        # Check if multiple dumbells are at the same block
                        for i in range(1, 4):
                            count = 0
                            for j in range(0, 3):
                                if next_state[j] == i:
                                    count = count + 1
                                if count > 1:
                                    valid = False
                                    break
                            if not valid:
                                break
                    else:
                        valid = False

                    if not valid:
                        possible_act[y] = -1
                    else:
                        # Determine action number to take
                        for i in range(0, 3):
                            if current_state[i] != next_state[i]:
                                # Formula: Since there are 3 actions per color, we can multiple
                                # the color by 3 to get to first action corresponding to that color.
                                # Then, we can add the block number and subtrat 1 to find the right
                                # action number
                                possible_act[y] = (i*3) + next_state[i] - 1
                                break

                self.action_matrix[x] = possible_act

            print(self.action_matrix)

        def image_callback(self, msg):

                self.view = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        def turn_around(self):
            if(self.initialized):
                print("here")
                relative_angle = pi
                angular_speed = 0.2
                twister = Twist()
                twister.angular.z = -abs(angular_speed)
                self.navigator.publish(twister)
                current_angle = 0
                firstTime = rospy.Time.now().to_sec()
                while current_angle < relative_angle:
                    curTime = rospy.Time.now().to_sec() - firstTime
                    current_angle = angular_speed*(curTime)
                twister.angular.z = 0
                self.navigator.publish(twister)
                print("done")
                self.turn = False
                self.block = True
                #self.done = False

        #function for detecting numbers and navigating to the block
        #currently just goes to the first number possible
        def find_block(self):
            while(self.block):
                print("finding block")
                image = self.view
                h, w, d = image.shape

                #cv2.imshow("window", image)
                #cv2.waitKey(3)
                pipeline = keras_ocr.pipeline.Pipeline()
                #rospy.sleep(1)
                prediction_groups = pipeline.recognize([image])
                #print(prediction_groups)
                #print(prediction_groups[0][0][1])
                min_x = 1000
                max_x = 0
                min_y = 1000
                max_y = 0
                for coords in prediction_groups[0][0][1]:
                    if coords[0] < min_x:
                        min_x = coords[0]
                    if coords[0] > max_x:
                        max_x = coords[0]
                    if coords[1] < min_y:
                        min_y = coords[1]
                    if coords[1] > max_y:
                        max_y = coords[1]

                cx = int((min_x + max_x) / 2)
                cy = int((min_y + max_y) / 2)
                cv2.circle(image, (cx, cy), 20, (255,0,0), -1)
                #cv2.imshow("window", image)
                #cv2.waitKey(3)
                prop_control = 0.3
                center = w/2
                error = (center - cx)
        
                twister = Twist()
                twister.linear.x = 0.3
                twister.angular.z = prop_control * error*3.1415/180
        
                self.navigator.publish(twister)


        #function for finding dumbbell and navigating to it
        #currently goes to red dumbbel
        def find_red_db(self):
            if(self.initialized):
                image = self.view
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                # # TODO: define the upper and lower bounds for what should be considered 'yellow'
                lower_red = numpy.array([0,50,50])
                upper_red = numpy.array([10,255,255])
                mask = cv2.inRange(hsv, lower_red, upper_red)

                self.find_db(image, mask)

        def find_green_db(self):
            if(self.initialized):
                image = self.view
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                # # TODO: define the upper and lower bounds for what should be considered 'yellow'
                lower_green = numpy.array([50,100,100])
                upper_green = numpy.array([70,255,255])
                mask = cv2.inRange(hsv, lower_green, upper_green)

                self.find_db(image, mask)

        def find_blue_db(self):
                image = self.view
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                # # TODO: define the upper and lower bounds for what should be considered 'yellow'
                lower_blue = numpy.array([110,50,50])
                upper_blue = numpy.array([130,255,255])
                mask = cv2.inRange(hsv, lower_blue, upper_blue)

                self.find_db(image, mask)

        def find_db(self,image, mask):
                # # this erases all pixels that aren't yellow
                h, w, d = image.shape
                search_top = int(h/2)
                search_bot = int(h/2 + 1)
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0

                # # using moments() function, the center of the yellow pixels is determined
                M = cv2.moments(mask)
                # # if there are any yellow pixels found
                if self.movement:
                    if M['m00'] > 0:
                            # center of the yellow pixels in the image
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])

                            # a red circle is visualized in the debugging window to indicate
                            # the center point of the yellow pixels
                            cv2.circle(image, (cx, cy), 20, (255,0,0), -1)
                    
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
                    
                            self.navigator.publish(twister)
                #self.turn_around()
                #self.done = True
                #cv2.imshow("window", image)
                #cv2.waitKey(3)

        def process_scan(self, data):
            twister = Twist()
            if self.movement:
                self.find_red_db()
                if data.ranges[0] <= 0.28:
                    print("robot slow down")
                    self.movement = False
                    # Go forward if not close enough to wall.
                    twister.linear.x = 0
                    self.navigator.publish(twister)
                    #self.dumbbell_grasp_position()

                    self.dumbbell_pickup_position()      
            if self.block:
                self.find_block()
                '''if data.ranges[0] <= 0.28:
                    print("robot slow down")
                    self.block = False
                    # Go forward if not close enough to wall.
                    twister.linear.x = 0
                    self.navigator.publish(twister)
                    self.dumbbell_grasp_position()'''
            if(self.turn):
                self.turn_around()


                    #self.dumbbell_pickup_position()     

        def run(self):
                rospy.spin()

if __name__ == '__main__':

    #TODO Change this init_node
        rospy.init_node('line_QLearner')
        q_learner = QLearner()
        q_learner.run()
