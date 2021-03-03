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

            #JOSHUA'S STUFF UNCOMMENT LATER
            '''self.initialize_state_matrix()
            #self.initialize_action_matrix()

            print('Initiliazation Complete')'''

            # # set up ROS / OpenCV bridge
            self.bridge = cv_bridge.CvBridge()

            # the interface to the group of joints making up the turtlebot3
            # openmanipulator arm
            self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

            # the interface to the group of joints making up the turtlebot3
            # openmanipulator gripper
            self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

            self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
            self.navigator = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            rospy.Subscriber("/scan", LaserScan, self.process_scan)

            #initiliazed set to true once all subscribers and publishers established
            self.movement = False
            self.leggoblock = False
            self.initialized = True

            #let gazebo start up properly
            rospy.sleep(1)


            #finding out orientation of blocks
            self.blocks = []    
            self.block_setup()
            print(self.blocks)

            self.dumbbell_grasp_position()

            self.movement = True

            '''self.dumbbell_grasp_position()
            self.movement = False
            self.block = False
            self.turn = False
            self.setup = True'''
            #self.dumbbell_grasp_position()
            #self.movement=True
            #self.block_setup()

        # The state matrix is a dict where the key is the state number and the value is an array
        # contain the red, green, and blue dumbell locations in that order.
        # For reference about locations: 0 = Origin,1 = Block 1, 2 = Block 2, 3 = Block 3

        #function to set up locations of the blocks
        def block_setup(self):
            #self.setup=False

            print("turn")
            self.turn_around(160)
            self.blocks = self.find_block2()
            self.turn_around(200)
            #self.dumbbell_grasp_position()
            #self.movement = True



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
                self.move_group_gripper.go(gripper_joint_goal, wait=True)
                self.move_group_gripper.stop()

        #call this to make robot close grasp and lift joints to carry dumbbell
        def dumbbell_pickup_position(self):
            if(self.initialized):
                print("pickup dumbbell")
                gripper_joint_goal = [0.002,0.002]
                self.move_group_gripper.go(gripper_joint_goal, wait=True)
                self.move_group_gripper.stop()     
                rospy.sleep(1) 
                arm_joint_goal = [0.05,0,0,-1.2]
                self.move_group_arm.go(arm_joint_goal, wait=True)
                # Calling ``stop()`` ensures that there is no residual movement
                self.move_group_arm.stop() 
                self.turn_around(180)
                rospy.sleep(1)
                self.leggoblock = True
                #self.turn = True

        def image_callback(self, msg):

                self.view = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        def turn_around(self, angle):
            #if(self.initialized):
            print("turning")
            print(angle)
            relative_angle = (angle * pi) / 180
            print(relative_angle)
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
                #self.turn = False
                #self.block = True
                #self.done = False


        #function for detecting numbers and navigating to the block
        #currently just goes to the first number possible
        def find_block2(self):
            print("finding block")
            rospy.sleep(1)
            image = self.view
            h, w, d = image.shape

            #cv2.imshow("window", image)
            #cv2.waitKey(3)
            pipeline = keras_ocr.pipeline.Pipeline()
            #rospy.sleep(1)
            prediction_groups = pipeline.recognize([image])
            print(prediction_groups)
            print(prediction_groups[0][0][1][0][0])
            print(prediction_groups[0][1][1][0][0])
            first = 999
            first_num = 0
            second_num = 0
            for i in range(2):
                if prediction_groups[0][i][1][0][0] < first:
                    first = prediction_groups[0][i][1][0][0]
                    first_num = int(prediction_groups[0][i][0])
                    if(second_num == 0):
                        second_num = int(prediction_groups[0][i][0])
                else:
                    second = prediction_groups[0][i][1][0][0]
                    second_num = int(prediction_groups[0][i][0])                                     
            print(first_num)
            print(second_num)
            nums = [1,2,3]
            nums.remove(first_num)
            nums.remove(second_num)
            third_num = nums[0]
            print(third_num)   
            return([first_num,second_num,third_num]) 

        #function for finding dumbbell and navigating to it
        #currently goes to red dumbbel
        def find_red_db(self):
            if(self.initialized):
                print("red dumbell")
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
                    print("lookin")
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
                            twister.linear.x = 0.2
                            twister.angular.z = prop_control * error*3.1415/180
                    
                            self.navigator.publish(twister)
                #self.turn_around(180)
                self.done = True
        def process_scan(self, data):
            twister = Twist()
            rospy.sleep(1)

            if self.movement:
                print("processing")
                self.find_blue_db()
                if data.ranges[0] <= 0.27:
                    print("robot slow down")
                    self.movement = False
                    # Go forward if not close enough to wall.
                    twister.linear.x = 0
                    self.navigator.publish(twister)
                    #self.dumbbell_grasp_position()

                    self.dumbbell_pickup_position()   
            if self.leggoblock:
                print("navigating to block")
                //self.find_block()
                if data.ranges[0] <= 0.4:
                    print("robot slow down")
                    self.leggoblock = False
                    # Go forward if not close enough to wall.
                    twister.linear.x = 0
                    self.navigator.publish(twister)
                    #self.dumbbell_grasp_position()
            '''   
            #if self.block:
            #   self.find_block()
                if data.ranges[0] <= 0.28:
                    print("robot slow down")
                    self.block = False
                    # Go forward if not close enough to wall.
                    twister.linear.x = 0
                    self.navigator.publish(twister)
                    self.dumbbell_grasp_position()
            if(self.turn):
                self.find_block()
                    #self.dumbbell_pickup_position()
            '''

        def run(self):
            '''rospy.sleep(1)
            if(self.setup):
                self.block_setup()'''
            rospy.spin()

if __name__ == '__main__':

    #TODO Change this init_node
        rospy.init_node('line_QLearner')
        q_learner = QLearner()
        q_learner.run()
