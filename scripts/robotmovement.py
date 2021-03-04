#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Point
from q_learning_project.msg import QMatrix, QMatrixRow, RobotMoveDBToBlock, QLearningReward
import moveit_commander
import keras_ocr
import math
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from operator import itemgetter

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

class RobotMovement:

        def __init__(self):
            #Once everything is initialized will set to true
            self.initialized = False

            #set up Subscriber and Publishers
            #TODO link correct function for Sub
            #self.q_reward = rospy.Subscriber('/q_learning/reward', QLearningReward, self.image_callback)
            self.q_matrix = rospy.Publisher('/q_learning/q_matrix', QMatrix, queue_size=10)
            self.bot_action = rospy.Publisher('/q_learning/robot_action', RobotMoveDBToBlock, queue_size=10)

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

            self.odometry = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
            self.position = None

            #let gazebo start up properly
            #the following two lists hold odometry locations of the blocks and dumbbells, relatively left to right
            block_coords = [(-1.4,-2),(-1.2,-0.002),(-1.4,2)]
            db_coords = [(0.7,0.3),(0.65,0),(0.7,-0.3)]
            rospy.sleep(1)

            self.initialized = True

            #setup dumbbells and blocks
            self.dumbbell_setup()
            self.block_setup()
            flag = 0
            for i in range(3):
                current_db = self.db_order[i][0]
                self.travel(0, 0)
                if(flag == 1):
                    self.turn_around(20)
                if(flag == 2):
                    self.turn_around(110)
                if(flag == 3):
                    self.turn_around(300)
                self.dumbbell_grasp_position() 
                self.arrived = True
                while(self.arrived):
                    self.find_num_db(current_db)
                self.grip_close()
                self.dumbbell_pickup_position()
                if(current_db == 0): #means its red
                    self.travel(block_coords[self.block_lineup.index(3)][0], block_coords[self.block_lineup.index(3)][1])
                    flag = 3
                elif(current_db == 1): #means its green
                    self.travel(block_coords[self.block_lineup.index(1)][0], block_coords[self.block_lineup.index(1)][1])
                    flag = 2
                else: #means its blue
                    self.travel(block_coords[self.block_lineup.index(2)][0], block_coords[self.block_lineup.index(2)][1])
                    flag = 1
                self.dumbbell_grasp_position()


        def image_callback(self, msg):

                self.view = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        def odometry_callback(self, data):
            self.position = data.pose.pose


        #function to figure out order of the blocks
        def block_setup(self):
            #turn to face two leftmost blocks
            self.turn_around(160)
            #identify what those two are using keras_ocr and deduce third block
            self.block_lineup = self.identify_numbers()
            #turn around to face all dbs
            self.turn_around(200)

        def dumbbell_setup(self):

            #obtain colors in image
            image = self.view
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            lower_red = numpy.array([0,50,50])
            upper_red = numpy.array([10,255,255])

            lower_green = numpy.array([50,100,100])
            upper_green = numpy.array([70,255,255])

            lower_blue = numpy.array([110,50,50])
            upper_blue = numpy.array([130,255,255])

            mask_low = [lower_red, lower_green, lower_blue]
            mask_upper = [upper_red, upper_green, upper_blue]

            range_list = []
            #for each general area detected, see which color it matches with and store x value
            #red is index 0, green is index 1, and blue is index 2
            for i in range(3):
                mask = cv2.inRange(hsv, mask_low[i], mask_upper[i])
                h, w, d = image.shape
                search_top = int(h/2)
                search_bot = int(h/2 + 1)
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0
                M = cv2.moments(mask)
                if M['m00'] > 0:
                    # center of the yellow pixels in the image
                    cx = int(M['m10']/M['m00'])
                    range_list.append([i,cx])
            #sort the list of colors based on the x value of the detected blocks. leftmost color will have smallest x
            range_list = sorted(range_list, key=itemgetter(1))
            self.db_order = range_list

        #function for detecting numbers and navigating to the block
        #currently just goes to the first number possible
        def identify_numbers(self):

            #grab current image and use keras_ocr to detect numbers (will see the two leftmost numbers)
            rospy.sleep(1)
            image = self.view
            h, w, d = image.shape
            pipeline = keras_ocr.pipeline.Pipeline()
            prediction_groups = pipeline.recognize([image])

            first = 999
            first_num = 0
            second_num = 0

            #using the x values of the rectangle bounds for the two detected numbers, figure out which number is leftmost and which is center.
            #smaller x value corresponds to leftmost number
            for i in range(2):
                if prediction_groups[0][i][1][0][0] < first:
                    first = prediction_groups[0][i][1][0][0]
                    first_num = int(prediction_groups[0][i][0])
                    if(second_num == 0):
                        second_num = int(prediction_groups[0][i][0])
                else:
                    second = prediction_groups[0][i][1][0][0]
                    second_num = int(prediction_groups[0][i][0])                                     

            #given first two numbers, figure out what the remaining rightmost number must be
            nums = [1,2,3]
            nums.remove(first_num)
            nums.remove(second_num)
            third_num = nums[0]

            #return list of the order of the blocks from left to right
            return([first_num,second_num,third_num]) 


        #helper function for rotating bot. used to survey the numbers on the blocks.
        def turn_around(self, angle):
            relative_angle = (angle * math.pi) / 180
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


        '''
        this function lets bot travel around map using odom node.
        followed this guide: https://www.theconstructsim.com/ros-qa-053-how-to-move-a-robot-to-a-certain-point-using-twist/
        positions obtained using teleoperation
        leftmost dumbbell = (0.7,0.3)
        center dumbbell = (0.65,0)
        rightmost dumbbell = (0.7,-0.3)
        leftmost block = (-1.4,-2)
        center block = (-1.2,-0.002)
        rightmost block = (-1.4,2)
        '''
        def travel(self, x, y):
            while True:
                goal = Point()
                speed = Twist()
                goal.x = x
                goal.y = y
                rot_q = self.position.orientation
                (roll, pitch,  theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
                inc_x = goal.x - self.position.position.x
                inc_y = goal.y - self.position.position.y

                angle_to_goal = math.atan2(inc_y, inc_x)
                distance = pow((pow(inc_x, 2) + pow(inc_y,2)), 0.5)
                if distance < 0.1:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    self.navigator.publish(speed)
                    break
                if abs(angle_to_goal- theta) > 0.3:
                    speed.linear.x = 0.0
                    prop_control = 0.3
                    speed.angular.z = prop_control
                else:
                    speed.linear.x = 0.5
                    speed.angular.z = 0.0

                self.navigator.publish(speed)


        #publish laser scan data to self.directly_ahead node. used in cv dumbbell algorithms to approximate best position for pickup
        def process_scan(self, data):
            self.directly_ahead = data.ranges[0]


        #redirects finding dbs based on the color index list
        def find_num_db(self,num):
            if num == 0:
                self.find_red_db()
            if num == 1:
                self.find_green_db()
            else:
                self.find_blue_db()

        #find red db in field of vision and approach
        def find_red_db(self):
            image = self.view
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_red = numpy.array([0,50,50])
            upper_red = numpy.array([10,255,255])
            mask = cv2.inRange(hsv, lower_red, upper_red)

            self.find_db(mask)

        #find green db in field of vision and approach
        def find_green_db(self):
            image = self.view
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_green = numpy.array([50,100,100])
            upper_green = numpy.array([70,255,255])
            mask = cv2.inRange(hsv, lower_green, upper_green)

            self.find_db(mask)

        #find blue db in field of vision and approach
        def find_blue_db(self):
            image = self.view
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_blue = numpy.array([110,50,50])
            upper_blue = numpy.array([130,255,255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

            self.find_db(mask)


        #function for navigating to specific dumbbell using CV. this is called after the main locator function, which helps the robot navigate closer. this function centers bot on dumbbell
        def find_db(self,mask):
                # # this erases all pixels that aren't yellow
                if(self.directly_ahead <= 0.25):
                    self.arrived = False 
                    twister = Twist()
                    twister.linear.x = 0
                    twister.angular.z = 0
            
                    self.navigator.publish(twister)
                    return
                image = self.view
                h, w, d = image.shape
                search_top = int(h/2)
                search_bot = int(h/2 + 1)
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0

                # # using moments() function, the center of the yellow pixels is determined
                M = cv2.moments(mask)
                # # if there are any yellow pixels found
                if M['m00'] > 0:
                        # center of the yellow pixels in the image
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])

                        # a red circle is visualized in the debugging window to indicate
                        # the center point of the specific color pixels
                        cv2.circle(image, (cx, cy), 20, (255,0,0), -1)
                
                        prop_control = 0.2
                        center = w/2
                        error = (center - cx)
                
                        twister = Twist()
                        twister.linear.x = 0.2
                        twister.angular.z = prop_control * error*3.1415/180
                
                        self.navigator.publish(twister)

        #call this to make robot stoop and open grasp. ready to grip dumbbell -> needs fixing. it doesnt pick it up well...
        def dumbbell_grasp_position(self):
            if(self.initialized):
                arm_joint_goal = [0.0,0.99,-0.942478,-.015] #instead of 0.309 do -.015
                self.move_group_arm.go(arm_joint_goal, wait=True)
                self.move_group_arm.stop()
                gripper_joint_goal = [0.016,0.016]
                self.move_group_gripper.go(gripper_joint_goal, wait=True)
                self.move_group_gripper.stop()


        #call this to make robot close grip
        def grip_close(self):
            if(self.initialized):
                gripper_joint_goal = [0.003,0.003]
                self.move_group_gripper.go(gripper_joint_goal, wait=True)
                self.move_group_gripper.stop()     

        #best config so far for carrying dumbbell
        def dumbbell_pickup_position(self):
                arm_joint_goal = [0.05,0,-0.347,-1.1] #[0.05,0,-0.347,-1.1] #[0.05,0,-0.347,-1.1]
                self.move_group_arm.go(arm_joint_goal, wait=True)
                # Calling ``stop()`` ensures that there is no residual movement
                self.move_group_arm.stop() 

        def run(self):
            rospy.spin()

if __name__ == '__main__':

        rospy.init_node('robot_movement')
        robot_movement = RobotMovement()
        robot_movement.run()
