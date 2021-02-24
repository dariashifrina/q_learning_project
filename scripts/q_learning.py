#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy, random
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from q_learning_project.msg import QMatrix, QMatrixRow, RobotMoveDBToBlock, QLearningReward

class QLearner:

        def __init__(self):
            #Once everything is initialized will set to true
            self.initialized = False

            #set up Subscriber and Publishers
            #TODO link correct function for Sub
            self.q_reward = rospy.Subscriber('/q_learning/reward', QLearningReward, self.q_algorithm)
            self.q_matrix_pub = rospy.Publisher('/q_learning/q_matrix', QMatrix, queue_size=10)
            self.bot_action = rospy.Publisher('/q_learning/robot_action', RobotMoveDBToBlock, queue_size=10)

            self.initialize_state_matrix()
            self.initialize_action_matrix()

            # Begin running the Q learning algorithm
            self.alpha = 1
            self.gamma = 1
            self.t = 0
            # Always begin with all blocks at the origin
            self.state = 0
            self.initialize_q_matrix()
            # TODO remove this (goes in the sub)
            self.q_algorithm(self.state)

            self.initialized = True
            print('Initiliazation Complete')

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


        # This creates the QMatrix
        # TODO publish initial q matrix
        def initialize_q_matrix(self):
            header = Header()
            # TODO https://answers.ros.org/question/60209/what-is-the-proper-way-to-create-a-header-with-python/
            header.stamp = rospy.Time.now()
            header.frame_id = "q_matrix"

            temp = QMatrixRow()
            temp.q_matrix_row = [0] * 9
            # 64 different states = 64 rows, with 9 possible actions each (columns)
            q_matrix = [temp] * 64

        # Queries the action matrix to determine the valid actions from the given state.
        # Output is an array of valid actions
        def valid_actions(self, state):
            all_actions = self.action_matrix[state]
            valid = []
            for x in all_actions:
                if x != -1:
                    valid.append(x)
            return valid

        # This function runs the actual algorithm
        def q_algorithm(self, state):
            valid = self.valid_actions(state)

            # Select a random actions
            action_num = valid[random.randint(0, len(valid) - 1)]
            action = RobotMoveDBToBlock()
            block_num = (action_num+1)%3
            if block_num == 0:
                block_num = 3
            action.block_id = block_num
            color = ''
            if action_num >= 0 and action_num <=2:
                color = 'red'
            elif action_num >= 3 and action_num <= 5:
                color = 'green'
            else:
                color = 'blue'
            action.robot_db = color

            # Perform the chosen action
            self.bot_action.publish(action)

        def run(self):
                rospy.spin()

if __name__ == '__main__':
        rospy.init_node('q_learning')
        q_learner = QLearner()
        q_learner.run()
