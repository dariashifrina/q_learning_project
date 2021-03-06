#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy, random, copy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from q_learning_project.msg import QMatrix, QMatrixRow, RobotMoveDBToBlock, QLearningReward

class QLearner:

        def __init__(self):
            #Once everything is initialized will set to true
            self.initialized = False
            self.exit_algo = False

            # Used to throw out bogus rewards. Believe this is a race condition
            # (see writeup for explanation, someone else had this problem on Slack too)
            self.last_msg_time = 0

            #set up Subscriber and Publishers
            self.q_reward = rospy.Subscriber('/q_learning/reward', QLearningReward, self.q_algorithm_pt2)
            self.q_matrix_pub = rospy.Publisher('/q_learning/q_matrix', QMatrix, queue_size=10)
            self.bot_action = rospy.Publisher('/q_learning/robot_action', RobotMoveDBToBlock, queue_size=10)
            rospy.sleep(1)
            self.initialize_state_matrix()
            self.initialize_action_matrix()

            # Begin running the Q learning algorithm
            self.alpha = 1
            self.gamma = 1
            self.t = 0
            self.converge_threshold = 20
            self.converge_count = 0
            # Always begin with all blocks at the origin
            self.actual_state = 0
            self.initialize_q_matrix()

            self.initialized = True
            print('Initiliazation Complete')

            # Begin executing the algorithm
            # self.robot_command(copy.deepcopy(self.actual_q_matrix), copy.deepcopy(self.state_matrix))
            self.q_algorithm_pt1()



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
        def initialize_q_matrix(self):
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "q_matrix"
            temp = QMatrixRow()
            temp.q_matrix_row = [0] * 9

            self.actual_q_matrix = QMatrix()
            self.actual_q_matrix.header = h
            # 64 different states = 64 rows, with 9 possible actions each (columns)
            self.actual_q_matrix.q_matrix = [temp] * 64

            # This variable is used to determine whether the matrix has converged
            self.prev_q_matrix = copy.deepcopy(self.actual_q_matrix)

            # Publish the initial, empty q_matrix to the topic
            self.q_matrix_pub.publish(self.actual_q_matrix)

        # Queries the action matrix to determine the valid actions from the given state.
        # Output is an array of valid actions
        def valid_actions(self):
            state = self.actual_state
            all_actions = self.action_matrix[state]
            valid = []
            for x in all_actions:
                if x != -1:
                    valid.append(x)
            return valid

        # Given the action performed, this determines the next state
        # and updates self.actual_state accordingly. This is important for performing the
        # next iteration of the Q-learning q_algorithm
        # This determines the next state by simply iterating through the state matrix until
         # it finds the right next state
        def determine_next_state(self):
            next_state = copy.deepcopy(self.state_matrix[self.actual_state])

            block_num = (self.action_num+1)%3
            if block_num == 0:
                block_num = 3
            # Red dumbell was moved
            if self.action_num/3 < 1:
                next_state[0] = block_num
            # Green dumbell was moved
            elif self.action_num/3 < 2:
                next_state[1] = block_num
            # Blue dumbell was moved
            else:
                next_state[2] = block_num

            # Iterate through all 64 states in state matrix until you find the matching
            # next state
            for x in range(0, 63):
                test_state = self.state_matrix[x]
                for y in range(0, 3):
                    if next_state[y] != test_state[y]:
                        break
                    # Will only get here is the next_state and test_state match exactly
                    # (aka found the right state)
                    if y == 2:
                        return x


        # After Q_matrix has converged, this command uses the matrix to determine
        # the order in which the robot should move
        def robot_command(self, matrix, state_array):
            current_state = 0
            number = -1
            final_order = []
            taken_action = []
            for x in range(0,3):
                # Determine the high value in the q matrix row
                max_value = -10000
                for y in range(0,9):
                    if matrix.q_matrix[current_state].q_matrix_row[y] > max_value:
                        max_value = matrix.q_matrix[current_state].q_matrix_row[y]
                # Collect actions that have this max value. Important in case there
                # are multiple actions that fit the criteria
                possible_actions = []
                count = 0
                for y in range(0,9):
                    if matrix.q_matrix[current_state].q_matrix_row[y] == max_value:
                        possible_actions.append(count)
                    count = count + 1
                # Choose the action to be taken, making sure that action has not been
                 # chosen before
                repeat = True
                while repeat:
                    number = numpy.random.choice(possible_actions)
                    passed = True
                    for y in taken_action:
                        if y == number:
                            passed = False
                            break
                    if passed:
                        repeat = False
                action = RobotMoveDBToBlock()
                block_num = (number+1)%3
                if block_num == 0:
                    block_num = 3
                action.block_id = block_num
                color = ''
                if number >= 0 and number <=2:
                    color = 'red'
                elif number >= 3 and number <= 5:
                    color = 'green'
                else:
                    color = 'blue'
                action.robot_db = color
                final_order.append(action)
                taken_action.append(number)
                # Determine the next state to look at
                next_state = state_array[current_state]

                block_num = (number+1)%3
                if block_num == 0:
                    block_num = 3
                # Red dumbell was moved
                if number/3 < 1:
                    next_state[0] = block_num
                # Green dumbell was moved
                elif number/3 < 2:
                    next_state[1] = block_num
                # Blue dumbell was moved
                else:
                    next_state[2] = block_num

                # Iterate through all 64 states in state matrix until you find the matching
                # next state
                for i in range(0, 63):
                    test_state = state_array[i]
                    for y in range(0, 3):
                        if next_state[y] != test_state[y]:
                            break
                        # Will only get here is the next_state and test_state match exactly
                        # (aka found the right state)
                        if y == 2:
                            current_state = i
            print(final_order)

        # This function runs selects and executes a randomly chosen action
        # q_algorithm_pt2 is called when a QLearningReward is published (this updates q_matrix)
        # with appropriate reward
        def q_algorithm_pt1(self):
            valid = self.valid_actions()

            # Select a random actions
            self.action_num = valid[random.randint(0, len(valid) - 1)]
            action = RobotMoveDBToBlock()
            block_num = (self.action_num+1)%3
            if block_num == 0:
                block_num = 3
            action.block_id = block_num
            color = ''
            if self.action_num >= 0 and self.action_num <=2:
                color = 'red'
            elif self.action_num >= 3 and self.action_num <= 5:
                color = 'green'
            else:
                color = 'blue'
            action.robot_db = color

            # Perform the chosen action
            self.bot_action.publish(action)

        # Callback function for /q_learning/reward topic. Updates the matrix, and increments the time
        # Additionally, checks to see if Q has converged and, if no, runs the algorithm again
        def q_algorithm_pt2(self, q_reward):
            # Used to determine whether reward occurred too quickly and show be ignored
            # See writeup for explanation
            msg_time = q_reward.header.stamp.secs + (q_reward.header.stamp.nsecs/1000000000)
            if self.last_msg_time + 0.08 < msg_time:
                initial_val = copy.deepcopy(self.actual_q_matrix.q_matrix[self.actual_state].q_matrix_row[self.action_num])
                reward = q_reward.reward

                # Find the next state. This is used to get the max reward from the next state
                next_state = self.determine_next_state()
                # Set this to arbitrarily low (negative) number
                next_state_reward = -1000000
                j = copy.deepcopy(self.actual_q_matrix.q_matrix[next_state].q_matrix_row)
                for x in j:
                    if x > next_state_reward:
                        next_state_reward = x

                # Calculate the net reward value to put into the q_matrix for this action
                future_val = self.gamma * (next_state_reward - initial_val)
                # If this is the last action before the world resets, do not include a future value
                # (q_matrix will not have an value for the future state, causing the value to be negative)
                if (self.t+1)%3 == 0:
                    total_val = initial_val + self.alpha * (reward - initial_val)
                else:
                    total_val = initial_val + self.alpha * (reward + future_val)

                # Update the value in the q_matrix
                z = copy.deepcopy(self.actual_q_matrix.q_matrix[self.actual_state])
                z.q_matrix_row[self.action_num] = total_val
                self.actual_q_matrix.q_matrix[self.actual_state] = z

                # Update the current state and increment the time
                self.actual_state = next_state
                self.t = self.t + 1

                # If all three DBs have been moved, the world has reset and we
                # likewise need to reset the state to 0
                if self.t%3 == 0:
                    self.actual_state = 0
                    rospy.sleep(0.25)

                # Publish updated q_matrix
                self.q_matrix_pub.publish(self.actual_q_matrix)

                # Check to see if converged. If not, begin loop again
                self.has_converged()
                if not self.exit_algo:
                    self.q_algorithm_pt1()
                else:
                    print('---Matrix has Converged---')
                    print(self.actual_q_matrix)
                    print(self.converge_count)
                    print(self.t)
                    print('---Matrix has Converged---')
                    # Generate order of actions to be taken
                    self.robot_command(copy.deepcopy(self.actual_q_matrix), copy.deepcopy(self.state_matrix))

            self.last_msg_time = msg_time

        # This function serves as the while loop in the q algorithm. It checks
        # to see if our q_matrix has converged after every set of moves (ie. after all DBs are moved).
        # Once the algorithm has converged a pre-determined number of times (as determined by)
        # self.converge_threshold), it will flip a flag so the program knows to exit the q q_learning
        # algorithm
        # The algorithm checks convergence by comparing values in the current q_matrix and a copy
        # of the previous iteration's q_matrix
        def has_converged(self):
            if self.t%3 != 0:
                self.exit_algo = False
            else:
                same = True
                for x in range(0,64):
                    current = self.actual_q_matrix.q_matrix[x]
                    previous = self.prev_q_matrix.q_matrix[x]
                    for y in range(0,9):
                        if current.q_matrix_row[y] != previous.q_matrix_row[y]:
                            self.exit_algo = False
                            same = False
                    # A previous state in the q_matrix was divergent
                    if not same:
                        break
                # Q_matrix has converged
                if same:
                    self.converge_count = self.converge_count + 1
                    if self.converge_count >= self.converge_threshold:
                        self.exit_algo = True
                    else:
                        self.prev_q_matrix = copy.deepcopy(self.actual_q_matrix)
                else:
                    self.prev_q_matrix = copy.deepcopy(self.actual_q_matrix)
                    self.converge_count = 0





        def run(self):
                rospy.spin()

if __name__ == '__main__':
        rospy.init_node('q_learning')
        q_learner = QLearner()
        q_learner.run()
