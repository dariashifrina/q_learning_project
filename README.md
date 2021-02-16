# Q-Learning Project

## Team Members: Joshua Soong, Daria Shifrina

## Implementation Plan

###   Q-learning algorithm
-   **Executing the Q-learning algorithm**
    

![](https://lh3.googleusercontent.com/hnlul0XhaNpAwx9ZUYSOJ_QOQMIeCt6X_QhAFaTO1XsYm3nI_icYyTzyl1gyp_f63WOFsPBTaITsvIFpHIV_xnmemnOuyBlBQy4bZO7M-BlkoK_vOZ-p2DtwB1ojLW_f45j64s1B)

  

We will likely break the algorithm down into smaller helper functions to modularize and compartmentalize each line of the above algorithm (taken from class slides). This will not only make it easier for us to work on each section of the code but crucially will help us isolate problems when we are debugging. To test this, we intend to test each component independently. For example, if we run the first “step” of our algorithm several times, we can expect our robot to move in a randomized fashion each time.

-   **Determining when the Q-matrix has converged**

We will store two states at all times, with state Q1 being an updated state of Q0. We will decide that our Q-matrix has converged after there are no observable changes to the Q-matrix with a new iteration, or when Q1 is almost equal to Q0. It is important to note that there might be instances when there’s no change, but in reality our matrix hasn’t actually converged. At this point, to account for any noise, we will run five additional iterations and verify that the Q-matrix is finalized if no changes occur within those iterations. To test this, we will use a for statement to loop through the iterations’ q-matrices and check that they are identical.

-   **Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward**
    

Once we have completely mapped out our policy, we can use it to determine how the robot should move. Specifically, if our robot finds itself in grid X, we can call grid X from our policy mapping, determine which direction offers the highest reward, and execute an action accordingly. This process can then be repeated for each grid the robot finds itself in. To test this, we can print out the policy mapping and see if the robot’s movements align with the expected action.

### Robot perception
    

**-   Determining the identities and locations of the three colored dumbbells**
    

To determine the location and identity, we will utilize the model_states_received() function along with the LaserScan and Camera topics. We will be basing this off of the class meeting 3 code. First, we will make a subscriber for ‘camera/rgb/image_raw’ and then convert the incoming ROS messages to OpenCV format and HSV. Given the HSV, we’ll convert it for the different color channels to distinguish the dumbbells.

  

 1. *HSV red*: lower bound [80,70,50], upper bound [100, 255, 255]
    
 2.  *HSV blue*: lower bound [110,50,50], upper bound [130,255,255]
    
   3. *HSV green*: lower bound [50,100,100], upper bound [70,255,255]

Using moments, we will determine the center of the colored pixels and get an estimated movie for the LaserScan location. To test this, we can implement print statements that output the color the robot sees when in front of a dumbbell ( and see if this conforms with what we see in the RViz camera).

-   **Determining the identities and locations of the three numbered blocks**
    

We intend to use the OpenCV and Python Tesseract libraries along with Camera to recognize the digits on the blocks. Python-tesseract is a wrapper for Google’s Tesseract-OCR Engine which is used to recognize text from images. We will use the same subscriber from dumbbell detection, get incoming ROS messages and use OpenCV to narrow down the black text region. Then, using Python Tesseract, we will translate the detected text into a number. For this to properly work, our bot will detect each block using LaserScan and position itself directly in front of them to get a 2D image. To test this, we can give our code a selection of test images and see if it can then correctly identify the text.

### Robot manipulation & movement
    

-   **Picking up and putting down the dumbbells with the OpenMANIPULATOR arm**
    

Since we know that the claw needs to be perpendicular to the dumbbell to pick it up and that our robot will be near the dumbbell before attempting to pick it up, we can employ inverse kinematics to determine the optimal joint angles. Additionally, when the robot is not holding a dumbbell we will have the claw fully open (and when it wants to hold a dumbbell it will close its claw). To test this, we can initialize the robot next to a dumbbell and see whether or not it successfully picks up (and puts down) the dumbbell.

-   **Navigating to the appropriate locations to pick up and put down the dumbbells**
    

We can navigate to the appropriate location using our action matrix, which can be populated with information from our converged q-learning algorithm. If needed, we can also use information from our LIDAR scan to ensure our robot does not crash into anything when executing instructions from the action matrix. To test this, we can print out our Q-matrix and actio matrix, use these to determine which action the robot should be taking, and visually check whether the robot is indeed following the expected instructions.

### Timeline
    

-   February 21: complete Q algorithm and robot dumbbell detection.
    
-   February 24: complete robot perception, incorporate both dumbbell and block detection.
    
-   February 26: complete robot manipulation & movement.
    

-   February 27: writeup. Record gif and rosbag.
    

-   February 28: finalize writeup. submit everything.
