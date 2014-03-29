To use this code you will need OpenNI, OpenNITracker, NiTE v1.5.2.23, and the python library sci-kit for machine learning classifiers.

OpenNI can be installed using apt-get like usual
OpenNI Tracker needs to be built from source. the git repo is here:
https://github.com/ros-drivers/openni_tracker

Note: you will need to modify the CMakeListsFiles and comment out any line that refers to NiTE

Download NiTE separately from the OpenNI website (make sure it's the correct version) and run the install script

sci-kit installation instructions can be found at:
http://scikit-learn.org/stable/


For any of the scripts you will first need to launch OpenNI and OpenNI tracker. Before you do, you should set the ROS param as follows:

rosparam set openni_tracker/camera_frame_id "camera_depth_optical_frame"

To launch the two:

roslaunch openni_launch openni.launch
rosrun openni_tracker openni_tracker

You should also almost always be running the parseTFToFV.py script. This script parses skeletal transforms that are tracked (users are tracked once they stand in front of the connect and make the psi symbol) and turns them into a feature vector that is published to
the topic pose_feature_vector. The message consists of 4 vectors (a vector from the head to each hand and elblow) and also reports the distance of the user's head from the kinect. To run:

rosrun pose_feature_vector parseTFToFV

Once run there is a few second delay between when the user is tracked and when data will start to be published (it will print the time remaining). This gives the user a chance to switch from the psi pose into some other pose

There are then two possible scripts that use the feature vector information. One for recording feature vector information to a file. That script is the psoe_fv_recorder. It takes two arguments a path to the file where the data will be saved and the "class label" (as an int) of the pose that will be recorded. Once information is published to the pose_feature_vector topic, it will start recording. However, recording to the file is also delayed by a few seconds. This means when you're done making poses you have a few seconds to walk over to the computer and hit cntr-c to kill the recorder so that the last few seconds of you moving to the computer will not be saved. Once you've saved a punch of different poses cat them all into a file to use as training data. The class labels have the following correspondance:
0: hug
1: high five
2: fist bump
3: null pose
4: psi pose
Example files are in the directory poseData. You should consider using data3_nh.csv. For the example in that file, high fives and fist bumps are always with the right hand.

The other script detect_pose.py takes one argument, the path to a training file, which is turned into a classifier using logistic regression. Once a user is tracked it will classify their pose according the logistic regression. If they hold the pose of a hug, fist bump, or high-five for 15 consecutive refresh cycles, it will cause baxter to enter the behavior for that. Note that you need to have ros pointed to baxter for the behaviors to actually work! Use Stephen's baxter script.