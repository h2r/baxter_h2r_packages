meldon_detection

sudo apt-get install:
libhdf5-7
libeigen3-dev

Additionally, make sure you have the following installed:
opencv
cv_bridge
image_transport

export PKG_CONFIG_PATH=$CATKIN_WS/src/meldon_detection/src/kdes/dependencies/matio:$PKG_CONFIG_PATH
cd $CATKIN_WS/src/meldon_detection/src/kdes/dependencies/matio
make CFLAGS=-DH5_USE_16_API
export LD_LIBRARY_PATH=$CATKIN_WS/src/meldon_detection/src/kdes/dependencies/matio/src/.libs:$LD_LIBRARY_PATH
cd ../../KernelDescriptors_CPU/
make
cd $CATKIN_WS
catkin_make

<Everything should now be built>

To run:
rosrun meldon_detection talker $CATKIN_WS
roslaunch baxter_ork detection.launch
rosrun object_recognition_ros client

The last step must be performed every time you wish to receive a labeling.
Labeled clusters are published to /labeled_objects in the form of RecognizedObjectArray.msg