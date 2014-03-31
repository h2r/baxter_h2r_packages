meldon_detection

```
sudo apt-get install libhdf5-7 libhdf5-dev libeigen3-dev
```

Additionally, make sure you have the following installed:

opencv
cv_bridge
image_transport

Set $CATKIN_WS to the appropriate directory 
```
export CATKIN_WS=$HOME/catkin_ws
```

Install the matio dependencies
```
export PKG_CONFIG_PATH=$CATKIN_WS/src/baxter_h2r_packages/meldon_detection/src/kdes/dependencies/matio:$PKG_CONFIG_PATH
cd $CATKIN_WS/src/baxter_h2r_packages/meldon_detection/src/kdes/dependencies/matio
make CFLAGS=-DH5_USE_16_API
sudo make install
export LD_LIBRARY_PATH=$CATKIN_WS/src/baxter_h2r_packages/meldon_detection/src/kdes/dependencies/matio/src/.libs:$LD_LIBRARY_PATH
cd ../../KernelDescriptors_CPU/
make
cd $CATKIN_WS
catkin_make
```

<Everything should now be built>

To run:
```
rosrun meldon_detection talker $CATKIN_WS
roslaunch baxter_ork detection.launch
rosrun object_recognition_ros client
```

The last step must be performed every time you wish to receive a labeling.
Labeled clusters are published to /labeled_objects in the form of RecognizedObjectArray.msg