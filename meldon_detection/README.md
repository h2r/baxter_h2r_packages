meldon_detection


Make sure you have the following installed:

opencv
cv_bridge
image_transport

Set $DECTECTION to the appropriate directory 
```
export DETECTION=$HOME/catkin_ws/src/baxter_h2r_packages/meldon_detection
```

Install the matio dependencies
```
export PKG_CONFIG_PATH=$DECTECTION/src/kdes/dependencies/matio:$PKG_CONFIG_PATH
cd $DECTECTION/src/kdes
./install-dependencies.sh #NOTE: this will yield an error referencing doxygen. This is fine.
export LD_LIBRARY_PATH=$DECTECTION/src/kdes/dependencies/matio/src/.libs:$LD_LIBRARY_PATH
cd KernelDescriptors_CPU/
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
