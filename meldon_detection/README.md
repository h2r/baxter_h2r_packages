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

<Everything should now
