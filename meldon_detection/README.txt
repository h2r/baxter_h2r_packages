meldon_detection

sudo apt-get install:
libhdf5-7
libeigen3-dev

Additionally, make sure you have the following installed:
opencv
cv_bridge
image_transport




cd $CATKIN_WS/src/meldon_detection/src/kdes/dependencies/matio
make CFLAGS=-DH5_USE_16_API
cd ../../KernelDescriptors_CPU/
make
cd $CATKIN_WS
make


At the end.