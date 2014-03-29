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
cd ../KernelDescriptors_CPU/
make
cd $CATKIN_WS
make


NOTE: Building matio will result in errors. These errors are from building documentation in a random dependency for matio. It doesn't matter as long as the terminal looks like:
lookup cache used 276/65536 hits=1951 misses=286
finished...
cd latex;.././../format_api.sh;.././../textopdf.sh
/bin/bash: .././../format_api.sh: No such file or directory
/bin/bash: .././../textopdf.sh: No such file or directory
make[2]: *** [../../doxygen/user_api/latex/libmatio.pdf] Error 127
make[2]: Leaving directory `/home/meldon/catkin_ws/src/meldon_detection/src/kdes/dependencies/matio/doxygen/user_api'
make[1]: *** [all-recursive] Error 1
make[1]: Leaving directory `/home/meldon/catkin_ws/src/meldon_detection/src/kdes/dependencies/matio/doxygen'
make: *** [all-recursive] Error 1

At the end.