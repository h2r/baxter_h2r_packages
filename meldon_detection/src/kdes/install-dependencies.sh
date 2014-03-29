#!/bin/sh
sudo apt-get install cmake libopenmpi-dev libhdf5-serial-dev libboost-all-dev
cd ./dependencies/matio/
./configure
make
sudo make install
cd ../Eigen_3.0.1
mkdir build
cd build
cmake ..
make
sudo make install
#cd ../../OpenCV-2.3.0
#mkdir build
#cd build
#cmake ..
#make
#sudo make install
sudo ldconfig
