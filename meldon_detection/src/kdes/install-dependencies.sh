#!/bin/sh
sudo apt-get install cmake libopenmpi-dev libhdf5-serial-dev libboost-all-dev libhdf5-7 libhdf5-dev libeigen3-dev
cd ./dependencies/matio/
./configure
make CFLAGS=-DH5_USE_16_API
sudo make install
sudo ldconfig
