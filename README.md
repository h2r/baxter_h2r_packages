To make and install
-------------------

This is now installable via a rosinstall file. It requires access to the RethinkRobotics github repos.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool merge --merge-replace -y https://raw.githubusercontent.com/davetcoleman/baxter_cpp/hydro-devel/baxter.rosinstall
wstool merge --merge-replace -y https://raw.githubusercontent.com/h2r/baxter_h2r_packages/master/h2r.wstool
wstool update
```

Install ros object recognition packages

```
$ sudo apt-get install ros-hydro-object-recognition-*
```


Build it
```
cd .. 
source /opt/ros/hydro/setup.bash
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro hydro -y
catkin_make
```


Running demo
-------------------
This demo expects that you first calibrate your kinect. Print a copy of table8_9_10.png and place it on a table where the kinect can view it as well as the left arm camera (after moving it to the correct position)
https://github.com/sniekum/ar_track_alvar/blob/groovy-devel/markers/table_8_9_10.png



```
$ cd ~/baxter_ws/
$ ./baxter.sh
$ rosrun baxter_tools enable_robot -e
```

Grab the left cuff and move the camera to view the alvar bundle printout.

```
$ roslaunch baxter_kinect_calibration baxter_bundle_calibrate.launch output_file:=$(rospack find baxter_kinect_calibration)/launch/kinect_publisher.launch
```

This demo launch file will expect to find a launch file that publishes the static transform between baxter and the kinect at the above location in baxter_kinect_calibration. 

```
$ roslaunch baxter_hackathon_demos baxter_speech2moveit.launch

```

Enjoy!!

-H2R @ Brown
