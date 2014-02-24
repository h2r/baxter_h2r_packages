To make and install
-------------------

Clone this package into the src directory of your catkin workspace

```
$ git clone http://github.com/h2r/baxter_hackathon
```

Build it
```
$ cd .. 
$ source devel/setup.bash
$ catkin_make
$ catkin_make install
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
