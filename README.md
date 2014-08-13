To make and install
-------------------

This is now installable via a rosinstall file. It requires access to the RethinkRobotics github repos.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool merge --merge-replace -y https://raw.githubusercontent.com/davetcoleman/baxter_cpp/hydro-devel/baxter.rosinstall
wstool merge --merge-replace -y https://raw.githubusercontent.com/h2r/baxter_h2r_packages/master/h2r.rosinstall
wstool merge https://raw.githubusercontent.com/h2r/baxter_h2r_packages/master/ork.rosinstall
wstool update
```

Need to remove one package that doesn't build without a lot of other dependencies
```
rm -rf checkerboard_detector
```

Build it
```
cd .. 
source /opt/ros/hydro/setup.bash
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro hydro -y
catkin_make
```


Running  demo
-------------------
This demo expects that you first calibrate your kinect. Print a copy of table8_9_10.png and place it on a table where the kinect can view it as well as the left arm camera (after moving it to the correct position)
https://github.com/sniekum/ar_track_alvar/blob/groovy-devel/markers/table_8_9_10.png



```
$ cd ~/baxter_ws/
$ ./baxter.sh
$ rosrun baxter_kinect_calibration setup_robot_left.sh
```

Grab the left cuff and move the camera to view the alvar bundle printout, as well as the kinect.

```
$ roslaunch baxter_kinect_calibration baxter_bundle_calibrate.launch
```

While this is running, view the transform so you can make use of it.

```
rosrun tf tf_echo /world /camera
```

Once satisfied with the transform, kill the calibration launch file.


Running ork_pick
-----------------------
```
roslaunch baxter_hackathon_demos ork_pick.launch x:=0.0 y:=0.0 z:=0.0 qx:=0.0 qy:=0.0 qz:=0.0 qw:=1.0
```

Replace the values for x,y,z,qx,qy,qz, and qw from the transform you found in the previous step.

The listen_pick node dies due to a race condition. You'll have to start it manually

```
rosrun listen_and_grasp listen_pick.py
```

Running speech2moveit
------------------------

This demo launch file will expect to find a launch file that publishes the static transform between baxter and the kinect in baxter_kinect_calibration/launch

```
$ roslaunch baxter_hackathon_demos baxter_speech2moveit.launch

```

Enjoy!!

-H2R @ Brown
