Unless you've installed baxter_h2r_packages according to its README, you will also need to get ar_track_alvar

```
cd ~/baxter_ws/src
git clone -b hydro-devel https://github.com/sniekum/ar_track_alvar
cd ..
catkin_make

```

Print a copy of table8_9_10.png and place it on a table where the kinect can view it as well as the left arm camera (after moving it to the correct position). The markers in the printout should be approximately 44-45 mm wide.
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

You can now publish a static transform with the following command
```
rosrun tf static_transform_publisher x y z qx qy qz qw /world /camera_link 10
```

You can now write a launch file with the following syntax
```
<launch>
    <node pkg="tf" type="static_transform_publisher" name="kinectTransformer" args="x y z qx qy qz qw /world /camera_link 10" />
</launch>
```



You can now bring down baxter_bundle_calibrate.launch
