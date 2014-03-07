Print a copy of table8_9_10.png and place it on a table where the kinect can view it as well as the left arm camera (after moving it to the correct position)
https://github.com/sniekum/ar_track_alvar/blob/groovy-devel/markers/table_8_9_10.png



```
$ cd ~/baxter_ws/
$ ./baxter.sh
$ rosrun baxter_tools enable_robot -e
```

Grab the left cuff and move the camera to view the alvar bundle printout. Run calibration

```
$ roslaunch baxter_kinect_calibration baxter_bundle_calibrate.launch
```

This publishes a transform to /camera_link. In order to make use of the transform, you will need to create your own static transform publisher.

To get the values, run
```
rosrun tf tf_echo /world /camera_link
```

You can now write a launch file with the following syntax
```
<launch>
    <node pkg="tf" type="static_transform_publisher" name="kinectTransformer"args="x y z qx qy qz qw" />
</launch>

```

You can now bring down baxter_bundle_calibrate.launch
