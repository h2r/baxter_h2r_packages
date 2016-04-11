baxter_kinect_calibration makes it easy to calibrate your Kinect device to your robot

Installation
-----------------------
Unless you've installed baxter_h2r_packages according to its README, you will also need to get ar_track_alvar

```
cd ~/baxter_ws/src
git clone -b hydro-devel https://github.com/sniekum/ar_track_alvar
cd ..
catkin_make

```

Usage
------------------------
First, verify that the left hand camera is open. 
Check by running 
```
rosrun baxter_tools camera_control.py -l 
```
If it is not on the list, you must close another camera so the left one shows through 

```
rosrun baxter_tools camera_control.py -c <camera name> 
```
And open the left one at max resolution (1280 by 800)
```
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
```


Print a copy of table8_9_10.png and place it on a table where the kinect can view it as well as the left arm camera. The markers in the printout should be approximately 44-45 mm wide.
https://github.com/sniekum/ar_track_alvar/blob/groovy-devel/markers/table_8_9_10.png

(there is a scaled version to work on the sunlab computers in the marker directory)

Run the following command. 

```
$ roslaunch baxter_kinect_calibration baxter_bundle_calibrate.launch
```

While this is running, view the transform so you can make use of it. 

```
rosrun tf tf_echo /reference/base /openni_link
```

You can now publish a static transform with the following command
```
rosrun tf static_transform_publisher x y z qx qy qz qw /world /openni_link 10
```

You can now write a launch file with the following syntax
```
<launch>
    <node pkg="tf" type="static_transform_publisher" name="kinectTransformer" args="x y z qx qy qz qw /world /openni_link 10" />
</launch>
```

You can now bring down baxter_bundle_calibrate.launch
