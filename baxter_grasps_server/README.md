The grasps server serves a series of possible grasps for detectable objects. All grasps are stored as yaml versions of moveit_msgs/Grasp messages. 

Usage
-----------------------------
```
roslaunch baxter_grasps_server server.launch
```


Annotating Grasps
-----------------------------
grasp_annotator.py and marker_grasp_annotator listen to topics to find where the objects are. They continuously update the pose of the object, so even if it moves, it should be ok.

grasp_annotator.py requires ORK object detection running. marker_annotator requires running baxter_indiv.launch in the baxter_kinect_calibration package.

fixed_grasp_annotator.py requires denoting the location of the object with baxter's gripper.



Generating Grasps
----------------------------
If your object has some decent symetry, you can write your own yaml description of a Grasp (see some of the included grasps for an example). The cylinder_grasp_generator and the spherical analogue will generate many grasps rotated around 0,0,z, where z is the z position of the grasp pose