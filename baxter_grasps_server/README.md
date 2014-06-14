The grasps server serves a series of possible grasps for detectable objects. All grasps are stored as yaml versions of moveit_msgs/Grasp messages. 

To run:

```
roslaunch baxter_grasps_server server.launch
```

Kinect must be calibrated with robot, and a static transform must be published


To launch object recognition, and grasping

```
cd ~/workspace
./baxter.sh
roslaunch openni_launch openni.launch depth_registration:=true

--------------------
cd ~/workspace
./baxter.sh
rosrun object_recognition_core -c $(rospack find tabletop)/conf/detection.object.ros.ork


--------------------
cd ~/workspace
./baxter.sh
roslaunch baxter_grasps_server server.launch
---------------------

cd ~/workspace
./baxter.sh
roslaunch baxter_control baxter_hardware.launch

--------------------

cd ~/workspace
./baxter.sh
roslaunch baxter_moveit_config demo.launch
```