The grasps server serves a series of possible grasps for detectable objects. All grasps are stored as yaml versions of moveit_msgs/Grasp messages. 

To run, you need to point it to the grasps directory

```
rosrun baxter_grasps_server grasp_server $(rospack find baxter_grasps_server)/grasps
```
