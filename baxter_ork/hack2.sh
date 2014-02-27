#!/bin/bash
rosrun object_recognition_ros server -c $(rospack find baxter_ork)/conf/detection2.table.ros.ork
