#!/usr/bin/env python
import roslib
roslib.load_manifest("baxter_kinect_calibration")

import argparse
import sys
import rospy
import baxter_interface
import moveit_commander
import tf
import math
from geometry_msgs.msg import PoseStamped,  Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header

from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

global control_arm, listener, group

center_position = [0.75,-0.0,0.0]
positions = [[0.6,-0.1,0], [0.1, 0.1, 0.1], [-0.1, -0.1, -0.1]]

def solveIK(pose):
    ns = "/ExternalTools/left/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    ikreq.pose_stamp.append(pose)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    if (resp.isValid[0]):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints;
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    

def moveToPose(pose, useMoveIt=False):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    goalPose = PoseStamped(header=hdr, pose=pose)
    if (useMoveIt):
        group.set_start_state_to_current_state()
        group.set_pose_target(goalPose)
        group.go()
    else:
        joints = solveIK(goalPose)
        control_arm.move_to_joint_positions(joints)

def moveToNeutral():
    control_arm.move_to_neutral()

def moveAndCapture(useMoveIt=False):
    calculated_positions = []
    commanded_positions = []
    for i in range(3):
        positions_row = [] 
        commanded_row = []
        for j in range(3):
            moveToNeutral()
            quat = quaternion_from_euler(math.pi,-0.5 * math.pi,0)
            orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            position = Point(x=center_position[0] + i*0.1, y=center_position[1] + j*0.15, z=0)
            commanded_row.append([position.x, position.y, position.z])
            pose = Pose(position=position, orientation=orientation)
            moveToPose(pose, useMoveIt)
            rospy.sleep(5)

            try:
                listener.waitForTransform("/world", "ar_marker_6", rospy.Time(0), rospy.Duration(1))
                transform = listener.lookupTransform("/world", "ar_marker_6", rospy.Time(0))
                origin = transform[0]
                positions_row.append([origin[0], origin[1], origin[2]])
            except tf.Exception:
                positions_row.append([0,0,0])
        calculated_positions.append(positions_row)
        commanded_positions.append(commanded_row)
    return commanded_positions, calculated_positions

def alvar_check(numTrials, useMoveIt = False):
    calculated_positions = []  
    commanded_positions = []
    for i in range(numTrials):
        print("Trial: " + str(i))
        commanded, calculated = moveAndCapture(useMoveIt)
        commanded_positions.append(commanded)
        calculated_positions.append(calculated)
    average_positions = []
    average_error = []
    average_bias = []

    for i in range(3):
        positions_row = []
        errors_row = []
        bias_row = []
        for j in range(3):
            sum = [0.0,0.0,0.0]
            trials = 0
            for k in range(numTrials):
                if (calculated_positions[k][i][j][0] != 0):
                    trials += 1
                    for l in range(3):
                        sum[l] += calculated_positions[k][i][j][l]
            positions_row.append([pos/trials for pos in sum])
            sum_error = [0.0,0.0,0.0]
            sum_bias = [0.0, 0.0, 0.0]
            for k in range(numTrials):
                if (calculated_positions[k][i][j][l] != 0):
                    for l in range(3):
                        sum_error[l] += math.fabs(positions_row[j][l] - calculated_positions[k][i][j][l])
                        sum_bias[l] += math.fabs(positions_row[j][l] - commanded_positions[k][i][j][l])
            errors_row.append([error/trials for error in sum_error])
            bias_row.append([bias/trials for bias in sum_bias])
        average_error.append(errors_row)
        average_positions.append(positions_row)
        average_bias.append(bias_row)

    for i in range(3):
        for j in range(3):
            print(str([round(v,4) for v in commanded_positions[0][i][j]]))
            print(str([round(v,4) for v in average_positions[i][j]]))
            print(str([round(v,4) for v in average_error[i][j]]))
            print(math.sqrt(average_error[i][j][0]*average_error[i][j][0] +
                            average_error[i][j][1]*average_error[i][j][1] +
                            average_error[i][j][2]*average_error[i][j][2])) 
            print(str([round(v,4) for v in average_bias[i][j]]))
            print(math.sqrt(average_bias[i][j][0]*average_bias[i][j][0] +
                            average_bias[i][j][1]*average_bias[i][j][1] +
                            average_bias[i][j][2]*average_bias[i][j][2])) 
                             
            print("\n")
if __name__ == '__main__':
    rospy.init_node('check_alvar')
    listener = tf.TransformListener()
    group = moveit_commander.MoveGroupCommander("left_arm")
    control_arm = baxter_interface.limb.Limb("left")
    #alvar_check(20, False)
    alvar_check(20, True)
