#!/usr/bin/env python

import roslib
roslib.load_manifest("baxter_props")

import argparse
import sys
import rospy
import baxter_interface

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from dynamic_reconfigure.server import Server
from baxter_examples.cfg import JointSpringsExampleConfig

class BaxterProps:
    def __init__(self):
        self.dynamic_cfg_srv = Server(JointSpringsExampleConfig,
                             lambda config, level: config)
    def hug(self, height, strength):
        baxter_interface.RobotEnable().enable()
        left_arm = baxter_interface.limb.Limb("left")
        right_arm = baxter_interface.limb.Limb("right")
        hug_prep_right = Pose(position=Point(x=0.9,y=-0.7,z=height), orientation=Quaternion(x=0,y=0.707,z=0,w=0.707))    
        hug_prep_left = Pose(position=Point(x=0.9,y=0.7,z=height), orientation=Quaternion(x=0,y=0.707,z=0,w=0.707)) 
      
        cur_left = left_arm.joint_angles()
        cur_right = right_arm.joint_angles()   
        start_left = self.solveIK(hug_prep_left, "left")
        start_right = self.solveIK(hug_prep_right, "right")
        joints_left = self.getIntermediateJointPositions(cur_left, start_left, numSteps=7000)
        joints_right = self.getIntermediateJointPositions(cur_right, start_right, numSteps=7000)
        self.moveTwoArms(left_arm, right_arm, joints_left, joints_right, False)
        
        hug_final_right = Pose(position=Point(x=0.4,y=0.5,z=height - 0.1), orientation=Quaternion(x=-0.5,y=0.5,z=0.5,w=0.5))    
        hug_final_left = Pose(position=Point(x=0.4,y=-0.5,z=height + 0.1), orientation=Quaternion(x=0.5,y=0.5,z=-0.5,w=0.5)) 
        end_left = self.solveIK(hug_final_left, "left")
        end_right = self.solveIK(hug_final_right, "right")
        joints_left = self.getIntermediateJointPositions(start_left, end_left, numSteps=7000)
        joints_right = self.getIntermediateJointPositions(start_right, end_right, numSteps=7000)
        self.moveTwoArms(left_arm, right_arm, joints_left, joints_right, True)
        rospy.sleep(3)

        cur_left = left_arm.joint_angles()
        cur_right = right_arm.joint_angles()   
        joints_left = self.getIntermediateJointPositions(cur_left, start_left, numSteps=7000)
        joints_right = self.getIntermediateJointPositions(cur_right, start_right, numSteps=7000)
        self.moveTwoArms(left_arm, right_arm, joints_left, joints_right, False)

    def bump(self, point, limb):

        # prep = Pose(position=Point(x=point.x-0.2,y=point.y,z=point.z), orientation=Quaternion(x=0.707,y=0,z=0,w=0.707))

        baxter_interface.RobotEnable().enable()
        arm = baxter_interface.limb.Limb(limb)
        #prep = Pose(position=Point(x=point.x-0.2,y=point.y,z=point.z), orientation=Quaternion(x=0,y=0.707,z=0,w=0.707))
        prep = Pose(position=Point(x=0.5,y=0.7,z=point.z), orientation=Quaternion(x=0,y=0.707,z=0,w=0.707))
        cur_angles = arm.joint_angles()
        start = self.solveIK(prep, limb)
        joints = self.getIntermediateJointPositions(cur_angles, start, numSteps=2500)
        self.moveOneArm(arm, joints, False)
        
        bumpP = Pose(position=point, orientation=Quaternion(x=0,y=0.707,z=0,w=0.707))    
        end = self.solveIK(bumpP, limb)
        joints = self.getIntermediateJointPositions(start, end, numSteps=2500)
        self.moveOneArm(arm, joints, False)
        rospy.sleep(0.5)

        #Translation: [0.165, 1.125, 0.479]
        # Rotation: in Quaternion [0.296, 0.846, 0.328, 0.298]
        '''
        - Translation: [0.333, 1.036, 0.416]
        - Rotation: in Quaternion [0.139, 0.752, 0.204, 0.612]
            in RPY [1.910, 1.041, 2.006]
        '''

        #finalP = Pose(position=Point(x=-0.2,y=0.862,z=0.348), orientation=Quaternion(x=0.429,y=0.839,z=0.230,w=0.244))
        #finalP = Pose(position=Point(x=point.x-.55,y=point.y+.35,z=point.z), orientation=Quaternion(x=0,y=0.707,z=0,w=0.707))
        finalP = Pose(position=Point(x=0.33,y=1.125,z=0.479), orientation=Quaternion(x=0.296,y=0.864,z=0.328,w=0.298))
        final = self.solveIK(finalP, limb)

        cur_angles = arm.joint_angles()
        joints = self.getIntermediateJointPositions(cur_angles, final, numSteps=2500)
        self.moveOneArm(arm, joints, False)

    def five(self, point, limb):
        print 'Entering high five method'
        baxter_interface.RobotEnable().enable()
        arm = baxter_interface.limb.Limb(limb)
        #[0.056, 1.063, 0.757]
        #[0.013, -0.009, 1.000, -0.026]
        #prep = Pose(position=Point(x=point.x-0.2,y=point.y,z=point.z), orientation=Quaternion(x=0.0,y=0,z=0,w=1))
        prep = Pose(position=Point(x=0.056,y=1.063,z=0.757), orientation=Quaternion(x=0.0,y=0,z=1,w=0))
        cur_angles = arm.joint_angles()
        start = self.solveIK(prep, limb)
        joints = self.getIntermediateJointPositions(cur_angles, start, numSteps=2500)
        self.moveOneArm(arm, joints, False)
        

        #[0.643, 0.814, 0.738]
        #[-0.068, 0.039, 0.987, 0.138]
        #final = Pose(position=point, orientation=Quaternion(x=0,y=0,z=0,w=1)) 
        #final = Pose(position=Point(x=0.643,y=0.841,z=0.738), orientation=Quaternion(x=-0.068,y=0.039,z=0.987,w=0.138))
        final = Pose(position=Point(x=0.643,y=0.841,z=0.738), orientation=Quaternion(x=0.0,y=0,z=1,w=0))    
        end = self.solveIK(final, limb)
        joints = self.getIntermediateJointPositions(start, end, numSteps=2500)
        self.moveOneArm(arm, joints, False)
        rospy.sleep(1.5)

        cur_angles = arm.joint_angles()
        joints = self.getIntermediateJointPositions(cur_angles, start, numSteps=2500)
        self.moveOneArm(arm, joints, False)

    def moveOneArm(self, arm, joints, useTorqueMode):
        count = len(joints)
        control_rate = rospy.Rate(1000)

        for i in range(count):
            self.moveJointsToPosition(arm, joints[i], useTorqueMode)
            control_rate.sleep()

        arm.exit_control_mode()

    def moveTwoArms(self, left_arm, right_arm, left_angles, right_angles, useTorqueMode):
        count = min(len(left_angles), len(right_angles))
        control_rate = rospy.Rate(1000)

        for i in range(count):
            self.moveJointsToPosition(left_arm, left_angles[i], useTorqueMode)
            self.moveJointsToPosition(right_arm, right_angles[i], useTorqueMode)
            control_rate.sleep()

        left_arm.exit_control_mode()
        right_arm.exit_control_mode()

    def moveJointsToPosition(self, limb, setPoint, useTorqueMode):
        if useTorqueMode:
            [springs, damping] = self.getSprings(limb)
            cur_pos = limb.joint_angles()
            cur_vel = limb.joint_velocities()
            cmd = dict()
            for joint in limb.joint_names():
                cmd[joint] = springs[joint] * (setPoint[joint]- cur_pos[joint])
                cmd[joint] -= damping[joint] * cur_vel[joint]
            limb.set_joint_torques(cmd)
        else:
            limb.set_joint_positions(setPoint)

    def getSprings(self, limb):
        
        springs = dict()
        damping = dict()
        for joint in limb.joint_names():
            springs[joint] = self.dynamic_cfg_srv.config[joint[-2:] +
                                                        '_spring_stiffness']
            damping[joint] = self.dynamic_cfg_srv.config[joint[-2:] +
                                                        '_damping_coefficient']
        return [springs, damping]


    def getIntermediateJointPositions(self, start, end, numSteps = 10000):
        joints = []
        for i in range(numSteps):
            t = float(i) / numSteps;
            jointState = dict()
            for key in start:
               jointState[key] = start[key] + t*(end[key] - start[key])
            joints.append(jointState)
        return joints


    def solveIK(self, pose, limb):
        ns = "/ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        goalPose = PoseStamped(header=hdr, pose=pose)


        ikreq.pose_stamp.append(goalPose)
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



if __name__ == '__main__':
    rospy.init_node("baxter_props")
    props = BaxterProps()
    props.hug(0.4, 10.0)
    #props.bump(Point(x=0.9, y=0.7, z=0.25), "left")
    #props.five(Point(x=0.5, y=0.7, z=0.5), "left")