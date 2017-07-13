#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np
from Forward_Kinematics import FK
from Inverse_Kinematics import IK

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        #return -1
        return CalculateIKResponse(joint_trajectory_list)
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            myIK = IK(myFK.T0_1, myFK.T1_2, myFK.T2_3, myFK.symbols, myFK.q1, myFK.q2, myFK.q3)
            
            # Extract end-effector position and orientation from request
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            rospy.loginfo("px:   [ %7.4f ] py:    [ %7.4f ] pz:  [ %7.4f ]", px, py, pz )

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [req.poses[x].orientation.x, req.poses[x].orientation.y,
                        req.poses[x].orientation.z, req.poses[x].orientation.w])

            rospy.loginfo("roll: [ %7.4f ] pitch: [ %7.4f ] yaw: [ %7.4f ]", roll, pitch, yaw )

            myIK.get_wrist_rot_matrix( roll, pitch, yaw )

            wx, wy, wz = myIK.get_wrist_pos( px, py, pz )
            rospy.loginfo("WC_x: [ %7.4f ] WC_y: [ %7.4f ] WC_z: [  %7.4f ]", wx, wy, wz )
        
            theta_1, theta_2, theta_3 = myIK.get_theta_123( wx, wy, wz )
            rospy.loginfo("theta_1: [ %7.4f ] theta_2: [ %7.4f ] theta_3: [ %7.4f ]", theta_1, theta_2, theta_3 )

            R_3_6 = myIK.transform_to_wc( theta_1, theta_2, theta_3 )
            theta_4, theta_5, theta_6 = myIK.get_theta_456( R_3_6 )
            rospy.loginfo("theta_4: [ %7.4f ] theta_5: [ %7.4f ] theta_6: [ %7.4f ]", theta_4, theta_5, theta_6 )
            
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [ theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 ]
            joint_trajectory_list.append(joint_trajectory_point)

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server', log_level=rospy.DEBUG)
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()



print "Pre-Calculating Transformation Matrices"
myFK = FK()

if __name__ == "__main__":
    IK_server()
