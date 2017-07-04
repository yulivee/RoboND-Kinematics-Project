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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
	    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i
	    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') 
            
            # Joint angle symbols
	    s = {alpha0:     0,  a0:      0, d1:  0.75,  
	         alpha1: -pi/2,  a1:   0.35, d2:     0, q2: q2-pi/2, 
	         alpha2:     0,  a2:   1.25, d3:     0,       
	         alpha3: -pi/2,  a3: -0.054, d4:  1.50,
	         alpha4:  pi/2,  a4:      0, d5:     0,
	         alpha5: -pi/2,  a5:      0, d6:     0,
	         alpha6:     0,  a6:      0, d7: 0.303, q7: 0}
      
            # Modified DH params
	    T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
			   [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
			   [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
			   [                   0,                   0,            0,               1]])
	    T0_1 = T0_1.subs(s)

	    T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
			   [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
			   [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
			   [                   0,                   0,            0,               1]])
	    T1_2 = T1_2.subs(s)

	    T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
			   [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
			   [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
			   [                   0,                   0,            0,               1]])
	    T2_3 = T2_3.subs(s)

	    T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
			   [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
			   [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
			   [                   0,                   0,            0,               1]])
	    T3_4 = T3_4.subs(s)
	    T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
			   [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
			   [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],                                                                                                           
			   [                   0,                   0,            0,               1]])
	    T4_5 = T4_5.subs(s)
	    T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
			   [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
			   [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],                                                                                                           
			   [                   0,                   0,            0,               1]])
	    T5_6 = T5_6.subs(s)
	    T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],                                                                                                           
			   [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
			   [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
			   [                   0,                   0,            0,               1]])
	    T6_G = T6_G.subs(s)

            
            # Define Modified DH Transformation matrix
	    # Correct the Gripper
	    R_z =  Matrix([[     cos(np.pi), -sin(np.pi),             0, 0],
			   [     sin(np.pi),  cos(np.pi),             0, 0],
			   [              0,           0,             1, 0],
			   [              0,           0,             0, 1]])

	    R_y =  Matrix([[  cos(-np.pi/2),           0, sin(-np.pi/2), 0],
			   [              0,           1,             0, 0],
			   [ -sin(-np.pi/2),           0, cos(-np.pi/2), 0],
			   [              0,           0,             0, 1]])

	    R_corr = simplify(R_z * R_y)



            # Create individual transformation matrices
	    T0_2 = simplify(T0_1 * T1_2) # base_link to link_2
	    T0_3 = simplify(T0_2 * T2_3) # base_link to link_3
	    T0_4 = simplify(T0_3 * T3_4) # base_link to link_4
	    T0_5 = simplify(T0_4 * T4_5) # base_link to link_5
	    T0_6 = simplify(T0_5 * T5_6) # base_link to link_6
	    T0_G = simplify(T0_6 * T6_G) # base_link to gripper_link
	    T_total = simplify(T0_G * R_corr) # Total HT between base_link and gripper_link with orientation correction applied

            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            end_effector_length = 0.303            

	    px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            print "px:",px," py:",py," pz:",pz
	    print "roll:",roll,"pitch:",pitch,"yaw:",yaw

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
	
	    R = tf.transformations.quaternion_matrix([req.poses[x].orientation.x, req.poses[x].orientation.y,req.poses[x].orientation.z, req.poses[x].orientation.w])
	    al, be, ga = tf.transformations.euler_from_matrix(R,axes = 'ryzx')

	    # End Effector Position
	    p_ee = Matrix([ [ px ], [ py ], [ pz ] ] )

	    # Define translation along X to Wrist Center
            t_wc = Matrix([ [ 0 ], [ 0 ], [ 1 ] ] )

	    # Build a rotation matrix from the Roll, Pitch and Yaw angles
	    Rrpy = Matrix([[    cos(yaw)*cos(pitch),   cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll),    cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll)],
			   [    sin(yaw)*cos(pitch),   sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll),    sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll)],
			   [            -sin(pitch),             cos(pitch)*sin(roll),                                       cos(pitch)*cos(roll)               ]])

            nx = Rrpy[ 2, 0 ]
            ny = Rrpy[ 2, 1 ]
            nz = Rrpy[ 2, 2 ]

            print "lx:",lx," ly:",ly," lz:",lz

	    d1 = 0.75
	    a2 = 1.25
	    d6 = 0

	    # Calculate Wrist Center
            wx = px - ( end_effector_lenght + d6 ) * nx
            wy = py - ( end_effector_lenght + d6 ) * ny
            wz = pz - ( end_effector_lenght + d6 ) * nz

            print "wx:",wx," wy:",wy," wz:",wz

	    
            # Calculate joint angles using Geometric IK method
            theta1 = atan2(wy, wx) 

	    s2 = wz - d1
            w_x_off = wx
            w_y_off = wy
	    r = sqrt( w_x_off**2 + w_y_off**2 )
	    theta2 = atan2(s2, r) - pi/25
	    s3 = wz - (a2 + d1)
	    theta3 = atan2(s3,r)

	    print "theta1,2,3:",theta1, theta2, theta3 
	     
	    ################################theta 4,5,6 calculation ############################################
	    R0_3_eval = Matrix([[    sin(theta2 + theta3)*cos(theta1),   cos(theta1)*cos(theta2 + theta3),   -sin(theta1)],
		[   sin(theta1)*sin(theta2 + theta3),   sin(theta1)*cos(theta2 + theta3),   cos(theta1)],
		[   cos(theta2 + theta3),           -sin(theta2 + theta3),          0]])


	    print "R0_3_eval:", R0_3_eval            


	    R3_6_eval = R0_3_eval.inv() * Rrpy

	    print "R3_6_eval:", R3_6_eval

	    theta5 = acos(R3_6_eval[1,2])
	    theta6 = asin(R3_6_eval[1,1] / (-sin(theta5)))
	    theta4 = asin(R3_6_eval[2,2] / sin(theta5))

	    print "theta4,5,6:",theta4,theta5,theta6
	

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
