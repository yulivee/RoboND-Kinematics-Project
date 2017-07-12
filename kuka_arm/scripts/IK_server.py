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

def init_transformation_matrices():
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
	T0_1 = build_transformation_matrix( q1, alpha0, a0, d1 )
	T1_2 = build_transformation_matrix( q2, alpha1, a1, d2 )
	T2_3 = build_transformation_matrix( q3, alpha2, a2, d3 )
	T3_4 = build_transformation_matrix( q4, alpha3, a3, d4 )
	T5_5 = build_transformation_matrix( q5, alpha5, a5, d5 )
	T5_6 = build_transformation_matrix( q6, alpha5, a5, d6 )
	T6_G = build_transformation_matrix( q7, alpha6, a6, d7 )

            
    # Define Modified DH Transformation matrix
	# Correct the Gripper by using an intrinsic rotation
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
	T0_G_corr = simplify(T0_G * R_corr) # Total HT between base_link and gripper_link with orientation correction applied

	return

def build_transformation_matrix( theta, alpha, a, d ):
	T = Matrix([[            cos(theta),           -sin(theta),           0,             a ],
                [ sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d ],
                [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d ],
                [                     0,                     0,           0,             1 ]])
	T = T.subs(s)

    return(T)

def get_wrist_pos(px, py, pz, Rrpy):
       
    end_effector_length = 0.453
    
    
    lx = Rrpy[ 0, 0 ]
    ly = Rrpy[ 1, 0 ]
    lz = Rrpy[ 2, 0 ]

    d6 = 0

    # Calculate Wrist Center
    wx = px - ( end_effector_length + d6 ) * lx
    wy = py - ( end_effector_length + d6 ) * ly
    wz = pz - ( end_effector_length + d6 ) * lz

    return wx, wy, wz

# Build a rotation matrix from the Roll, Pitch and Yaw angles
def get_wrist_rot_matrix( roll, pitch, yaw ):
    
    Rrpy = Matrix([[    cos(yaw)*cos(pitch),   cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll),    cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll)],
                   [    sin(yaw)*cos(pitch),   sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll),    sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll)],
                   [            -sin(pitch),             cos(pitch)*sin(roll),                                       cos(pitch)*cos(roll)               ]])

    return Rrpy

# Calculate joint angles using Geometric IK method
def get_theta_123 (wx, wy, wz):
    
    theta1 = atan2(wy, wx)
    a_1 = 0.35
    a_2 = 1.25
    d_1 = 0.75
    d_4 = 1.50
    a_3 = -0.054
    
    
    wc_dist_x = sqrt(wx**2 + wy**2) - a_1 # Subtract a1 as horizontal offset from robot-base
    wc_dist_z = wz - d_1 # Subtract d1 as vertical offset from robot base
    
    x_c = sqrt( wc_dist_x**2 + wc_dist_z**2 )
    y_c = wc_dist_z

    L_25 = sqrt(x_c**2 + y_c**2)
    L_35 = sqrt(a_3**2+d_4**2)

    beta_4 = atan2(a_3,d_4)
    #beta_3 = acos((L_25**2-a_2**2-L_35**2)/(-2*L_35*a_2))
    cos_beta_3 = (( -L_25**2 + a_2**2 + L_35**2) / ( 2 * a_2 * L_35 ))
    beta_3 = atan2( cos_beta_3, sqrt( 1 - cos_beta_3**2 ))
    theta3 = np.pi/2 - beta_3 - beta_4
    
    beta_2 = atan2( y_c, x_c )
    #beta_1 = acos( ( L_35**2 - L_25**2 - a_2**2 ) / ( -2 * L_25 * a_2 ) )
    cos_beta_1 = ( ( -L_35**2 + L_25**2 + a_2**2 ) / ( 2 * L_25 * a_2 ) )
    beta_1 = atan2(  cos_beta_1, sqrt( 1 - cos_beta_1**2 ) ) 
    theta2 = np.pi/2 - beta_1 - beta_2

    return [ theta1.evalf(), theta2, theta3 ]

def transform_to_wc(theta1, theta2, theta3, Rrpy):
 
    R0_3 = simplify( T0_1 * T1_2 * T2_3 )
    R0_3 = R0_3.evalf(q1: theta1, q2: theta2, q3: theta3)
    R3_6 = ( R0_3.inv() * Rrpy ) * R0_6
	
	return R0_3


def get_theta_456(R3_6, Rrpy):

	# plain wrong
	r11 = R3_6[0, 0]
	r21 = R3_6[1, 0]
	r31 = R3_6[2, 0]
	r32 = R3_6[2, 1]
	r33 = R3_6[2, 2]

	theta4 = atan2(r21, r11).evalf()
	theta5 = atan2(-r31, sqrt(r11 ** 2 + r21 ** 2)).evalf()
	theta6 = atan2(r32, r33).evalf()
    #theta5 = acos( R3_6_eval[1,2] )
    #theta6 = asin( R3_6_eval[1,1] / ( -sin(theta5) ) )
    #theta4 = asin( R3_6_eval[2,2] / sin(theta5) )

    return theta4.evalf(), theta5.evalf(), theta6.evalf()

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
            
			# Extract end-effector position and orientation from request
			px = req.poses[x].position.x
			py = req.poses[x].position.y
			pz = req.poses[x].position.z

			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
					[req.poses[x].orientation.x, req.poses[x].orientation.y,
						req.poses[x].orientation.z, req.poses[x].orientation.w])

			Rrpy = get_wrist_rot_matrix( roll, pitch, yaw )
			# see function get_wrist_pos on the calculation of the wrist center
			wx, wy, wz = get_wrist_pos( px, py, pz , Rrpy )
		
			theta_1, theta_2, theta_3 = get_theta_123( wx, wy, wz )

			R_0_3 = transform_to_wc( theta_1, theta_2, theta_3, Rrpy )
			theta_4, theta_5, theta_6 = get_theta_456( R_0_3, Rrpy )
			
			# Populate response for the IK request
			# In the next line replace theta1,theta2...,theta6 by your joint angle variables
			joint_trajectory_point.positions = [ theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 ]
			joint_trajectory_list.append(joint_trajectory_point)

			rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    init_transformation_matrices()
    rospy.spin()

if __name__ == "__main__":
    IK_server()
