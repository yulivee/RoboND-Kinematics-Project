#!/usr/bin/env python
from sympy import *
from sympy.matrices import Matrix
import numpy as np

class IK:

    def __init__(self,R0_3, symbols, q1, q2, q3, R_corr):
        self.end_effector_length =  0.303 #0.453
        self.Rrpy = None
        self.a_1 = 0.35
        self.a_2 = 1.25
        self.d_1 = 0.75
        self.d_4 = 1.50
        self.d_6 = 0
        self.a_3 = -0.054
        self.R0_3 = R0_3
	self.q1 = q1
	self.q2 = q2
	self.q3 = q3
        self.symbols = symbols
	self.R_corr = R_corr
	self.last_t4 = 0.1
	self.last_t5 = 0.1
	self.last_t6 = 0.1


    # Build a rotation matrix from the Roll, Pitch and Yaw angles
    def get_wrist_rot_matrix(self, roll, pitch, yaw):

        #R_z =  Matrix([[     cos(yaw), -sin(yaw),             0],
        #               [     sin(yaw),  cos(yaw),             0],
        #               [              0,           0,             1]])

        #R_y =  Matrix([[  cos(pitch),           0, sin(pitch)],
        #               [              0,           1,             0],
        #               [ -sin(pitch),           0, cos(pitch)]])
    
        #R_x = Matrix([[ 1,              0,        0],
        #             [ 0,        cos(roll), -sin(roll)],
        #             [ 0,        sin(roll),  cos(roll)]])
    
        #Rrpy = simplify ( R_z * R_y * R_x )
            
     
        # Rrpy = Rz * Ry * Rx. Values precalculated to save precious time
        Rrpy = Matrix([[    cos(yaw)*cos(pitch),   cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll),    cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll)],
                         [    sin(yaw)*cos(pitch),   sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll),    sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll)],
                         [            -sin(pitch),             cos(pitch)*sin(roll),                                       cos(pitch)*cos(roll)               ]])

        self.Rrpy = Rrpy
        return 

    def get_wrist_pos(self, px, py, pz):
               
        Rrpy = self.Rrpy

        lx = Rrpy[ 0, 0 ]
        ly = Rrpy[ 1, 0 ]
        lz = Rrpy[ 2, 0 ]


        # Calculate Wrist Center
        wx = px - ( self.end_effector_length + self.d_6 ) * lx
        wy = py - ( self.end_effector_length + self.d_6 ) * ly
        wz = pz - ( self.end_effector_length + self.d_6 ) * lz
        
        # Modify Rrpy to take in transformation from gripper
        Rrpy = Rrpy.row_join(Matrix([[px],[py],[pz]]))
        Rrpy = Rrpy.col_join(Matrix([[0,0,0,1]]))
        self.Rrpy = Rrpy

        return wx, wy, wz

    #Calculate joint angles using Geometric IK method
    def get_theta_123 (self, wx, wy, wz):
        
        theta1  = atan2(wy, wx).evalf()
        
	r = sqrt( wx**2 + wy**2) - self.a_1
        z_c = wz - self.d_1 # Subtract d1 as vertical offset from robot base
        
	L_25 = sqrt(r**2 + z_c**2)
        L_35 = sqrt(self.a_3**2+ self.d_4**2)
        
        beta_2 = atan2( z_c, r )
        cos_beta_1 = ( ( L_35**2 - L_25**2 - self.a_2**2 ) / ( - 2 * L_25 * self.a_2 ) )
        beta_1 = atan2(  sqrt( 1 - cos_beta_1**2 ), cos_beta_1 ) 
        theta2 = np.pi/2 - beta_1 - beta_2
        
        beta_4 = atan2( - self.a_3, self.d_4)
        cos_beta_3 = ( L_25**2 - self.a_2**2 - L_35**2) / ( -2 * self.a_2 * L_35 )
        beta_3 = atan2( sqrt( 1 - cos_beta_3**2 ), cos_beta_3 )
        theta3 = np.pi/2 - beta_3 - beta_4


        return theta1.evalf(), theta2.evalf(), theta3.evalf()

    def transform_to_wc(self, theta1, theta2, theta3):
     
        R0_3 = self.R0_3
        R0_3 = R0_3.evalf(subs={self.q1: theta1, self.q2: theta2, self.q3: theta3})
        R3_6_eval = R0_3.inv() * self.Rrpy * self.R_corr
        return R3_6_eval        

    def get_theta_456(self, R3_6_eval):
        r12 = R3_6_eval[0,1]
        r13 = R3_6_eval[0,2]

        r21 = R3_6_eval[1,0]
        r22 = R3_6_eval[1,1]
        r23 = R3_6_eval[1,2]

        r32 = R3_6_eval[2,1]
        r33 = R3_6_eval[2,2]
        
	# when theta5 is 0 we encounter a wrist singularity and cannot obtain theta4 and theta6 from r33/r13 r22/r21
        # in this case we set theta6 to its last value, theta 5 to zero and calculate theta4 in correspondence to theta6
	if np.abs(r23) is not 1:
            sin_q5 = sqrt(r13**2 + r33**2).evalf()
            theta5 = atan2( sin_q5, r23 ).evalf()
            if( sin_q5 < 0 ):
                theta4 = atan2( -r33,  r13).evalf()
                theta6 = atan2(  r22, -r21).evalf()
            else:
                theta4 = atan2(  r33, -r13).evalf()
                theta6 = atan2( -r22,  r21).evalf()
        else:
            theta6 = self.last_t6
	    if ( r23 == 1  ):
	        theta5 = 0
                theta4 = -theta6 + atan2( -r12,  -r32).evalf()
	    else:
                theta5 = 0
                theta4 = theta6 + atan2( r12,  -r32).evalf()
        
        self.last_t4 = theta4
        self.last_t5 = theta5
        self.last_t6 = theta6
        
        return theta4, theta5, theta6
