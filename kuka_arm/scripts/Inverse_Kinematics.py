#!/usr/bin/env python
from sympy import *
from sympy.matrices import Matrix
import numpy as np

class IK:

    def __init__(self,R0_3, symbols, q1, q2, q3, R_corr):
        self.end_effector_length = 0.453 # 0.303 #0.453 #0.303
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
        
        theta1 = atan2(wy, wx)
        
	r = sqrt( wx**2 + wy**2) - self.a_1
        #x_c = wx - self.a_1 # Subtract a1 as horizontal offset from robot-base
        z_c = wz - self.d_1 # Subtract d1 as vertical offset from robot base
        
        #L_25 = sqrt(x_c**2 + z_c**2)
	L_25 = sqrt(r**2 + z_c**2)
        L_35 = sqrt(self.a_3**2+ self.d_4**2)
        
        beta_2 = atan2( z_c, r )
        #beta_2 = atan2( z_c, x_c )
        cos_beta_1 = ( ( L_35**2 - L_25**2 - self.a_2**2 ) / ( - 2 * L_25 * self.a_2 ) )
        #beta_1 = acos ( cos_beta_1 )
        beta_1 = atan2(  sqrt( 1 - cos_beta_1**2 ), cos_beta_1 ) 
        #beta_12 = atan2(  - sqrt( 1 - cos_beta_1**2 ), cos_beta_1 ) 
        theta2 = np.pi/2 - beta_1 - beta_2
	# flipped 
        
        beta_4 = atan2( - self.a_3, self.d_4)
        cos_beta_3 = ( L_25**2 - self.a_2**2 - L_35**2) / ( -2 * self.a_2 * L_35 )
        #beta_3 = acos ( cos_beta_3 )
        beta_3 = atan2( sqrt( 1 - cos_beta_3**2 ), cos_beta_3 )
        #beta_32 = atan2( - sqrt( 1 - cos_theta_3**2 ), cos_theta_3 )
        theta3 = np.pi/2 - beta_3 - beta_4


        return theta1.evalf(), theta2.evalf(), theta3.evalf()

    def transform_to_wc(self, theta1, theta2, theta3):
     
        R0_3 = self.R0_3
        R0_3 = R0_3.evalf(subs={self.q1: theta1, self.q2: theta2, self.q3: theta3})
        R3_6_eval = R0_3.inv() * self.Rrpy * self.R_corr
        return R3_6_eval        

    def get_theta_456(self, R3_6_eval):
        r13 = R3_6_eval[0,2]
        r33 = R3_6_eval[2,2]
        r23 = R3_6_eval[1,2]
        r22 = R3_6_eval[1,1]
        r21 = R3_6_eval[1,0]
        
        sin_q5 = sqrt(r13**2 + r33**2).evalf()
	print "sin(q5): ", sin_q5
        theta41 = atan2( -r33,  r13).evalf()
        theta61 = atan2(  r22, -r21).evalf()
        theta42 = atan2(  r33, -r13).evalf()
        theta62 = atan2( -r22,  r21).evalf()

	print "theta 4:", theta41, "alternative theta 4: ", theta42
	print "theta 6:", theta61, "alternative theta 6: ", theta62
               
        theta5 = atan2( sin_q5, r23 ).evalf()
        
        if( sin_q5 < 0 ):
            theta4 = atan2( -r33,  r13).evalf()
            theta6 = atan2(  r22, -r21).evalf()
        else:
            theta4 = atan2(  r33, -r13).evalf()
            theta6 = atan2( -r22,  r21).evalf()
        
        return theta4, theta5, theta6
