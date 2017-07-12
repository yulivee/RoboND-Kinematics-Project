#!/usr/bin/env python
from sympy import symbols, cos, sin, pi, sqrt, simplify
from sympy.matrices import Matrix
import numpy as np

class IK:

    def __init__(self):
        self.end_effector_length = 0.453 #0.303
        self.Rrpy = None
        self.a_1 = 0.35
        self.a_2 = 1.25
        self.d_1 = 0.75
        self.d_4 = 1.50
        self.d_6 = 0
        self.a_3 = -0.054
        self.T0_1 = None
        self.T1_2 = None 
        self.T2_3 = None
        self.symbols = None

    # Build a rotation matrix from the Roll, Pitch and Yaw angles
    def get_wrist_rot_matrix(self, roll, pitch, yaw):
            
        # Rrpy = Rx * Ry * Rz
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
        wx = px - ( self.end_effector_length + self.d6 ) * lx
        wy = py - ( self.end_effector_length + self.d6 ) * ly
        wz = pz - ( self.end_effector_length + self.d6 ) * lz
        
        # Modify Rrpy to take in transformation from gripper
        Rrpy = Rrpy.row_join(Matrix([[px],[py],[pz]]))
        Rrpy = Rrpy.col_join(Matrix([[0,0,0,1]]))
        self.Rrpy = Rrpy

        return wx, wy, wz

    #Calculate joint angles using Geometric IK method
    def get_theta_123 (self, wx, wy, wz):
        
        theta1 = atan2(wy, wx)
        
        x_c = sqrt( wx**2 + wy**2 ) - a_1 # Subtract a1 as horizontal offset from robot-base
        z_c = wz - d_1 # Subtract d1 as vertical offset from robot base

        L_25 = sqrt(x_c**2 + z_c**2)
        L_35 = sqrt(self.a_3**2 + self.d_4**2)

        cos_theta_3 = ( L_25**2 - self.a_2**2 - L_35**2) / ( 2 * self.a_2 * L_35 )
        #theta3 = atan2( cos_theta_3, sqrt( 1 - cos_theta_3**2 ))
        q3 = atan2( sqrt( 1 - cos_theta_3**2 ), cos_theta_3 )
        theta3 = ( q3 - np.pi/2 ).evalf()
        print("theta3: ", theta3)
        theta3_alt = ( -q3 + np.pi/2 ).evalf()
        print("theta3 alternative:", theta3_alt)
        
        # theta2 = atan2( z_c, x_c ) - atan2(a_2*sin(theta3), a_2+L_35*cos(theta3))
        # theta2b = atan2( z_c, x_c ) - atan2(a_2*sin(theta3b), a_2+L_35*cos(theta3b))
        beta_2 = atan2( z_c, x_c )
        cos_beta_1 = ( ( -L_35**2 + L_25**2 + self.a_2**2 ) / ( 2 * L_25 * self.a_2 ) )
        beta_1 = atan2(  cos_beta_1, sqrt( 1 - cos_beta_1**2 ) ) 
        theta2 = np.pi/2 - beta_1 - beta_2

        print("theta1,2,3:",theta1, theta2, theta3)
            
        return theta1.evalf(), theta2.evalf(), theta3.evalf()

    def transform_to_wc(self, theta1, theta2, theta3):
     
        R0_3 = simplify( T0_1 * T1_2 * T2_3 )
        #R0_3 = Matrix([[    sin(theta2 + theta3)*cos(theta1),   cos(theta1)*cos(theta2 + theta3),   -sin(theta1)],
                       #[   sin(theta1)*sin(theta2 + theta3),   sin(theta1)*cos(theta2 + theta3),   cos(theta1)],
                       #[   cos(theta2 + theta3),           -sin(theta2 + theta3),          0]])
        #print("R0_3:", R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3}))
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
        R3_6_eval = R0_3.inv() * self.Rrpy
        return R3_6_eval        

    def get_theta_456(self, R3_6_eval):
        r13 = R3_6_eval[0,2]
        r33 = R3_6_eval[2,2]
        r23 = R3_6_eval[1,2]
        r22 = R3_6_eval[1,1]
        r21 = R3_6_eval[1,0]
        
        #print("sin(q5): ", sin_q5_var)
        # sin(q5)² * ( cos(q4)² + sin(q4)²)
        # cos(q4)² + sin(q4)² = 1
        # => sqrt(r13² + r33²) = sin(q5)
        sin_q5 = sqrt(r13**2 + r33**2).evalf()
               
        theta5 = atan2( sin_q5, r23 ).evalf()
        
        if( sin_q5 < 0 ):
            theta4 = atan2(-r33,  r13).evalf()
            theta6 = atan2( r22, -r21).evalf()
        else:
            theta4 = atan2( r33,  -r13).evalf()
            theta6 = atan2( -r22, r21).evalf()
        
        return theta4, theta5, theta6
