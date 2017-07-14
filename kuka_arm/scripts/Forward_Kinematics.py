#!/usr/bin/env python
from sympy import *
from sympy.matrices import Matrix
import numpy as np

class FK:

    def __init__(self):
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	
	self.q1 = q1
	self.q2 = q2
	self.q3 = q3
	self.q4 = q4
	self.q5 = q5
	self.q6 = q6

        # Joint angle symbols
        s = {alpha0:     0,  a0:      0, d1:  0.75,  
             alpha1: -pi/2,  a1:   0.35, d2:     0, q2: q2-pi/2, 
             alpha2:     0,  a2:   1.25, d3:     0,       
             alpha3: -pi/2,  a3: -0.054, d4:  1.50,
             alpha4:  pi/2,  a4:      0, d5:     0,
             alpha5: -pi/2,  a5:      0, d6:     0,
             alpha6:     0,  a6:      0, d7: 0.303, q7: 0}

        self.symbols = s

        # Modified DH params
        T0_1 = self.build_transformation_matrix( q1, alpha0, a0, d1 )
        T1_2 = self.build_transformation_matrix( q2, alpha1, a1, d2 )
        T2_3 = self.build_transformation_matrix( q3, alpha2, a2, d3 )
        T3_4 = self.build_transformation_matrix( q4, alpha3, a3, d4 )
        T4_5 = self.build_transformation_matrix( q5, alpha4, a4, d5 )
        T5_6 = self.build_transformation_matrix( q6, alpha5, a5, d6 )
        T6_G = self.build_transformation_matrix( q7, alpha6, a6, d7 )
        R_corr = self.get_gripper_correction()

        T0_2 = simplify( T0_1 * T1_2 )      # base_link to link_2
        T0_3 = simplify( T0_2 * T2_3 )      # base_link to link_3
        T0_4 = simplify( T0_3 * T3_4 )      # base_link to link_4
        T0_5 = simplify( T0_4 * T4_5 )      # base_link to link_5
        T0_6 = simplify( T0_5 * T5_6 )      # base_link to link_6
        T0_G = simplify( T0_6 * T6_G )      # base_link to link_G
        T0_G_corr = simplify(T0_G * R_corr) # Total HT between base_link and gripper_link with orientation correction applied

        self.T0_1 = T0_1
        self.T1_2 = T1_2
        self.T2_3 = T2_3
        self.T0_G_corr = T0_G_corr

        return

    def get_gripper_correction(self):
        #Correct the Gripper
        R_z =  Matrix([[     cos(np.pi), -sin(np.pi),             0, 0],
                       [     sin(np.pi),  cos(np.pi),             0, 0],
                       [              0,           0,             1, 0],
                       [              0,           0,             0, 1]])

        R_y =  Matrix([[  cos(-np.pi/2),           0, sin(-np.pi/2), 0],
                       [              0,           1,             0, 0],
                       [ -sin(-np.pi/2),           0, cos(-np.pi/2), 0],
                       [              0,           0,             0, 1]])

        R_corr = simplify(R_z * R_y)
        
        return R_corr

    def build_transformation_matrix( self, theta, alpha, a, d ):
        # Define Modified DH Transformation matrix
        T = Matrix([[            cos(theta),           -sin(theta),           0,             a ],
                    [ sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d ],
                    [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d ],
                    [                     0,                     0,           0,             1 ]]) 
        T = T.subs(self.symbols)
           
        return(T)

    def calculate_FK_pose( self, theta1, theta2, theta3, theta4, theta5, theta6 ):
    
        EEpos = self.T0_G_corr.evalf( subs={ self.q1: theta1, self.q2: theta2, self.q3: theta3, self.q4: theta4, self.q5: theta5, self.q6: theta6 } )
        Test = Matrix([[1.0],[1.0],[1.0],[1.0]])
	EEpos = simplify( EEpos * Test )
        ex = EEpos[0,0]
        ey = EEpos[1,0]
        ez = EEpos[2,0]
        return ex, ey, ez
