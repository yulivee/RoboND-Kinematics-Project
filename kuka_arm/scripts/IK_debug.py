from sympy import *
from time import time
from mpmath import radians
import tf
from Forward_Kinematics import FK
from Inverse_Kinematics import IK

'''
Format of test case is [ [[EE position],[EE orientation as rpy]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generate test cases can be added to the test_cases dictionary
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[[[2.3099,0          ,1.9465      ], # EE Position
                  [0     ,-0.00014835,0     ,1    ]],# EE orientation in quaternions
                  [1.8499,0          ,1.9464      ], # WC location
                  [0     ,0          ,0     ,0,0,0]],# joint angles
              5:[[[-0.16657, -1.5542    , 1.4995                    ], # EE Position
                  [0.32196 , -0.28668   , -0.74205 ,0.51334         ]],# EE orientation in quaternions
                  [-0.32358, -1.5542    , 1.4995                    ], # WC location
                  [1.38    , -0.16      , -3.30 ,0.54 ,0.16 ,3.51   ]],# joint angles
              6:[[[-0.5640 , 0.75628    , 3.6653                    ], # EE Position
                  [-0.0889 , -0.58475   , 0.42441  ,0.68558         ]],# EE orientation in quaternions
                  [-0.7211 , 0.75628    , 3.6653                    ], # WC location
                  [2.48    , 0.17       , -1.55 ,-1.03 ,0.65 ,-0.88 ]],# joint angles
             }


def test_code(test_case):
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    ########################################################################################
    ## Insert IK code here starting at: Define DH parameter symbols
    myIK = IK(myFK.R0_3, myFK.symbols, myFK.q1, myFK.q2, myFK.q3, myFK.R_corr)
    
    # Extract end-effector position and orientation from request
    px = position.x
    py = position.y
    pz = position.z

    print "Px, Py, Pz provided by testdata (udacity)"
    print "px:   [ %7.4f ] py:    [ %7.4f ] pz:   [ %7.4f ]" % ( px, py, pz )

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y,
                orientation.z, orientation.w])

    print "Roll, Pitch, Yaw provided by testdata (udacity)"
    print "roll: [ %7.4f ] pitch: [ %7.4f ] yaw:  [ %7.4f ]" % ( roll, pitch, yaw )
    print ""

    myIK.get_wrist_rot_matrix( roll, pitch, yaw )

    wx, wy, wz = myIK.get_wrist_pos( px, py, pz )
    print "Calculated values for WC (mine)"
    print "WC_x: [ %7.4f ] WC_y:  [ %7.4f ] WC_z: [  %7.4f ]" % ( wx, wy, wz )
    print "Expected values for WC (udacity)"
    print "WC_x: [ %7.4f ] WC_y:  [ %7.4f ] WC_z: [  %7.4f ]" % ( test_case[1][0], test_case[1][1], test_case[1][2] )
    print ""

    theta1, theta2, theta3 = myIK.get_theta_123( wx, wy, wz )
    print "Calculated values for Theta1 - Theta3 (mine)"
    print "theta1: [ %7.4f ] theta2: [ %7.4f ] theta3: [ %7.4f ]" % ( theta1, theta2, theta3 )
    print "Expected values for Theta1 - Theta3 (udacity)"
    print "theta1: [ %7.4f ] theta2: [ %7.4f ] theta3: [ %7.4f ]" % ( test_case[2][0], test_case[2][0], test_case[2][0] )
    print ""

    R3_6 = myIK.transform_to_wc( theta1, theta2, theta3 )
    theta4, theta5, theta6 = myIK.get_theta_456( R3_6 )
    print "Calculated values for Theta4 - Theta6 (mine)"
    print "theta4: [ %7.4f ] theta5: [ %7.4f ] theta6: [ %7.4f ]" % ( theta4, theta5, theta6 )
    print "Expected values for Theta4 - Theta6 (udacity)"
    print "theta4: [ %7.4f ] theta5: [ %7.4f ] theta6: [ %7.4f ]" % ( test_case[2][3], test_case[2][4], test_case[2][5] )
    print ""


    ## Ending at: Populate response for the IK request
    ########################################################################################
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ex, ey, ez = myFK.calculate_FK_pose( theta1, theta2, theta3, theta4, theta5, theta6 )
    j5x, j5y, j5z = myFK.calculate_J5_pose( theta1, theta2, theta3, theta4, theta5, theta6 )
    print "Joint5 (mine)"
    print "J5_x: [ %7.4f ] J5_y:  [ %7.4f ] J5_z: [  %7.4f ]" % ( j5x, j5y, j5z )

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [ wx, wy, wz ] # <--- Load your calculated WC values in this array
    your_ee = [ ex, ey, ez ] # <--- Load your calculated end effector value from your forward kinematics
    # your_ee = [ 1, 1, 1 ] # <--- Load your calculated end effector value from your forward kinematics
    ######################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple posisiotns. It is best to add your forward kinmeatics to \
           \nlook at the confirm wether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    print "Pre-Calculating Transformation Matrices"
    myFK = FK()
    print "Starting Testcases"
    for test_case_number in range(1, 2):    
        print "---------------------------- Testcase Number ", test_case_number, " ----------------------------"
        test_code(test_cases[test_case_number])
