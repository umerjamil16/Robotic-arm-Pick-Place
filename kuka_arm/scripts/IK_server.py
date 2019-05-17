#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
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

from sympy import * #
from time import time
from mpmath import radians #

from sympy.interactive.printing import init_printing
from sympy import symbols, cos, sin, pi, simplify, atan2, acos, asin

from sympy.matrices import Matrix
init_printing(use_unicode=False, wrap_line=False)
from math import sqrt, pow

import numpy as np


def clip_angle(x, min, max):
    if x < min:
        x = min
    elif x > max:
        x = max
    return x

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        ### Create symbols for joint variables
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6, a7 = symbols('a0:8')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7 = symbols('alpha0:8')
        theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1:7')
        # DH Parameters
        s = {
            alpha0: 0,      a0:   0,         
            alpha1: -pi/2,  a1:   0.35,     d1: 0.75,   
            alpha2: 0,      a2:   1.25,     d2: 0,      
            alpha3: -pi/2,  a3:   -0.054,   d3: 0,      
            alpha4: pi/2,   a4:   0,        d4: 1.5,   
            alpha5: -pi/2,  a5:   0,        d5: 0,
            alpha6: 0,      a6:   0,        d6: 0,      
            alpha7: 0,      a7:   0,        d7:0.303
            }

            ### Define functions for Rotation Matrices about x, y, and z given specific angle.
        T0_1 = Matrix([
        [            cos(q1),            -sin(q1),            0,              a0],
        [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
        [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
        [                 0,                  0,            0,              1]
        ])

        #for the constant offest as per robot configuration//q2 = 

        T1_2 = Matrix([
        [            cos(q2 - pi/2),            -sin(q2 - pi/2),            0,              a1],
        [sin(q2 - pi/2)*cos(alpha1), cos(q2 - pi/2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
        [sin(q2 - pi/2)*sin(alpha1), cos(q2 - pi/2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
        [                 0,                  0,            0,              1]
        ])

        T2_3 = Matrix([
        [            cos(q3),            -sin(q3),            0,              a2],
        [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
        [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
        [                 0,                  0,            0,              1]
        ])

        T3_4 = Matrix([
        [            cos(q4),            -sin(q4),            0,              a3],
        [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
        [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
        [                 0,                  0,            0,              1]
        ])

        T4_5 = Matrix([
        [            cos(q5),            -sin(q5),            0,              a4],
        [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
        [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
        [                 0,                  0,            0,              1]
        ])

        T5_6 = Matrix([
        [            cos(q6),            -sin(q6),            0,              a5],
        [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
        [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
        [                 0,                  0,            0,              1]
        ])

        T6_G = Matrix([
        [            cos(0),            -sin(0),            0,              a7],
        [sin(0)*cos(alpha6), cos(0)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
        [sin(0)*sin(alpha6), cos(0)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
        [                 0,                  0,            0,              1]
        ])


        ## Homogeneous Transformation matrix from base_link to Gripper
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G


        # Create symbols
    #DONE
    #
    # Create Modified DH parameters
    #DONE
    #
    # Define Modified DH Transformation matrix
    #DONE
    #
    # Create individual transformation matrices
    #DONE
    #
    # Extract rotation matrices from the transformation matrices
    #
    #
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            r0G_0o = Matrix([
                [px],
                [py],
                [pz]
            ])

            r, p, y = symbols("r p y")
            ###these values are supposed to be from the ROS
            q = symbols("q")

            def rot_x(q):
                R_x = Matrix([[ 1,              0,        0],
                        [ 0,        cos(q), -sin(q)],
                        [ 0,        sin(q),  cos(q)]])

                return R_x.evalf(subs={q: q})
                
            def rot_y(q):              
                R_y = Matrix([[ cos(q),        0,  sin(q)],
                        [       0,        1,        0],
                        [-sin(q),        0,  cos(q)]])

                return R_y.evalf(subs={q: q})

            def rot_z(q):    
                R_z = Matrix([[ cos(q), -sin(q),        0],
                        [ sin(q),  cos(q),        0],
                        [ 0,              0,        1]])
                return R_z.evalf(subs={q: q})

            Rot_Error = rot_z(pi) * rot_y(-pi/2)


            ROT_EE = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * Rot_Error
            Rrpy = ROT_EE
            #= ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw}) ##this will give us end effector postion for given Roll, pitch, and yaw
        #    print(Rrpy)                

            r05_0G = Rrpy * Matrix([ [0], [0], [s[d7]] ])

            rWc_0o = r0G_0o - r05_0G 

        #    print(rWc_0o)

            WCx = rWc_0o[0,0]
            WCy = rWc_0o[1,0]
            WCz = rWc_0o[2,0]

            #From Figure 2:
            # #theta 1
            q1 = atan2(WCy, WCx)

            # For theta 2
            # From figure 3:

            side_a = s[d4]
            side_c = s[a2]

            side_b_x = sqrt(WCx*WCx + WCy*WCy) - s[a1]
            side_b_y = WCz - s[d1]
            side_b = sqrt(side_b_x*side_b_x + side_b_y*side_b_y) 


            phi1 = atan2(side_b_y, side_b_x)

            angle_a = acos( (side_c*side_c + side_b*side_b - side_a*side_a)/(2*side_b*side_c) )

            angle_b = acos( (side_c*side_c + side_a*side_a - side_b*side_b)/(2*side_a*side_c) )

            angle_c = acos( (side_b*side_b + side_a*side_a - side_c*side_c)/(2*side_a*side_b) )

            q2 = pi/2 - angle_a - phi1

            ##For theta 3
            #From figure 4

            phi2 = atan2(s[a3], s[d4])

            q3 = pi/2 - angle_b - phi2

            # Clipping angles as per URDF file
            theta1 = clip_angle(q1, radians(-185),  radians(185))
            theta2 = clip_angle(q2, radians(-45),   radians(85))
            theta3 = clip_angle(q3, radians(-210),  radians(155-90))

            #Remaining task:
            # 1. Clipping of the above angels according to URDF file

            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]

            R0_3 = R0_3.subs(s)
            R0_3 = R0_3.evalf(subs = {'q1': theta1, 'q2': theta2, 'q3': theta3})

            R3_6 = R0_3.transpose() * Rrpy

            R3_6_np = np.array(R3_6).astype(np.float64)

            q4 = atan2(R3_6_np[2,2], -R3_6_np[0,2])
            q5 = atan2(sqrt(R3_6_np[0,2]*R3_6_np[0,2] + R3_6_np[2,2]*R3_6_np[2,2]), R3_6_np[1,2])
            q6 = atan2(-R3_6_np[1,2], R3_6_np[1,0])
            # Euler angles from rotation matrix
            # More informaiton can be found in hte Euler Angles from a Rotation Matrix section
            theta4 = clip_angle(q4, radians(-350), radians(350))
            theta5 = clip_angle(q5, radians(-125), radians(125))
            theta6 = clip_angle(q6, radians(-350), radians(350))



        # Compensate for rotation discrepancy between DH parameters and Gazebo
        #
        #
        # Calculate joint angles using Geometric IK method
        #
        #
            ###

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