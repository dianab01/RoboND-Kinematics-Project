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
import numpy as np
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import matplotlib.pyplot as plt

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### FK code here
        # Create symbols
		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
		a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
		d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
		q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta

		# Numerical values for the non-zero parameters
		alpha1_val = -pi/2
		alpha3_val = -pi/2
		alpha4_val =  pi/2
		alpha5_val = -pi/2

		a1_val = 0.35
		a2_val = 1.25
		a3_val = -0.054

		d1_val = 0.75
		d4_val = 1.50
		d7_val = 0.303


		# Create Modified DH parameters
		s = {alpha0:     0, a0:    0,  d1:  0.75, q1:		  q1,
		     alpha1: -pi/2, a1: 0.35,  d2:     0, q2: -pi/2 + q2,
		     alpha2:     0, a2: 1.25,  d3:     0, q3:		  q3,
		     alpha3: -pi/2, a3:-0.054, d4:  1.50, q4:		  q4,
		     alpha4:  pi/2, a4:    0,  d5:     0, q5:		  q5,
		     alpha5: -pi/2, a5:    0,  d6:     0, q6: 		  q6,
		     alpha6:     0, a6:    0,  d7: 0.303, q7: 		   0}


		# Define Modified DH Transformation matrix
		def DH_TransformationMatrix(alpha, a, d, q):
			T = Matrix([[              cos(q),           -sin(q),            0,             a],
	                       [sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha), -sin(alpha)*d],
	                       [sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),  cos(alpha)*d],
	                       [                0,                 0,            0,             1]])
			return T
	    

	    # Create individual transformation matrices
		T0_1 = DH_TransformationMatrix(alpha0, a0, d1, q1).subs(s)
		T1_2 = DH_TransformationMatrix(alpha1, a1, d2, q2).subs(s)
		T2_3 = DH_TransformationMatrix(alpha2, a2, d3, q3).subs(s)
		T3_4 = DH_TransformationMatrix(alpha3, a3, d4, q4).subs(s)
		T4_5 = DH_TransformationMatrix(alpha4, a4, d5, q5).subs(s)
		T5_6 = DH_TransformationMatrix(alpha5, a5, d6, q6).subs(s)
		T6_G = DH_TransformationMatrix(alpha6, a6, d7, q7).subs(s)

		T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

		# Array for the position errors of the gripper, for plotting
		position_error = np.array([])

		# Extract rotation matrices from the transformation matrices

	        # Initialize service response
	        joint_trajectory_list = []
	        for x in xrange(0, len(req.poses)):
				# IK code starts here
				joint_trajectory_point = JointTrajectoryPoint()

				# Extract end-effector position and orientation from request
				# px,py,pz = gripper position
				# roll, pitch, yaw = gripper orientation
				px = req.poses[x].position.x
	 			py = req.poses[x].position.y
				pz = req.poses[x].position.z

				(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
	                [req.poses[x].orientation.x, req.poses[x].orientation.y,
	                    req.poses[x].orientation.z, req.poses[x].orientation.w])

	            ### IK code here

			    # Compensate for rotation discrepancy between DH parameters and Gazebo
				r, p, y = symbols('r p y')

				# Define rotation matrices 
				Rot_x =  Matrix([[ 1,         0,        0],
			                     [ 0,    cos(r),  -sin(r)],
			                     [ 0,    sin(r),   cos(r)]]) #Roll
			                  
				Rot_y = Matrix([[ cos(p),     0,  sin(p)],
			                    [      0,     1,       0],
			                    [-sin(p),     0,  cos(p)]]) #Pitch
			        
				Rot_z = Matrix([[ cos(y), -sin(y),     0],
			                    [ sin(y),  cos(y),     0],
			                    [ 0,            0,     1]]) #Yaw

				
			    # Compensate for rotation discrepancy between DH parameters and Gazebo
				Rot_err = Rot_z.subs(y, pi) * Rot_y.subs(p, -pi/2.) 

				# Compute rotation matrix of the gripper
				Rot_G = Rot_z * Rot_y * Rot_x

				# Account for the rotation discrepancy 
				Rot_G = Rot_G * Rot_err
			 	Rot_G = Rot_G.subs({'r': roll, 'p': pitch, 'y': yaw})

			 	# Get position of the gripper in matrix form
			 	G_pos = Matrix([[px], [py], [pz]])

			    # Calculate wrist center coordinates
				wc = G_pos - d7_val*Rot_G[:,2]
				
				# Calculate joint angles using Geometric IK method
				theta1 = atan2(wc[1], wc[0])

				#Calculate theta2 based on the triangle formed by origins of joints 2, 3 and the wrist center
				#Side of the triangle are:
				A_side = sqrt(a3_val * a3_val + d4_val * d4_val) 
				C_side = a2_val 
				B_side = sqrt(pow((sqrt(wc[0]*wc[0] + wc[1]*wc[1]) - a1_val), 2) + pow((wc[2] - d1_val), 2))#sqrt(r_c**2 + s_c**2)
				#Triangle's angles
				a_angle = acos((B_side*B_side + C_side*C_side - A_side*A_side) / (2*B_side*C_side))
				b_angle = acos((A_side*A_side + C_side*C_side - B_side*B_side) / (2*A_side*C_side))

			    
				theta2 = pi/2 - a_angle - atan2(wc[2] - d1_val, sqrt(wc[0]*wc[0] + wc[1]*wc[1]) - a1_val)
			    
				theta3 = pi/2 - b_angle - atan2(abs(a3_val), d4_val)

		    	#Rotation matrix from base link to the third link
				R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
				R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

				R3_6 = R0_3.T * Rot_G

				
			    #Get Euler Angles from the Rotation Matrices
			    # sin(theta4) / cos(theta4)
				theta4 = atan2(R3_6[2,2], -R3_6[0,2])
			    # sin(theta5) / cos(theta5)
				theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
			    # sin(theta6) / cos(theta6)
				theta6 = atan2(-R3_6[1,1], R3_6[1,0])

		        ###

		        # Populate response for the IK request
				joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
				joint_trajectory_list.append(joint_trajectory_point)


				# Gripper error
				FK = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

				# Position of the gripper, as from the Forward Kinematics
				calc_g_pose = [FK[0,3], FK[1,3], FK[2,3]] 

				err_g_x = abs(calc_g_pose[0] - px)
				err_g_y = abs(calc_g_pose[1] - py)
				err_g_z = abs(calc_g_pose[2] - pz)
				g_offset = sqrt(err_g_x**2 + err_g_y**2 + err_g_z**2)
				position_error = np.append(position_error, g_offset)

		
		# Plot in a figure the errors as computed from the Inverse Kinematics equations, compared to the actual positions
		#plt.figure()
		#plt.plot(position_error)
		#plt.show()

		rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # Initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
