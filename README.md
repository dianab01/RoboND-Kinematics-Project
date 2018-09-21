## Project: Kinematics Pick & Place

---

**For the project implementation, the following has been performed:**  


1. ROS Workspace has been set.
2. Code from the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) has been cloned/ downloaded (in the ***src*** directory).  
3. Launch the simulation in demo mode (by setting demo flag to `true` in `inverse_kinematics.launch`, found in `src/RoboND-Kinematics-Project/kuka_arm/launch/`).
4. Kinematic Analysis for the robot, as described below.
6. Implementing and testing the kinematic equations derived. 


[//]: # (Image References)

[image1]: ./misc_images/img1.jpg
[image2]: ./misc_images/img2.jpg
[image3]: ./misc_images/img3.jpg
[image4]: ./misc_images/img4.jpg
[image5]: ./misc_images/2-dropping-object.png
[image6]: ./misc_images/2-initial-position.png
[image7]: ./misc_images/2-moving-to-shelf.png
[image8]: ./misc_images/2-reaching-drop-off.png


---

### Kinematic Analysis
#### 1. After running the forward_kinematics demo, from the URDF file containing the description of the Kuka KR210, the following analysis has been performed, which will be used to define the parameters:

![alt text][image1]

#### 2. From the image above, the following DH parameters table have been derived:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 |0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | pi/2 | 0 | 0 | q6
6->G | 0 | 0 | 0.303 | q7

A function as written below has been used to create the individual transformation matrices about the joints, based on the DH Parameter table.

```

def DH_TransformationMatrix(alpha, a, d, q):
			T = Matrix([[              cos(q),           -sin(q),            0,             a],
	                       [sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha), -sin(alpha)*d],
	                       [sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),  cos(alpha)*d],
	                       [                0,                 0,            0,             1]])
			return T
```

Individual matrices are created by calling the above function with the corresponding parameters, such as:
```

T0_1 = DH_TransformationMatrix(alpha0, a0, d1, q1).subs(s)
```

#### 3. In order to solve the Inverse Kinematics problem of the robotic arm, the parameters characterizing each joint have been computed, considering that the position of the end effector (and of the wrist center respectively) is known. The equations for theta1, theta2 and theta3 are solved geometrically, by appling `Sine and Cosine Laws`, as described in the images below.

`theta1` is calculated by projecting the wrist center, tight to the first joint, to the ground plane.
![alt text][image2]

This gives the formula: `theta1 = atan2(wc_y, wc_x)`.

`theta2` is taken from the plane defined by Z and the line representing the hypotenuse of the triangle (O,wc_x, wc_y), where O is the origin.
![alt text][image3]

While side A is the d4 parameter, the link offset of joint 4, and side C is a2, the link length between joint 2 and joint 3, side B is taken from the lower, right-side right-angled triangle:
```

B = sqrt(pow((sqrt(wc[0]*wc[0] + wc[1]*wc[1]) - a1_val), 2) + pow((wc[2] - d1_val), 2))
```

As for `theta3`, we assume that theta2 = 0, and on the same triangle as above, it represents the difference up to `pi/2` from angle b and the angle `gamma`, written as:
```

gamma = atan2(d4, a3)
```
![alt text][image4]

Both angle a and angle b have been determined by using the [Law of Sines](https://en.wikipedia.org/wiki/Law_of_sines):
```

a_angle = acos((B_side*B_side + C_side*C_side - A_side*A_side) / (2*B_side*C_side))
b_angle = acos((A_side*A_side + C_side*C_side - B_side*B_side) / (2*A_side*C_side))
```

Furthermore, theta4, theta5 and theta6 are taken directly from the rotation matrix, as it follows:
```

theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])

```  
where R0_3 represents the rotation between joint 3 and the gripper.


### Project Implementation

The implementation of the project was focused on creating a clear and easy to understand Inverse Kinematics analysis, while following the suggested main steps.

Values of the DH parameters have been declared separately so that the time taken to subtitute their values from the dictionary can be spared.
Whenever it was possible, it has been opted for matrix/ vectorial computations, such as it is the case of the wrist center coordinates.

At the same time, rather than computing the inverse of the `R0_3` matrix using the LU decomposition, it has been taken into account that rotation matrices are orthogonal, which means their inverse is represented by the transpose.

In  order to determine an error of the Inverse Kinematics approximation, the computed values are input to a Forward Kinematics analysis. Thus, the error between the expected end-effector position (which is given) and the computed position can be determined and plotted. The section of the code which plots these values (two for each object-moving cycle) has been left commented, since it affects the simulation speed on the virtual machine on which it was tested.

