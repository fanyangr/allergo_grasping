# Brief Introduction to the Allegro hand code

Fan Yang Sep 9 2019



1. The structure of the code is as follows: First there is a higher level control thread called sai2 running constantly. It runs the state machine and writes the command torque to the global variable control_torque. While there is a lower level controller running another thread called ioThreadProc. This thread constantly computes the gravity compensation torques and adds it to the control torque, and finally sends that to the Allegro hand. My work mainly focuses on the Sai2 thread, while I still did some subtle modifications to the lower level thread.
2. The joint angle variables in this code has 2 formats, one for the lower lever driver,  using array to store the angles,  and the other one for Sai2, which use Eigen vector to store the variables. They have different orders and even different signs. We use the function sai2_to_driver and driver_to_sai2 to convert. 
3. The code is modified from an original Allegro hand grasping code. Since time is limited, there may be some little problems needed to be fixed.  For example, there may be some functions and variables that are useless and can be removed. I don't have time to do that when I interned there. Please feel free to delete some part of the code.

2. The Allegro hand model is essentially a branching robot. In practice, Since we only have the kinematic model of the whole hand, how can we perform an operational space controller on a specific finger tip. For example, how to configurate a certain finger to pregrasp configuration. We deal with this problem using the very simple method, just compute the joint torque as if it's not a branching robot, and then block the variables that have nothing to do with the target joints. This method work in some cases, but not all the cases. For example, if you want to do dynamic decomposition, this method doesn't work.

# FAQ 

Q1: Why 2 gravity compensations are used in this code？

A1: The one used in the lower level control loop is the gravity compensation provided by the BHand library, but they are not accurate enough, in order to compensate the error in the gravity compensation, we created another gravity compensation force vector ( It should be named gravity compensation error compensation though)

Q2: Why the mass and CoM in the urdf file is different from the actual value?

A2: Because as described in Q1, the second gravity compensation in Sai2 function is not actually the gravity compensation, but used to correct the error in the first gravity compensation provided by BHand library.

Q3: Why do we update the CoM?

A3: Ideally, the position of CoM shouldn't change, but during our experiment, we don't know for sure where the CoM is, so we estimate a CoM based on the grasping points.

Q4: Why do we have a friction compensation?

A4: Because we want to control to apply a tiny little force to the finger tip, then it has to compensate the friction. The variable may not be accurate, but It already works better than without it.

Q5: In FINGER_MOVE_CLOSE state, why do we use a function called make_contact rather than just write it out.

A5: Because I think we may reuse this function, but actually I didn't reuse it.

Q6: For detecting the surface normal, why do we first use the 2 fingers then the thumb?

A6: Because in this case, the unused finger may help to prevent the object from moving.

Q7: Why do we have a predefined normal flag?

A7: Because when debugging for grasping a box. It may be helpful to decompose the problem into several distinct parts.

Q8: Why do we have a check state?

A8: In this code we actually didn't have a check state, but we can use other theory to check whether the current grasp is a legal grasp or not.

Q9: Why do we compute the desired force again after we position our fingers and get ready to grasp?

A9: Because the actual grasping points may not be the points desired. So we do it again to get a more accurate force.

Q10: What's the bounce_back function and why do we use it?

A10: When applying the force, We want to maintain our contact points, because our forces are computed based on the current contact points. The bounce_back function tries to do the position control in the directions that are vertical to the desired force. We can try other approach, like compute the desired force based on the finger positions at each time step.

Q11: Why do we have a delay_counter?

A11: When we finish each step, we don't want to go to the next step immediately. Because it can damage the motor and lead to some great forces because of the inertia and they are hard to control.

Q12: Why do we have a q_offset?

A12: Because there is some joint slippages. For example, when you command the joint angle to be 0, but actually they may not be what you want. 

Q13: Why do we go to pregrasp configuration several times in the detect_surface_normal function?
A13: Because we wants to avoid singularities and joint limit.





TODO: 

1. It seems troublesome to compute the torque of the unused finger in each state, we can compute it at the last.
2. Delete the unused code to make it clearer.
3. Try sliding down approach to get more contact points
4. Try to optimize internal forces in the optimization part instead of the applying forces. Please ask Shameek about the details.
5. Find some ways to deal with the wire stiffness on the thumb



