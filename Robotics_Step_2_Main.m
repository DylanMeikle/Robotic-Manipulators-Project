%% Forward Kinamatics

clc; clear;
syms l1 l2 l3 d1 d2 d3 theta1 theta2 theta3
%DH table specific to our robot
DH =    [l1 d1 0 0;
        l2 0 0 theta2;
        l3 0 0 theta3];

%Each row is a different position to test
q = [0 pi/4 pi/4    %Pose 1
    25 0 pi/2       %Pose 2
    50 -pi/4 -pi/2];%Pose 3

%Each row is a set of link lengths for a robot
l = [1245 685 685 %Link Lengths set 1
    1000 250 250];%Link Lengths set 2

%Creating the robot object (they only vary by link lengths we supply as we
%give them both the same DH parameters
bot1 = manipulator(DH,l(1,:));
bot2 = manipulator(DH,l(2,:));

%Calculating Transformation matrices for first set of link lengths
T1_1 = bot1.fkine(q(1,:)) 
T2_1 = bot1.fkine(q(2,:))
T3_1 = bot1.fkine(q(3,:))



%Calculating Transformation matrices for second set of link lengths
T1_2 = bot2.fkine(q(1,:))
T2_2 = bot2.fkine(q(2,:))
T3_2 = bot2.fkine(q(3,:))

%Same as above but evaluated to decimal numbers for conveniance of reading
%results
T1_1 = eval(T1_1)
T2_1 = eval(T2_1)
T3_1 = eval(T3_1)

T1_2 = eval(T1_2) 
T2_2 = eval(T2_2)
T3_3 = eval(T3_2) 



%% Inverse Kinamatics 


%Calculating the inverse kinematics for pose 1
[d1 theta2 theta3] = bot1.ikine([1729.4 1169.4 0]);
%Transformation matrix calculated previouly to check results
T1_1
%Validating inverse kinematics results by running joint angles back through
%the forward kinematics to compare transformation matrices
eval(bot1.fkine([d1 theta2(1,1) theta3(1,1)]))
eval(bot1.fkine([d1 theta2(1,2) theta3(1,2)]))

%Calculating the inverse kinematics for pose 1
[d1 theta2 theta3] = bot1.ikine([1930 685 20]);
T2_1
eval(bot1.fkine([d1 theta2(1,1) theta3(1,1)]))
eval(bot1.fkine([d1 theta2(1,2) theta3(1,2)]))

%Calculating the inverse kinematics for pose 1
[d1 theta2 theta3] = bot1.ikine([1245 -968.7 50]);
T3_1
eval(bot1.fkine([d1 theta2(1,1) theta3(1,1)]))
eval(bot1.fkine([d1 theta2(1,2) theta3(1,2)]))

%% Jacobian


q = [0 pi/4 pi/4
    25 0 pi/2
    50 -pi/4 -pi/2];

l = [1245 685 685
    1000 250 250];
q_dot = [10 10 10] %mm/s rad/s rad/s

%Calculating Jacobians for link sets 1 and 2 without the poses assigned
J1 = bot1.Jacobian()
J2 = bot2.Jacobian();

%Assigning poses to the Jacobians for link length sets 1 and 2 in 3
%different poses
J1_1 = bot1.Jacobian(q(1,:))
J2_1 = bot2.Jacobian(q(1,:))

J1_2 = bot1.Jacobian(q(2,:))
J2_2 = bot2.Jacobian(q(2,:))

J1_3 = bot1.Jacobian(q(3,:))
J2_3 = bot2.Jacobian(q(3,:))


% J1_vel = eval(bot1.Jacobian(q(1,:))) * 
%Here we're calculating the angular and linear velocities for Jacobians
%J1_1 and J2_1 (Using link set 1 and link set 2, respectively) to compare
%how the different in link lengths impacts the velocities
Jv1_1 = J1_1(1:3,:);
Jw1_1 = J1_1(4:6,:);
v1_1 = eval(Jv1_1 * q_dot.') %.' just transposes them to column instead of row vectors
w1_1 = Jw1_1 * q_dot.'

Jv2_1 = J2_1(1:3,:);
Jw2_1 = J2_1(4:6,:);
v2_1 = eval(Jv2_1 * q_dot.')
w2_1 = Jw2_1 * q_dot.'