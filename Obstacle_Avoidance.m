%Obstacle Avoidance code
clc; clear;
syms l1 l2 l3 d1 d2 d3 theta1 theta2 theta3

l = [1245 685 685]; %Link Lengths set 1
DH =    [l1 d1 0 0;
        l2 0 0 theta2;
        l3 0 0 theta3];

%Each row is a different position to test
q = [0 pi/4 pi/4    %Pose 1
    25 0 pi/2       %Pose 2
    50 -pi/4 -pi/2];%Pose 3
numJoints = 3;
O = zeros(1,2,numJoints);

%Potential Field Parameters 
U_att = zeros(1,2,numJoints);           %
F_att = zeros(1,2,numJoints);           %Force from attractive field on a given joint
q = zeros(1,3);                         %Current Position in 3D space
q_f = zeros(1,3);                       %Final Position in 3D space
zeta = zeros(1,numJoints);              %Constant to be tuned for strengh of attractive field on a given joint 
Nu = zeros(1,numJoints);                %Constant to be tuned for strength of repulsive field on a given joint
d = 0;                                  %Distance that defines transition from conic to parabolic well
rho = 0;                                %Distance that defines if repulsive forces act on a joint

%Should explain effets of parameters in the report for bonus marks

bot1 = manipulator(DH,l);

[T, A_Mats] = bot1.fkine();
T = eval(T);

[d1, theta2, theta3] = bot1.ikine(q(1,:));

J = bot1.Jacobian();
%or, if you want it for a specifc case:
%J = bot1.Jacobian(q(1,:))

pos =   [0, 0, 0;      %Position 1
        10, 10, 10];    %Position 2
obs =   [5, 5];

dt = 0.02;  %Size of time step
time = 10;  %Total time

for time = 0:dt:time
    %Apply force field to each joint
    for joint = 1:numJoints
        
        %Get the position of every origin
        O(:,:,joint) = A_Mats(4,1:3,joint);
        origin_start = subs(O(:,:,joint),[d1 theta2, theta3],pos(1,:));
        origin_final = subs(O(:,:,joint),[d1 theta2, theta3],pos(2,:));

        if norm(origin_start - origin_final) <= d
            F_att(:,:,joint) = -zeta(joint)*(origin_start - origin_final); %Parabolic
        else
            F_att(:,:,joint) = -d*zeta(joint)*((origin_start - origin_final)/norm(origin_start - origin_final)); %Conic
        end

         
        
    end

end

figure;
%p_pos = plot(obs)
p_obs = plot(obs(1), obs(2), 'ro', 'LineWidth', 2);
axis([0 10 0 10]);
