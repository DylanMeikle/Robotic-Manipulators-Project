%% Obstacle Avoidance code

%% Setup
clc; clear;
syms l1 l2 l3 d1 d2 d3 theta1 theta2 theta3 real


l = [1245 685 685]; %Link Lengths set 1
DH =    [l1 d1 0 0;
        l2 0 0 theta2;
        l3 0 0 theta3];
bot1 = manipulator(DH,l);

%Each row is a different position to test

q = [0 pi/4 pi/4    %Starting Pose 
    0 0 0];         %Next Pose
numJoints = 3;
O = sym(zeros(3,1,numJoints));

%Potential Field Parameters 

F_att = zeros(1,3,numJoints);           %Force from attractive field on a given joint
F_rep = zeros(1,3,numJoints);           %Force from repulsive field on a given joint
q = zeros(2,3);                         %Current Position in 3D space
zeta = zeros(1,numJoints);              %Constant to be tuned for strengh of attractive field on a given joint 
Nu = zeros(1,numJoints);                %Constant to be tuned for strength of repulsive field on a given joint
d = 0;                                  %Distance that defines transition from conic to parabolic well
rho = 0;                                %Region of influence -> Distance that defines if repulsive forces act on a joint
epsilon = [1, pi/16, pi/16];            %Region of Convergence ->
alpha = [1, pi/16, pi/16];              %Defines step size

%dt = 0.02;  %Size of time step
%time_tot = 10;  %Total time

tau = zeros(1, 3, numJoints);              %Torques acting on joints
step = zeros(1, 3, numJoints);          %Step size for each loop


%Should explain effets of parameters in the report for bonus marks

[T, A_Mats] = bot1.fkine();

obs =   [5 5 5        %Centrifuge
        ];              %Flask

%Calculating starting position of EE based on initial joint angles
pos(1,:) = bot1.fkine(q(1,:));
%Calculating final joint angles based on desired EE position
pos(2,:) = [10, 10, 10];
q(2,:) = bot1.ikine(pos(2,:));



%% Execution


%{

For time step 1:
-Determine attractive forces on joint 1
-Determine repulsive forces on joint 1
-Determine torques acting on joint 1 from attactive and repulsive forces
... complete for all other joints
-Determine net torque on system 



%}


%for time = 0:dt:time_tot %Don't think this is the correct way to do this
while q(1,:) - q(2,:) < epsilon(:)
   %Apply force field to each joint
   for joint = 0:numJoints
       
        % Get the position of every origin for every time step
        [origin_start, origin_next] = getOrigin(O,numJoints,pos);
    
        % Attractive Forces
        F_att() = Attractive_Forces(joint, origin_start, origin_next, d, zeta(joint));
       
        % Repulsive Forces
        F_rep() = Repulsive_Force(joint,nu,rho,rho_min,origin_start);
        

        %Calculate resultant torque/force on each joints

        tau(:,:,joint) = torques(F_att, F_rep, q(1,:), joint);
        
        %Step needs to hold 3 3x1 matrices (d1, theta2, and theta3 for each
        %joint
        step(:,:,joint) = tau(:,:,joint)/norm(tau);
   end
    
   
   q(2,:) = q(1,:) + step*alpha;

end


%% Functions

%Function to get the current and next origin for a joint
function [origin_start, origin_next] = getOrigin(O,pos,joint,A_Mats)
   
    O(:,:,joint) = A_Mats(1:3, 4,joint);
    %If there's nothing to substitue this try/catch will solve that. 
    try
        origin_start = subs(O(:,:,joint),[d1, theta2, theta3],pos(1,:));
        origin_next = subs(O(:,:,joint),[d1, theta2, theta3],pos(2,:));
    catch
        origin_start = O(:,:,joint);
        origin_next = O(:,:,joint);
    end
    
end

%Function that returns attractive force on each joint
function F_att = Attractive_Forces(joint, origin_start, origin_next, d, zeta)

    if norm(origin_start - origin_next) <= d
        F_att(:,:,joint) = -zeta(joint)*(origin_start - origin_next); %Parabolic
    else
        F_att(:,:,joint) = -d*zeta(joint)*((origin_start - origin_next)/norm(origin_start - origin_next)); %Conic
    end
        
end


function F_rep = Repulsive_Force(joint,nu,rho,rho_min,origin_start)

    if rho_min <= rho
        %CONFIRM THIS FUNCTION. I AM NOT SURE IF I DID THIS RIGHT
        F_rep(:,:,joint) = nu(joint)*((1/rho_min) - (1/rho))*(1/rho_min^2)*((origin_start - rho_min)/norm(origin_start - rho_min));
    else
        F_rep(:,:,joint) = 0;
    end
    
end


function tau = torque(F_att, F_rep, q,joint)
        
    J = bot1.Jacobian(q);
    
    %Figure out how to set jacobians in this equation
    tau_att(:,:,joint) = transpose(J())*F_att(:,:,joint);
    tau_rep(:,:,joint) = transpose(J())*F_rep(:,:,joint);
    
    tau(:,:, joint) = tau_att(:,:,joint) + tau_rep(:,:,joint);

end


function path()
    for via = 1:size(Q_via,1)-1
    
        
        [a0,a1,a2,a3,a4,a5] = quin(Q_via(via,joint), Q_via(via+1, joint), tf);
        
        t = t_seg;
        %Need .^ to raising each individual matrix element 
        q = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
        qd = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
        qdd = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;

        pos_all(via,:,joint) = q;
        vel_all(via,:,joint) = qd;
        acc_all(via,:,joint) = qdd;
    
    
  
    
    end

end



