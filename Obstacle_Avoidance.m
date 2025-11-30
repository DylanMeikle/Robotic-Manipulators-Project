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
%epsilon = [1, pi/16, pi/16];            %Region of Convergence ->
epsilon = 1;
alpha = [1, pi/16, pi/16];              %Defines step size
%Tunable Parameters
zeta = [0.5 0.5 0.5];                         %Constant to be tuned for strengh of attractive field on a given joint 
nu = [5 5 5];                           %Constant to be tuned for strength of repulsive field on a given joint
d = 50;                                  %Distance that defines transition from conic to parabolic well
rho = 200;                                %Region of influence -> Distance that defines if repulsive forces act on a joint



%dt = 0.02;  %Size of time step
%time_tot = 10;  %Total time

tau = zeros(1, numJoints);              %Torques acting on joints
step = zeros(1, numJoints);             %Step size for each loop


%Should explain effets of parameters in the report for bonus marks

[T, A_Mats] = bot1.fkine();
J = bot1.Jacobian();
%A_Mats(:,:,2) = A_Mats(:,:,1)*A_Mats(:,:,2);
%A_Mats(:,:,3) = A_Mats(:,:,2)*A_Mats(:,:,3);

obs =   [5 5 5        %Centrifuge
        1225 770 550];              %Flask

path = 1;

%Changes which flask is treated as an obstacle 
if path == 1
    %In path 1, flask 2 is treated as obstacle
    obs(2,:) = [1225 970 500];      %Flask 2 - Obstacle
    pos(1,:) = [1225 770 550];      %Flask 1 - Starting Position
    pos(2,:) = [1225 750 1640];     %Goal
else
    %In path 2, flask 1 is treated as obstacle
    obs(2,:) = [1225 770 550];      %Flask 1 - Obstacle
    pos(1,:) = [1225 970 500];      %Flask 2 - Starting Position
    pos(2,:) = [1225 1000 2000];    %Goal
end


%Calculating startin joint angles based on EE starting position
q(1,:) = bot1.ikine(pos(1,:));
%Calculating final joint angles based on desired EE position
q(2,:) = bot1.ikine(pos(2,:));

%Preallocate space for 3, 3x3 matrices for the velocity jacobian for every
%origin
%3 rows (for velocity jacobian) x 3 columns (numJoints) x 3 matrices (for each origin = numjoint)
J = zeros(3,numJoints,numJoints);
J = Jv(bot1);
path_points = [];

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
while norm(q(1,:) - q(2,:)) > epsilon
   %Apply force field to each joint
   for joint = 1:numJoints
       
        % Get the position of every origin for every time step
        [origin_start, origin_next] = getOrigin(O,pos,joint,A_Mats);

    
        % Attractive Forces
        F_att(:,:,joint) = Attractive_Forces(origin_start, origin_next, d, zeta(joint));
       
        % Repulsive Forces
        F_rep(:,:,joint) = Repulsive_Force(pos(1,:),obs,rho,nu(joint),origin_start);
        
        %Calculate resultant torque/force on each joints
        
        tau(:,:,joint) = torques(F_att(:,:,joint), F_rep(:,:,joint), J(:,:,joint));
        
        %Step needs to hold 3 3x1 matrices (d1, theta2, and theta3 for each
        %joint
        step(:,joint) = tau(:,:,joint)/norm(tau(:,joint));
        q(1,joint) = q(1,joint) + step(:,joint)'.*alpha; 
   end
    
   
   [Trans, ~] = bot1.fkine(q(1,:));
   pos(1,:) = Trans(1:3,4)';
   path_points = [path_points; pos(1,:)];   % record EE position each step
    

end
path_points;
plotPath(path_points, pos, obs);

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
function F_att = Attractive_Forces(origin_start, origin_next, d, zeta)
    %Origin_start, Origin_next: 3x1
    %zeta: scalar
    if norm(origin_start - origin_next) <= d
        F_att = -zeta*(origin_start - origin_next); %Parabolic
    else
        F_att = -d*zeta*((origin_start - origin_next)/norm(origin_start - origin_next)); %Conic
    end
        
end


function F_rep = Repulsive_Force(pos, obs, rho, nu, origin_start)
    % pos: 1x3 current EE position
    % obs: Nx3 obstacle positions
    % rho: influence distance
    % nu: scalar
    F_rep = zeros(3,1);
    %If the position of the EE if within the bounds of an obstacle
    for i = 1:size(obs,1)
        dist = norm(pos - obs(i,:));
        if dist <= rho
            F_rep = F_rep + nu*((1/dist) - (1/rho))*(1/dist^2)*(origin_start - obs(i,:)'); 
        end
    end
    
end


function tau = torques(F_att, F_rep,J)
        
    %J = bot1.Jacobian(q);
    
    
    %Figure out how to set jacobians in this equation
    %tau_att = transpose(J)*transpose(F_att);
    %tau_rep = transpose(J)*transpose(F_rep);
    
    %tau = tau_att + tau_rep;

    % F_att_joint, F_rep_joint: 3x1
    % J: 3x3 velocity Jacobian for this joint
    tau = J.' * (F_att + F_rep).';


end

function J = Jv(bot1)
    
    z = zeros(3,1);

    J(:,:,1) = [cross(bot1.z(:,1),bot1.O(:,2)), z, z];
    J(:,:,2) = [cross(bot1.z(:,1),bot1.O(:,3)), cross(bot1.z(:,2),(bot1.O(:,3)-bot1.O(:,2))), z];
    J(:,:,3) = [cross(bot1.z(:,1),bot1.O(:,4)), cross(bot1.z(:,2),(bot1.O(:,4)-bot1.O(:,2))), cross(bot1.z(:,3),(bot1.O(:,4)-bot1.O(:,3)))];

end

function plotPath(path_points, pos, obs)
    figure;
    hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Manipulator End-Effector Path');

    % Obstacles
    scatter3(obs(:,1), obs(:,2), obs(:,3), 100, 'k', 'filled');
    % Start & goal
    scatter3(pos(1,1), pos(1,2), pos(1,3), 100, 'g', 'filled');
    scatter3(pos(2,1), pos(2,2), pos(2,3), 100, 'r', 'filled');

    % Path
    plot3(path_points(:,1), path_points(:,2), path_points(:,3), 'b-', 'LineWidth', 2);

    % Optional animation
    hEE = scatter3(NaN, NaN, NaN, 60, 'b', 'filled');
    for i = 1:size(path_points,1)
        set(hEE, 'XData', path_points(i,1), 'YData', path_points(i,2), 'ZData', path_points(i,3));
        drawnow;
        pause(0.02);
    end
end



%{ 
%Some Commands I've been using for testing
syms d1 theta2 theta3
z0 = [0;0;1];
o0 = [0;0;0];
z1 = [0;0;1];
o1 = [1245;0;d1];
z2 = z1;
z3 = z2;
o2 = [685*cos(theta2) + 1245;685*sin(theta2);d1];
o3 = [685*cos(theta2 + theta3) + 685*cos(theta2) + 1245;685*sin(theta2 + theta3) + 685*sin(theta2);d1];
Jv03 = [cross(z0,o3), cross(z1,(o3-o1)), cross(z2,(o3-o2))]



%}

