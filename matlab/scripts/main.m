clear;
close all;
clc;
set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);

addpath('../FetchRobot/');
addpath('../scripts/');

robot = Fetch(false);
%% Lets test and compare joint configurations to that of gazebo
q = zeros(1,7);
robot.model.animate(q)
robot.model.teach


% still need to adjust link 7 to account for gazebo end-effector

%% Test trajectory
q1 = [-0.7754   -0.0560   -2.1892   -1.9326   -4.8977   -2.1664   -2.7673];
T2 = transl(-1.08, 0.03, 0.19);
q2 = robot.model.ikcon(T2,q1);

steps = 50;
qMatrix = jtraj(q1,q2,steps);

% Plot the results
for i=1:size(qMatrix,1)
    robot.model.animate(qMatrix(i,:));
    
    drawnow();
end

%% ROS
rosinit
%% gripper
gripper_pub = rospublisher('/matlab_gripper_action', 'std_msgs/Bool');
gripper_msg = rosmessage(gripper_pub);
gripper_msg.Data = 1;
send(gripper_pub,gripper_msg);

%% arm
arm_pub = rospublisher('/matlab_joint_config', 'sensor_msgs/JointState');
arm_msg = rosmessage(arm_pub);
arm_msg.Position = q;
send(arm_pub,arm_msg);
%%
rosshutdown
