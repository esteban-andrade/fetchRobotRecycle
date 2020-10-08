clear;
close all;
clc;
set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);

addpath('../FetchRobot/');
addpath('../scripts/');
addpath('../GUI/');

robot = Fetch('fetch');
robot.Move2Pose([0.2,0.3,0.6]);
%% Use Fetch.m class to move robot (matlab and Gazebo) simultaneously

% move to pose
robot.Move2Pose([0.3,0.3,0.6]);
robot.Move2Pose([0.5,0.7,0.2]);
robot.Move2Pose([-0.5,0.7,0.6])

% Now move to joint configuration
robot.Move2JointState([0 0 0 0 0 0 0])
robot.Move2JointState([pi/4 0 0 0 0 pi/2 0])
robot.Move2JointState([pi/5 0.8 0 -1.2 0 pi/2 0])
robot.Move2JointState([pi/3 0.8 0 -pi/1.5 0 pi/2 0])

% Close and Open Gripper
robot.OpenGripper(false);
pause(2);
robot.OpenGripper(true);

%% Lets test and compare joint configurations to that of gazebo
% q = zeros(1,7);
% robot.model.animate(q)
% robot.model.teach
% 
% 
% % still need to adjust link 7 to account for gazebo end-effector
% 
% %% Test trajectory
% q1 = [-0.7754   -0.0560   -2.1892   -1.9326   -4.8977   -2.1664   -2.7673];
% T2 = transl(-1.08, 0.03, 0.19);
% q2 = robot.model.ikcon(T2,q1);
% 
% steps = 50;
% qMatrix = jtraj(q1,q2,steps);
% 
% % Plot the results
% for i=1:size(qMatrix,1)
%     robot.model.animate(qMatrix(i,:));
%     
%     drawnow();
% end
% 
% %% ROS
% rosinit
% %% gripper publisher
% gripper_pub = rospublisher('/matlab_gripper_action', 'std_msgs/Bool');
% gripper_msg = rosmessage(gripper_pub);
% gripper_msg.Data = 1;
% send(gripper_pub,gripper_msg);
% 
% %% arm publisher
% arm_pub = rospublisher('/matlab_joint_config', 'sensor_msgs/JointState');
% arm_msg = rosmessage(arm_pub);
% arm_msg.Position = q1;
% send(arm_pub,arm_msg);
% 
% %% can pose subscriber
% sub = rossubscriber('/aruco_single/position');
% pause(1);
% msg2 = receive(sub,10)
% %%
% rosshutdown
