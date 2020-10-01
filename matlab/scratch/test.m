close all
clear all
clc
%% This Script its only for testing purposes. The main one  will be located under main. 
% This  will only be use for development.




%%

addpath('../FetchRobot/');
addpath('../scripts/');


%%
%fetch = Fetch(false);
fetch = FetchRobot('fetch');
%fetch.model.teach()
set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);


p = zeros(1, 7);
animate(fetch.model,p)





%%

rosIP = '192.168.20.10';   % IP address of ROS-enabled machine  

rosinit(rosIP,11311); % Initialize ROS connection
%%
sub = rossubscriber('/joint_states');
pub1 = rospublisher('/matlab_joint_config','sensor_msgs/JointState')
pub2 = rospublisher('/matlab_gripper_action','std_msgs/Bool')
pause(1);
%%
gripper_status = false; 
gripper_msg = rosmessage('std_msgs/Bool');
gripper_msg.Data= gripper_status;
send(pub2,gripper_msg)


%%

msg = receive(sub,1)



%%
joints_names= msg.Name;

constrained_joints_names = joints_names(7:13);

get_jointstates = msg.Position(7:13);


%%

%%animate(fetch.model,get_jointstates')

%%


q1 = fetch.model.getpos
%%
q_matrix = interpolateJointAnglesFetch(q1,get_jointstates',50);

%%
motion(q_matrix,fetch.model)


%% Adjust Data

entire_joints = msg.Position;
%%
temp_matrix = entire_joints(7:13)-get_jointstates;
%%

entire_joints(7:13) = temp_matrix


%%
sent = rosmessage(pub1);
%sent.Position =entire_joints;


send(pub1,sent);
%%
rosshutdown



%%

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
