clear;
close all;
clc;
set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);

addpath('FetchRobot/');
addpath('scripts/');

%robot = Fetch(false)
fetch = FetchRobot('fetch');

%% Lets test and compare joint configurations to that of gazebo
q = [0 pi/2 pi/4 -pi/2 0 -pi/4 0];
%robot.model.animate(q)
%robot.model.teach
animate(fetch.model,q)

% still need to adjust link 7 to account for gazebo end-effector
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
msg = receive(sub,1);

%%
joints_names= msg.Name;

constrained_joints_names = joints_names(7:13);

get_jointstates = msg.Position(7:13);
%%


q1 = fetch.model.getpos
q_matrix = interpolateJointAnglesFetch(q1,get_jointstates',50);
motion(q_matrix,fetch.model)

%%
p = zeros(1, 7);
joint_msg = rosmessage('sensor_msgs/JointState');
joint_msg.Position = p;
send(pub1,joint_msg);

%%
gripper_status = true;
gripper_msg.Data= gripper_status;
send(pub2,gripper_msg);

%%
rosshutdown