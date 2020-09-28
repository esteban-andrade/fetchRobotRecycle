close all
clear all
clc
%%
fetch = Fetch(false);

%fetch.model.teach()

%%


%%

rosIP = '192.168.20.10';   % IP address of ROS-enabled machine  

rosinit(rosIP,11311); % Initialize ROS connection

sub = rossubscriber('/joint_states');
pause(1);

msg = receive(sub,10)



%%
joints_names= msg.Name;

constrained_joints_names = joints_names(7:13);

get_jointstates = msg.Position(7:13);

%%


q1 = fetch.model.getpos
%%
q_matrix = interpolateJointAnglesFetch(q1,get_jointstates',50);





%%


rosshutdown