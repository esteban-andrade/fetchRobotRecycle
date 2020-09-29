close all
clear all
clc

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
pause(1);

msg = receive(sub,10)



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


%%


rosshutdown