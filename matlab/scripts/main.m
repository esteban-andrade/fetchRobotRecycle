clear;
close all;
clc;
set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);

addpath('../FetchRobot/');
addpath('../scripts/');

robot = Fetch('fetch');
ros_data = rosData;
%% Will update the position of the MATLAB instance in order to create accuracy on the motion in gazebo
robot.getGazeboState();
robot.Move2JointState([-1.2812 0.1257 -3.0902 1.7200 0.0000 2.0091 0.0000],50)
%% Move to can pose

pose = ros_data.getCanPosition

robot.RMRC2Pose(3,0.2,[pose(1), pose(2), pose(3)+0.2]);
robot.OpenGripper(1)
robot.RMRC2Pose(3,0.02,pose);

%% pick up can and raise
robot.OpenGripper(0)
pause(1);
robot.RMRC2Pose(3,0.1,[pose(1), pose(2), pose(3)+0.2]);

%% Need to get exact bin location. This will do for now
% drop left side
robot.RMRC2JointState(3,0.2,[ 0.9538 -0.6826 -6.2832 0.0376 -0.0005 2.1816 -1.8031]);

% drop right side
% robot.RMRC2JointState(3,0.2,[-0.9517 -0.7973 -6.2828 0.0316 -0.0003 2.1769 -1.5690]);
robot.OpenGripper(1)

