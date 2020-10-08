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
ros_data.getPointCloud; 
pose = ros_data.getCanPosition

robot.RMRC2Pose(3,0.1,[pose(1), pose(2), pose(3)+0.15]);
robot.OpenGripper(1)
robot.RMRC2Pose(2,0.02,pose);

%% pick up can and raise
robot.OpenGripper(0)
pause(1);

robot.RMRC2Pose(5,0.2,[pose(1), pose(2), pose(3)+0.2]);

%% Go to bin
bin_position = ros_data.getBinLocalPosition
robot.RMRC2Pose(2,0.2,[bin_position(1), bin_position(2), bin_position(3)]);
%% pause(3)
robot.OpenGripper(1)