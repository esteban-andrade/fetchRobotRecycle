clear;
close all;
clc;
set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);

addpath('../FetchRobot/');
addpath('../scripts/');

robot = Fetch('fetch');
%% Will update the position of the MATLAB instance in order to create accuracy on the motion in gazebo
robot.getGazeboState();
%%
robot.Move2Pose([0.2,0.3,0.6],50);
%% Use Fetch.m class to move robot (matlab and Gazebo) simultaneously

% move to pose
robot.Move2Pose([0.3,0.3,0.6],50);
robot.Move2Pose([0.5,0.7,0.2],50);
robot.Move2Pose([-0.5,0.7,0.6],50);

% Now move to joint configuration
robot.Move2JointState([0 0 0 0 0 0 0],50)
robot.Move2JointState([pi/4 0 0 0 0 pi/2 0],50)
robot.Move2JointState([pi/5 0.8 0 -1.2 0 pi/2 0],50)
robot.Move2JointState([pi/3 0.8 0 -pi/1.5 0 pi/2 0],50)

% Close and Open Gripper
robot.OpenGripper(false);
pause(2);
robot.OpenGripper(true);

%%

robot.RMRC2Pose(3,0.02,[0.2,0.3,0.6])

%%
robot.RMRC2Pose(3,0.02,[0.5,0.7,0.2])

%%

robot.RMRC2Pose(3,0.02,[-0.5,0.7,0.6])

%%

robot.RMRC2Pose(3,0.02,[0.3,0.3,0.6])

%%
robot.RMRC2JointState(3,0.02,[pi/4 0 0 0 0 pi/2 0])