clear;
close all;
clc;
set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);

addpath('FetchRobot/');
addpath('scripts/');

robot = Fetch(false)
%% Lets test and compare joint configurations to that of gazebo
q = [0 pi/2 pi/4 -pi/2 0 -pi/4 0];
robot.model.animate(q)
robot.model.teach


% still need to adjust link 7 to account for gazebo end-effector