clear;
close all;
clc;
set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);

addpath('../FetchRobot/');
addpath('../scripts/');
addpath('../GUI/');

robot = Fetch('fetch');
ros_data = rosData;
fetch_motion = fetchMotion;
gui = app1;
gui.addRobot(robot);
%% Will update the position of the MATLAB instance in order to create accuracy on the motion in gazebo
robot.getGazeboState();

% robot.Move2JointState([-1.2812 0.1257 -3.0902 1.7200 0.0000 2.0091 0.0000],50)
%% point cloud
ros_data.getPointCloud; 

%% Move to can pose

pose = ros_data.getCanPosition;
robot.OpenGripper(1)
time = fetch_motion.calculateTime((robot.model.fkine(robot.model.getpos)), pose);
robot.RMRC2Pose(time,0.02,[pose(1), pose(2), pose(3)+0.2]);

time = fetch_motion.calculateTime(robot.model.fkine(robot.model.getpos), pose);
robot.RMRC2Pose(time,0.02,pose);

% pick up can and raise
% while ros_data.checkForMatchingQ(robot, robot.model.getpos) == 0
%     pause(1);
%     ros_data.checkForMatchingQ(robot, robot.model.getpos)
% end
%%
robot.OpenGripper(0)
pause(1);

robot.RMRC2Pose(5,0.02,[pose(1), pose(2), pose(3)+0.2]);

%% Go to bin
bin_position = ros_data.getBinLocalPosition
time = fetch_motion.calculateTime(robot.model.fkine(robot.model.getpos), pose);
robot.Move2Pose([bin_position(1), bin_position(2), bin_position(3)],ceil(time/0.02));
pause(3)
robot.OpenGripper(1)
