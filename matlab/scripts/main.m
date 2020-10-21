clear;
close all;
clc;
set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);

addpath('../FetchRobot/');
addpath('../scripts/');
addpath('../GUI/');

robot = Fetch('fetch');
fetch_motion = fetchMotion;

%% Rember to press 'start' on GUI before running this

robot.Move2JointState([1.2812 0.1257 -3.0902 1.7200 0.0000 2.0091 0.0000],50)
%% point cloud
robot.obtainPcloud; 

%% Move to can pose

pose = robot.ros_data.getCanPosition;
robot.OpenGripper(1)
time = fetch_motion.calculateTime((robot.model.fkine(robot.model.getpos)), pose);
robot.RMRC2Pose(time,0.02,[pose(1), pose(2), pose(3)+0.2]);

time = fetch_motion.calculateTime(robot.model.fkine(robot.model.getpos), pose);
robot.RMRC2Pose(time,0.02,pose);
%
while(robot.active_traj == 0)
    pause(0.5);
end
%
robot.OpenGripper(0)
pause(1);

robot.RMRC2Pose(5,0.02,[pose(1), pose(2)-0.07, pose(3)+0.2]);

% Go to bin
while(robot.active_traj == 0)
    pause(0.5);
end
bin_position = robot.ros_data.getBinLocalPosition
time = fetch_motion.calculateTime(robot.model.fkine(robot.model.getpos), bin_position);
robot.Move2Pose([bin_position(1), bin_position(2), bin_position(3)],ceil(time/0.02));
while(robot.active_traj == 0)
    pause(1);
end
robot.OpenGripper(1)
