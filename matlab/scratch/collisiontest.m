clear;
close all;
clc;
% set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);

robot = Fetch('fetch');
fetch_motion = fetchMotion;

%%
xyz = robot.ros_data.getPointCloud;
robot.ros_data.plotPointCloud

%%
t = GetLinkPoses(robot.model.getpos, robot);

x1 = min(xyz(:,1));
y1 = min(xyz(:,2));
z1 = min(xyz(:,3));
x2 = max(xyz(:,1));
y2 = max(xyz(:,2));
z2 = max(xyz(:,3));

for i=3:length(t)
    T = t(:,:,i);
    if T(1,4) >= x1 && T(1,4) <= x2
        if T(2,4) >= y1 && T(2,4) <= y2
            if T(3,4) >= z1 && T(3,4) <= z2
                disp('collision');
                break
            end
        end
    end
    
    if i == length(t)
        disp('no collision');
    end
end

%%

x = xyz(:, 1);
y = xyz(:, 2);
z = xyz(:, 3);
inRangeX = x >= x1 & x <= x2;
inRangeY = y >= y1 & y <= y2;
inRangeZ = z >= z1 & z <= z2;
% Find out rows where it's in range on all three axes.
pointsInRange = inRangeX & inRangeY & inRangeZ;
% Extract those points in the box to an output array.
boxPoints = xyz(pointsInRange, :);

%%
hold on;
% New values for the ellipsoid (guessed these, need proper model to work out correctly)
centerPoint = [0,0,0];
radii = [0.08,0.08,0.08;
        0.08,0.08,0.08;
        0.08,0.08,0.25;
        0.08,0.2,0.08;
        0.08,0.08,0.25;
        0.08,0.15,0.08;
        0.08,0.08,0.16;
        0.06,0.06,0.05];
    
for i = 1:7
    if i == 4 || i==6 || i== 7 || i==8
        [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(i,1), radii(i,2), radii(i,3) );
        robot.model.points{i} = [X(:),Y(:),Z(:)];
        warning off
        robot.model.faces{i} = delaunay(robot.model.points{i});    
        warning on;
    end
end

robot.model.plot3d(robot.model.getpos);
axis equal
camlight
%%
hold on;
for i=1:size(t,3)
    trplot(:,:,i);
end



%% GetLinkPoses - from Lab5
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

links = robot.model.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.model.base;

    for i = 1:length(links)
        L = links(1,i);

        current_transform = transforms(:,:, i);

        current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
        transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
        transforms(:,:,i + 1) = current_transform;
    end
end


