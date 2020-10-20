clear;
close all;
clc;
% set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);

robot = Fetch('fetch');
ros_data = rosData;
fetch_motion = fetchMotion;

%%
xyz = ros_data.getPointCloud;

%%
t = GetLinkPoses(robot.model.getpos, robot);

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


%% create mesh
function [self,mesh] = createmesh(bricksPoses)
            %   Create and place 9 bricks from ply file
            %   input:      array of brick poses
            %   output1:    class object
            %   output2:    mesh array of type patch
            
            [f,v,data] = plyread('ply/Brick.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            for i=1:size(bricksPoses,1)
                mesh(i) = trisurf(f,v(:,1)+bricksPoses(i,1),v(:,2)+bricksPoses(i,2), v(:,3)+bricksPoses(i,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            end
            %%
            for i=1:length(robot.plyData)
                vertexColours{i} = [robot.plyData{1}.vertex.red, robot.plyData{1}.vertex.green, robot.plyData{1}.vertex.blue]/255;
            end
            
            f = robot.faceData;
            v = robot.vertexData;
            
            for i=1:size(bricksPoses,1)
                mesh(i) = trisurf(f,v(:,1)+bricksPoses(i,1),v(:,2)+bricksPoses(i,2), v(:,3)+bricksPoses(i,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            end
        end