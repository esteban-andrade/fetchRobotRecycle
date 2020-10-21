classdef collision
    %COLLISION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        robot;
        pcloud;
    end
    
    methods
        %% Constructor
        % robot - seriallink robot model
        % pcloud - point cloud matrix (nx3)
        function self = collision(robot, pcloud)
           self.robot = robot;
           self.pcloud = pcloud;
        end
        
        %% check if collision has ocurred
        function result = checkCollision(self)
            % get link poses
            t = self.GetLinkPoses(self.robot.model.getpos, self.robot);

            % define minimum and maximum (xyz) search area
            x1 = min(self.pcloud(:,1));
            y1 = min(self.pcloud(:,2));
            z1 = min(self.pcloud(:,3));
            x2 = max(self.pcloud(:,1));
            y2 = max(self.pcloud(:,2));
            z2 = max(self.pcloud(:,3));

            % check if link pose is inside pointcloud
            for i=3:length(t)
                T = t(:,:,i);
                if T(1,4) >= x1 && T(1,4) <= x2
                    if T(2,4) >= y1 && T(2,4) <= y2
                        if T(3,4) >= z1 && T(3,4) <= z2
                            result = 1;
                            break
                        end
                    end
                end

                if i == length(t)
                    result = 0;
                end
            end
        end
        
        %% Update Ellipse (not used in final)
        function [centerPoint, radii] = updateEllipse(self)
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
%                     self.robot.model.points{i} = [X(:),Y(:),Z(:)];
                    warning off
                    self.robot.model.faces{i} = delaunay(self.robot.model.points{i});    
                    warning on;
                end
                
            end
            
%             self.robot.model.plot3d(self.robot.model.getpos);
        end
        
        %% Determine Algebraic distance (not used in final)
        function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

        algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                      + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                      + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end
        
        %% GetLinkPoses - from Lab5
        % q - robot joint angles
        % robot -  seriallink robot model
        % transforms - list of transforms
        function [ transforms ] = GetLinkPoses(self, q, robot)

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
    end
end

