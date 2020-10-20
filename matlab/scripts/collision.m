classdef collision
    %COLLISION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        robot;
        pcloud;
    end
    
    methods
        function self = collision(robot, pcloud)
           self.robot = robot;
           self.pcloud = pcloud;
        end
        
        function checkCollision(self)
            [centerPoint, radii] = self.updateEllipse;
            
            q = self.robot.model.getpos;
            tr = self.robot.model.fkine(q);
            cubePointsAndOnes = [inv(tr) * [self.pcloud,ones(size(self.pcloud,1),1)]']';
            updatedCubePoints = cubePointsAndOnes(:,1:3);
            
            algebraicDist = ((updatedCubePoints(:,1)-centerPoint(1))/radii(1)).^2 ...
                      + ((updatedCubePoints(:,2)-centerPoint(2))/radii(2)).^2 ...
                      + ((updatedCubePoints(:,3)-centerPoint(3))/radii(3)).^2;
                  
            pointsInside = find(algebraicDist < 1);
            disp(['2.9: There are now ', num2str(size(pointsInside,1)),' points inside']);
        end
        
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
                    self.robot.model.points{i} = [X(:),Y(:),Z(:)];
                    warning off
    %                 self.robot.model.faces{i} = delaunay(self.robot.model.points{i});    
                    warning on;
                end
                
            end
            
%             self.robot.model.plot3d(self.robot.model.getpos);
        end
        
        function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

        algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                      + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                      + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end
    end
end

