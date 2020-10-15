classdef rosData
    %ROSDATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        can_position_sub;
        pc_sub;
        odom_sub;
    end
    
    methods
        function self = rosData
            
            
            disp('init')
        end
        
        function pose = getCanPosition(self)
            self.can_position_sub = rossubscriber('/aruco_single/position');
            can_position_msg = receive(self.can_position_sub);
            raw_pos = can_position_msg.Vector;
            pose = [raw_pos.X-0.02, raw_pos.Z+0.1, (raw_pos.Y + 0.99)];
        end
        
        function xyz = getPointCloud(self)
            self.pc_sub = rossubscriber('/head_camera/depth_registered/points');
            pc_msg = receive(self.pc_sub);
            xyz = readXYZ(pc_msg);
            xyz = [xyz(:,1) xyz(:,3) xyz(:,2)];
            xyz(:,3) = -xyz(:,3)+1.3;
            
            p = -0.3840;
            xyz(:,2) = xyz(:,2)*cos(p)-xyz(:,3)*sin(p);
            xyz(:,3) = xyz(:,2)*sin(p)+xyz(:,3)*cos(p);
            xyz(:,2) = xyz(:,2) -0.35;
            hold on;
            pcshow(xyz);
            xlabel('X(m)')
            ylabel('Y(m)')
            zlabel('Z(m)')
            title('Detect a Cylinder in a Point Cloud')
            axis equal;
        end
        
        
        function local_bin_position = getBinLocalPosition(self)
            self.odom_sub = rossubscriber('/odom');
            odom_msg = receive(self.odom_sub);
            
            global_bin_position = [0.821506 4.180764];
            odom_msg.Pose.Pose.Position.Y = -odom_msg.Pose.Pose.Position.Y;
            y = global_bin_position(2) - odom_msg.Pose.Pose.Position.X;
            x = global_bin_position(1) - odom_msg.Pose.Pose.Position.Y;
            local_bin_position = [x y 1];
        end
    end
end

