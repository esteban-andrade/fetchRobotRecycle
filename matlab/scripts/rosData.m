classdef rosData
    %ROSDATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        can_position_sub;
        pc_sub;
    end
    
    methods
        function self = rosData
            
            
            disp('init')
        end
        
        function pose = getCanPosition(self)
            self.can_position_sub = rossubscriber('/aruco_single/position');
            can_position_msg = receive(self.can_position_sub);
            raw_pos = can_position_msg.Vector;
            pose = [raw_pos.X-0.02, raw_pos.Z+0.1, (raw_pos.Y + 1.0)];
        end
        
        function xyz = getPointCloud(self)
            self.pc_sub = rossubscriber('/head_camera/depth_registered/points');
            pc_msg = receive(self.pc_sub);
            xyz = readXYZ(pc_msg);
        end
    end
end

