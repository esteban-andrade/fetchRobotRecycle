classdef Fetch < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-2 2 -2 2 -0.3 2];   
        
        %> Flag to indicate if gripper is used
        useGripper = false;        
    end
    
    methods%% Class for Fetch robot simulation
function self = Fetch(useGripper)
    self.useGripper = useGripper;
    
%> Define the boundaries of the workspace

        
% robot = 
self.GetFetchRobot();
% robot = 
self.PlotAndColourRobot();%robot,workspace);
%drawnow
end

%% GetFetchRobot
% Given a name (optional), create and return a Fetch robot model
function GetFetchRobot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['Fetch_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

    L(1) = Link('d',0.05,   'a',0.117,  'alpha',-pi/2,   'qlim',deg2rad([-92 92]),   'offset', 0);   %shoulder pan
    L(2) = Link('d',0,      'a',0,      'alpha',pi/2,   'qlim',deg2rad([-87 70]),   'offset',pi/2); %shoulder lift
    L(3) = Link('d',0.35,   'a',0,      'alpha',-pi/2,  'qlim',deg2rad([-360 360]), 'offset', 0);   %uperarm roll
    L(4) = Link('d',0,      'a',0,      'alpha',pi/2,   'qlim',deg2rad([-129 129]),'offset', 0);    %elbow flex
    L(5) = Link('d',0.32,   'a',0,      'alpha',-pi/2,   'qlim',deg2rad([-360,360]), 'offset',0);    %forearm roll
    L(6) = Link('d',0,      'a',0,      'alpha',pi/2,  'qlim',deg2rad([-125,125]), 'offset', 0);   %wrist flex
    L(7) = Link('d',0.15,   'a',0,      'alpha',0,      'qlim',deg2rad([-360,360]), 'offset', 0);   %wrist roll
    
    self.model = SerialLink(L,'name',name);
    
    % Rotate robot to the correct orientation
    self.model.base = transl([0 0 0.45]);
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['FetchLink',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['FetchLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end        
    end
end
