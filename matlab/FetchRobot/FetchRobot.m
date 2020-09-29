classdef FetchRobot < handle
    properties
        %> Robot model
        model;
        volume = [];
        computedVolume;
        transveralReach;
        verticalReach;
        arcRadius;
        name;
        %> workspace
        workspace = [-2, 2, -2, 2, -0.3, 2];
        % workspace = [-1.6 1.6 -1.6 1.6 -0.3 1];  %change workspace for easier approach
        %> If we have a tool model which will replace the final links model, combined ply file of the tool model and the final link models
        toolModelFilename = []; % Available are: 'DabPrintNozzleTool.ply';
        toolParametersFilename = []; % Available are: 'DabPrintNozzleToolParameters.mat';
    end

    methods %% Class for UR3 robot simulation


        function self = FetchRobot(acronym);
            self.name = ['Fetch ', acronym];
            self.GetFetchRobot(self.name);
            self.PlotAndColourRobot(); %

            drawnow

        end

        %% GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetFetchRobot(self, name)
            pause(0.001);
           
            %These were the default DH PARAMATERS provided from the manifacturer
    L(1) = Link('d',0.05,   'a',0.117,  'alpha',pi/2,   'qlim',deg2rad([-92 92]),   'offset', 0);   %shoulder pan
    L(2) = Link('d',0,      'a',0,      'alpha',pi/2,   'qlim',deg2rad([-87 70]),   'offset',deg2rad(-80)); %shoulder lift
    L(3) = Link('d',0.35,   'a',0,      'alpha',-pi/2,  'qlim',deg2rad([-360 360]), 'offset', pi);   %uperarm roll
    L(4) = Link('d',0,      'a',0,      'alpha',pi/2,   'qlim',deg2rad([-129 129]),'offset', 0);    %elbow flex
    L(5) = Link('d',0.32,   'a',0,      'alpha',pi/2,   'qlim',deg2rad([-360,360]), 'offset',pi);    %forearm roll
    L(6) = Link('d',0,      'a',0,      'alpha',-pi/2,  'qlim',deg2rad([-125,125]), 'offset',0);   %wrist flex
    L(7) = Link('d',0.15,   'a',0,      'alpha',0,      'qlim',deg2rad([-360,360]), 'offset', 0);   %wrist roll
           
%     L(1) = Link('d',0.05,   'a',0.117,  'alpha',pi/2,   'qlim',deg2rad([-92 92]),   'offset', pi/2);   %shoulder pan
%     L(2) = Link('d',0,      'a',0,      'alpha',pi/2,   'qlim',deg2rad([-87 70]),   'offset',0); %shoulder lift
%     L(3) = Link('d',0.35,   'a',0,      'alpha',-pi/2,  'qlim',deg2rad([-360 360]), 'offset', 0);   %uperarm roll
%     L(4) = Link('d',0,      'a',0,      'alpha',pi/2,   'qlim',deg2rad([-129 129]),'offset', -pi/2);    %elbow flex
%     L(5) = Link('d',0.32,   'a',0,      'alpha',pi/2,   'qlim',deg2rad([-360,360]), 'offset',pi);    %forearm roll
%     L(6) = Link('d',0,      'a',0,      'alpha',-pi/2,  'qlim',deg2rad([-125,125]), 'offset', -pi/2);   %wrist flex
%     L(7) = Link('d',0.15,   'a',0,      'alpha',0,      'qlim',deg2rad([-360,360]), 'offset', 0);   %wrist roll
%     
    self.model = SerialLink(L,'name',name);
    
            self.name = name;
             % Rotate robot to the correct orientation
    self.model.base = transl([0 0 0.45]);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self) %robot,workspace)
            for linkIndex = 0:self.model.n
                [faceData, vertexData, plyData{linkIndex + 1}] = plyread(['FetchLink', num2str(linkIndex), '.ply'], 'tri'); %#ok<AGROW>
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            if ~isempty(self.toolModelFilename)
                [faceData, vertexData, plyData{self.model.n + 1}] = plyread(self.toolModelFilename, 'tri');
                self.model.faces{self.model.n+1} = faceData;
                self.model.points{self.model.n+1} = vertexData;
                toolParameters = load(self.toolParametersFilename);
                self.model.tool = toolParameters.tool;
                self.model.qlim = toolParameters.qlim;
                warning('Please check the joint limits. They may be unsafe')
            end
            % Display robot
            self.model.plot3d(zeros(1, self.model.n), 'noarrow', 'workspace', self.workspace);
            if isempty(findobj(get(gca, 'Children'), 'Type', 'Light'))
                camlight
            end
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles, 'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex + 1}.vertex.red, ...
                        plyData{linkIndex + 1}.vertex.green, ...
                        plyData{linkIndex + 1}.vertex.blue] / 255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end

     


       
    end
end