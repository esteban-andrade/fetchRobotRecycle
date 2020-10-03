classdef Fetch < handle
    properties
        %> Robot model
        model;
        volume = [];
        name;
        t;
        deltaT;
        steps;
        delta;
        %RMRC Param
        epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
        W = diag([1 1 1 1 1 1]);   %  W * xdot
       
        
        %> workspace
        workspace = [-2, 2, -2, 2, -0.3, 2];
        % workspace = [-1.6 1.6 -1.6 1.6 -0.3 1];  %change workspace for easier approach
        
        %> If we have a tool model which will replace the final links model, combined ply file of the tool model and the final link models
        toolModelFilename = []; % Available are: 'DabPrintNozzleTool.ply';
        toolParametersFilename = []; % Available are: 'DabPrintNozzleToolParameters.mat';
    end
    
    properties %(Access = private)
        gripper_pub;
        gripper_msg;
        arm_pub;
        arm_msg;
    end

    methods %% Class for Fetch robot simulation


        function self = Fetch(acronym);
            self.name = ['Fetch ', acronym];
            self.GetFetchRobot(self.name);
            self.PlotAndColourRobot(); %

            drawnow

            rosshutdown;
            rosinit;
            
            % gripper publisher
            self.gripper_pub = rospublisher('/matlab_gripper_action', 'std_msgs/Bool');
            self.gripper_msg = rosmessage(self.gripper_pub);
            

            % arm publisher
            self.arm_pub = rospublisher('/matlab_joint_config', 'sensor_msgs/JointState');
            self.arm_msg = rosmessage(self.arm_pub);
            
        end

        %% GetFetchRobot
        % Given a name (optional), create and return a Fetch robot model
        function GetFetchRobot(self, name)
            pause(0.001);
            
            L(1) = Link('d',0.05,   'a',0.117,  'alpha',-pi/2,   'qlim',deg2rad([-92 92]),   'offset', 0);   %shoulder pan
            L(2) = Link('d',0,      'a',0,      'alpha',pi/2,   'qlim',deg2rad([-87 70]),   'offset',pi/2); %shoulder lift
            L(3) = Link('d',0.35,   'a',0,      'alpha',-pi/2,  'qlim',deg2rad([-360 360]), 'offset', 0);   %uperarm roll
            L(4) = Link('d',0,      'a',0,      'alpha',pi/2,   'qlim',deg2rad([-129 129]),'offset', 0);    %elbow flex
            L(5) = Link('d',0.32,   'a',0,      'alpha',-pi/2,   'qlim',deg2rad([-360,360]), 'offset',0);    %forearm roll
            L(6) = Link('d',0,      'a',0,      'alpha',pi/2,  'qlim',deg2rad([-125,125]), 'offset', 0);   %wrist flex
            L(7) = Link('d',0.15,   'a',0,      'alpha',0,      'qlim',deg2rad([-360,360]), 'offset', 0);   %wrist roll

            self.model = SerialLink(L,'name',name);

            % Rotate robot to the correct orientation
            self.model.base = transl([0 0 0.45]) * trotz(pi/2);
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
        
        %% Move Manipulator to Pose
        function Move2Pose(self, pose,setSteps)
            steps = setSteps;
            q1 = self.model.getpos;                                              % Derive joint angles for required end-effector transformation
            T2 = transl(pose);                                                   % Define a translation matrix
            q2 = self.model.ikcon(T2,q1);
            qMatrix = fetchMotion.interpolateJointAnglesFetch(q1,q2,steps);
            fetchMotion.motion(qMatrix,self)
        end
        
        %% Move Manipulator to Joint state
        function Move2JointState(self, q,setSteps)
            
            if length(q) == 7
                steps = setSteps;
                q1 = self.model.getpos;                                              % Derive joint angles for required end-effector transformation
                q2 = q;
                qMatrix = fetchMotion.interpolateJointAnglesFetch(q1,q2,steps);
                fetchMotion.motion(qMatrix,self)
            else
                disp('Too few or too many elements in q');
            end
        end
        %%
        function RMRC2Pose(self,time,deltaTime,pose)
            self.t = time;
            self.deltaT=deltaTime;
            self.steps = self.t/self.deltaT;
            self.delta=2*pi/self.steps;
            q1 = self.model.getpos;                                              % Derive joint angles for required end-effector transformation
            T2 = transl(pose);                                                   % Define a translation matrix
            q2 = self.model.ikcon(T2,q1);
            
            qMatrix = fetchMotion.RMRCPose(self,T2);
         
            self.model.plot(qMatrix,'trail','r-')
            
            
        end
        %%
        function RMRC2JointState(self)
            
        end
        
        
        
        %% Open/Close Gripper
        function OpenGripper(self, state)
            self.gripper_msg.Data = state;
            send(self.gripper_pub,self.gripper_msg);
            pause(0.02);
        end
    end
end