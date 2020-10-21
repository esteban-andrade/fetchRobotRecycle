classdef fetchMotion
    
    properties
        endEffector
    end
    
    methods(Static)
        function qMatrix = interpolateJointAnglesFetch(q1, q2, steps) % this function will get generate a trapezoidal trajectory. 
            %is more accurate than jtraj
            s = lspb(0, 1, steps); % First, create the scalar function
            qMatrix = nan(steps, 7); % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i, :) = (1 - s(i)) * q1 + s(i) * q2; % Generate interpolated joint angles
            end
        end
        
     %% Move arm and publish joint angles and velocities
        function motion(qMatrix, robot) %pass Q matrix and corresponding robot
            
            logData = ['transforms-data.mat']; % log data into a mat file
            %subcriber to people detection topic 
             people_sub = rossubscriber('/person_detector/status','std_msgs/Bool');
            for step = 1:size(qMatrix, 1) % iterate between rows of Q matrix
                
               
                if robot.collision.checkCollision == 0 % check for collision
                
                    if robot.gui.StartButton.Value == 1 %check for gui status 
                        msg = receive(people_sub);  %get message from rostopic 
                        human_detected_status = msg.Data; 
                        
                        if human_detected_status==false % if human detected false(environment is not safe)
                            % Set emergency subroutine
                            robot.gui.EmergencyStopButton.Value = 1;
                            robot.gui.setEstop;                 
                            robot.q_before_pause = [];
                            robot.active_traj = 0;
                            disp('Human Detected')
                        end

                        %animate(robot,qMatrix(step,:));
                        robot.model.animate(qMatrix(step, :));
                        endEffector = robot.model.fkine(qMatrix(step, :)); %#ok<NASGU>

                        %Logs the data in the file premade earlier
                        if (step == size(qMatrix, 1)) %if this is the last step of the arm

                            transformsData.name = robot.model.name;
                            transformsData.pose = robot.model.fkine(qMatrix(step, :));
                            text = [robot.name, ' End-Effector Transform '];
                            disp(text);
                            disp(transformsData.pose);

                            if isfile(logData)
                                %check if the file exists then add to the mat file
                                a = load(logData);
                                transformsData = [a.transformsData; transformsData]; %adds data to the end of the mat file
                                save(logData, 'transformsData');
                            else
                                %if the file does not exist, save
                                save(logData, 'transformsData');
                            end

                        end

                        
                        drawnow() %draw robot 
                        robot.arm_msg.Position = qMatrix(step,:); % send current joint config
                        robot.arm_msg.Velocity=1.2;     % sent static joint vel
                        send(robot.arm_pub,robot.arm_msg); 

                        % reset variables if last step in qMatrix is being
                        % executed for (GUI)
                        if step == size(qMatrix,1)
                            robot.q_before_pause = [];
                            robot.active_traj = 0;
                        end
                    else
                        if robot.gui.EmergencyStopButton.Value == 1
                            robot.q_before_pause = [];
                            robot.active_traj = 0;
                        else
                            robot.q_before_pause = qMatrix(end,:);
                        end
                        break;
                    end
                    robot.active_traj = 1;
                else
                    robot.gui.EmergencyStopButton.Value = 1;
                    robot.gui.setEstop;
                    robot.q_before_pause = [];
                    robot.active_traj = 0;
                    disp('collision detected')
                    break;
                end
            end
            robot.gui.updateAll;
        end
        
        %% Move arm and publish joint angles and velocities using RMRC
         function RMRCmotion(qMatrix,qdot, robot) %pass Q matrix and corresponding robot
            
            logData = ['transforms-data.mat']; % log data into a mat file
             %subcriber to people detection topic 
            people_sub = rossubscriber('/person_detector/status','std_msgs/Bool');
            for step = 1:size(qMatrix, 1) % iterate between rows of Q matrix
                
                if robot.collision.checkCollision == 0 % check for collision
                
                    if robot.gui.StartButton.Value == 1 % check for gui status 
                        %animate(robot,qMatrix(step,:));
                        msg = receive(people_sub);
                        human_detected_status = msg.Data;
                        
                        if human_detected_status==false % if human detected false(environment is not safe)
                            % Set emergency subroutine
                            robot.gui.EmergencyStopButton.Value = 1;
                            robot.gui.setEstop;
                            robot.q_before_pause = [];
                            robot.active_traj = 0;
                            disp('Human Detected')
                        end
                        
                        robot.model.animate(qMatrix(step, :));
                        endEffector = robot.model.fkine(qMatrix(step, :)); %#ok<NASGU>

                        %Logs the data in the file premade earlier
                        if (step == size(qMatrix, 1)) %if this is the last step of the arm

                            transformsData.name = robot.model.name;
                            transformsData.pose = robot.model.fkine(qMatrix(step, :));
                            text = [robot.name, ' End-Effector Transform '];
                            disp(text);
                            disp(transformsData.pose);

                            if isfile(logData)
                                %check if the file exists then add to the mat file
                                a = load(logData);
                                transformsData = [a.transformsData; transformsData]; %adds data to the end of the mat file
                                save(logData, 'transformsData');
                            else
                                %if the file does not exist, save
                                save(logData, 'transformsData');
                            end

                        end


                        drawnow()  %draw robot 
                        robot.arm_msg.Position = qMatrix(step,:); % send joint config
                        robot.arm_msg.Velocity=qdot(step,:)/5; % send jont vel with added gain of 0.2.
                        send(robot.arm_pub,robot.arm_msg);
                        
                       % reset variables if last step in qMatrix is being
                       % executed (for GUI)
                        if step == size(qMatrix,1)
                            robot.q_before_pause = [];
                            robot.active_traj = 0;
                        end
                    else
                        if robot.gui.EmergencyStopButton.Value == 1
                            robot.q_before_pause = [];
                            robot.active_traj = 0;
                        else
                            robot.q_before_pause = qMatrix(end,:);
                        end
                        break;
                    end
                    robot.active_traj = 1;
                else
                    robot.gui.EmergencyStopButton.Value = 1;
                    robot.gui.setEstop;
                    robot.q_before_pause = [];
                    robot.active_traj = 0;
                    disp('collision detected')
                    break;
                end
            end
            robot.gui.updateAll;
        end
        
        
        %%
        
        function [qMatrix,qdot] = RMRCPose(robot,targetPose)
            W = diag([1 1 1 1 1 1]);
            m = zeros(robot.steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(robot.steps,7);       % Array for joint angles
            qdot = zeros(robot.steps,7);          % Array for joint velocities
            theta = zeros(3,robot.steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,robot.steps);             % Array for x-y-z trajectory
            positionError = zeros(3,robot.steps); % For plotting trajectory error
            angleError = zeros(3,robot.steps);    % For plotting trajectory error
            
            initialPose = robot.model.fkine(robot.model.getpos); % get intial pose
            initialRPY = tr2rpy(initialPose); % get intial RPY
            targetRPY = tr2rpy(targetPose); % get target RPY
            
            s = lspb(0,1,robot.steps);   % trapezoidal trajectory 
            for i=1:robot.steps
                x(1,i) = (1-s(i))*initialPose(1, 4) + s(i)*targetPose(1, 4);% Points in x
                x(2,i) = (1-s(i))*initialPose(2, 4) + s(i)*targetPose(2, 4);% Points in y
                x(3,i) = (1-s(i))*initialPose(3, 4) + s(i)*targetPose(3, 4);% Points in z
                theta(1,i) = (1-s(i))*initialRPY(1) + s(i)*targetRPY(1);% Roll angle 
                theta(2,i) = (1-s(i))*initialRPY(2) + s(i)*targetRPY(2); % Pitch angle
                theta(3,i) = (1-s(i))*initialRPY(3) + s(i)*targetRPY(3); % Yaw angle
            end

            qMatrix(1,:)= robot.model.getpos; % set initial step to current pose
            
            for i = 1:robot.steps-1
                T=robot.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/robot.deltaT)*(Rd - Ra);                                       % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric
                linear_velocity = (1/robot.deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                            % Calculate end-effector velocity to reach next waypoint
                J = robot.model.jacob0(qMatrix(i,:));                                          % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < robot.epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/robot.epsilon)*5E-2;
                else
                    lambda = 0;
                end
                
                N = null(J);
                invJ = pinv(J'*J + lambda *eye(7))*J';  % DLS Inverse
                qdot(i,:) = (invJ*xdot)';  % Solve the RMRC equation (you may need to transpose the vector)
                for j = 1:7 % Loop through joints 1 to 7
                    % check status for joints 1,3,5 and add extra DLS for extra safety of motion.  
                    if j == 1 || j == 3 || j == 5
                        if qMatrix(i,j) >= 180
                            qMatrix(i,j) = mod(qMatrix(i,j), 180);
                        elseif qMatrix(i,j) <= -180
                            qMatrix(i,j) = mod(qMatrix(i,j), 180);
                        end
                    end
                    if qMatrix(i,j) + robot.deltaT*qdot(i,j) < robot.model.qlim(j,1) % If next joint angle is lower than joint lim
                        qdot(i,j) = 0; % stop motor
                    elseif qMatrix(i,j) + robot.deltaT*qdot(i,j) > robot.model.qlim(j,2)  % If next joint angle is greater than joint limit
                        qdot(i,j) = 0; %stop motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + robot.deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                              
                angleError(:,i) = deltaTheta;                
                
            end                                           
            
        end
        
        %% function to calculate the dynamic time based on distance to pose 
        function time= calculateTime(T, goalpose)
            % get distance between transform and target pose 
            dist = norm(T(1:3,4)' - goalpose(:));
            
            % constant and constrans 
            max_steps = 50;
            min_steps = 5;
            max_dist = 0.94;
            min_dist = 0.01;
            deltaT = 0.02;
            %eqation based on slope. Linear approach for result y =mx+b
            gain = (dist - min_dist)/max_dist; % get m 
            steps = gain*max_steps + min_steps; % resolve for number of steps 
            time = steps * deltaT; % get time to be used on RMRC
        end
        
    end
end

