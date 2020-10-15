classdef fetchMotion
    
    properties
        endEffector
    end
    
    methods(Static)
        function qMatrix = interpolateJointAnglesFetch(q1, q2, steps)
            s = lspb(0, 1, steps); % First, create the scalar function
            qMatrix = nan(steps, 7); % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i, :) = (1 - s(i)) * q1 + s(i) * q2; % Generate interpolated joint angles
            end
        end
     %%    
        function motion(qMatrix, robot) %pass Q matrix and corresponding robot
            
            logData = ['transforms-data.mat']; % log data into a mat file
            
            for step = 1:size(qMatrix, 1) % iterate between rows of Q matrix
                
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
                drawnow()
                robot.arm_msg.Position = qMatrix(step,:);
                send(robot.arm_pub,robot.arm_msg);
%                 pause(0.01);
            end
        end
        %%
         function RMRCmotion(qMatrix,qdot, robot) %pass Q matrix and corresponding robot
            
            logData = ['transforms-data.mat']; % log data into a mat file
            
            for step = 1:size(qMatrix, 1) % iterate between rows of Q matrix
                
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
                drawnow()
                robot.arm_msg.Position = qMatrix(step,:);
                robot.arm_msg.Velocity=qdot(step,:);
                send(robot.arm_pub,robot.arm_msg);
%                 pause(0.01);
            end
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
            
            initialPose = robot.model.fkine(robot.model.getpos);
            initialRPY = tr2rpy(initialPose);
            targetRPY = tr2rpy(targetPose);
            
             s = lspb(0,1,robot.steps);   
            for i=1:robot.steps
                x(1,i) = (1-s(i))*initialPose(1, 4) + s(i)*targetPose(1, 4);
                x(2,i) = (1-s(i))*initialPose(2, 4) + s(i)*targetPose(2, 4);
                x(3,i) = (1-s(i))*initialPose(3, 4) + s(i)*targetPose(3, 4);
                theta(1,i) = (1-s(i))*initialRPY(1) + s(i)*targetRPY(1);
                theta(2,i) = (1-s(i))*initialRPY(2) + s(i)*targetRPY(2);
                theta(3,i) = (1-s(i))*initialRPY(3) + s(i)*targetRPY(3);
            end

            qMatrix(1,:)= robot.model.getpos;
            
            for i = 1:robot.steps-1
                T=robot.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/robot.deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';  
                linear_velocity = (1/robot.deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];
                J = robot.model.jacob0(qMatrix(i,:));                                          % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < robot.epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/robot.epsilon)*5E-2;
                else
                    lambda = 0;
                end
                
                N = null(J);
                invJ = pinv(J'*J + lambda *eye(7))*J';
                qdot(i,:) = (invJ*xdot)';
                for j = 1:7
%                     if j == 3 || j == 5 || j == 7
%                         if qMatrix(i,j) >= 180
%                             qMatrix(i,j) = mod(qMatrix(i,j), -180);
%                         elseif qMatrix(i,j) <= -180
%                             qMatrix(i,j) = mod(qMatrix(i,j), -180);
%                         end
%                     end
                    if qMatrix(i,j) + robot.deltaT*qdot(i,j) < robot.model.qlim(j,1)
                        qdot(i,j) = 0;
                    elseif qMatrix(i,j) + robot.deltaT*qdot(i,j) > robot.model.qlim(j,2)
                        qdot(i,j) = 0;
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + robot.deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                
                
            end                                           
            
        end
        %%
        
    end
end

