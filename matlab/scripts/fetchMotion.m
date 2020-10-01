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
                pause(0.01);
            end
        end
        
        
        %%
        
        function RMRC()
            
        end
        
    end
end

