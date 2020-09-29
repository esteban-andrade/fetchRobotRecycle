function motion(qMatrix, robot) %pass Q matrix and corresponding robot

logData = ['transforms-data.mat']; % log data into a mat file

for step = 1:size(qMatrix, 1) % iterate between rows of Q matrix
    
    %animate(robot,qMatrix(step,:));
    robot.animate(qMatrix(step, :));
    endEffector = robot.fkine(qMatrix(step, :)); %#ok<NASGU>
    
    %Logs the data in the file premade earlier
    if (step == size(qMatrix, 1)) %if this is the last step of the arm
        
        transformsData.name = robot.name;
        transformsData.pose = robot.fkine(qMatrix(step, :));
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
end
end