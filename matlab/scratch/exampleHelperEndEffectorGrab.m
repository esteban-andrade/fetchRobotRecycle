function q = exampleHelperEndEffectorGrab(validator, obsName, qini, ax)
%exampleHelperEndEffectorGrab 

% the last 3 digits are gripper fingers


q = qini;
N = 20;
fingerRange = linspace(0,1,20);
for j = 1:N
    q(end-2:end) = fingerRange(j);
    validator.StateSpace.RigidBodyTree.show(q, 'Parent', ax, 'PreservePlot', false, 'Frames', 'off');
    drawnow
    if ~validator.isStateValid(q)
        break;
    end
end

validator.attachTOEE(obsName, q);
validator.StateSpace.RigidBodyTree.show(q, 'PreservePlot',false, 'Parent', ax, 'Frames', 'off');
validator.StateSpace.NominalConfig = q;
end

