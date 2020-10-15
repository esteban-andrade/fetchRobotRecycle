function q = exampleHelperEndEffectorRelease(validator, qini, ax)
%exampleHelperEndEffectorRelease 

q = qini;
N = 20;
d = 0.05;

q = qini;
for j = 1:N
    q(end-2:end) = q(end-2:end) - j*d;
    validator.StateSpace.RigidBodyTree.show(q, 'Parent', ax, 'PreservePlot', false, 'Frames', 'off');
    drawnow
    if ~validator.isStateValid(q) || any(q(end-2:end) - d < 0)
        break;
    end
end

validator.removeFromEE(q, ax);
validator.StateSpace.RigidBodyTree.show(q, 'Parent', ax, 'frames', 'off', 'PreservePlot', false);
p = findobj(ax, 'type', 'patch', 'DisplayName','workpiece_mesh');
delete(p);
validator.StateSpace.NominalConfig = q;

end

