function qMatrix = interpolateJointAnglesFetch(q1, q2, steps)
s = lspb(0, 1, steps); % First, create the scalar function
qMatrix = nan(steps, 7); % Create memory allocation for variables
for i = 1:steps
    qMatrix(i, :) = (1 - s(i)) * q1 + s(i) * q2; % Generate interpolated joint angles
end
end
