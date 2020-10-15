%% Point cloud test

addpath('../scripts/');
hold on;

ros_data = rosData;

xyz = ros_data.getPointCloud; 

pc = pcshow(xyz);
% h = scatter3(xyz(:,1),xyz(:,2),xyz(:,3))
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Detect a Cylinder in a Point Cloud')
axis equal;

%%
while 1
    xyz = ros_data.getPointCloud; 
    pc = pcshow(xyz);
end
