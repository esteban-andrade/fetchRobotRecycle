%% Point cloud test
addpath('../scripts/');

ros_data = rosData;

xyz = ros_data.getPointCloud;

pcshow(xyz)
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Detect a Cylinder in a Point Cloud')
%% cylinder detection
maxDistance = 1;
roi = [0,0.5,-0.5,0.5,0.5,1];
ptcloud =pointCloud(xyz);
sampleIndices = findPointsInROI(ptcloud,roi);
referenceVector = [0,0,1];
model = pcfitcylinder(ptcloud,maxDistance,referenceVector,...
        'SampleIndices',sampleIndices);
hold on
plot(model)
%% camera calibration
focalLength    = [554, 554]; 
principalPoint = [320.5, 240.5];
imageSize      = [480, 640];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
tagSize = 0.04;