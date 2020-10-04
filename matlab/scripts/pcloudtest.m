%%
marker = 0;
% while marker == 0
    sub = rossubscriber('/head_camera/rgb/image_raw');
%     pause(0.1);
    msg2 = receive(sub, 10);
    img = readImage(msg2);
    figure;
    imshow(img)
% end

%%
close;
clc;
sub1 = rossubscriber('/head_camera/depth_registered/points');
pc_msg = receive(sub1, 10);

figure;
% scatter3(pc_msg);
xyz = readXYZ(pc_msg);
pcshow(xyz)
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Detect a Cylinder in a Point Cloud')
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