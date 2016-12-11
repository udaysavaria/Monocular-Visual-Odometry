% Monocular Visual Odometry
% Data is read from the left camera images and intrinsics provided from KITTI dataset 

% Read "INDEX"th datastream 
INDEX = '02';
imagedir = ['E:\KITTI_VISION_Dataset\data_odometry_gray\dataset\sequences\' INDEX '\image_0'];
images = imageDatastore(imagedir);

% Read the first 1000 images from each datastream
number = min(1000, size(images.Files,1));
images.Files = images.Files(1:number);

Num = size(images.Files);
reprojErrorThresh = 2;

% Camera intrinsics, provided in KITTI dataset
K = [7.188560000000e+02 0.000000000000e+00 6.071928000000e+02 ;...
     0.000000000000e+00 7.188560000000e+02 1.852157000000e+02 ;...
     0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 ]';
 
cameraIntrinsics = cameraParameters('IntrinsicMatrix', K);

%% Ground Truth

gtdir = ['D:/KITTI_VISION_Dataset/data_odometry_poses/dataset/poses/' INDEX '.txt'];
data = load(gtdir);

Location = {};
for i = 1:size(data,1)
    Location{i} = data(i,[4,8,12]);
end
Location = Location';

Orientation = {};
for i = 1:size(data,1)
    Orientation{i} = [data(i,1:3); data(i,5:7); data(i, 9:11)]';
end
Orientation = Orientation';

viewPointIndex = 1:size(data,1);
viewPointIndex = viewPointIndex';

groundTruthPoses = table(viewPointIndex, Location, Orientation);

%% Initialize video objects

vidNameFeatures = ['features_Bundle_',INDEX,'.avi' ];
vidNameTrajectory = ['trajectory_Bundle_',INDEX,'.avi' ];

v1 = VideoWriter(vidNameFeatures);
v2 = VideoWriter(vidNameTrajectory);
open(v1);
open(v2);


%% Initialize a view set

vSet = viewSet;
latestFrame = readimage(images, 1);
previousFrame = undistortImage(latestFrame, cameraIntrinsics); 


% Detect features. 
prevPoints = detectSURFFeatures(previousFrame, 'MetricThreshold', 500);

% Select a subset of features, uniformly distributed throughout the image.
numPoints = 150;
prevPoints = selectUniform(prevPoints, numPoints, size(previousFrame));

% Extract features. Using 'Upright' features improves matching quality if 
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(previousFrame, prevPoints, 'Upright', true);


% Add the first view. Place the camera associated with the first view
% at the origin, oriented along the Z-axis.
viewPointIND = 1;
vSet = addView(vSet, viewPointIND, 'Points', prevPoints, 'Orientation', eye(3), 'Location', [0 0 0]);

%% Initialize Visualization

trajFig = figure;
axis([-220, 320, -140, 30, -50, 600]);
view(gca, 3);
set(gca, 'CameraUpVector', [0, -1, 0]);
camorbit(gca, -120, 0, 'data', [0, 1, 0]);
hold on
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');

cameraSize = 6;
% Estimated 6DOF camera pose
camEstimated = plotCamera('Size', cameraSize, 'Location', vSet.Views.Location{1}, 'Orientation', vSet.Views.Orientation{1},'Color', 'g', 'Opacity', 0);

% Ground truth 6DOF camera pose
camActual = plotCamera('Size', cameraSize, 'Location', groundTruthPoses.Location{1}, 'Orientation', groundTruthPoses.Orientation{1}, 'Color', 'b', 'Opacity', 0);

% Initialize camera trajectories.
trajectoryEstimated = plot3(0, 0, 0, 'g-');
trajectoryActual    = plot3(0, 0, 0, 'b-');

legend('Estimated Trajectory', 'Actual Trajectory');
title('Camera Trajectory');

%% Estimate the Pose of the Second View

matchedFigure = figure;
for viewPointIND = 2: 2

    latestFrame = readimage(images, viewPointIND);
    I = undistortImage(latestFrame, cameraIntrinsics); 

    [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(prevFeatures, I);
    matchedPoints1 = prevPoints(indexPairs(:,1));
    matchedPoints2 = currPoints(indexPairs(:,2));
    
    
    figure(matchedFigure)
    set(gca,'nextplot','replacechildren'); 
    showMatchedFeatures(previousFrame,I,matchedPoints1,matchedPoints2);
    frame = getframe;
    writeVideo(v1,frame);

    % Estimate pose of the second viewPoint with respect to the first one
    % by calculating the essential matrix
    
    [orient, loc, inlierIdx] = helperEstimateRelativePose(prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), cameraIntrinsics);
    
    loc = loc + vSet.Views.Location{viewPointIND - 1};
    orient = orient * vSet.Views.Orientation{viewPointIND - 1};
    

    % Exclude epipolar outliers.
    indexPairs = indexPairs(inlierIdx, :);
    
    vSet = addView(vSet, viewPointIND, 'Points', currPoints, 'Orientation', orient,'Location', loc);
    vSet = addConnection(vSet, viewPointIND-1, viewPointIND, 'Matches', indexPairs);

    % Normalize translation
    vSet = applyScaleFactor( vSet, groundTruthPoses );

    helperUpdateCameraPlots(viewPointIND, camEstimated, camActual, poses(vSet), groundTruthPoses);
    helperUpdateCameraTrajectories(viewPointIND, trajectoryEstimated, trajectoryActual, poses(vSet), groundTruthPoses);

    frame = getframe(trajFig);
    writeVideo(v2,frame);

    previousFrame = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;
end
    
%% Pose estimation for frame #3 and up
for viewPointIND = 3:Num
    latestFrame = readimage(images, viewPointIND);
    I = undistortImage(latestFrame, cameraIntrinsics); 
    
    % Match the points with known 3D position from the previous frame to
    % the features extracted from the new frame 

    [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(prevFeatures, I);
    matchedPoints1 = prevPoints(indexPairs(:,1));
    matchedPoints2 = currPoints(indexPairs(:,2));
    
    
    figure(matchedFigure)
    showMatchedFeatures(previousFrame,I,matchedPoints1,matchedPoints2);
    frame = getframe;
    writeVideo(v1,frame);
      
    % Apply triangulation to calculate the 3D position of the matched
    % points given the camera pose in the previous two frames
    [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet, cameraIntrinsics, indexPairs, currPoints);
     
    % Apply PnP to estimate the 6DOF camera pose given the known 3D
    % position of feature points in the current frame 
    [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, cameraIntrinsics, 'Confidence', 99, 'MaxReprojectionError', 1);
    
    vSet = addView(vSet, viewPointIND, 'Points', currPoints, 'Orientation', orient, 'Location', loc);
    vSet = addConnection(vSet, viewPointIND-1, viewPointIND, 'Matches', indexPairs);    
    vSet = applyScaleFactor( vSet, groundTruthPoses );
    
    % Run Bundle adjustment once every 15 frames
    if(mod(viewPointIND,15) == 0)
        
      % Find point tracks in the last 15 views and triangulate.
        windowSize = 30;
        startFrame = max(1, viewPointIND - windowSize);
        
        tracks = findTracks(vSet, startFrame:viewPointIND);
        camPoses = poses(vSet, startFrame:viewPointIND);
        
        InitPose = camPoses.Location{1};
        InitOrientation = camPoses.Orientation{1};
        [xyzPoints, reprojErrors] = triangulateMultiview(tracks, camPoses, cameraIntrinsics);
                                
        fixedIds = [startFrame, startFrame+1];
        idx = reprojErrors < reprojErrorThresh;
        
        [~, camPoses] = bundleAdjustment(xyzPoints(idx, :), tracks(idx),camPoses, cameraIntrinsics,'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9,'RelativeTolerance', 1e-9, 'MaxIterations', 400);
        
        vSet = updateView(vSet, camPoses); 
        vSet = applyBundleNormalization(vSet, groundTruthPoses,  startFrame, InitPose, InitOrientation);    
    end
    
    
    % Update camera trajectory plot.
    helperUpdateCameraPlots(viewPointIND, camEstimated, camActual, poses(vSet), groundTruthPoses);
    helperUpdateCameraTrajectories(viewPointIND, trajectoryEstimated,trajectoryActual, poses(vSet), groundTruthPoses);
    frame = getframe(trajFig);
    writeVideo(v2,frame);   
    
    previousFrame = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
end

camPoses_total = zeros(size(vSet.Views,1) * 3, 4);

for k = 1: size(vSet.Views, 1)
   camPoses_total(3*(k-1)+1:3*(k-1)+1 + 2, :) = [vSet.Views.Orientation{k}, vSet.Views.Location{k}']; 
end

textFileName = ['resultsBundle_', INDEX, '.txt'];
matFileName = ['resultsBundle_', INDEX, '.mat'];

delete(textFileName);
delete(matFileName);

dlmwrite(textFileName, camPoses_total, 'delimiter', ' ');
save(matFileName,'camPoses_total', 'groundTruthPoses');

hold off
close(v1);
close(v2);
