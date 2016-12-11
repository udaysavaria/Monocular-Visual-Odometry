% helperNormalizeViewSet Translate and scale camera poses to align with ground truth
%  vSet = helperNormalizedViewSet(vSet, groundTruth) returns a view set
%  with the camera poses translated to put the first camera at the origin
%  looking along the Z axes, and scaled to match the scale of the ground
%  truth. vSet is a viewSet object. groundTruth is a table containing the
%  actual camera poses.
%
%  See also viewSet, table

% Copyright 2016 The MathWorks, Inc. 

function vSet = applyBundleNormalization(vSet, groundTruth, startFrame, InitPose, InitOrientation)


camPoses = poses(vSet);

% Move the first camera to the origin.
locations = cat(1, camPoses.Location{startFrame:end});
locations = locations - InitPose;

locationsGT  = cat(1, groundTruth.Location{startFrame:height(camPoses)});
locationsGT = locationsGT - locationsGT(1,:);

magnitudes   = sqrt(sum(locations.^2, 2));
magnitudesGT = sqrt(sum(locationsGT.^2, 2));
scaleFactor = median(magnitudesGT(2:end) ./ magnitudes(2:end));

% Scale the locations
locations = locations .* scaleFactor;
locations = locations + InitPose;

% camPoses.Location{startFrame:end} = num2cell(locations, 2);
for i = 1: size(locations,1)
   camPoses.Location{startFrame + i - 1} = locations(i,:); 
end

% Rotate the poses so that the first camera points along the Z-axis
R = InitOrientation;
R2 = camPoses.Orientation{startFrame}'; % Inverse of the orientation of the start frame after bundle adjustment
for i = startFrame:height(camPoses)
    camPoses.Orientation{i} =  R * R2 * camPoses.Orientation{i} ;
end

vSet = updateView(vSet, camPoses);
