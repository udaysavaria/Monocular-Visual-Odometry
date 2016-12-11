function [ vSet ] = applyScaleFactor( vSet, groundTruthPoses )
% Calculate the scale factor given the odometry data and apply that to 
% the estimated camera poses
estimatedPoses = poses(vSet);
numPoses = size(estimatedPoses,1);

lastEstimatedTranslation = estimatedPoses.Location{numPoses} - estimatedPoses.Location{numPoses - 1};
lastOdometryTranslation = groundTruthPoses.Location{numPoses} - groundTruthPoses.Location{numPoses - 1};
scaleFactor = norm(lastOdometryTranslation)/norm(lastEstimatedTranslation);
estimatedPoses.Location{numPoses} = scaleFactor * lastEstimatedTranslation + estimatedPoses.Location{numPoses - 1};

vSet = updateView(vSet, estimatedPoses);
end


