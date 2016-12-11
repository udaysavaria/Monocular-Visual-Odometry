function [matchedPoints1, matchedPoints2] = matchOdometryFeatures(I1, I2)
disp('===========================');
disp('Detecting Features');

Loc1 = [];
Loc2 = [];
[Loc1(:, 1), Loc1(:, 2), score1]  = detectInterestPoints(I1);
[Loc2(:, 1), Loc2(:, 2), score2]  = detectInterestPoints(I2);

disp('Exctracting Features');

[features1] = extractPixelFeatures(I1, Loc1(:, 1), Loc1(:, 2), 15);
[features2] = extractPixelFeatures(I2, Loc2(:, 1), Loc2(:, 2), 15);

disp('Matching Features');

indexPairs = computeFeatureMatch(features1, features2);

matchedPoints1 = Loc1(indexPairs(:,1),:);
matchedPoints2 = Loc2(indexPairs(:,2),:);

disp('Done');



% --------------------------------------------------------------------------

% disp('===========================');
% disp('Detecting SURF Features');
%
% points1 = detectSURFFeatures(I1);
% points2 = detectSURFFeatures(I2);
%
% disp('Exctracting Features');
%
% [features1,valid_points1] = extractFeatures(I1,points1);
% [features2,valid_points2] = extractFeatures(I2,points2);
%
% disp('Matching Features');
%
%
% % indexPairs = matchFeature(features1,features2);
% indexPairs = computeFeatureMatch(features1,features2);
%
% matchedPoints1 = valid_points1(indexPairs(:,1),:);
% matchedPoints2 = valid_points2(indexPairs(:,2),:);
%
% disp('Done');
%
% figure; showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2);
