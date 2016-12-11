function [matches] = computeFeatureMatch(features1, features2)
threshold = 0.70;
dist = pdist2(features1, features2, 'euclidean');
[sorted_dist, indices] = sort(dist, 2);
ratio = (sorted_dist(:,1)./sorted_dist(:,2));
score = 1./ratio(ratio < threshold);
matches = zeros(size(score,1), 2);

matches(:,1) = find(ratio < threshold);
matches(:,2) = indices(ratio < threshold, 1);

[score, ind] = sort(score, 'descend');
matches = matches(ind,:);
