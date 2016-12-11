function [x, y, confidence] = detectInterestPoints(image)

alpha=0.04;
gaussian = fspecial('Gaussian', [25 25], 1);
[gx, gy] = imgradientxy(gaussian);

xgrad = imfilter(image, gx);
ygrad = imfilter(image, gy);

featureSize = 16;
% Supress gradients near the edges
xgrad([(1:featureSize) end-featureSize+(1:featureSize)],:) = 0;
xgrad(:, [(1:featureSize) end-featureSize+(1:featureSize)]) = 0;
ygrad([(1:featureSize) end-featureSize+(1:featureSize)],:) = 0;
ygrad(:, [(1:featureSize) end-featureSize+(1:featureSize)]) = 0;

large_gaussian = fspecial('Gaussian', [25 25], 2);

ixx = imfilter(xgrad.*xgrad, large_gaussian);
ixy = imfilter(xgrad.*ygrad, large_gaussian);
iyy = imfilter(ygrad.*ygrad, large_gaussian);
harris = ixx.*iyy - ixy.*ixy - alpha.*(ixx+iyy).*(ixx+iyy);

thresholded = harris > 10*mean2(harris); %Adaptive threshold

components = bwconncomp(thresholded);
width = components.ImageSize(1);
x = zeros(components.NumObjects, 1);
y = zeros(components.NumObjects, 1);
confidence = zeros(components.NumObjects, 1);
for ii=1:(components.NumObjects)
    pixel_ids = components.PixelIdxList{ii};
    pixel_values = harris(pixel_ids);
    [max_value, max_id] = max(pixel_values);
    x(ii) = floor(pixel_ids(max_id)/ width);
    y(ii) = mod(pixel_ids(max_id), width);
    confidence(ii) = max_value;
end
