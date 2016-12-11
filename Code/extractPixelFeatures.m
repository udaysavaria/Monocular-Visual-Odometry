function [features] = extractPixelFeatures(image, x, y, feature_width)
[h,w] = size(image);
num_points = size(x,1);
f = [];
w = (feature_width-1)/2;
features = [];

for i = 1:num_points
      f = image(y(i)-w:y(i)+w, x(i)-w:x(i)+w);
      f = reshape(f, 1, []);
      features = [features; f];
end
