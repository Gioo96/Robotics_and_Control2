function y_img = y_view(y, y_max, img_height)

% y_view - conversion of y's coordinates from world to image reference 
% frame
%
% Inputs:
% y          : y coordinate in RF_W
% y_max      : height of the map in RF_W
% img_height : height of the map in RF_image
%
% Output:
% y_img      : y coordinate in RF_image

y_img = (y_max - y) * img_height/y_max;

end