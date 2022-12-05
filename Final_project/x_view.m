function x_img = x_view(x, x_max, img_width)

% x_view - conversion of x's coordinates from world to image reference 
% frame
%
% Inputs:
% x         : x coordinate in RF_W
% x_max     : width of the map in RF_W
% img_width : width of the map in RF_image
%
% Output:
% x_img     : x coordinate in the RF_image

x_img = x * img_width/x_max;

end