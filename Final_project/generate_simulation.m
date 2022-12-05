function F = generate_simulation(map, path_x, path_y, p_x, p_y, theta)

car_width = 0.3;
car_height = 0.5;
car_x_body = [1 -1 -1 1].*car_height/2;
car_y_body = [1 1 -1 -1].*car_width/2;
car_xy_body = cat(1, car_x_body, car_y_body);

% inizialization
f = figure;
f.Visible = 'off';
imshow(map.img.data) % image
hold on
plot(x_view (path_x, map.x_max, map.img.width), ...
     y_view(path_y, map.y_max, map.img.height)); % reference signal

R = [cos(theta(1)) -sin(theta(1)); sin(theta(1)) cos(theta(1))];
car_xy_world = R * car_xy_body + [p_x(1); p_y(1)] * [1 1 1 1];
car_xy_world(1,:) = x_view(car_xy_world(1,:), map.x_max, map.img.width);
car_xy_world(2,:) = y_view(car_xy_world(2,:), map.y_max, map.img.height);
car = patch(car_xy_world(1,:), car_xy_world(2,:), 'yellow'); % draw the car

hold off
axis equal manual
set(gca,'nextplot','replacechildren');

ratio = 50;

v = VideoWriter('Simulation.mp4', 'MPEG-4');
v.FrameRate = 1000/ratio;
open(v);

loops = length(p_x);
F(floor(loops/ratio)) = struct('cdata',[],'colormap',[]);

for k=1:loops
    waitbar(k/loops);
    if mod(k, ratio) == 1
        %update car position
        R = [cos(theta(k)) -sin(theta(k)); sin(theta(k)) cos(theta(k))];
        car_xy_world = R * car_xy_body + [p_x(k); p_y(k)] * [1 1 1 1];
        car_xy_world(1,:) = x_view(car_xy_world(1,:), map.x_max, map.img.width);
        car_xy_world(2,:) = y_view(car_xy_world(2,:), map.y_max, map.img.height);

        %rendering
        car.Vertices = car_xy_world';

        F(ceil(k/ratio)) = getframe(gca);
        
    end

end
writeVideo(v,F);
close(v)

end