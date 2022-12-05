function [path_x, path_y, path_theta, path_v, path_w] = generate_reference(x_points, y_points, ...
    x0, y0, theta0, Ts, kind_reference, x_der0, y_der0)

% generate_reference - This function returns the reference position, the reference orientation, 
% the reference linear velocity and the reference angular velocity (only for a specific path) 
% given a collection of steering points of the path 
% 
% Inputs:
% x_points       : x's coordinates of the steering points of the path
% y_points       : y's coordinates of the steering points of the path
% x0             : x coordinate of initial position
% y0             : y coordinate of initial position
% theta0         : initial orientation of the path in radians
% Ts             : sampling time in seconds
% Kind_reference : kind_reference = 0 --> horizontal/vertical/curvilinear lines
%                  kind_reference = 1 --> yellow pathway
%   
% Outputs:
% path_x         : x-position reference signal
% path_y         : y-position reference signal
% path_theta     : orientation reference signal
% path_v         : linear velocity reference signal  <-- if kind_reference = 0
% path_w         : angular velocity reference signal <-- if kind_reference = 0 

path_x = [];
path_y = [];
path_theta = [];
path_v = [];
path_w = [];

% Constant tangential velocity
v = 0.5;

if kind_reference == 0

    path_theta_points = zeros(1,length(x_points)); % orientation at steering points
    path_theta_points(1) = theta0; 
    
    for k = 1:length(x_points)-1 % Iterate through subpaths
        % A subpath between two steering points can be of three types:
        % CASE 1) vertical subpath (uniform linear motion)
        % CASE 2) horizontal subpath (uniform linear motion)
        % CASE 3) steering subpath (uniform circular motion)
        
        if x_points(k) == x_points(k+1)                 % CASE 1
            distance = abs(y_points(k+1) - y_points(k));
            duration = distance / v; 
            
            curr_path_v = ones(1, round(duration/Ts)) .* v; % const lin. vel.
            curr_path_w = zeros(1, round(duration/Ts)); % zero angular vel.
            path_theta_points(k+1) = path_theta_points(k); % const orientation
           
        elseif y_points(k) == y_points(k+1)             % CASE 2
            distance = abs(x_points(k+1) - x_points(k));
            duration = distance / v; 
            
            curr_path_v = ones(1, round(duration/Ts)) .* v; % const lin. vel.
            curr_path_w = zeros(1, round(duration/Ts));  % zero angular vel.
            path_theta_points(k+1) = path_theta_points(k); % const orientation
     
        else                                            % CASE 3
            
            radius = abs(x_points(k+1) - x_points(k)); % steering radius
            
            % Computation of the steering direction
            sense = 0; 
            switch path_theta_points(k)
                case 0
                    if y_points(k+1) - y_points(k) > 0
                        sense = +1;
                        path_theta_points(k+1) = pi/2;
                    else 
                        sense = -1;
                        path_theta_points(k+1) = -pi/2; 
                    end
                
                case pi/2
                    if x_points(k+1) - x_points(k) > 0
                        sense = -1;
                        path_theta_points(k+1) = 0;
                    else 
                        sense = +1;
                        path_theta_points(k+1) = pi;
                    end
               
                case pi
                    if y_points(k+1) - y_points(k) > 0
                        sense = -1;
                        path_theta_points(k+1) = pi/2; 
                    else 
                        sense = +1;
                        path_theta_points(k+1) = -pi/2;
                    end
                
                case -pi/2
                    if x_points(k+1) - x_points(k) > 0
                        sense = +1;
                        path_theta_points(k+1) = 0;
                    else 
                        sense = -1;
                        path_theta_points(k+1) = pi;
                    end
            end
    
            distance = 0.5 * pi * radius; % subpath's arclength
            duration = distance / v;  
            w = sense * v / radius; % const angular vel. of the subpath
            
            curr_path_v = ones(1, round(duration/Ts)) .* v;
            curr_path_w = ones(1, round(duration/Ts)) .* w;
    
        end
        
        % Concatenate the current subpath's reference signals
        path_v = cat(2, path_v, curr_path_v);
        path_w = cat(2, path_w, curr_path_w);
    
    end
        
        % Final values of the reference signals
        path_v = cat(2, path_v, path_v(end));
        path_w = cat(2, path_w, path_w(end));

    path_x = zeros(1,length(path_v));
    path_y = zeros(1, length(path_v));
    path_theta = zeros(1, length(path_v));
    
    % Set initial conditions
    path_x(1) = x0; 
    path_y(1) = y0;
    path_theta(1) = theta0;
    
    % Integration
    for k = 2:length(path_v)
        path_theta(k) =  path_theta(k-1) + path_w(k) * Ts; 
        path_x(k) = path_x(k-1) + path_v(k) * cos(path_theta(k)) * Ts;
        path_y(k) = path_y(k-1) + path_v(k) * sin(path_theta(k)) * Ts;
    
    end
else

    duration = 40; % [s]
    num_samples = duration/Ts; % Number of samples
    time_points = 1:height(x_points);

    % Interpolate x vs t
    cs_x = spline(time_points,[x_der0; x_points; 0]);
    time = linspace(time_points(1),time_points(end),num_samples);
    path_x = ppval(cs_x,time);
    
    % Interpolate y vs t
    cs_y = spline(time_points,[y_der0; y_points; 0]);
    path_y = ppval(cs_y,time);

    % Orientation reference
    path_theta = atan2(diff(path_y), diff(path_x)); 
    path_theta = cat(2, path_theta, [path_theta(end)]); % Padding
    deg2rad = pi/180;
    for i=2:length(path_theta)
        if path_theta(i) - path_theta(i-1) > 350*deg2rad
            path_theta(i) = path_theta(i) - 360*deg2rad;
        elseif path_theta(i) - path_theta(i-1) < -350*deg2rad
            path_theta(i) = path_theta(i) + 360*deg2rad;
        end
    end

end
end
