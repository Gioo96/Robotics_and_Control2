%% Unicycle / Camera / Environment parameters

% Unicycle Paramneters
load unicycle.mat

% Rad2deg
rad2deg = 180/pi;

% Deg2rad 
deg2rad = pi/180;

%
% Unicycle
%
% Dimensions / Rotations / Translations
unicycle.width = 0.3;
unicycle.height = 0.5;
unicycle.b = 0.2;
unicycle.R_BC = [0 1;-1 0]; % Camera to Body
unicycle.T_BC1= [unicycle.height/2; unicycle.b/2]; % C1-B translation
unicycle.T_BC2 = [unicycle.height/2; -unicycle.b/2]; % C2-B translation
unicycle.dangerous_distance = 0.7; % Dangerous distance
unicycle.diffDrive.omegaWheelMax = 40;
unicycle.diffDrive.omegaWheelMin = -40;

%
% Camera
%
camera.depth = 3;
camera.focal_length = 0.02;
camera.fov = 140 * pi/180;

%
% Map
%
map.x_max = 15;
map.y_max = 10;
% Image of the map
map.img.data = imread('environ.png');
% Image size
img_size = size(map.img.data);
map.img.width = img_size(2);
map.img.height = img_size(1);
clear img_size
%% Derivator

derivator.wc = 2*pi*1; %2*pi*1
derivator.delta = 1/sqrt(2); %1/sqrt(2)
%% Controller

% Tracking controller
controller.tracking.damp = 0.5;
controller.tracking.a = 5;
controller.tracking.K1 = 2 * controller.tracking.damp * controller.tracking.a; 
controller.tracking.K2 = 5;
controller.tracking.K3 = controller.tracking.K1;

% Posture controller
controller.posture.Kv = 1; % 1
controller.posture.Kw = 12; % 12
controller.posture.Kdelta = 12; % 12
%% Scheduler

% Scheduler
scheduler.threshold_p = 0.03;
scheduler.threshold_theta = 4*deg2rad;
%% Simulation parameters

simulation.step_size = 1e-3; %[s]
simulation.stop_time = 70; %[s]
simulation.starting_controller = 1; % 1 -> tracking controller, 2 -> posture controller, 3 -> zero controller