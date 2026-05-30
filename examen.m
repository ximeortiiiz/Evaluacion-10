%% EXAMPLE: Differential drive vehicle following waypoints using the
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

T = readtable("puntos_xime.csv");

nombres = lower(string(T.Properties.VariableNames));
ix = find(nombres == "x", 1);
iy = find(nombres == "y", 1);

if ~isempty(ix) && ~isempty(iy)
    x = T{:, ix};
    y = T{:, iy};
else
    esNum   = varfun(@isnumeric, T, 'OutputFormat','uniform');
    colsNum = find(esNum);
    x = T{:, colsNum(end-1)};
    y = T{:, colsNum(end)};
end

waypoints = [x y];

if isempty(waypoints)
    error("No se leyeron waypoints. Revisa el archivo puntos.xlsx.");
end

disp("Waypoints importados:");
disp(waypoints);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:265;         % Time array

% Pose inicial = primer waypoint (theta = 0)
initPose = [waypoints(1,1); waypoints(1,2); 0];
pose = zeros(3,numel(tVec));     % Pose matrix
pose(:,1) = initPose;

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.20;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 4;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w];                         % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
end