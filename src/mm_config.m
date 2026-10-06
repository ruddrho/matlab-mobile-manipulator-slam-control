function c = mm_config()
% SI units throughout. Invented educational robot; not a commercial plant.
c.dt = 0.05; c.armDt = 0.004; c.seed = 17;
c.world = [-1.5 11.5 -2.0 9.5]; c.grid = 0.06;
% x, y, conservative footprint radius, height; types control only appearance.
c.obstacles = [2.15 .30 .35 .95;3.8 2.0 .38 .75;5.15 3.0 .32 .85; ...
 1.8 3.65 .22 .70;6.75 .75 .28 .65;2.2 5.0 .36 .85; ...
 4.15 5.55 .40 1.05;5.65 8.55 .32 .90;8.0 8.0 .38 .85; ...
 10.25 5.6 .35 1.05;7.45 4.65 .35 .75;9.6 1.8 .38 .80; ...
 3.15 8.3 .33 .80;5.7 5.1 .32 .70;10.35 7.65 .28 .75];
c.obstacleTypes = {'drum','carton','box','cone','carton','carton', ...
 'drum','drum','box','drum','carton','box','carton','box','cone'};
% Long inspection-cell route, followed forward and backward between stations.
c.route = [.05 0;.35 1.7;.45 4.8;2.4 6.7;5.3 7.3;8.5 6.4; ...
 9.0 4.0;8.0 2.7;4.4 4.25;5.85 3.9];
c.cornerRadius = .75; c.crossingFractions = [.26 .65];
c.crossingHalfWidth = 1.3; c.crossingLead = 1.7;
c.agvPark = [10.55 -.8]; c.scanDuration = 6;
c.baseRadius = 0.59; c.margin = 0.12;
c.wheelRadius = 0.16; c.track = 0.67;
c.vmax = 0.48; c.wmax = 1.4; c.accel = 0.65;
c.brake = 0.8; c.reaction = 0.15; c.lidarRange = 5;
c.lidarAngles = linspace(-pi,pi,181); c.lidarNoise = 0.004;
c.agvRadius = 0.40; c.agvSpeed = 0.38;
c.pickBase = [0.05 0 0]; c.goalBase = [5.85 3.9 0];
c.dock = [0 0 0];
% Pad rectangles [xmin ymin xmax ymax]; keep the entire rotating base clear.
c.stationPads = [.72 -.40 1.24 .40;6.55 3.49 7.11 4.11];
c.stationMargin = .06;
c.blocks = [0.98 -0.19 0.17; 0.98 0.19 0.17];
c.stack = [6.83 3.8 0.17; 6.83 3.8 0.37];
c.blockSize = [0.18 0.18 0.20]; c.payload = 1;
c.arm.h = 0.62; c.arm.L = [0.62 0.52]; c.arm.tool = 0.14;
c.arm.axes = [0 0 1;0 1 0;0 1 0;1 0 0;0 1 0;1 0 0]';
c.arm.offset = [0 0 .62;.62 0 0;.52 0 0;0 0 0;0 0 0;0 0 .14]';
c.arm.com = c.arm.offset/2;
c.arm.mass = [2.0 2.2 1.5 .35 .30 .25];
c.arm.inertia = zeros(3,3,6);
for i=1:6
 ell=norm(c.arm.offset(:,i)); mass=c.arm.mass(i);
 % Isotropic positive inertia is an explicit approximation for this model.
 c.arm.inertia(:,:,i)=eye(3)*mass*(ell^2/12+0.035^2/2);
end
c.arm.rotor = [0.04 .055 .035 .012 .012 .008]';
c.arm.friction = [.15 .18 .12 .035 .035 .025]';
c.arm.limit = repmat([-pi pi],6,1);
c.arm.torqueLimit = [40 65 45 12 12 8]';
c.arm.qHome = [0 -1.8 2.65 0 -0.85 0]';
c.arm.Kp = [80 90 90 55 55 45]';
c.arm.Kd = 2*sqrt(c.arm.Kp);
c.output = fullfile(fileparts(fileparts(mfilename('fullpath'))),'results');
end
