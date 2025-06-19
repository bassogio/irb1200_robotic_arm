clc; clear; close all;

%% 1. DH constants (cm) ------------
dh = struct('d1',399,  ... % base → axis 2
            'a2',350,  ... % axis 2 → axis 3
            'a3', 42,  ... % axis 3 link offset
            'd4',351,  ... % axis 4 link
            'd6', 82);     % flange/tool offset

%% 2. Joint limits (radians) -------
% [min, max] for axes 1…6  (ABB IRB1200-5/0.9; adapt if needed)
degLimits = [ -170,  170;      % θ1
               -85,  120;      % θ2
              -120,  158;      % θ3
              -400,  400;      % θ4
              -125,  125;      % θ5
              -400,  400 ];    % θ6
radLimits = deg2rad(degLimits);

%% 3. Sampling parameters ----------
N = 20000;                       % number of random configs
thetas = zeros(6,N);

for i = 1:6
    thetas(i,:) = radLimits(i,1) + (radLimits(i,2)-radLimits(i,1)) * rand(1,N);
end

%% 4. Forward kinematics loop -------
points = zeros(3,N);

for k = 1:N
    [T06, ~, ~] = forwardKinematics(dh, thetas(:,k));
    points(:,k) = T06(1:3,4);   % store x,y,z
end

%% 5. Plotting ----------------------
figure('Name','IRB1200 Workspace','Color','w');

% 5a. 3-D workspace
subplot(2,2,1);
scatter3(points(1,:), points(2,:), points(3,:), 5, '.');
axis equal; grid on;
xlabel('X [cm]'); ylabel('Y [cm]'); zlabel('Z [cm]');
title('3-D Workspace');

% 5b. XY projection
subplot(2,2,2);
scatter(points(1,:), points(2,:), 5, '.');
axis equal; grid on;
xlabel('X [cm]'); ylabel('Y [cm]');
title('XY View');

% 5c. XZ projection
subplot(2,2,3);
scatter(points(1,:), points(3,:), 5, '.');
axis equal; grid on;
xlabel('X [cm]'); ylabel('Z [cm]');
title('XZ View');

% 5d. YZ projection
subplot(2,2,4);
scatter(points(2,:), points(3,:), 5, '.');
axis equal; grid on;
xlabel('Y [cm]'); ylabel('Z [cm]');
title('YZ View');

sgtitle('ABB IRB1200 Reachable Workspace');

%% 6. (Optional) show one arm pose ---
% Pick the first sampled pose and overlay its stick model in 3-D view
hold(subplot(2,2,1),'on');
[~, origins, ~] = forwardKinematics(dh, thetas(:,1));
plot3(origins(1,:), origins(2,:), origins(3,:), 'k-', 'LineWidth', 1.5);
plot3(origins(1,:), origins(2,:), origins(3,:), 'ro', 'MarkerSize', 4, ...
      'MarkerFaceColor','r');
legend('Reachable points','Sample pose','Location','best');
