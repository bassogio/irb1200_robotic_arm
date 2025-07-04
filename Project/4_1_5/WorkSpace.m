clc; clear; close all;      

% === Robot constants ===
params = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

% === Discretisation step size ===
angle_step    = 10;                        % Sampling interval (deg)
theta_range   = deg2rad(0:angle_step:180); % 0 to 180° in radians

% === Initialise point cloud ===
workspace_points = [];                     % Preallocate empty array

% === Loop over all combinations of θ1, θ2, θ3 ===
fprintf('Calculating reachable points...\n');
total   = numel(theta_range)^3;            % Total number of samples
counter = 0;                               % Initialize progress counter

for t1 = theta_range
    for t2 = theta_range
        for t3 = theta_range

            % Fix tool orientation by setting last three joints to zero
            thetas = [t1; t2; t3; 0; 0; 0];

            % Compute forward kinematics numerically
            [T06, ~, ~] = forwardKinematics(params, thetas);
            p = T06(1:3, 4);               % Extract end-effector XYZ
            workspace_points(end+1, :) = p';   % Append point

            % Update progress every ~10%
            counter = counter + 1;
            if mod(counter, round(total/10)) == 0
                fprintf('%.0f%% complete\n', 100 * counter / total);
            end
        end
    end
end

% === Plot workspace only (no robot) ===
figure('Name', 'IRB 1200 Workspace Views (numeric FK)', ...
       'NumberTitle', 'off');

% 1) Isometric 3-D scatter
subplot(2,2,1);
scatter3(workspace_points(:,1), workspace_points(:,2), ...
         workspace_points(:,3), 10, 'b', 'filled');
title('Isometric View');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
axis equal; grid on; view(45, 30);

% 2) Top view (X–Y plane)
subplot(2,2,2);
scatter(workspace_points(:,1), workspace_points(:,2), 10, 'filled');
title('Top View (X–Y)');
xlabel('X [m]'); ylabel('Y [m]');
axis equal; grid on;

% 3) Front view (X–Z plane)
subplot(2,2,3);
scatter(workspace_points(:,1), workspace_points(:,3), 10, 'filled');
title('Front View (X–Z)');
xlabel('X [m]'); ylabel('Z [m]');
axis equal; grid on;

% 4) Side view (Y–Z plane)
subplot(2,2,4);
scatter(workspace_points(:,2), workspace_points(:,3), 10, 'filled');
title('Side View (Y–Z)');
xlabel('Y [m]'); ylabel('Z [m]');
axis equal; grid on;


% === Compute workspace limits ===
min_vals = min(workspace_points);
max_vals = max(workspace_points);

fprintf('\nEstimated Workspace Limits:\n');
fprintf('X: %.3f m to %.3f m\n', min_vals(1), max_vals(1));
fprintf('Y: %.3f m to %.3f m\n', min_vals(2), max_vals(2));
fprintf('Z: %.3f m to %.3f m\n', min_vals(3), max_vals(3));



