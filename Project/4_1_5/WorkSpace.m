% plotWorkspace_IRB1200_numeric.m
% ------------------------------------------------------------
% Workspace sampling using *numeric* forwardKinematics().
% Mirrors the structure of the symbolic script you provided.
% ------------------------------------------------------------
clc;
clear;
close all;

% === Robot constants (metres) ===
params = struct( ...
    'd1', 0.399, ...
    'a2', 0.350, ...
    'a3', 0.042, ...
    'd4', 0.351, ...
    'd6', 0.082 ...
);

% === Discretisation step size ===
angle_step = 10;                   % degrees
theta_range = deg2rad(0:angle_step:180);  % 0–180° in radians

% === Initialise point cloud ===
workspace_points = [];

% === Loop over all combinations of θ1, θ2, θ3 ===
fprintf('Calculating reachable points...\n');
total   = numel(theta_range)^3;
counter = 0;

for t1 = theta_range
    for t2 = theta_range
        for t3 = theta_range
            
            % Joint vector (θ4–θ6 = 0 → fixed tool orientation)
            thetas = [t1; t2; t3; 0; 0; 0];
            
            % Numeric forward kinematics
            [T06, ~, ~] = forwardKinematics(params, thetas);
            p = T06(1:3, 4);                % end-effector position
            workspace_points = [workspace_points; p']; %#ok<AGROW>
            
            % Progress display (10 % increments)
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

% Plot 1: 3-D isometric
subplot(2,2,1);
scatter3(workspace_points(:,1), workspace_points(:,2), ...
         workspace_points(:,3), 10, 'b', 'filled');
title('Isometric View');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
axis equal; grid on; view(45, 30);

% Plot 2: Top view (X-Y)
subplot(2,2,2);
scatter(workspace_points(:,1), workspace_points(:,2), 10, 'filled');
title('Top View (X-Y)');
xlabel('X [m]'); ylabel('Y [m]');
axis equal; grid on;

% Plot 3: Front view (X-Z)
subplot(2,2,3);
scatter(workspace_points(:,1), workspace_points(:,3), 10, 'filled');
title('Front View (X-Z)');
xlabel('X [m]'); ylabel('Z [m]');
axis equal; grid on;

% Plot 4: Side view (Y-Z)
subplot(2,2,4);
scatter(workspace_points(:,2), workspace_points(:,3), 10, 'filled');
title('Side View (Y-Z)');
xlabel('Y [m]'); ylabel('Z [m]');
axis equal; grid on;
