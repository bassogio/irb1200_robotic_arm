clc;
clear;
close all;

%% Load symbolic forward kinematics
run('irb1200_ForwardKinematics.m');

% === Robot constants ===
params = struct( ...
    'd1', 0.399, ...
    'a2', 0.350, ...
    'a3', 0.042, ...
    'd4', 0.351, ...
    'd6', 0.082 ...
);

% === Discretization step size ===
angle_step = 10;  % degrees
theta_range = deg2rad(0:angle_step:180);  

% === Symbolic substitution list ===
symbol_list = {
    theta1, theta2, theta3, theta4, theta5, theta6, ...
    d1, a2, a3, d4, d6
};

% Fixed DH constants
fixed_params = {
    params.d1, params.a2, params.a3, ...
    params.d4, params.d6
};

% === Initialize point cloud ===
workspace_points = [];

% === Loop over all combinations of joint angles ===
fprintf('Calculating reachable points...\n');
total = numel(theta_range)^3;
progress = 0;

for t1 = theta_range
    for t2 = theta_range
        for t3 = theta_range
            % Use default (zero) values for last 3 joints (orientation)
            angles = {t1, t2, t3, 0, 0, 0};
            values = [angles, fixed_params];
            
            % Compute end-effector position from T06
            T_num = double(subs(T06_simplified, symbol_list, values));
            p = T_num(1:3, 4);
            workspace_points = [workspace_points; p'];
            
            % Optional: progress display
            progress = progress + 1;
            if mod(progress, round(total/10)) == 0
                fprintf('%.0f%% complete\n', 100 * progress / total);
            end
        end
    end
end

%% === Pick a random pose to show robot ===
rand_pose = [theta_range(randi(length(theta_range))), ...
             theta_range(randi(length(theta_range))), ...
             theta_range(randi(length(theta_range))), ...
             0, 0, 0];

% === Evaluate positions of all joints for that pose ===
T_chain = {eye(4), T01, T02, T03, T04, T05, T06};
joint_positions = zeros(3, 7);
values_rand = [num2cell(rand_pose), fixed_params];

for j = 1:7
    Tj = double(subs(T_chain{j}, symbol_list, values_rand));
    joint_positions(:, j) = Tj(1:3, 4);
end

%% === 4-PLOT DISPLAY ===
figure('Name', 'IRB 1200 Workspace Views', 'NumberTitle', 'off');

% Plot 1: 3D isometric with robot
subplot(2,2,1);
scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), 10, 'b', 'filled');
hold on;
plot3(joint_positions(1,:), joint_positions(2,:), joint_positions(3,:), '-ro', 'LineWidth', 2);
title('Isometric View + Robot Pose');
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
