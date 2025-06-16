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

% === Initialize lines container ===
workspace_lines = {};  % each cell will hold one line

% === Generate reachable workspace lines ===
fprintf('Calculating reachable lines...\n');
for t1 = theta_range
    for t3 = theta_range
        line_points = [];
        for t2 = theta_range
            angles = {t1, t2, t3, 0, 0, 0};
            values = [angles, fixed_params];

            T_num = double(subs(T06_simplified, symbol_list, values));
            p = T_num(1:3, 4);
            line_points = [line_points; p'];
        end
        workspace_lines{end+1} = line_points;
    end
end

%% === Plot: 4 Views of the Workspace ===
figure('Name', 'IRB 1200 Workspace (Lines Only)', 'NumberTitle', 'off');

% Plot 1: 3D Isometric View
subplot(2,2,1);
hold on;
for i = 1:length(workspace_lines)
    pts = workspace_lines{i};
    plot3(pts(:,1), pts(:,2), pts(:,3), 'b-');
end
title('Isometric View');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
axis equal; grid on; view(45, 30);

% Plot 2: Top View (X-Y)
subplot(2,2,2);
hold on;
for i = 1:length(workspace_lines)
    pts = workspace_lines{i};
    plot(pts(:,1), pts(:,2), 'b-');
end
title('Top View (X-Y)');
xlabel('X [m]'); ylabel('Y [m]');
axis equal; grid on;

% Plot 3: Front View (X-Z)
subplot(2,2,3);
hold on;
for i = 1:length(workspace_lines)
    pts = workspace_lines{i};
    plot(pts(:,1), pts(:,3), 'b-');
end
title('Front View (X-Z)');
xlabel('X [m]'); ylabel('Z [m]');
axis equal; grid on;

% Plot 4: Side View (Y-Z)
subplot(2,2,4);
hold on;
for i = 1:length(workspace_lines)
    pts = workspace_lines{i};
    plot(pts(:,2), pts(:,3), 'b-');
end
title('Side View (Y-Z)');
xlabel('Y [m]'); ylabel('Z [m]');
axis equal; grid on;
