clc;
clear;
close all;

% Load symbolic transformations (T01–T06)
run('irb1200_ForwardKinematics.m');

% Define DH constants
params = struct( ...
    'd1', 0.399, ...
    'a2', 0.350, ...
    'a3', 0.042, ...
    'd4', 0.351, ...
    'd6', 0.082 ...
);

% === Define test joint configurations (degrees) ===
test_angles_deg = [
    0   0    0    0   0   0;
    90  0    0    0   0   0;
    90  90   0    0   0   0;
    90  90  -90   0   0   0;
    90  90  -90  90   0   0;
    90  90  -90  90  90   0;
    90  90  -90  90  90  90
];

% === Auto-count number of configurations ===
num_configs = size(test_angles_deg, 1);

% === Subplot layout (3 columns, dynamic rows) ===
num_cols = 3;
num_rows = ceil(num_configs / num_cols);

% === Prepare transformation chain ===
T_chain = {eye(4), T01, T02, T03, T04, T05, T06};

symbol_list = {
    theta1, theta2, theta3, theta4, theta5, theta6, ...
    d1, a2, a3, d4, d6
};

values_template = {
    0, 0, 0, 0, 0, 0, ...
    params.d1, params.a2, params.a3, params.d4, params.d6
};

% === Create main figure ===
figure('Name', 'IRB 1200 Configurations', 'NumberTitle', 'off');

for k = 1:num_configs
    subplot(num_rows, num_cols, k);

    % Set current joint angles
    angles_rad = deg2rad(test_angles_deg(k, :));
    values = values_template;
    values(1:6) = num2cell(angles_rad);

    % Evaluate joint positions
    positions = zeros(3, 7);
    for j = 1:7
        T = T_chain{j};
        T_num = double(subs(T, symbol_list, values));
        pos = T_num(1:3, 4);
        positions(:, j) = pos;
    end

    % Plot stick figure
    plot3(positions(1,:), positions(2,:), positions(3,:), '-o', ...
        'LineWidth', 2, 'MarkerSize', 6);
    grid on;
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(['Angles: [', num2str(test_angles_deg(k,:)), ']']);
    view(3);
end
