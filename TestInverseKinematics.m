clc;
clear;
close all;

% Load symbolic forward kinematics (defines T06_simplified)
run('irb1200_ForwardKinematics.m');

% === Robot constants ===
params = struct( ...
    'd1', 0.399, ...
    'a2', 0.350, ...
    'a3', 0.042, ...
    'd4', 0.351, ...
    'd6', 0.082 ...
);

% === Ask for joint angles in degrees ===
theta_deg = input('Enter joint angles [theta1 theta2 theta3 theta4 theta5 theta6] in degrees:\n');

% === Convert to radians ===
theta_rad = deg2rad(theta_deg);

% === Create substitution list ===
symbol_list = {
    theta1, theta2, theta3, theta4, theta5, theta6, ...
    d1, a2, a3, d4, d6
};

values = {
    theta_rad(1), theta_rad(2), theta_rad(3), ...
    theta_rad(4), theta_rad(5), theta_rad(6), ...
    params.d1, params.a2, params.a3, params.d4, params.d6
};

% === Evaluate FK ===
T06_numeric = double(subs(T06_simplified, symbol_list, values));
position = T06_numeric(1:3, 4);

% === Output XYZ ===
fprintf('\nEnd-effector Position:\n');
fprintf('X = %.3f m\n', position(1));
fprintf('Y = %.3f m\n', position(2));
fprintf('Z = %.3f m\n', position(3));
