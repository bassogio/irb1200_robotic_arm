clc;
clear all;
close all;

%% Step 0: Define Robotic arm constants (imaginary joint is represented as 00)

% D = [0.399 0 0 0 0 0.082];
D = [0.399 0 0 00 0 0 0];
% A = [0 0.35 0.351 0.042 0 0];
A = [0 0 0.35 0.042 0.351 0 0.082]; 


% Alpha = deg2rad([90 0 -90 90 0 0]);
% theta_zero_state = [0 90 -90 0 0 0];

Alpha = deg2rad([0 90 0 00 90 90 0]);
theta_zero_state = [0 0 90 00 -90 0 0];

%% Step 1: Define test joint angles (degrees) (00 represents imaginary joint and should't be changed)
test_angles_deg = [0 0 0 00 0 0 0];

%% Step 2: Compute FK
Theta = deg2rad(test_angles_deg+theta_zero_state);
[position, T06] = forward_kinematics_Guy(D, A, Theta, Alpha);

fprintf('[FORWARD KINEMATICS]\n');
fprintf('Given joint angles (deg): [%s]\n', num2str(test_angles_deg));
fprintf('Computed End-Effector Position:\n');
fprintf('X = %.3f m\nY = %.3f m\nZ = %.3f m\n', position(1), position(2), position(3));

%% Step 3: Compute IK (from FK result)
theta_solved_rad = inverse_kinematics_Guy(position, T06);
theta_solved_deg = rad2deg(theta_solved_rad);

fprintf('\n[INVERSE KINEMATICS]\n');
fprintf('Recovered joint angles (deg): [%s]\n', num2str(theta_solved_deg, '%.2f '));

%% Step 4: Compare
fprintf('\n[COMPARISON]\n');
disp('Original (deg):'); disp(test_angles_deg);
disp('Recovered (deg):'); disp(theta_solved_deg);
