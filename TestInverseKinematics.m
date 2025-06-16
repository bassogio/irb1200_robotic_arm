clc;
clear;
close all;

%% Step 1: Define test joint angles (degrees)
test_angles_deg = [0 0 0 0 0 0];
test_angles_rad = deg2rad(test_angles_deg);

%% Step 2: Compute FK
[position, T06] = forward_kinematics_irb1200(test_angles_rad);

fprintf('[FORWARD KINEMATICS]\n');
fprintf('Given joint angles (deg): [%s]\n', num2str(test_angles_deg));
fprintf('Computed End-Effector Position:\n');
fprintf('X = %.3f m\nY = %.3f m\nZ = %.3f m\n', position(1), position(2), position(3));

%% Step 3: Compute IK (from FK result)
theta_solved_rad = inverse_kinematics_irb1200(position, T06);
theta_solved_deg = rad2deg(theta_solved_rad);

fprintf('\n[INVERSE KINEMATICS]\n');
fprintf('Recovered joint angles (deg): [%s]\n', num2str(theta_solved_deg, '%.2f '));

%% Step 4: Compare
fprintf('\n[COMPARISON]\n');
disp('Original (deg):'); disp(test_angles_deg);
disp('Recovered (deg):'); disp(theta_solved_deg);
