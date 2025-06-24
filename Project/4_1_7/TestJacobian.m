clc; clear; close all;

% === Robot DH constants ===
params = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

% === Test joint configuration (deg) ===
test_angles = [0, -45, 45, 0, 90, 0];

q = deg2rad(test_angles(:));  % Column vector

% === Compute Jacobians ===
[J, Jv, Jw] = Jacobian(params, q);

% === Display results ===
disp('Linear velocity Jacobian J_v:');
disp(Jv);

disp('Angular velocity Jacobian J_w:');
disp(Jw);

disp('Combined Geometric Jacobian J ( [J_v; J_w] ):');
disp(J);
