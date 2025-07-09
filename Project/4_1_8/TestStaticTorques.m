clc; clear; close all;

%% === Robot DH constants (millimeters) ===
params = struct( ...
    'd1', 399, ...
    'a2', 350, ...
    'a3', 42, ...
    'd4', 351, ...
    'd6', 82);

%% === Test joint configuration (degrees) ===
test_angles_deg = [0, -45, 45, 0, 90, 0];
q_test = deg2rad(test_angles_deg(:));  % Convert to radians column vector

%% === Payload parameters ===
payload_mass = 1.0;    % Mass in kg
gravity_accel = 9.81;  % Gravity in m/s^2

%% === Compute static torques ===
tau_static = staticTorquesForLoad(params, q_test, payload_mass, gravity_accel);

%% === Display results ===
fprintf('\n=== Static Joint Torques for holding %.2f kg payload ===\n', payload_mass);
for i = 1:length(tau_static)
    fprintf('Joint %d: %.3f Nm\n', i, tau_static(i));
end
