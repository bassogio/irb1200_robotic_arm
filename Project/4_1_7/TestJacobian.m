%% TestJacobian.m
clc; clear; close all;

% --- Robot DH constants (cm)
params = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

% --- Test joint configurations (deg)
% test_angles = [ ...
%      0   0    0   0   0   0;
%     90   0    0   0   0   0;
%     90  90    0   0   0   0;
%     90  90  -90   0   0   0;
%     90  90  -90  90   0   0;
%     90  90  -90  90  90   0;
%     90  90  -90  90  90  90 ];
test_angles = [0, -45, 45, 0, 90, 0];

% --- Pick one of the test poses and compute Jacobian
q = deg2rad(test_angles(1,:)).';      % 4th pose, column vector
J = Jacobian(params,q);
disp('Geometric Jacobian (base frame):');
disp(J);
