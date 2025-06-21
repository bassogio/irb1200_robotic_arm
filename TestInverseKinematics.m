% TestInverseKinematics.m  –  verify every IK solution
clc; clear; close all;

%% 1. Robot constants (mm)  ─────────────────────────────────────────────
dh = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

%% 2. Desired TCP point (mm)
xyz = [250; 5; 791];

%% 3. Inverse kinematics  (rad)  ───────────────────────────────────────
Q = InverseKinematics(dh, xyz);     % 6×N matrix

%% 4. Print table θ₁…θ₆ (deg) + FK XYZ (mm)  ───────────────────────────
N = size(Q,2);
fprintf('\n%-7s | %6s%6s%6s%6s%6s%6s || %9s%9s%9s\n', ...
        'Sol #', 'θ1', 'θ2', 'θ3', 'θ4', 'θ5', 'θ6', 'X  (mm)', 'Y  (mm)', 'Z  (mm)');
fprintf('%s\n', repmat('-',1,7+1+6*6+3+3*9));   % divider

for k = 1:N
    thetaDeg = rad2deg(Q(:,k));

    % Forward kinematics → end-effector XYZ (mm)
    [~,origins,~] = forwardKinematics(dh, Q(:,k));
    p = origins(:,end);           % 3×1

    fprintf('%7d | %6.1f%6.1f%6.1f%6.1f%6.1f%6.1f || %9.1f%9.1f%9.1f\n', ...
            k, thetaDeg, p);
end
