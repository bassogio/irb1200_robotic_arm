clc;    
clear;
close all;

% Load symbolic FK definitions
run('irb1200_ForwardKinematics.m');

%% === Symbolic DH Parameters ===
syms d1 a2 a3 d4 d6 real
syms theta1 theta2 theta3 real

%% === Extract rotation and position components from T06 ===
% T06_simplified = [nx ox ax px;
%                   ny oy ay py;
%                   nz oz az pz;
%                   0  0  0  1 ];
nx = T06_simplified(1,1); ox = T06_simplified(1,2); ax = T06_simplified(1,3); px = T06_simplified(1,4);
ny = T06_simplified(2,1); oy = T06_simplified(2,2); ay = T06_simplified(2,3); py = T06_simplified(2,4);
nz = T06_simplified(3,1); oz = T06_simplified(3,2); az = T06_simplified(3,3); pz = T06_simplified(3,4);

%% === Compute wrist center position (Px, Py, Pz) ===
Px = px - d6 * ax;
Py = py - d6 * ay;
Pz = pz - d6 * az;

Px = collect(Px, cos(theta1));  % Expected: r*cos(theta1)
Py = collect(Py, sin(theta1));  % Expected: r*sin(theta1)

% Planar radial distance
r = sqrt(Px^2 + Py^2);

% === Define two theta1 options ===
theta1_a = simplify(atan2(Py, Px));               % First solution
theta1_b = simplify(atan2(-Py, -Px));             % Second solution (rotated by pi)

%% === Solve theta2 and theta3 for both theta1 options ===
for i = 1:2
    if i == 1
        th1 = theta1_a;
        label = 'theta1_a';
    else
        th1 = theta1_b;
        label = 'theta1_b';
    end
    

    
    % Evaluate wrist center projection magnitude
    x = simplify(r);                % planar distance doesn't depend on theta1 sign
    z = simplify(Pz - d1);          % vertical height

    % Law of cosines: c^2 = a^2 + b^2 − 2ab*cos(theta) where c = sqrt(x^2 + z^2), a = a2, b = a3
    cos_theta3 = simplify((x^2 + z^2 - a2^2 - a3^2)/(2*a2*a3))

    sin_theta3_pos = simplify(sqrt(1 - cos_theta3^2));
    sin_theta3_neg = simplify(-sqrt(1 - cos_theta3^2));

    % Elbow-Up
    theta3_up = simplify(atan2(sin_theta3_pos, cos_theta3));
    k1_up = a2 + a3 * cos_theta3;
    k2_up = a3 * sin_theta3_pos;
    theta2_up = simplify(atan2(z, x) - atan2(k2_up, k1_up));

    % Elbow-Down
    theta3_down = simplify(atan2(sin_theta3_neg, cos_theta3));
    k2_down = a3 * sin_theta3_neg;
    theta2_down = simplify(atan2(z, x) - atan2(k2_down, k1_up));

    % === Output ===
    % fprintf('\n==== %s with Elbow-UP ====\n', label);
    % disp('theta1 ='); pretty(th1)
    % disp('theta2 ='); pretty(theta2_up)
    % disp('theta3 ='); pretty(theta3_up)
    % 
    % fprintf('\n==== %s with Elbow-DOWN ====\n', label);
    % disp('theta1 ='); pretty(th1)
    % disp('theta2 ='); pretty(theta2_down)
    % disp('theta3 ='); pretty(theta3_down)
end
