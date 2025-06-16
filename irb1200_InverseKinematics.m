% clc;    
% clear;
% close all;

% Load symbolic FK definitions
run('irb1200_ForwardKinematics.m');

%% Symbolic DH Parameters ===
syms d1 a2 a3 d4 d6 real
syms theta1 theta2 theta3 real

%% Extract rotation and position components from T06 ===
% T06_simplified = [nx ox ax px;
%                   ny oy ay py;
%                   nz oz az pz;
%                   0  0  0  1 ];
nx = T06_simplified(1,1); ox = T06_simplified(1,2); ax = T06_simplified(1,3); px = T06_simplified(1,4);
ny = T06_simplified(2,1); oy = T06_simplified(2,2); ay = T06_simplified(2,3); py = T06_simplified(2,4);
nz = T06_simplified(3,1); oz = T06_simplified(3,2); az = T06_simplified(3,3); pz = T06_simplified(3,4);

%% Compute wrist center position (Px, Py, Pz) ===
Px = px - d6 * ax;
Py = py - d6 * ay;
Pz = pz - d6 * az;

Px = collect(Px, cos(theta1));  % Expected: r*cos(theta1)
Py = collect(Py, sin(theta1));  % Expected: r*sin(theta1)

% Planar radial distance
r = sqrt(Px^2 + Py^2);

% Define two theta1 options ===
theta1_a = simplify(atan2(Py, Px));               % First solution
theta1_b = simplify(atan2(-Py, -Px));             % Second solution (rotated by pi)

%% Solve theta2 and theta3 for both theta1 options ===
for i = 1:2
    if i == 1
        th1 = theta1_a;
        label = 'theta1_a';
    else
        th1 = theta1_b;
        label = 'theta1_b';
    end
    
    % Triangle used to compute theta3
    %
    %              Wrist Center (x, z)
    %                     ●
    %                    / \
    %               a₃  /   \  a₂
    %                  /     \
    %                 / θ₃    \
    %                ●---------●
    %               Joint 2    → (x, z) from base
    %
    % This triangle connects:
    % - a₂: link 2 (from joint 2 to elbow)
    % - a₃: link 3 (from elbow to wrist center)
    % - √(x² + z²): the straight-line distance from joint 2 to wrist center
    %
    % Law of Cosines:
    %   c² = a² + b² − 2ab·cos(θ)
    %   where:
    %     c = √(x² + z²)
    %     a = a₂
    %     b = a₃
    %
    % Rearranged:
    %   cos(θ₃) = (x² + z² - a₂² - a₃²) / (2·a₂·a₃)

    % Evaluate wrist center projection magnitude
    x = simplify(r);                % planar distance doesn't depend on theta1 sign
    z = simplify(Pz - d1);          % vertical height

    % Law of cosines: c^2 = a^2 + b^2 − 2ab*cos(theta) where c = sqrt(x^2 + z^2), a = a2, b = a3
    cos_theta3 = simplify((x^2 + z^2 - a2^2 - a3^2)/(2*a2*a3));
    
    % sin²(θ) + cos²(θ) = 1 -> sin(θ) = ±sqrt(1 - cos²(θ))
    sin_theta3_pos = simplify(sqrt(1 - cos_theta3^2));
    sin_theta3_neg = simplify(-sqrt(1 - cos_theta3^2));
    
    % Triangle used to compute theta2 (based on theta3)
    %
    %                    Wrist Center (x, z)
    %                          ●
    %                         /|
    %                        / |
    %                   a₃  /  |  k₂ = a₃·sin(θ₃)
    %                      /   |
    %                     /    |
    %                    ●-----●
    %                Joint 2    → k₁ = a₂ + a₃·cos(θ₃)
    %
    % This triangle is formed between:
    % - The horizontal projection from joint 2 to wrist: k₁ = a₂ + a₃·cos(θ₃)
    % - The vertical projection due to elbow bend:       k₂ = a₃·sin(θ₃)
    % - The direct line to the wrist:                    (x, z) from base
    %
    % The triangle is used to compute θ₂ using:
    %
    %     θ₂ = atan2(z, x) - atan2(k₂, k₁)
    %
    % Geometrically:
    % - atan2(z, x) is the angle from base to wrist center
    % - atan2(k₂, k₁) is the internal correction angle between links

    % Elbow-Up
    theta3_up = simplify(atan2(sin_theta3_pos, cos_theta3));
    k1_up = a2 + a3 * cos_theta3;
    k2_up = a3 * sin_theta3_pos;
    theta2_up = simplify(atan2(z, x) - atan2(k2_up, k1_up));

    % Elbow-Down
    theta3_down = simplify(atan2(sin_theta3_neg, cos_theta3));
    k2_down = a3 * sin_theta3_neg;
    theta2_down = simplify(atan2(z, x) - atan2(k2_down, k1_up));

    % Output ===
    % fprintf('\n=%s with Elbow-UP ====\n', label);
    % disp('theta1 ='); pretty(th1)
    % disp('theta2 ='); pretty(theta2_up)
    % disp('theta3 ='); pretty(theta3_up)
    % 
    % fprintf('\n=%s with Elbow-DOWN ====\n', label);
    % disp('theta1 ='); pretty(th1)
    % disp('theta2 ='); pretty(theta2_down)
    % disp('theta3 ='); pretty(theta3_down)
end

%% Extract R from T06
R = T06(1:3,1:3);  % Rotation part of full transformation

%% Compute R03 (rotation matrix from base to joint 3)
% Assuming you already have symbolic T01, T12, T23:
T03 = simplify(T01 * T12 * T23);
R03 = T03(1:3,1:3);

%% Compute R36 = transpose(R03) * R
R36 = simplify(transpose(R03) * R);

r11 = R36(1,1); r12 = R36(1,2); r13 = R36(1,3);
r21 = R36(2,1); r22 = R36(2,2); r23 = R36(2,3);
r31 = R36(3,1); r32 = R36(3,2); r33 = R36(3,3);

%% === Extract theta4, theta5, theta6 from R36 ===
% Compute theta5 using norm of column 3 entries r13, r23
theta5 = simplify(atan2(sqrt(r13^2 + r23^2), r33));

% Use theta5 to compute theta4 and theta6:
% To avoid dividing by zero, we assume sin(theta5) ≠ 0
% Otherwise, gimbal lock occurs

% Define sin(theta5) terms
s5_pos = simplify(sqrt(r13^2 + r23^2));
s5_neg = simplify(-sqrt(r13^2 + r23^2));

% Two solutions for theta5
theta5_pos = simplify(atan2(s5_pos, r33));
theta5_neg = simplify(atan2(s5_neg, r33));

% Corresponding solutions for theta4 and theta6
theta4_pos = simplify(atan2(r13 / s5_pos, r23 / s5_pos));
theta6_pos = simplify(atan2(r31 / s5_pos, r32 / s5_pos));

theta4_neg = simplify(atan2(r13 / s5_neg, r23 / s5_neg));
theta6_neg = simplify(atan2(r31 / s5_neg, r32 / s5_neg));


