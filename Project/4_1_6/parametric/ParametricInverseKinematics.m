clc; clear; close all;

% Load symbolic FK definitions
run('irb1200_ForwardKinematics.m');  % must define T01, T12, T23, T06

% Define symbolic variables
syms d1 a2 a3 d4 d6 real
syms theta1 theta2 theta3 theta4 theta5 theta6 real
syms Px_tip Py_tip Pz_tip real

% Step 1: Wrist center
ax = T06(1,3); ay = T06(2,3); az = T06(3,3);
px = T06(1,4); py = T06(2,4); pz = T06(3,4);

Px = px - d6 * ax;
Py = py - d6 * ay;
Pz = pz - d6 * az;

% Step 2: Theta1 candidates
theta1_a = atan2(Py, Px);
theta1_b = theta1_a + pi;

% Initialize solution cell array
solutions = {};

for th1 = [theta1_a, theta1_b]
    % Step 3: Compute s and r for theta3 (ζ)
    s = Pz - d1;
    r = sqrt((Px - a2 * cos(th1))^2 + (Py - a2 * sin(th1))^2);

    % Step 4: Solve θ3 (zeta) using cosine law
    L3 = d4 + d6;
    cos_zeta = (s^2 + r^2 - a2^2 - L3^2) / (2 * a2 * L3);
    sin_zeta = sqrt(1 - cos_zeta^2);  % positive and negative options
    zeta_pos = atan2(sin_zeta, cos_zeta);
    zeta_neg = atan2(-sin_zeta, cos_zeta);
    
    % θ3 = -(90 + ζ)
    theta3_pos = -(pi/2 + zeta_pos);
    theta3_neg = -(pi/2 + zeta_neg);

    for zeta = [zeta_pos, zeta_neg]
        theta3 = -(pi/2 + zeta);  % Removed simplify to avoid symbolic overload
        
        % Step 5: Solve θ2
        Omega = atan2(s, r);
        lambda = atan2(L3 * sin(zeta), a2 + L3 * cos(zeta));
        theta2 = -(Omega - lambda) - pi/2;  % DO NOT simplify this
        
        % Store θ1–θ3
        solutions{end+1} = [th1; theta2; theta3];
    end
end

% Step 6: Extract R03
T03 = simplify(T01 * T12 * T23);
R03 = T03(1:3,1:3);

% Step 7: Extract rotation part from full transform
R06 = T06(1:3,1:3);

% Step 8: R36 = R03^T * R06
R36 = simplify(transpose(R03) * R06);

r11 = R36(1,1); r12 = R36(1,2); r13 = R36(1,3);
r21 = R36(2,1); r22 = R36(2,2); r23 = R36(2,3);
r31 = R36(3,1); r32 = R36(3,2); r33 = R36(3,3);

% Step 9: Solve theta4, theta5, theta6 using ZYZ Euler
theta5 = atan2(sqrt(r31^2 + r32^2), r33);
theta4 = atan2(r32, r31);
theta6 = atan2(r23, -r13);

% Alternative (flipped) solutions
theta5_flip = atan2(-sqrt(r31^2 + r32^2), r33);
theta4_flip = theta4 + pi;
theta6_flip = theta6 + pi;

% Print one full solution as example
fprintf('\n=== Example IK solution (θ1–θ3) ===\n');
pretty(solutions{1})
disp('=== Theta4–6 ===')
pretty([theta4; theta5; theta6])

disp('=== Flipped solution (θ4–6) ===')
pretty([theta4_flip; theta5_flip; theta6_flip])
