% clc;     
% clear; 
% close all;

% Load symbolic FK definitions
run('irb1200_ForwardKinematics.m');

%% Symbolic DH Parameters ===
syms d1 a2 a3 d4 d6 real
syms theta1 theta2 theta3 real

%% Extract rotation and position components from T06 ===
nx = T06_simplified(1,1); ox = T06_simplified(1,2); ax = T06_simplified(1,3); px = T06_simplified(1,4);
ny = T06_simplified(2,1); oy = T06_simplified(2,2); ay = T06_simplified(2,3); py = T06_simplified(2,4);
nz = T06_simplified(3,1); oz = T06_simplified(3,2); az = T06_simplified(3,3); pz = T06_simplified(3,4);

%% Compute wrist center
Px = px - d6 * ax;
Py = py - d6 * ay;
Pz = pz - d6 * az;

r = sqrt(Px^2 + Py^2);
theta1 = simplify(atan2(Py, Px));   

x = simplify(r);
z = simplify(Pz - d1);

cos_theta3 = simplify((x^2 + z^2 - a2^2 - a3^2)/(2*a2*a3));
sin_theta3 = simplify(sqrt(1 - cos_theta3^2));  % <-- Elbow-UP only

theta3 = simplify(atan2(sin_theta3, cos_theta3));
k1 = a2 + a3 * cos_theta3;
k2 = a3 * sin_theta3;
theta2 = simplify(atan2(z, x) - atan2(k2, k1));

%% Compute R03 and R36
T03 = simplify(T01 * T12 * T23);
R03 = T03(1:3,1:3);
R = T06(1:3,1:3);
R36 = simplify(transpose(R03) * R);

r11 = R36(1,1); r12 = R36(1,2); r13 = R36(1,3);
r21 = R36(2,1); r22 = R36(2,2); r23 = R36(2,3);
r31 = R36(3,1); r32 = R36(3,2); r33 = R36(3,3);

% theta5 positive only
theta5 = simplify(atan2(sqrt(r13^2 + r23^2), r33));  % sin(theta5) > 0

theta4 = simplify(atan2(r13 / sin(theta5), r23 / sin(theta5)));
theta6 = simplify(atan2(r31 / sin(theta5), r32 / sin(theta5)));

%% Store best solution
best_solution = [theta1; theta2; theta3; theta4; theta5; theta6];  
