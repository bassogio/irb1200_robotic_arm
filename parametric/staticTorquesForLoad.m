clc; clear; close all;

%% === Declare symbolic variables ===
syms theta1 theta2 theta3 theta4 theta5 theta6 real
syms d1 a2 a3 d4 d6 real
syms M g real

% Collect joint angles
thetas = [theta1; theta2; theta3; theta4; theta5; theta6];

%% === Define DH parameters as symbolic struct ===
dh.d1 = d1;
dh.a2 = a2;
dh.a3 = a3;
dh.d4 = d4;
dh.d6 = d6;

%% === Compute Forward Kinematics for Origins and Rotations ===
% We'll define the same per-joint transformation matrices
% The individual transformations (as in your FK example):
T01 = [ cos(theta1)   0  -sin(theta1)   0;
        sin(theta1)   0   cos(theta1)   0;
             0       -1       0        d1;
             0        0       0         1 ];

T12 = [ sin(theta2)   cos(theta2)  0   a2*sin(theta2);
       -cos(theta2)   sin(theta2)  0  -a2*cos(theta2);
            0              0       1       0;
            0              0       0       1 ];

T23 = [ cos(theta3)   0  -sin(theta3)   a3*cos(theta3);
        sin(theta3)   0   cos(theta3)   a3*sin(theta3);
             0       -1       0         0;
             0        0       0         1 ];

T34 = [ cos(theta4)   0   sin(theta4)   0;
        sin(theta4)   0  -cos(theta4)   0;
            0         1       0        d4;
            0         0       0         1 ];

T45 = [ cos(theta5)   0  -sin(theta5)   0;
        sin(theta5)   0   cos(theta5)   0;
            0        -1       0         0;
            0         0       0         1 ];

T56 = [ -cos(theta6)   sin(theta6)   0    0;
        -sin(theta6)  -cos(theta6)   0    0;
             0             0         1   d6;
             0             0         0    1 ];

% Chain
T_matrices = {T01, T12, T23, T34, T45, T56};
T_cumulative = sym(eye(4));
origins = sym(zeros(3,7));
origins(:,1) = [0;0;0];

z_axes = sym(zeros(3,6));
z_axes(:,1) = [0;0;1];  % Base z-axis

for i = 1:6
    T_cumulative = simplify(T_cumulative * T_matrices{i}, 'Steps', 50);
    origins(:,i+1) = T_cumulative(1:3,4);
    z_axes(:,i) = T_cumulative(1:3,3);
end

%% === Compute the Jacobian Symbolically ===
J_linear = sym(zeros(3,6));
J_angular = sym(zeros(3,6));
pe = origins(:,end);

for i = 1:6
    J_linear(:,i) = simplify(cross(z_axes(:,i), pe - origins(:,i)), 'Steps', 50);
    J_angular(:,i) = z_axes(:,i);
end

J = [J_linear; J_angular];

%% === Define External Wrench ===
F_ext = [0; 0; -M * g; 0; 0; 0];

%% === Compute Static Torques ===
tau_static = simplify( transpose(J) * F_ext, 'Steps', 50 );

%% === Display Results ===
disp('=== Symbolic Jacobian ===');
disp(simplify(J, 'Steps', 100));

disp('=== Symbolic Static Torques τ for holding load M ===');
disp(tau_static);
