%% plotTrajectoryAndSpeed.m
% Prompts for goal pose and then computes
%   • Joint‐space interpolation
%   • End‐effector path (3D)
%   • End‐effector speed vs. time

clc; clear; close all;

%% === 0) Prompt for goal pose ===
goalX = input('Enter Goal X (cm): ');
goalY = input('Enter Goal Y (cm): ');
goalZ = input('Enter Goal Z (cm): ');
alpha_deg = input('Enter roll α (deg): ');
beta_deg  = input('Enter pitch β (deg): ');
gamma_deg = input('Enter yaw γ (deg): ');

% Convert to radians and form vectors
P_end   = [goalX; goalY; goalZ];
rpy_end = deg2rad([alpha_deg; beta_deg; gamma_deg]);

%% === 1) (Optional) Define start pose — here we assume start at the origin, zero orientation
P_start   = [0;0;0];
rpy_start = [0;0;0];

%% === 2) Build homogeneous transforms
R_start = eul2rotm(rpy_start','ZYX');
T_start = [R_start, P_start; 0 0 0 1];

R_end   = eul2rotm(rpy_end',  'ZYX');
T_end   = [R_end,   P_end;   0 0 0 1];

%% === 3) Inverse kinematics for start & end
Q0 = InverseKinematics(dh, P_start);  % may return several solutions
Qf = InverseKinematics(dh, P_end);

% Pick the first solution from each
q_start = Q0(:,1);
q_end   = Qf(:,1);

%% === 4) Time discretization & interpolation
T_total = 5;      % seconds
N       = 100;    % samples
t       = linspace(0, T_total, N);
dt      = t(2) - t(1);

Q_traj = zeros(6, N);
for i = 1:6
    Q_traj(i,:) = linspace(q_start(i), q_end(i), N);
end

%% === 5) Forward kinematics to get end‐effector path
pos_traj = zeros(3, N);
for k = 1:N
    [T06, ~, ~] = forwardKinematics(dh, Q_traj(:,k));
    pos_traj(:,k) = T06(1:3,4);
end

%% === 6) Plot 3D path
figure;
plot3(pos_traj(1,:), pos_traj(2,:), pos_traj(3,:), '-o','MarkerSize',4);
xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
title('End-Effector 3D Path'); grid on; axis equal;

%% === 7) Compute speed via Jacobian
speed = zeros(1, N-1);
for k = 1:N-1
    q_dot = (Q_traj(:,k+1) - Q_traj(:,k)) / dt;
    [~, J_lin, ~] = Jacobian(dh, Q_traj(:,k)');
    v = J_lin * q_dot;
    speed(k) = norm(v);
end

%% === 8) Plot speed vs time
figure;
plot(t(1:end-1), speed, 'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Speed (cm/s)');
title('End-Effector Speed vs. Time'); grid on;
