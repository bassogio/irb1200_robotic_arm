clc;
clear;
close all;

% Load symbolic transformations
run('irb1200_ForwardKinematics.m');

% Link lengths
d_vals = [0.399, 0.35, 0.42, 0.351, 0.082];

% Define joint angle ranges in degrees (step size of 30° for speed)
theta1_range = -170:30:170;
theta2_range = -135:30:100;
theta3_range = -70:30:200;
theta4_range = -270:30:270;
theta5_range = -130:30:130;
theta6_range = -360:30:360;


% Initialize workspace point cloud
workspace_points = [];

% Sample configurations 
for t1 = theta1_range
    for t2 = theta2_range
        for t3 = theta3_range
            for t4 = theta4_range
                for t5 = theta5_range
                    for t6 = theta6_range
                        joint_angles_deg = [t1, t2, t3, t4, t5, t6];
                        joint_angles_rad = deg2rad(joint_angles_deg);

                        % Substitute into symbolic matrices
                        T01_num = double(subs(T01, [theta1, d1], [joint_angles_rad(1), d_vals(1)]));
                        T12_num = double(subs(T12, [theta2, d2], [joint_angles_rad(2), d_vals(1)]));
                        T23_num = double(subs(T23, [theta3, d3], [joint_angles_rad(3), d_vals(2)]));
                        T34_num = double(subs(T34, [theta4, d4], [joint_angles_rad(4), d_vals(2)]));
                        T45_num = double(subs(T45, theta5, joint_angles_rad(5)));
                        T56_num = double(subs(T56, [theta6, d6], [joint_angles_rad(6), d_vals(3)]));

                        % Forward kinematics
                        T06 = T01_num * T12_num * T23_num * T34_num * T45_num * T56_num;
                        p = T06(1:3, 4);
                        workspace_points = [workspace_points, p];
                    end
                end
            end
        end
    end
end

%% === 4.1.5.2.1 – Isometric View ===
figure;
scatter3(x, y, z, 10, 'filled'); hold on;

% Plot a sample pose for the robot
joint_angles_deg = [0, 0, 0, 0, 0, 0];
origins = compute_origins(joint_angles_deg);
plot3(origins(1, :), origins(2, :), origins(3, :), 'k-o', 'LineWidth', 2);

view(45, 30);  % Isometric view
axis equal;
grid on;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Isometric View: Robot Pose + Workspace');

%% === 4.1.5.2.2 – X-Y View ===
figure;
scatter(x, y, 10, 'filled');
axis equal;
grid on;
xlabel('X [m]');
ylabel('Y [m]');
title('Top View (X-Y) of Workspace');

%% === 4.1.5.2.3 – X-Z View ===
figure;
scatter(x, z, 10, 'filled');
axis equal;
grid on;
xlabel('X [m]');
ylabel('Z [m]');
title('Front View (X-Z) of Workspace');

%% === 4.1.5.2.4 – Y-Z View ===
figure;
scatter(y, z, 10, 'filled');
axis equal;
grid on;
xlabel('Y [m]');
ylabel('Z [m]');
title('Side View (Y-Z) of Workspace');
