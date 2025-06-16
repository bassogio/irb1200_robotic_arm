clc;    
clear;
close all;

% Define a range of joint configurations (degrees)
configurations = [
    0   0   0    0   0   0;
    90  0   0    0   0   0;
    90  90  0    0   0   0;
    90  90  -90  0   0   0;
    90  90  -90  90  0   0;
    90  90  -90  90  90  0;
    90  90  -90  90  90  90
];

figure;
for i = 1:size(configurations, 1)
    joint_angles_deg = configurations(i, :);

    % Compute joint origins for current configuration
    origins = compute_origins(joint_angles_deg);

    % Plot
    clf;
    plot3(origins(1, :), origins(2, :), origins(3, :), 'k-o', 'LineWidth', 2, 'MarkerSize', 6);
    grid on; axis equal;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    title(sprintf('IRB 1200 Stick Figure | Joint Angles = [%s]', num2str(joint_angles_deg)));

    % Label joints
    for j = 1:7
        text(origins(1, j), origins(2, j), origins(3, j), sprintf('  J%d', j-1));
    end

    pause(1);  % Wait a second before next update
end
