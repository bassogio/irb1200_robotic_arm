function Simulate()
    % --- DH Parameters for IRB1200 ---
    dh = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

    % Start configuration is HOME (all zeros)
    q_home = zeros(6,1);
    [~, origins_home, ~] = forwardKinematics(dh, q_home);
    start_pos = origins_home(:,end); 
    start_rpy = [0; 0; 0];  % Home orientation (degrees)

    % Define target position and orientation
    end_pos = [600; 0; 800];       % Absolute target in mm
    end_rpy = [-90; 0; -90];           % Target RPY in degrees

    % --- Check if end_pos is reachable before path planning ---
    sol_test = InverseKinematics(dh, end_pos);
    if isempty(sol_test)
        warning('WARNING: The selected end_pos may be unreachable with this orientation!');
    else
        disp('INFO: The selected end_pos is reachable.');
    end

    % --- Path generation ---
    N = 100;
    dt = 0.1;  % seconds between steps

    % Interpolate position (mm)
    path_positions = [linspace(start_pos(1), end_pos(1), N);
                       linspace(start_pos(2), end_pos(2), N);
                       linspace(start_pos(3), end_pos(3), N)];

    % Interpolate RPY (degrees)
    path_rpys = [linspace(start_rpy(1), end_rpy(1), N);
                  linspace(start_rpy(2), end_rpy(2), N);
                  linspace(start_rpy(3), end_rpy(3), N)];

    % --- For storing robot joint angles ---
    joint_trajectory = zeros(6,N);

    % --- Compute IK for all positions ---
    for k = 1:N
        goal_pos_mm = path_positions(:,k);
        % NOTE: path_rpys(:,k) is planned but unused here!
        sol = InverseKinematics(dh, goal_pos_mm);
        if isempty(sol)
            warning('IK failed at step %d - reusing previous joint angles.', k);
            joint_trajectory(:,k) = joint_trajectory(:,max(k-1,1));
        else
            joint_trajectory(:,k) = sol(:,1);
        end
    end

    % --- Animate the robot along the path ---
    figure('Name','IRB 1200 Planned Motion','NumberTitle','off');
    ax = axes;
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
    grid(ax, 'on'); hold(ax, 'on');
    view(ax, 45, 30);

    % --- Store EE positions for velocity calculation ---
    EE_positions = zeros(3,N);

    for k = 1:N
        cla(ax);
        theta_rad = joint_trajectory(:,k);
        [~, origins, rots] = forwardKinematics(dh, theta_rad);

        % Store TCP position
        EE_positions(:,k) = origins(:,end);

        % Plot links
        plot3(ax, origins(1,:)/1000, origins(2,:)/1000, origins(3,:)/1000, '-ok', ...
            'LineWidth', 2, 'MarkerSize', 4);
        hold(ax, 'on');

        % Plot path so far
        plot3(ax, EE_positions(1,1:k)/1000, EE_positions(2,1:k)/1000, EE_positions(3,1:k)/1000, 'b--');

        % Draw coordinate axes
        axisLen = 50;  % mm
        cols3 = {'r','g','b'};
        labels = {'X','Y','Z'};
        for j = 1:size(origins,2)
            O = origins(:,j)/1000;
            R = rots(:,:,j);
            for ii = 1:3
                v = R(:,ii) * (axisLen/1000);
                line(ax, [O(1), O(1)+v(1)], [O(2), O(2)+v(2)], [O(3), O(3)+v(3)], ...
                    'Color', cols3{ii}, 'LineWidth', 2);
                tip = O + v*1.1;
                text(ax, tip(1), tip(2), tip(3), sprintf('%s%d', labels{ii}, j-1), ...
                    'FontSize', 9, 'FontWeight', 'bold');
            end
        end

        title(ax, sprintf('Step %d/%d', k, N));
        setAxesEqual3D(ax);
        pause(0.01);
    end

    % --- Plot the full 3D path in a new figure ---
    figure('Name','End-Effector Trajectory','NumberTitle','off');
    plot3(EE_positions(1,:)/1000, EE_positions(2,:)/1000, EE_positions(3,:)/1000, 'b', 'LineWidth', 2);
    grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('End-Effector Path in 3D Space');

    % --- Compute velocity magnitude over time (simple difference) ---
    velocities = vecnorm(diff(EE_positions,1,2)/dt, 2, 1);
    time_vec = dt*(1:(N-1));

    figure('Name','Velocity vs Time (Simple)','NumberTitle','off');
    plot(time_vec, velocities, 'r', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (mm/s)');
    title('End-Effector Linear Velocity Magnitude (Simple Difference)');

    % --- Jacobian-based velocity estimation ---
    joint_velocities = diff(joint_trajectory,1,2) / dt;
    ee_velocities = zeros(6, N-1);

    for k = 1:(N-1)
        theta_k = joint_trajectory(:,k);
        q_dot_k = joint_velocities(:,k);
        [J, ~, ~] = Jacobian(dh, theta_k);
        ee_velocities(:,k) = J * q_dot_k;
    end

    % Plot Cartesian linear speed from Jacobian
    linear_speed = vecnorm(ee_velocities(1:3,:),1,1);

    figure('Name','EE Linear Speed via Jacobian','NumberTitle','off');
    plot(time_vec, linear_speed, 'g', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Linear Speed (mm/s)');
    title('End-Effector Linear Velocity via Jacobian');

    % --- Plot comparison on same figure
    figure('Name','Linear Velocity Comparison','NumberTitle','off');
    plot(time_vec, velocities, 'r', 'LineWidth', 2); hold on;
    plot(time_vec, linear_speed, 'g', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (mm/s)');
    legend('Simple diff (position)','Jacobian-based');
    title('Comparison of Linear Velocity Estimates');
    
    % --- Plot Cartesian linear velocity components
    figure('Name','EE Linear Velocity Components (Jacobian)','NumberTitle','off');
    plot(time_vec, ee_velocities(1,:), 'r', 'LineWidth', 2); hold on;
    plot(time_vec, ee_velocities(2,:), 'g', 'LineWidth', 2);
    plot(time_vec, ee_velocities(3,:), 'b', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (mm/s)');
    legend('Vx','Vy','Vz');
    title('End-Effector Linear Velocity Components (Jacobian)');
    
    % --- Plot Cartesian angular velocity components
    figure('Name','EE Angular Velocity Components (Jacobian)','NumberTitle','off');
    plot(time_vec, ee_velocities(4,:), 'r', 'LineWidth', 2); hold on;
    plot(time_vec, ee_velocities(5,:), 'g', 'LineWidth', 2);
    plot(time_vec, ee_velocities(6,:), 'b', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    legend('Wx','Wy','Wz');
    title('End-Effector Angular Velocity Components (Jacobian)');

end

function setAxesEqual3D(ax)
    xL = xlim(ax); yL = ylim(ax); zL = zlim(ax);
    xMid = mean(xL); yMid = mean(yL); zMid = mean(zL);
    xRange = diff(xL); yRange = diff(yL); zRange = diff(zL);
    maxRange = max([xRange, yRange, zRange]);
    xlim(ax, [xMid - maxRange/2, xMid + maxRange/2]);
    ylim(ax, [yMid - maxRange/2, yMid + maxRange/2]);
    zlim(ax, [zMid - maxRange/2, zMid + maxRange/2]);
end
