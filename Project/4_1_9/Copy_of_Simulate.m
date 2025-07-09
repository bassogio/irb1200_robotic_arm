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
    end_rpy = [-90; 0; 90];           % Target RPY in degrees

    % --- Check if end_pos is reachable before path planning ---
    goal_test = [end_pos; end_rpy];
    sol_test = InverseKinematicsRPY(dh, goal_test);
    if isempty(sol_test)
        warning('WARNING: The selected end_pos may be unreachable with this orientation!');
    else
        disp('INFO: The selected end_pos is reachable.');
    end

    % --- Path generation ---
    N = 100;
    T_total = 10;        % total motion duration in seconds
    t_vec = linspace(0, T_total, N);
    dt = T_total / (N - 1);
    s_profile = trapezoidalProfile(t_vec, T_total, 0.2*T_total);  

    % Interpolate position (mm)
    path_positions = start_pos * ones(1,N) + (end_pos - start_pos) * s_profile;
    path_rpys = start_rpy * ones(1,N) + (end_rpy - start_rpy) * s_profile;


    % --- For storing robot joint angles ---
    joint_trajectory = zeros(6,N);

    % --- Compute IK for all positions ---
    for k = 1:N
        goal_pos_mm = path_positions(:,k);
        goal_rpy_deg = path_rpys(:,k);
        goal_pose = [goal_pos_mm; goal_rpy_deg];
        sol = InverseKinematicsRPY(dh, goal_pose);
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
        plot3(ax, origins(1,:), origins(2,:), origins(3,:), '-ok', ...
            'LineWidth', 2, 'MarkerSize', 4);
        hold(ax, 'on');

        % Plot path so far
        plot3(ax, EE_positions(1,1:k), EE_positions(2,1:k), EE_positions(3,1:k), 'b--');

        % Draw coordinate axes
        axisLen = 50;  % mm
        cols3 = {'r','g','b'};
        labels = {'X','Y','Z'};
        for j = 1:size(origins,2)
            O = origins(:,j);
            R = rots(:,:,j);
            for ii = 1:3
                v = R(:,ii) * (axisLen);
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
    plot3(EE_positions(1,:), EE_positions(2,:), EE_positions(3,:), 'b', 'LineWidth', 2);
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

function s = trapezoidalProfile(t, T, Ta)
    % Compute trapezoidal time-scaling function s(t) in [0,1]
    % t - time vector
    % T - total duration
    % Ta - acceleration (and deceleration) time

    Vmax = 1 / (T - Ta);  % Ensure area under v(t) = 1

    s = zeros(size(t));
    for i = 1:length(t)
        if t(i) < Ta
            s(i) = 0.5 * Vmax / Ta * t(i)^2;
        elseif t(i) < T - Ta
            s(i) = Vmax * (t(i) - Ta/2);
        else
            dt_dec = t(i) - (T - Ta);
            s(i) = 1 - 0.5 * Vmax / Ta * (T - t(i))^2;
        end
    end
end
