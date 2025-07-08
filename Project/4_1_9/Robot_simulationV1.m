%% Complete ABB IRB 1200 Robot Simulation with PID Control
% This script implements steps 3-9 of your requirements

clear; clc; close all;

%% ABB IRB one‑two‑zero‑zero – PID test with ode45 integration
clear; clc; close all;

% =====================  Parameters  ================================
dh.d1 = 0.399;            % m
dh.a2 = 0.350;
dh.a3 = 0.042;
dh.d4 = 0.351;
dh.d6 = 0.082;

theta_offset = [0; -pi/2; 0; 0; 0; -pi];   % joint‑angle offsets

masses = [5, 4, 3, 2, 1, 0.5];             % kg
g = 9.81;                                   % m s⁻²

%% Step 3: Dynamical Equations of Motion
% The equations of motion for a 6-DOF robot are:
% M(q)*q̈ + C(q,q̇)*q̇ + G(q) = τ
% where:
%   M(q) - mass matrix (6x6)
%   C(q,q̇) - Coriolis/centrifugal matrix (6x6)
%   G(q) - gravity vector (6x1)
%   τ - joint torques (6x1)

fprintf('=== Step 3: Dynamical Equations of Motion ===\n');
fprintf('The robot dynamics are governed by:\n');
fprintf('M(q)*q̈ + C(q,q̇)*q̇ + G(q) = τ\n\n');

%% Step 5: Choose start position with zero velocity
% Initial configuration - robot arm extended
q_start = [0; -pi/3; pi/6; 0; -pi/4; 0];  % Initial joint angles (rad) - arm extended
qDot_start = zeros(6,1);                   % Zero initial velocities

fprintf('=== Step 5: Initial Configuration ===\n');
fprintf('Start position (degrees): [%s]\n', num2str(rad2deg(q_start')));
fprintf('Start velocity (rad/s): [%s]\n', num2str(qDot_start'));
fprintf('Robot will fall under gravity from this position\n\n');

%% Step 7: Choose target position
q_target = [pi/4; pi/3; -pi/4; pi/6; pi/3; pi/2];  % Target joint angles (rad)

fprintf('=== Step 7: Target Configuration ===\n');
fprintf('Target position (degrees): [%s]\n\n', num2str(rad2deg(q_target')));

%% Step 8: Design PID Controller  (unchanged gains)
Kp = diag([200, 1500, 200, 1000, 200, 100]);
Ki = diag([0,45, 0, 0, 0, 0]);
Kd = diag([40, 280, 40, 40, 40, 40]);

integral_limit = 100; % rad·s clamp

% === new: viscous-damping matrix reused by both phases ============
B = diag([2, 2, 1, 0.5, 0.3, 0.2]);   % N·m·s  per joint
% ==================================================================
fprintf('=== Step 8: PID Controller Design ===\n');
fprintf('PID controller with gravity compensation\n');
fprintf('Higher gains for faster response\n\n');

%% Step 4 & 6: Simulation with Zero Input (Free Motion)
fprintf('=== Step 4 & 6: Free Motion Simulation (Robot Falling Under Gravity) ===\n');
fprintf('Simulating robot falling with ZERO input torques...\n');
fprintf('The robot will fall due to gravity alone (τ = 0)\n\n');

% Simulation parameters
t_span = [0, 3];  % Reduced time to see the falling motion clearly
dt = 0.001;       % Time step
t_sim = t_span(1):dt:t_span(2);

% Initial state vector [q; qDot]
x0 = [q_start; qDot_start];

% % Simulate with zero input (τ = 0)
% fprintf('Running simulation with τ = [0, 0, 0, 0, 0, 0] Nm\n');
% [t_free, x_free] = ode45(@(t,x) robotDynamics(t, x, zeros(6,1), dh, masses, g), ...
%                          t_sim, x0);
% 
% % Extract positions and velocities
% q_free = x_free(:, 1:6);
% qDot_free = x_free(:, 7:12);
% 
% % Calculate potential and kinetic energy to verify falling motion
% PE_initial = calculatePotentialEnergy(dh, masses, g, q_start);
% PE_final = calculatePotentialEnergy(dh, masses, g, q_free(end,:)');
% KE_final = 0.5 * qDot_free(end,:) * computeMassMatrix(dh, masses, q_free(end,:)') * qDot_free(end,:)';
% 
% fprintf('\nEnergy Analysis:\n');
% fprintf('Initial Potential Energy: %.2f J\n', PE_initial);
% fprintf('Final Potential Energy: %.2f J\n', PE_final);
% fprintf('Final Kinetic Energy: %.2f J\n', KE_final);
% fprintf('Energy Conservation Check: Initial PE = %.2f J, Final PE + KE = %.2f J\n', ...
%         PE_initial, PE_final + KE_final);
% 
% %  Create stick figure animation for free motion
% figure('Name', 'Robot Falling Under Gravity (Zero Input)', 'Position', [100, 100, 800, 600]);
% subplot(2,1,1);
% for i = 1:30:length(t_free)  % Adjusted step for smoother animation
%     clf;
%     subplot(2,1,1);
% 
%     % Draw initial position in gray
%     drawRobot3D(dh, q_start', 'k--');
%     hold on;
% 
%     % Draw current position
%     drawRobot3D(dh, q_free(i,:));
%     title(sprintf('Robot Falling Under Gravity - Time: %.2f s (τ = 0)', t_free(i)));
%     legend('Initial Position', 'Current Position', 'Location', 'northwest');
%     axis equal;
%     view(45, 30);
%     grid on;
% 
%     % Add text showing zero torque
%     text(-0.8, -0.8, 1.3, 'Input Torques: τ = [0, 0, 0, 0, 0, 0] Nm', ...
%          'FontSize', 12, 'FontWeight', 'bold', 'Color', 'red');
% 
%     subplot(2,1,2);
%     plot(t_free(1:i), rad2deg(q_free(1:i,:)));
%     xlabel('Time (s)');
%     ylabel('Joint Angles (degrees)');
%     title('Joint Angles vs Time (Free Fall Motion)');
%     legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'Location', 'best');
%     grid on;
%     xlim(t_span);
%     ylim([-180, 180]);
% 
%     drawnow;
% 
%     % Pause at first and last frame
%     if i == 1 || i >= length(t_free)-30
%         pause(1);
%     end
% end

%% Step 9: Simulation with PID Control
fprintf('\n=== Step 9: PID Controlled Motion Simulation ===\n');
fprintf('Simulating robot with PID control to reach target...\n');

% Extended simulation time for controlled motion
t_span_control = [0, 10];
t_sim_control = t_span_control(1):dt:t_span_control(2);

% Simulate with PID control
integral_error = zeros(6,1);  % Initialize integral term
prev_error = zeros(6,1);      % Initialize previous error for derivative

% Preallocate arrays
q_control = zeros(length(t_sim_control), 6);
qDot_control = zeros(length(t_sim_control), 6);
tau_control = zeros(length(t_sim_control), 6);

% Initial conditions
q_control(1,:) = q_start';
qDot_control(1,:) = qDot_start';

% Main simulation loop
for i = 1:length(t_sim_control)-1
    q_current  = q_control(i,:)';
    qDot_current = qDot_control(i,:)';
    
    % PID control terms
    error          = q_target - q_current;
    integral_error = integral_error + error * dt;
    integral_error = max(min(integral_error,integral_limit),-integral_limit);
    derivative_error = (i==1) * zeros(6,1) + (i>1) * ((error - prev_error)/dt);
    
    tau_pid   = Kp*error + Ki*integral_error + Kd*derivative_error;
    tau_damp  = -B * qDot_current;                 % <-- added damping
    tau_total = tau_pid + tau_damp;                % total applied torque
    
    tau_control(i,:) = tau_total';                 % store for plotting
    
    % Dynamics integration
    [M,C,G] = computeDynamicsMatrices(dh,masses,g,q_current,qDot_current);
    qDDot   = (M + 1e-6*eye(6)) \ (tau_total - C*qDot_current - G);
    qDDot   = max(min(qDDot,50),-50);              % accel clamp
    
    qDot_control(i+1,:) = (qDot_current + qDDot*dt)';
    q_control(i+1,:)    = (q_current    + qDot_current*dt + 0.5*qDDot*dt^2)';
    prev_error = error;
end

% Create stick figure animation for PID controlled motion
figure('Name', 'PID Controlled Motion with Gravity Compensation', 'Position', [950, 100, 800, 600]);
for i = 1:100:length(t_sim_control)
    clf;
    subplot(2,2,[1,3]);
    
    % Draw initial position in gray
    drawRobot3D(dh, q_start', 'k:');
    hold on;
    
    % Draw current position
    drawRobot3D(dh, q_control(i,:));
    hold on;
    
    % Draw target configuration in red
    drawRobot3D(dh, q_target', 'r--');
    title(sprintf('PID Control with Gravity Compensation - Time: %.2f s', t_sim_control(i)));
    axis equal;
    view(45, 30);
    grid on;
    legend('Initial', 'Current', 'Target', 'Location', 'best');
    
    subplot(2,2,2);
    plot(t_sim_control(1:i), rad2deg(q_control(1:i,:)));
    hold on;
    plot(t_sim_control, rad2deg(repmat(q_target', length(t_sim_control), 1)), 'k--');
    xlabel('Time (s)');
    ylabel('Joint Angles (degrees)');
    title('Joint Angles vs Time');
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'Location', 'best');
    grid on;
    xlim(t_span_control);
    ylim([-200, 200]);
    
    subplot(2,2,4);
    plot(t_sim_control(1:i), tau_control(1:i,:));
    xlabel('Time (s)');
    ylabel('Control Torque (Nm)');
    title('Control Torques vs Time (Including Gravity Compensation)');
    legend('τ1', 'τ2', 'τ3', 'τ4', 'τ5', 'τ6', 'Location', 'best');
    grid on;
    xlim(t_span_control);
    ylim([-150, 150]);
    
    drawnow;
end

%% Summary Plots
figure('Name', 'Simulation Results Summary', 'Position', [100, 100, 1200, 800]);

% Joint angles comparison
subplot(2,2,1);
plot(t_free, rad2deg(q_free), '--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Angles (degrees)');
title('Free Motion (Zero Input)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'Location', 'best');
grid on;

subplot(2,2,2);
plot(t_sim_control, rad2deg(q_control), 'LineWidth', 1.5);
hold on;
plot(t_sim_control, rad2deg(repmat(q_target', length(t_sim_control), 1)), 'k--');
xlabel('Time (s)');
ylabel('Joint Angles (degrees)');
title('PID Controlled Motion');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q1_{target}', 'q2_{target}', ...
        'q3_{target}', 'q4_{target}', 'q5_{target}', 'q6_{target}', 'Location', 'best');
grid on;

% Tracking error
subplot(2,2,3);
error_control = rad2deg(repmat(q_target', length(t_sim_control), 1) - q_control);
plot(t_sim_control, error_control);
xlabel('Time (s)');
ylabel('Tracking Error (degrees)');
title('Position Tracking Error');
legend('e1', 'e2', 'e3', 'e4', 'e5', 'e6', 'Location', 'best');
grid on;

% Control effort
subplot(2,2,4);
plot(t_sim_control, tau_control);
xlabel('Time (s)');
ylabel('Control Torque (Nm)');
title('Control Effort');
legend('τ1', 'τ2', 'τ3', 'τ4', 'τ5', 'τ6', 'Location', 'best');
grid on;

% Print final statistics
fprintf('\n=== Simulation Results ===\n');
fprintf('Final tracking errors (degrees):\n');
final_error = rad2deg(q_target' - q_control(end,:));
for i = 1:6
    fprintf('  Joint %d: %.3f°\n', i, final_error(i));
end
%% Helper Functions for Energy Calculations

function PE = calculatePotentialEnergy(dh, masses, g, q)
    % Calculate total potential energy of the robot
    [~, origins, rots] = forwardKinematics(dh, q);
    com_positions = computeCOMPositions(dh, origins, rots);
    
    PE = 0;
    for i = 1:6
        % PE = m*g*h where h is the z-coordinate
        PE = PE + masses(i) * g * com_positions(3, i);
    end
end

function M = computeMassMatrix(dh, masses, q)
    % Simplified function to get just the mass matrix
    [~, origins, rots] = forwardKinematics(dh, q);
    com_positions = computeCOMPositions(dh, origins, rots);
    
    n = 6;
    M = zeros(n, n);
    
    for k = 1:n
        J_com = computeLinkJacobian(origins, rots, com_positions(:,k), k);
        Jv = J_com(1:3, :);
        Jw = J_com(4:6, :);
        I_k = computeInertia(dh, masses(k), k, rots(:,:,k+1));
        M = M + masses(k) * (Jv' * Jv) + Jw' * I_k * Jw;
    end
    
    M = 0.5 * (M + M');
    M = M + 1e-8 * eye(n);
end

function dxdt = robotDynamics(t, x, tau, dh, masses, g)
    % Extract states
    q = x(1:6);
    qDot = x(7:12);
    B = diag([2, 2, 1, 0.5, 0.3, 0.2]);   % N·m·s  (example values)
    tau_damp = -B * qDot; % adding dissipation of energy     
    
    % Get dynamics matrices
    [M, C, G] = computeDynamicsMatrices(dh, masses, g, q, qDot);
    
    % Add regularization to prevent singularity
    epsilon = 1e-6;
    M_reg = M + epsilon * eye(6);
    
    % Check condition number
    cond_num = cond(M_reg);
    if cond_num > 1e10
        warning('Mass matrix is poorly conditioned: cond(M) = %e', cond_num);
        % Increase regularization
        M_reg = M + 1e-3 * eye(6);
    end
    
    % Compute acceleration with regularized mass matrix
    qDDot    = M_reg \ (tau + tau_damp - C*qDot - G);
    
    % Limit accelerations to prevent numerical instability
    max_accel = 100;  % rad/s^2
    qDDot = max(min(qDDot, max_accel), -max_accel);
    
    % Return derivatives
    dxdt = [qDot; qDDot];
end

function drawRobot3D(dh, q, lineStyle)
    if nargin < 3
        lineStyle = 'b-';
    end
    
    % Get forward kinematics
    [~, origins, ~] = forwardKinematics(dh, q);
    
    % Draw links
    plot3(origins(1,:), origins(2,:), origins(3,:), lineStyle, 'LineWidth', 3);
    hold on;
    
    % Draw joints
    plot3(origins(1,:), origins(2,:), origins(3,:), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    
    % Draw base
    plot3(0, 0, 0, 'ks', 'MarkerSize', 15, 'MarkerFaceColor', 'k');
    
    % Draw end-effector
    plot3(origins(1,end), origins(2,end), origins(3,end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    % Set axis properties
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    axis equal;
    xlim([-1, 1]);
    ylim([-1, 1]);
    zlim([0, 1.5]);
end

%% Core Functions Used in Simulation

function [T06, origins, rots] = forwardKinematics(dh, thetas)
    % Computes forward kinematics for a 6-DOF robot
    % Inputs:
    %   dh      - struct with Denavit–Hartenberg constants:
    %               dh.d1, dh.a2, dh.a3, dh.d4, dh.d6
    %   thetas  - 6×1 vector of joint angles (radians)
    %
    % Outputs:
    %   T06     - 4×4 homogeneous transform from base to end-effector
    %   origins - 3×7 matrix of joint origins in base frame
    %   rots    - 3×3×7 array of rotation matrices for each joint frame

    % === Ensure exactly six joint angles provided ===
    assert(numel(thetas)==6, 'Must supply a 6×1 vector of thetas');

    % === Unpack only the DH link offsets we need ===
    d1 = dh.d1;
    a2 = dh.a2;
    a3 = dh.a3;
    d4 = dh.d4;
    d6 = dh.d6;

    % === Apply any joint-specific angular offsets ===
    % From the article, Table I shows these offsets
    t = [ thetas(1);
          thetas(2) - pi/2;  % offset of -90 degrees
          thetas(3);
          thetas(4);
          thetas(5);
          thetas(6) - pi ];   % offset of -180 degrees

    % === Define the rest of the DH parameters for all six joints ===
    % From Table I in the article
    a     = [   0,  a2,  a3,   0,   0,   0 ];    % link lengths
    d     = [  d1,   0,   0,  d4,   0,  d6 ];    % link offsets  
    alpha = [-pi/2,  0, -pi/2, pi/2, -pi/2, 0 ]; % link twists (converted from degrees)

    % === Preallocate outputs ===
    T       = eye(4);                         % running transform
    origins = zeros(3,7);                     % positions of each joint
    rots    = repmat(eye(3), [1,1,7]);        % rotation of each joint
    origins(:,1) = [0;0;0];                   % base at origin
    rots(:,:,1) = eye(3);                     % base rotation

    % === Loop through each joint to build the chain ===
    for i = 1:6
        ct = cos(t(i));   st = sin(t(i));      % joint rotation
        ca = cos(alpha(i)); sa = sin(alpha(i));% twist rotation

        % Standard DH transform from frame i-1 to frame i
        Ai = [ ...
            ct,    -st*ca,   st*sa,   a(i)*ct;
            st,     ct*ca,  -ct*sa,   a(i)*st;
             0,        sa,      ca,      d(i);
             0,         0,       0,       1   ];

        % Accumulate transform
        T = T * Ai;

        % Extract and store origin and rotation for this joint
        origins(:,i+1) = T(1:3,4);
        rots(:,:,i+1)  = T(1:3,1:3);
    end

    % === Final transform from base to end-effector ===
    T06 = T;
end

function J_link = computeLinkJacobian(origins, rots, p_target, link_num)
    % Compute Jacobian for a specific point (e.g., COM) up to link_num
    J_link = zeros(6, 6);
    
    for i = 1:link_num
        % z-axis of joint i
        z_i = rots(:, 3, i);
        % position of joint i
        p_i = origins(:, i);
        
        % Linear velocity contribution
        J_link(1:3, i) = cross(z_i, p_target - p_i);
        % Angular velocity contribution
        J_link(4:6, i) = z_i;
    end
end

function I = computeInertia(dh, mass, link_num, R)
    % Compute inertia tensor for uniform rod in world frame
    % R is the rotation matrix from link frame to world frame
    
    % Get link length and add minimum inertia to prevent singularities
    min_inertia = 0.001 * mass;  % Minimum inertia value
    
    if link_num == 1
        L = dh.d1;  % vertical link
        % Inertia about COM for rod along z-axis
        I_local = mass * diag([L^2/12 + min_inertia, L^2/12 + min_inertia, min_inertia]);
    elseif link_num == 2
        L = dh.a2;  % horizontal link
        % Inertia about COM for rod along x-axis
        I_local = mass * diag([min_inertia, L^2/12 + min_inertia, L^2/12 + min_inertia]);
    elseif link_num == 3
        L = dh.a3;  % horizontal link
        % Inertia about COM for rod along x-axis
        I_local = mass * diag([min_inertia, L^2/12 + min_inertia, L^2/12 + min_inertia]);
    elseif link_num == 4
        L = dh.d4;  % vertical link
        % Inertia about COM for rod along z-axis
        I_local = mass * diag([L^2/12 + min_inertia, L^2/12 + min_inertia, min_inertia]);
    elseif link_num == 5
        % Link 5 has no physical extent, use small inertia
        I_local = mass * 0.01 * eye(3);
    else  % link_num == 6
        L = dh.d6;  % vertical link
        % Inertia about COM for rod along z-axis
        I_local = mass * diag([L^2/12 + min_inertia, L^2/12 + min_inertia, min_inertia]);
    end
    
    % Transform to world frame
    I = R * I_local * R';
    
    % Ensure symmetry and positive definiteness
    I = 0.5 * (I + I');
    I = I + 1e-6 * eye(3);
end

function [M, C, G] = computeDynamicsMatrices(dh, masses, g, q, qDot)
    % Computes the dynamics matrices M, C, G from the Lagrangian
    % Using the property that T = 0.5*qDot'*M(q)*qDot
    % 
    % Inputs:
    %   dh     - struct with DH parameters
    %   masses - 1×6 vector of link masses (kg)
    %   g      - gravitational acceleration (m/s^2)
    %   q      - 6×1 vector of joint angles (radians)
    %   qDot   - 6×1 vector of joint velocities (rad/s)
    %
    % Outputs:
    %   M      - 6×6 mass/inertia matrix
    %   C      - 6×6 Coriolis/centrifugal matrix
    %   G      - 6×1 gravity vector
    
    % Ensure q and qDot are column vectors
    q = q(:);
    qDot = qDot(:);
    
    n = 6;  % number of joints
    
    % Initialize matrices
    M = zeros(n, n);
    C = zeros(n, n);
    G = zeros(n, 1);
    
    % Get forward kinematics
    [~, origins, rots] = forwardKinematics(dh, q);
    
    % Compute center of mass positions
    com_positions = computeCOMPositions(dh, origins, rots);
    
    % === Compute Mass Matrix M(q) ===
    % M_ij = sum over all links of (m_k * Jv_k^T * Jv_k + Jw_k^T * I_k * Jw_k)
    % where Jv_k and Jw_k are evaluated at the COM of link k
    
    for k = 1:n  % for each link
        % Get Jacobian for this link's COM
        J_com = computeLinkJacobian(origins, rots, com_positions(:,k), k);
        Jv = J_com(1:3, :);  % linear velocity Jacobian
        Jw = J_com(4:6, :);  % angular velocity Jacobian
        
        % Get inertia tensor in world frame
        I_k = computeInertia(dh, masses(k), k, rots(:,:,k+1));
        
        % Contribution to mass matrix
        M = M + masses(k) * (Jv' * Jv) + Jw' * I_k * Jw;
    end
    
    % Ensure symmetry (numerical errors can break this)
    M = 0.5 * (M + M');
    
    % Add small regularization to ensure positive definiteness
    epsilon = 1e-8;
    M = M + epsilon * eye(n);
    
    % === Compute Gravity Vector G(q) ===
    % G_i = ∂V/∂q_i = -g^(0)^T * Σ(m_i * J_Li)
    % where J_Li is the linear Jacobian to the COM of link i
    
    g_vector = [0; 0; -g];  % gravity vector in world frame (z pointing up)
    
    for i = 1:n  % for each joint i
        G(i) = 0;
        for k = 1:n  % for each link k
            % Get Jacobian for link k's COM
            J_com = computeLinkJacobian(origins, rots, com_positions(:,k), k);
            J_linear = J_com(1:3, :);  % linear velocity part
            
            % Contribution to gravity term for joint i
            % G_i = -g^T * m_k * J_linear(:,i)
            G(i) = G(i) - g_vector' * masses(k) * J_linear(:, i);
        end
    end
    
    % === Compute Coriolis Matrix C(q, qDot) ===
    % Using Christoffel symbols: C_ij = sum_k (0.5 * (∂M_ij/∂q_k + ∂M_ik/∂q_j - ∂M_jk/∂q_i) * qDot_k)
    
    % Compute derivatives of M numerically
    dM = zeros(n, n, n);  % dM(:,:,k) = ∂M/∂q_k
    delta = 1e-6;
    
    for k = 1:n
        q_plus = q;
        q_plus(k) = q_plus(k) + delta;
        
        % Compute M at q + delta
        M_plus = zeros(n, n);
        [~, origins_plus, rots_plus] = forwardKinematics(dh, q_plus);
        com_plus = computeCOMPositions(dh, origins_plus, rots_plus);
        
        for link = 1:n
            J_com = computeLinkJacobian(origins_plus, rots_plus, com_plus(:,link), link);
            Jv = J_com(1:3, :);
            Jw = J_com(4:6, :);
            I_link = computeInertia(dh, masses(link), link, rots_plus(:,:,link+1));
            M_plus = M_plus + masses(link) * (Jv' * Jv) + Jw' * I_link * Jw;
        end
        
        % Numerical derivative
        dM(:,:,k) = (M_plus - M) / delta;
    end
    
    % Compute C using Christoffel symbols
    for i = 1:n
        for j = 1:n
            for k = 1:n
                C(i,j) = C(i,j) + 0.5 * (dM(i,j,k) + dM(i,k,j) - dM(j,k,i)) * qDot(k);
            end
        end
    end
end

function com_positions = computeCOMPositions(dh, origins, rots)
    % Helper function to compute all COM positions
    com_positions = zeros(3, 6);
    
    % Link 1: COM at d1/2 along z0
    com_positions(:, 1) = [0; 0; dh.d1/2];
    
    % Link 2: COM at a2/2 from joint 2
    T_temp = eye(4);
    T_temp(1:3, 1:3) = rots(:,:,2);
    T_temp(1:3, 4) = origins(:,2);
    com_positions(:, 2) = T_temp(1:3, 1:3) * [dh.a2/2; 0; 0] + T_temp(1:3, 4);
    
    % Link 3: COM at a3/2 from joint 3
    T_temp(1:3, 1:3) = rots(:,:,3);
    T_temp(1:3, 4) = origins(:,3);
    com_positions(:, 3) = T_temp(1:3, 1:3) * [dh.a3/2; 0; 0] + T_temp(1:3, 4);
    
    % Link 4: COM at d4/2 along z3
    T_temp(1:3, 1:3) = rots(:,:,4);
    T_temp(1:3, 4) = origins(:,4);
    com_positions(:, 4) = T_temp(1:3, 1:3) * [0; 0; dh.d4/2] + T_temp(1:3, 4);
    
    % Link 5: COM at joint 5
    com_positions(:, 5) = origins(:,5);
    
    % Link 6: COM at d6/2 from joint 6
    T_temp(1:3, 1:3) = rots(:,:,6);
    T_temp(1:3, 4) = origins(:,6);
    com_positions(:, 6) = T_temp(1:3, 1:3) * [0; 0; dh.d6/2] + T_temp(1:3, 4);
end

