function [J, J_linear, J_angular] = Jacobian(dhParams, jointAngles)
% Inputs:
%   dhParams     - struct or table of DH parameters (a, alpha, d, theta offsets)
%   jointAngles - 1×n vector of joint positions (in radians or degrees)
%
%   returns:
%     J_linear   - 3×n linear velocity sub-Jacobian
%     J_angular  - 3×n angular velocity sub-Jacobian
%     J          - 6×n full Jacobian [J_linear; J_angular]

    % === Step 1: Forward kinematics to get positions & orientations ===
    % origins: 3×(n+1) matrix of frame origins, including base (col 1) to end-effector (col end)
    % rots:    3×3×n array of rotation matrices R_i from base to each joint frame
    [~, jointPositions, rotationMatrices] = forwardKinematics(dhParams, jointAngles);

    % === Step 2: Preallocate sub-Jacobians ===
    n = length(jointAngles);
    J_linear  = zeros(3, n);
    J_angular = zeros(3, n);

    % === Step 3: Compute end-effector position ===
    endEffectorPos = jointPositions(:, end);

    % === Step 4: Populate each column of J ===
    for idx = 1:n
        % axis vector of joint idx (z-axis of its local frame)
        axisVec   = rotationMatrices(:, 3, idx);
        % position of joint idx origin
        jointPos  = jointPositions(:, idx);
        % linear part: z_i × (p_e - p_i)
        J_linear(:, idx)  = cross(axisVec, endEffectorPos - jointPos);
        % angular part: z_i
        J_angular(:, idx) = axisVec;
    end

    % === Step 5: Assemble full Jacobian === 
    J = [J_linear; J_angular];
end
