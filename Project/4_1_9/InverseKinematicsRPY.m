function Q = InverseKinematicsRPY(dh, goal)
    % Inputs:
    %   dh    - struct with DH parameters
    %   goal  - 6x1 vector [X; Y; Z; Roll; Pitch; Yaw] in mm and degrees
    %
    % Output:
    %   Q     - 6xN matrix of joint angles (radians), columns are solutions

    % === Unpack goal ===
    Px = goal(1);
    Py = goal(2);
    Pz = goal(3);
    roll  = deg2rad(goal(4));
    pitch = deg2rad(goal(5));
    yaw   = deg2rad(goal(6));

    % === Build desired rotation matrix R06 ===
    R06 = rpy2rotm(roll, pitch, yaw);

    % === Solve joints 1-3 as before (position only IK) ===
    position_only_solutions = InverseKinematics(dh, [Px; Py; Pz]);

    % === For each position solution, compute wrist angles 4-6 ===
    Q = [];
    for i = 1:size(position_only_solutions,2)
        th1 = position_only_solutions(1,i);
        th2 = position_only_solutions(2,i);
        th3 = position_only_solutions(3,i);

        q123 = [th1; th2; th3];

        % Compute T03 from FK using only joints 1-3
        [T03, ~, ~] = forwardKinematics_partial(dh, q123);

        R03 = T03(1:3,1:3);

        % Solve for wrist rotation
        R36 = R03' * R06;

        % Extract wrist Euler angles (Z-Y-X / Roll-Pitch-Yaw)
        [th4, th5, th6] = rotm2rpy(R36);

        % Combine full solution
        q_full = [th1; th2; th3; th4; th5; th6];
        Q = [Q, q_full];
    end
end

function R = rpy2rotm(roll, pitch, yaw)
    % Standard ZYX convention
    Rz = [cos(yaw) -sin(yaw) 0;
          sin(yaw)  cos(yaw) 0;
               0         0   1];

    Ry = [cos(pitch) 0 sin(pitch);
               0     1     0;
         -sin(pitch) 0 cos(pitch)];

    Rx = [1     0          0;
          0 cos(roll) -sin(roll);
          0 sin(roll)  cos(roll)];

    R = Rz * Ry * Rx;
end

function [roll, pitch, yaw] = rotm2rpy(R)
    % Standard ZYX convention
    pitch = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
    if abs(cos(pitch)) < 1e-6
        % Gimbal lock
        roll = 0;
        yaw = atan2(-R(2,3), R(2,2));
    else
        roll = atan2(R(3,2), R(3,3));
        yaw  = atan2(R(2,1), R(1,1));
    end
end

function [T03, origins, rots] = forwardKinematics_partial(dh, thetas)
    % Only computes up to joint 3 for positioning
    assert(numel(thetas) == 3, 'Need 3 angles for partial FK');
    t_full = [thetas; 0; 0; 0];

    [T_full, origins, rots] = forwardKinematics(dh, t_full);
    T03 = rots(:,:,4);
    T03 = [T03 origins(:,4); 0 0 0 1];
end
