function Q = InverseKinematicsRPY(dh, goal, rpy_deg)
% Extended IK to solve for position and orientation
%
% Inputs:
%   dh       - DH parameters struct
%   goal     - 3×1 position (mm)
%   rpy_deg  - 3×1 desired orientation in RPY (degrees)
%
% Output:
%   Q        - 6×N matrix of joint solutions (radians)

    d1_val = dh.d1;
    L2     = dh.a2;
    L3     = dh.d4 + dh.d6;

    Px = goal(1);
    Py = goal(2);
    Pz = goal(3);

    % Desired end-effector orientation as rotation matrix
    R_target = eul2rotm(deg2rad(rpy_deg'), 'ZYX');

    % Solve for position part
    th1_1 = atan2(Py, Px);
    th1_2 = th1_1 + pi;
    baseAngles = [th1_1, th1_2];

    Q = [];

    s = Pz - d1_val;
    r = hypot(Px, Py);

    for i = 1:2
        th1 = baseAngles(i);
        cz = (r^2 + s^2 - L2^2 - L3^2) / (2 * L2 * L3);
        if abs(cz) > 1
            continue;
        end

        phi1 = atan2( sqrt(1 - cz^2), cz);
        phi2 = atan2(-sqrt(1 - cz^2), cz);
        phiAngles = [phi1, phi2];

        for j = 1:2
            phi = phiAngles(j);
            th3 = -(pi/2 + phi);

            th2a = computeTh2(r, s, L2, L3, phi);
            th2b = computeTh2Comp(r, s, L2, L3, phi);

            % === Wrist orientation solution ===
            for th2 = [th2a, th2b]
                th123 = [th1; th2; th3];

                % Compute T03
                [~, ~, ~, T03] = forwardKinematicsUpTo3(dh, th123);
                R03 = T03(1:3,1:3);

                % Compute required wrist rotation
                R36 = R03' * R_target;

                % Solve wrist angles from R36
                [th4, th5, th6] = wristAnglesFromR36(R36);

                % Collect solution
                Q = [Q, [th1; th2; th3; th4; th5; th6]];
            end
        end
    end
end

% === Helper: primary θ2 computation ===
function th2 = computeTh2(r, s, L2, L3, phi)
    omega  = atan2(s, r);
    lambda = atan2(L3*sin(phi), L2 + L3*cos(phi));
    th2    = -((omega - lambda) - pi/2);
end

% === Helper: complementary θ2 computation ===
function th2 = computeTh2Comp(r, s, L2, L3, phi)
    omega  = atan2(s, -r);
    lambda = atan2(L3*sin(phi), L2 + L3*cos(phi));
    th2    = -((omega - lambda) - pi/2);
end

% === Helper: Forward Kinematics up to joint 3 ===
function [origins, rots, Ts, T03] = forwardKinematicsUpTo3(dh, jointAngles)
    [Ts, origins, rots] = forwardKinematics(dh, [jointAngles; 0; 0; 0]);
    T03 = Ts(:,:,3);
end

% === Helper: Wrist angle extraction ===
function [th4, th5, th6] = wristAnglesFromR36(R)
    th5 = atan2( sqrt(R(1,3)^2 + R(2,3)^2), R(3,3) );

    if abs(th5) < 1e-6
        th4 = 0;
        th6 = atan2(-R(1,2), R(1,1));
    else
        th4 = atan2(R(2,3), R(1,3));
        th6 = atan2(R(3,2), -R(3,1));
    end
end
