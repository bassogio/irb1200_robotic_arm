function theta_best = inverse_kinematics_Guy(position, T06, ref_theta)
    % Returns the best inverse kinematics solution for ABB IRB1200
    % Inputs:
    %   position   : [x; y; z] end-effector position (not used, but kept for legacy)
    %   T06        : 4x4 homo
    % geneous transform of end-effector pose
    %   ref_theta  : 1x6 vector of desired joint angles to match (default = zeros)
    % Output:
    %   theta_best : 1x6 joint angles [rad] closest to ref_theta

    % Robot dimensions (in meters)
    d1 = 0.399; a2 = 0.350; a3 = 0.042; d4 = 0.351; d6 = 0.082;

    % Default reference angles
    if nargin < 3
        ref_theta = [0 0 0 0 0 0];
    end

    % Extract wrist center from T06
    T06 = double(T06);  % Ensure numeric
    px = T06(1,4); py = T06(2,4); pz = T06(3,4);
    ax = T06(1,3); ay = T06(2,3); az = T06(3,3);

    Px = px - d6 * ax;
    Py = py - d6 * ay;
    Pz = pz - d6 * az;

    x = sqrt(Px^2 + Py^2);
    z = Pz - d1;

    % Two possible theta1 values
    theta1_opts = [atan2(Py, Px), atan2(-Py, -Px)];

    % Compute theta3 using cosine law
    D = (x^2 + z^2 - a2^2 - a3^2) / (2 * a2 * a3);
    D = min(max(D, -1), 1);  % Clamp to [-1, 1]

    theta3_opts = [atan2(sqrt(1 - D^2), D), atan2(-sqrt(1 - D^2), D)];

    % Store all valid solutions
    solutions = [];

    for i = 1:2
        theta1 = theta1_opts(i);
        for j = 1:2
            theta3 = theta3_opts(j);
            k1 = a2 + a3 * cos(theta3);
            k2 = a3 * sin(theta3);
            theta2 = atan2(z, x) - atan2(k2, k1);

            % Compute R03
            R01 = rotz(rad2deg(theta1)) * rotx(-90);
            R12 = roty(rad2deg(theta2));
            R23 = rotx(-90) * roty(rad2deg(theta3));
            R03 = R01 * R12 * R23;

            % Compute R36
            R06 = T06(1:3, 1:3);
            R36 = R03' * R06;

            % Extract rotation matrix elements
            r13 = R36(1,3); r23 = R36(2,3); r33 = R36(3,3);
            r31 = R36(3,1); r32 = R36(3,2);

            for flip = [+1, -1] % wrist flip
                theta5 = atan2(flip * sqrt(r13^2 + r23^2), r33);

                if abs(sin(theta5)) < 1e-6
                    theta4 = 0;
                    theta6 = atan2(-R36(1,2), R36(1,1));
                else
                    theta4 = atan2(flip * r13, flip * r23);
                    theta6 = atan2(flip * r31, flip * r32);
                end

                theta_sol = real([theta1, theta2, theta3, theta4, theta5, theta6]);
                solutions = [solutions; theta_sol];
            end
        end
    end

    % Pick solution closest to reference
    diffs = vecnorm(solutions - ref_theta, 2, 2);
    [~, idx] = min(diffs);
    theta_best = solutions(idx, :);
end

% === Helper functions ===
function R = rotx(t)
    t = deg2rad(t);
    R = [1 0 0;
         0 cos(t) -sin(t);
         0 sin(t) cos(t)];
end

function R = roty(t)
    t = deg2rad(t);
    R = [cos(t) 0 sin(t);
         0 1 0;
        -sin(t) 0 cos(t)];
end

function R = rotz(t)
    t = deg2rad(t);
    R = [cos(t) -sin(t) 0;
         sin(t)  cos(t) 0;
         0       0      1];
end
