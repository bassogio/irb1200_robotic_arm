function Q = InverseKinematics(dh, goal)
    % Inputs:
    %   dh    - struct with DH parameters:
    %             dh.d1, dh.a2, dh.d4, dh.d6
    %   goal  - 3×1 vector [Px; Py; Pz] target position (same units as DH)
    %
    % Output:
    %   Q     - 6×N matrix of joint angles (radians), columns are solutions

    % === 1) Unpack relevant DH link lengths ===
    d1_val = dh.d1;                  % base height
    L2     = dh.a2;                  % second link length
    L3     = dh.d4 + dh.d6;          % combined third+tool length

    % === 2) Extract target coordinates ===
    Px = goal(1); 
    Py = goal(2);
    Pz = goal(3);

    % === 3) Compute two base rotation (θ1) options ===
    th1_1      = atan2(Py, Px);      % “elbow in” solution
    th1_2      = th1_1 + pi;         % “elbow out” solution (flip 180°)
    baseAngles = [th1_1, th1_2];

    % === 4) Prepare storage for solutions ===
    Q = [];                          % will grow to 6×N

    % === 5) Precompute planar coordinates for the 2D IK subproblem ===
    s = Pz - d1_val;                 % vertical offset from joint2
    r = hypot(Px, Py);               % radial distance in base plane

    % === Loop over each base angle and both elbow bends ===
    for i = 1:2
        th1 = baseAngles(i);

        % Law of cosines for θ3
        cz = (r^2 + s^2 - L2^2 - L3^2) / (2 * L2 * L3);
        if abs(cz) > 1
            continue;  % target unreachable for this base angle
        end

        % Two possible elbow angles φ (positive & negative bend)
        phi1 = atan2( sqrt(1 - cz^2),  cz );
        phi2 = atan2(-sqrt(1 - cz^2),  cz );
        phiAngles = [phi1, phi2];

        for j = 1:2
            phi = phiAngles(j);
            th3 = -(pi/2 + phi);     % map φ to robot’s joint-3 convention

            % Compute two θ2 roots (primary and “complementary”)
            th2a = computeTh2(r, s, L2, L3, phi);
            th2b = computeTh2Comp(r, s, L2, L3, phi);

            % For simplicity, set wrist joints θ4–θ6 = 0 (tool orientation fixed)
            sol1 = [th1; th2a; th3; 0; 0; 0];
            sol2 = [th1; th2b; th3; 0; 0; 0];

            % Append both solutions
            Q = [Q, sol1, sol2];
        end
    end
end

% === Helper: primary θ2 computation ===
function th2 = computeTh2(r, s, L2, L3, phi)
    omega  = atan2(s, r);                  % angle from horizontal to target
    lambda = atan2(L3*sin(phi), L2 + L3*cos(phi));
    % adjust by robot’s zero-offset convention
    th2    = -((omega - lambda) - pi/2);
end

% === Helper: complementary θ2 computation ===
function th2 = computeTh2Comp(r, s, L2, L3, phi)
    omega  = atan2(s, -r);                 % flip radial sign for second root
    lambda = atan2(L3*sin(phi), L2 + L3*cos(phi));
    th2    = -((omega - lambda) - pi/2);
end
