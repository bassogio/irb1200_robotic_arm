function Q = InverseKinematics(dh, goal)
    % 1) Unpack DH parameters
    d1_val = dh.d1;
    L2     = dh.a2;
    L3     = dh.d4 + dh.d6;

    % 2) Target coordinates
    Px = goal(1);
    Py = goal(2);
    Pz = goal(3);

    % 3) Base-joint th1 solutions
    th1_1      = atan2(Py, Px);
    th1_2      = th1_1 + pi;
    baseAngles = [th1_1, th1_2];

    % 4) Preallocate solution matrix
    Q = [];

    % 5) Common planar terms
    s = Pz - d1_val;
    r = hypot(Px, Py);

    % 6) Loop over base angles and elbow configurations
    for i = 1:2
        th1 = baseAngles(i);
        cz  = (r^2 + s^2 - L2^2 - L3^2) / (2 * L2 * L3);
        if abs(cz) > 1
            continue;  % out of reach
        end
        % two elbow options
        phi1 = atan2( sqrt(1 - cz^2),  cz);
        phi2 = atan2(-sqrt(1 - cz^2),  cz);
        phiAngles = [phi1, phi2];

        for j = 1:2
            phi = phiAngles(j);
            th3 = -(pi/2 + phi);
            % two th2 roots
            th2a = computeTh2(r, s, L2, L3, phi);
            th2b = computeTh2Comp(r, s, L2, L3, phi);
            % append solutions (th4–th6 = 0)
            Q = [Q, [th1; th2a; th3; 0; 0; 0], [th1; th2b; th3; 0; 0; 0]];
        end
    end
end

function th2 = computeTh2(r, s, L2, L3, phi)
    omega  = atan2(s, r);
    lambda = atan2(L3*sin(phi), L2 + L3*cos(phi));
    th2     = -((omega - lambda) - pi/2);
end

function th2 = computeTh2Comp(r, s, L2, L3, phi)
% computeTh2Comp  Second solution for joint 2
    omega  = atan2(s, -r);
    lambda = atan2(L3*sin(phi), L2 + L3*cos(phi));
    th2     = -((omega - lambda) - pi/2);
end
