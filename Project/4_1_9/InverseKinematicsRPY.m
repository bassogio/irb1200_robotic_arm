function Q = InverseKinematicsRPY(dh, pose)
    % Inputs:
    %   dh   – struct: dh.d1, dh.a2, dh.d4, dh.d6
    %   pose – 6×1 [Px; Py; Pz; roll; pitch; yaw] (radians)
    %
    % Output:
    %   Q    – 6×M matrix of joint solutions

    % Unpack position & orientation
    Px    = pose(1);  Py    = pose(2);  Pz    = pose(3);
    roll  = pose(4);  pitch = pose(5);  yaw   = pose(6);

    % Precompute desired end-effector rotation (ZYX Euler)
    R_des = eul2rotm([yaw, pitch, roll], 'ZYX');

    % Link geometry
    d1_val = dh.d1;
    L2 = dh.a2;
    L3 = dh.d4 + dh.d6;

    % Base-angle candidates
    s = Pz - d1_val;
    r = hypot(Px, Py);
    th1_1 = atan2(Py, Px);
    th1_2 = th1_1 + pi;
    baseAngles = [th1_1, th1_2];

    Q = [];  % collect solutions

    for iBase = 1:2
      th1 = baseAngles(iBase);

      % 2D elbow subproblem
      cz = (r^2 + s^2 - L2^2 - L3^2) / (2*L2*L3);
      if abs(cz)>1, continue; end
      phiAngles = [ atan2( sqrt(1-cz^2), cz ), atan2(-sqrt(1-cz^2), cz) ];

      for phi = phiAngles
        th3 = -(pi/2 + phi);
        th2a = computeTh2(r, s, L2, L3, phi);
        th2b = computeTh2Comp(r, s, L2, L3, phi);
        for th2 = [th2a, th2b]
          % --- compute R03 from first three joints ---
          A1 = dhA(dh, th1, 1);
          A2 = dhA(dh, th2 - (1==2)*pi/2, 2);  % include joint offset for joint-2
          A3 = dhA(dh, th3, 3);
          R03 = (A1*A2*A3)(1:3,1:3);

          % --- wrist rotation ---
          R36 = R03' * R_des;
          % extract via rotm2eul:
          eulZYX = rotm2eul(R36, 'ZYX'); 
          th6 = eulZYX(1);
          th5 = eulZYX(2);
          th4 = eulZYX(3);

          % primary solution
          sol = [th1; th2; th3; th4; th5; th6];
          Q = [Q, sol];

          % optional flipped wrist
          sol_flip = [th1; th2; th3; th4+pi; -th5; th6+pi];
          Q = [Q, sol_flip];
        end
      end
    end
end

function A = dhA(dh, theta, jointIdx)
    % build single-joint DH transform using your alpha, a, d
    alpha = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0];
    a     = [0, dh.a2, dh.a3, 0, 0, 0];
    d     = [dh.d1, 0, 0, dh.d4, 0, dh.d6];
    ct = cos(theta);  st = sin(theta);
    ca = cos(alpha(jointIdx)); sa = sin(alpha(jointIdx));
    A = [ ct, -st*ca,  st*sa,  a(jointIdx)*ct;
          st,  ct*ca, -ct*sa,  a(jointIdx)*st;
          0,     sa ,     ca ,  d(jointIdx);
          0,      0 ,      0 ,      1       ];
end

% (computeTh2 and computeTh2Comp unchanged)
