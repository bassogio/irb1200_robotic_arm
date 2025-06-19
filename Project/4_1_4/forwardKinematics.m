function [T06, origins, rots] = forwardKinematics(dh, thetas)
% forwardKinematics  Compute 0→6 transform, joint origins, and rotations.
% If `thetas` is omitted or empty, returns symbolic expressions in
% theta1…theta6.

    % If no angles provided, define symbolic parameters
    if nargin < 2 || isempty(thetas)
        syms theta1 theta2 theta3 theta4 theta5 theta6 real
        thetas = [theta1; theta2; theta3; theta4; theta5; theta6];
    end

    % Unpack DH constants
    d1 = dh.d1; a2 = dh.a2; a3 = dh.a3; d4 = dh.d4; d6 = dh.d6;

    % Apply joint-specific offsets (same as before)
    t = [ thetas(1);
          thetas(2) - pi/2;
          thetas(3);
          thetas(4);
          thetas(5);
          thetas(6) - pi ];

    % DH arrays
    a     = [   0,  a2,  a3,   0,   0,   0 ];
    d     = [  d1,   0,   0,  d4,   0,  d6 ];
    alpha = [-pi/2,  0, -pi/2, pi/2,-pi/2, 0 ];

    % Initialize outputs
    T       = eye(4);
    origins = sym(zeros(3,7));           % make symbolic if needed
    rots    = sym(repmat(eye(3),[1,1,7]));
    origins(:,1) = [0;0;0];

    % Build transforms
    for i = 1:6
        ct = cos(t(i));  st = sin(t(i));
        ca = cos(alpha(i));  sa = sin(alpha(i));
        Ai = [ ...
            ct,    -st*ca,   st*sa,   a(i)*ct;
            st,     ct*ca,  -ct*sa,   a(i)*st;
             0,        sa,      ca,      d(i);
             0,         0,       0,       1   ];
        T = T * Ai;
        origins(:,i+1) = T(1:3,4);
        rots(:,:,i+1)  = T(1:3,1:3);
    end

    T06 = T;
end
