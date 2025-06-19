function [T06, origins, rots] = forwardKinematics(dh, thetas)
% forwardKinematics  Compute 0→6 transform, joint origins, and rotations.
%   Input:
%     dh     – struct with fields d1,a2,a3,d4,d6
%     thetas – 6×1 vector of joint angles [θ1;…;θ6] in radians
%
%   Output:
%     T06     – 4×4 homogeneous transform from base→tool
%     origins – 3×7 matrix of joint positions (columns 1→7)
%     rots    – 3×3×7 array of joint rotation matrices

    assert(numel(thetas)==6, 'Must supply a 6×1 vector of thetas');

    % Unpack DH constants
    d1 = dh.d1;  a2 = dh.a2;  a3 = dh.a3;
    d4 = dh.d4;  d6 = dh.d6;

    % Apply joint-specific offsets
    t = [ thetas(1);
          thetas(2) - pi/2;
          thetas(3);
          thetas(4);
          thetas(5);
          thetas(6) - pi];

    % DH parameters
    a     = [   0,  a2,  a3,   0,   0,   0 ];
    d     = [  d1,   0,   0,  d4,   0,  d6 ];
    alpha = [-pi/2,  0, -pi/2, pi/2, -pi/2, 0 ];

    % Preallocate
    T       = eye(4);
    origins = zeros(3,7);
    rots    = repmat(eye(3), [1,1,7]);
    origins(:,1) = [0;0;0];

    % Build chain
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
