function [T06, origins, rots] = forwardKinematics(dh, thetas)
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
    t = [ thetas(1);
          thetas(2) - pi/2;
          thetas(3);
          thetas(4);
          thetas(5);
          thetas(6) - pi ];

    % === Define the rest of the DH parameters for all six joints ===
    a     = [   0,  a2,  a3,   0,   0,   0 ];    % link lengths
    d     = [  d1,   0,   0,  d4,   0,  d6 ];    % link offsets
    alpha = [-pi/2,  0, -pi/2, pi/2, -pi/2, 0 ];  % link twists

    % === Preallocate outputs ===
    T       = eye(4);                         % running transform
    origins = zeros(3,7);                     % positions of each joint
    rots    = repmat(eye(3), [1,1,7]);        % rotation of each joint
    origins(:,1) = [0;0;0];                   % base at origin

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
