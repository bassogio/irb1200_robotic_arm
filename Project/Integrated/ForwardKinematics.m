function [T06, origins, rots] = ForwardKinematics(dh, t)
% ForwardKinematics  Compute 0→6 transform, joint origins, and rotations.
%   Input:
%     dh – struct with fields a, d, alpha. Contains arrays of same size.
%     t  – 6×1 vector of joint angles [θ1;…;θ6] in radians.
%
%   Output:
%     T06     – 4×4 homogeneous transform from base→tool
%     origins – 3×7 matrix of joint positions (columns 1→7)
%     rots    – 3×3×7 array of joint rotation matrices

assert(numel(t)==6, 'Must supply a 6×1 vector of thetas');

% Unpack DH constants
a = dh.a;
d = dh.d;
alpha = dh.alpha;

% Preallocate
T       = (eye(4));
origins = sym(zeros(3,7));
rots    = repmat(sym(eye(3)), [1,1,7]);
origins(:,1) = [0;0;0];

% Build chain
for i = 1:6
    Ai = CreateTransMat(d(i), a(i), t(i), alpha(i));
    T = simplify(T * Ai);
    origins(:,i+1) = T(1:3,4);
    rots(:,:,i+1)  = T(1:3,1:3);
end

T06 = T;
end