function A = CreateTransMat(d, a, t, alpha)
% Creates a transformation matrix using forward kinematics.
% Input:
%   d - Distance of the current X axis along the previous Z axis.
%   a - Distance of the previous Z axis along the current X axis.
%   t - Angle between previous and current X axes, measured in a plane normal to previous Z axis.
%   alpha - Angle between previous and current Z axes, measured in a plane normal to current X axis.
%   
% Output:
%   A - 4X4 Transformation matrix
%    __                                         __
%   |   3X3 Rotation matrix     3X1 Origin vector |
%   |__   1X3 zero vector               1       __|

ct = cos(t);  st = sin(t);
ca = cos(alpha);  sa = sin(alpha);
A = [ ...
    ct,    -st*ca,   st*sa,   a*ct;
    st,     ct*ca,  -ct*sa,   a*st;
    0,        sa,      ca,      d;
    0,         0,       0,       1   ];

end