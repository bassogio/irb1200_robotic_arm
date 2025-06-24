%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  ik_irb1200.m  –  Full analytical-plus-geometric inverse kinematics
%  for an ABB IRB-1200-style 6-DOF manipulator (R-R-R-R-R-R).
%
%  INPUTS
%    dh   – struct with modified DH constants
%           .d1, .a2, .a3, .d4, .d6 (units must match goal position)
%    goal – 6×1 vector [Px; Py; Pz; roll; pitch; yaw]  (radians)
%
%  OUTPUT
%    Q    – 6×N matrix, each column is a complete joint solution [θ1…θ6]
%
%  NOTES
%  • θ1–θ3 obtained geometrically (planar + law-of-cosines).
%  • θ4–θ6 obtained analytically from the residual rotation ^3R6
%    using Z-Y-Z Euler extraction with singularity handling.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Q = ik_irb1200(dh, goal)

% --- Unpack goal pose ----------------------------------------------------
Px   = goal(1);  Py   = goal(2);  Pz   = goal(3);
roll = goal(4);  pitch = goal(5); yaw  = goal(6);

% Convert RPY (ZYX) → rotation matrix
R06 = eul2rotm([yaw, pitch, roll], 'ZYX');   %#ok<EMUL>

% --- DH short-names ------------------------------------------------------
d1 = dh.d1;  a2 = dh.a2;  d4 = dh.d4;  d6 = dh.d6;
L2 = a2;                         % 2nd link length
L3 = d4 + d6;                    % 3rd link + tool

% --- Pre-compute planar terms for θ1..θ3 -------------------------------
s = Pz - d1;                     % vertical offset
r = hypot(Px, Py);               % radial distance in base plane

% Two candidate base rotations
th1_candidates = [atan2(Py, Px), atan2(Py, Px) + pi];

% Container for all 8 full solutions
Q = [];

for th1 = th1_candidates
    % Rotate wrist point into the arm plane ------------------------------
    cz = (r^2 + s^2 - L2^2 - L3^2)/(2*L2*L3);
    if abs(cz) > 1, continue; end       % unreachable
    
    % Two elbow configurations
    phi = atan2( sqrt(1 - cz^2), cz);   % elbow-down
    for phi = [ phi, -phi ]            % elbow-up
        th3 = -(pi/2 + phi);            % map to robot joint-3
        
        % Two θ2 solutions (mirror about wrist line)
        th2 = primaryTheta2(r, s, L2, L3, phi);
        th2 = [th2, complementTheta2(r, s, L2, L3, phi)];
        
        for th2 = th2
            % Forward kinematics to frame{3}
            T03 = forwardKinematics(dh, [th1; th2; th3; 0; 0; 0]);
            R03 = T03(1:3,1:3);
            
            % Residual wrist rotation
            R36 = R03.' * R06;
            
            % Analytical extraction of θ4 θ5 θ6  (Z-Y-Z Euler)
            if abs(R36(3,3)) < 1-1e-9
                th5 = atan2( sqrt(R36(3,1)^2 + R36(3,2)^2), R36(3,3) );
                th4 = atan2( R36(2,3), R36(1,3) );
                th6 = atan2( R36(3,2),-R36(3,1) );
            else  % Singularity: θ5 ≈ 0 or π
                th5 = 0;
                if R36(3,3) < 0 % θ5 ≈ π
                    th5 = pi;
                end
                th4 = 0;
                th6 = atan2(-R36(1,2), -R36(1,1));
            end
            
            % Append full solution
            Q = [Q, [th1; th2; th3; th4; th5; th6]]; %#ok<AGROW>
        end
    end
end
end
%% ===================================================================== %%
%  Helper: primary θ2 (elbow-down in current half-plane)
function th2 = primaryTheta2(r, s, L2, L3, phi)
omega  = atan2(s, r);
lambda = atan2(L3*sin(phi), L2 + L3*cos(phi));
th2    = -((omega - lambda) - pi/2);
end
%% ===================================================================== %%
%  Helper: complementary θ2 (mirror across radial line)
function th2 = complementTheta2(r, s, L2, L3, phi)
omega  = atan2(s, -r);
lambda = atan2(L3*sin(phi), L2 + L3*cos(phi));
th2    = -((omega - lambda) - pi/2);
end
%% ===================================================================== %%
%  Forward kinematics for IRB1200 (modified DH, radians, returns 4×4)
function T = forwardKinematics(dh, thetas)

% Unpack
d1 = dh.d1;  a2 = dh.a2;  a3 = dh.a3;  d4 = dh.d4;  d6 = dh.d6;

% Joint offsets (IRB1200 specific)
t = [ thetas(1);
      thetas(2) - pi/2;
      thetas(3);
      thetas(4);
      thetas(5);
      thetas(6) - pi ];

a     = [   0,  a2,  a3,   0,   0,   0 ];
d     = [  d1,   0,   0,  d4,   0,  d6 ];
alpha = [-pi/2,  0, -pi/2, pi/2, -pi/2, 0 ];

T = eye(4);
for i = 1:6
    ct = cos(t(i));  st = sin(t(i));
    ca = cos(alpha(i)); sa = sin(alpha(i));
    A = [ ct, -st*ca,  st*sa, a(i)*ct;
          st,  ct*ca, -ct*sa, a(i)*st;
           0,     sa,     ca,       d(i);
           0,      0,      0,        1  ];
    T = T * A;
end
end
