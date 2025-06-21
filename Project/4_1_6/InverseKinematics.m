function Q = InverseKinematics(dh, goal_cm)
%InverseKinematics  Analytic IK for ABB-IRB1200 (all 8 pose branches)
%   Q = InverseKinematics(dh, goal_cm)
%   • dh        – struct with d1,a2,a3,d4,d6  (mm or “cm-as-mm”, must
%                 match forwardKinematics)
%   • goal_cm   – 3×1 desired TCP position   [x;y;z]  (same units)
%   • Q         – 6×N matrix of joint angles (rad)   (N ≤ 8)

%%---------------- Constant geometry ---------------------------------------
d1 = dh.d1;   a2 = dh.a2;   a3 = dh.a3;
d4 = dh.d4;   d6 = dh.d6;

L3   = hypot(a3, d4);              % “virtual” link-3
beta = atan2(d4, a3);              % built-in offset of link-3

%%---------------- Desired wrist centre ------------------------------------
Px = goal_cm(1);   Py = goal_cm(2);   Pz = goal_cm(3);
Pw = [Px; Py; Pz - d6];             % subtract tool length (R06 = I)

r = hypot(Pw(1), Pw(2));            % projection radius
s = Pw(3) - d1;                     % height above J1

%%---------------- Enumerate branches --------------------------------------
sol = [];
shoulder = [ 1, -1 ];               % Theta₁ front / back-over-shoulder
elbow    = [ 1, -1 ];               % Theta₃ elbow-down / elbow-up
wrist    = [ 1, -1 ];               % Theta₅ positive / negative

for s1 = shoulder
    r1 = s1 * r;                    % signed radius in the chosen half-plane
    
    %% Arm triangle -------------------------------------------------------
    cosPsi = (r1^2 + s^2 - a2^2 - L3^2) / (2*a2*L3);
    if abs(cosPsi) > 1,  continue,  end   % out of reach
    
    psi = acos(cosPsi);             % angle between a₂ and L₃
    
    for s2 = elbow
        Theta1 = atan2(Py, Px) + (s1==-1)*pi;
        Theta3 =  s2*psi  -  beta;
        
        k1 = a2 + L3*cos(s2*psi);
        k2 =      L3*sin(s2*psi);
        
        % ************  corrected formula for Theta₂  *************************
        Theta2 = -atan2(s, r1) + atan2(-s2*k2, k1) + pi/2;
        % *****************************************************************
        
        %% Wrist frame ----------------------------------------------------
        T03   = fk3(Theta1, Theta2, Theta3, d1, a2, a3);   % fast FK up to joint-3
        R36   = T03(1:3,1:3).';                % because R06 = I
        
        % ************  corrected wrist extraction  ***********************
        Theta5mag = atan2( hypot(R36(1,3), R36(2,3)),  R36(3,3) );
        
        for s3 = wrist
            Theta5 =  s3 * Theta5mag;
            
            if abs(sin(Theta5)) < 1e-6          % singular (straight wrist)
                Theta4 = atan2( R36(2,1),  R36(1,1) );
                Theta6 = 0;
            else
                Theta4 = atan2(-R36(1,2), -R36(2,2));
                Theta6 = atan2(-R36(2,1),  R36(1,1));
                if s3 == -1                % mirror branch
                    Theta4 = wrapToPi(Theta4 + pi);
                    Theta6 = wrapToPi(Theta6 + pi);
                end
            end
        % *****************************************************************
            
            q   = [Theta1; Theta2; Theta3; Theta4; Theta5; Theta6];
            T06_fk = forwardKinematics(dh, q);      % call FK and store the 4×4 matrix
            err    = norm( T06_fk(1:3,4) - goal_cm );% compare the XYZ positions

            if err < 1e-3                  % 1 mm tolerance
                sol(:,end+1) = wrapToPi(q); %#ok<AGROW>
            end
        end
    end
end

if isempty(sol)
    error('Target outside workspace – no IK solution found.');
end

Q = uniquetol(sol.', 1e-6, 'ByRows',true).';  % remove duplicates
end
% ========================================================================

function T03 = fk3(t1,t2,t3,d1,a2,a3)
% Fast FK for the first three joints only (no d4 yet)
A1 = dhA(t1, d1, 0, -pi/2);
A2 = dhA(t2-pi/2, 0, a2, 0);
A3 = dhA(t3, 0, a3, -pi/2);
T03 = A1*A2*A3;
end

function A = dhA(theta,d,a,alpha)
ct = cos(theta);  st = sin(theta);
ca = cos(alpha);  sa = sin(alpha);
A  = [ ct, -st*ca,  st*sa, a*ct;
       st,  ct*ca, -ct*sa, a*st;
        0,     sa,     ca,    d;
        0,      0,      0,    1 ];
end
