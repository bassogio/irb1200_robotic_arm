function solutions = inverseKinematics(x, y, z, dh)
% inverseKinematicsIRB1200  Compute all IK solutions for ABB IRB-1200
% Inputs:
%   x, y, z   : desired wrist-center position (cm)
%   dh        : struct with fields d1, a2, a3, d4, d6 (cm)
% Output:
%   solutions : struct array with fields theta1…theta6 (radians)

    % 1. Get symbolic FK
    [T06_sym, ~, rots_sym] = forwardKinematics(dh, []);
    T06_sym = simplify(T06_sym);

    % 2. Extract axes and positions
    syms px py pz ax ay az real
    ax = T06_sym(1,3); ay = T06_sym(2,3); az = T06_sym(3,3);
    px = T06_sym(1,4); py = T06_sym(2,4); pz = T06_sym(3,4);

    % 3. Wrist center
    Wx = px - dh.d6 * ax;
    Wy = py - dh.d6 * ay;
    Wz = pz - dh.d6 * az;

    % 4. Substitute target
    Wx = simplify(subs(Wx, {px,py,pz}, {x,y,z}));
    Wy = simplify(subs(Wy, {px,py,pz}, {x,y,z}));
    Wz = simplify(subs(Wz, {px,py,pz}, {x,y,z}));

    % 5. Planar radius and vertical offset
    r = sqrt(Wx^2 + Wy^2);
    z2 = Wz - dh.d1;

    % 6. Theta1 solutions
    t1 = [ atan2(Wy, Wx);
          atan2(-Wy, -Wx) ];

    % Pre-allocate
    solIdx = 1;
    solutions = struct('theta1', {}, 'theta2', {}, 'theta3', {}, ...
                       'theta4', {}, 'theta5', {}, 'theta6', {});

    for i1 = 1:2
        theta1 = t1(i1);

        % 7. Theta3 via law of cosines
        cos3 = (r^2 + z2^2 - dh.a2^2 - dh.a3^2)/(2*dh.a2*dh.a3);
        sin3_u =  sqrt(1 - cos3^2);
        sin3_d = -sin3_u;
        t3 = [ atan2(sin3_u, cos3);
               atan2(sin3_d, cos3) ];

        % 8. Theta2 geometric
        for i3 = 1:2
            theta3 = t3(i3);
            k1 = dh.a2 + dh.a3*cos3;
            k2 = dh.a3*sin3_u * (1 - 2*(i3==2));  
            theta2 = atan2(z2, r) - atan2(k2, k1);

            % 9. Build numeric R03 and R06
            % Substitute angles into FK
            [T06_num, origins, rots_num] = forwardKinematics(dh, ...
                                             [theta1,theta2,theta3,0,0,0]);
            R03 = rots_num(:,:,4);
            R06 = T06_num(1:3,1:3);
            R36 = R03.' * R06;

            % 10. Theta4,5,6 from R36
            t5 = atan2( sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3) );
            t4 = atan2( R36(2,3)/sin(t5), R36(1,3)/sin(t5) );
            t6 = atan2(-R36(3,1)/sin(t5),  R36(3,2)/sin(t5) );

            % 11. Store solution
            solutions(solIdx).theta1 = double(theta1);
            solutions(solIdx).theta2 = double(theta2);
            solutions(solIdx).theta3 = double(theta3);
            solutions(solIdx).theta4 = double(t4);
            solutions(solIdx).theta5 = double(t5);
            solutions(solIdx).theta6 = double(t6);
            solIdx = solIdx + 1;
        end
    end
end
