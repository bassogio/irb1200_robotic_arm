function [position, T06_eval] = forward_kinematics_irb1200(joint_angles)
    % joint_angles: 1x6 vector [theta1, ..., theta6] in radians

    % Robot dimensions
    d1_val = 0.399; a2_val = 0.350; a3_val = 0.042; d4_val = 0.351; d6_val = 0.082;

    % Declare symbolic variables
    syms theta1 theta2 theta3 theta4 theta5 theta6 real
    syms d1 a2 a3 d4 d6 real

    % Define transformation matrices (symbolic)
    T01 = [ cos(theta1)   0  -sin(theta1)   0;
            sin(theta1)   0   cos(theta1)   0;
                 0       -1       0        d1;
                 0        0       0         1 ];

    T12 = [ sin(theta2)   cos(theta2)  0   a2*sin(theta2);
           -cos(theta2)   sin(theta2)  0  -a2*cos(theta2);
                0              0       1       0;
                0              0       0       1 ];

    T23 = [ cos(theta3)   0  -sin(theta3)   a3*cos(theta3);
            sin(theta3)   0   cos(theta3)   a3*sin(theta3);
                 0       -1       0         0;
                 0        0       0         1 ];

    T34 = [ cos(theta4)   0   sin(theta4)   0;
            sin(theta4)   0  -cos(theta4)   0;
                0         1       0        d4;
                0         0       0         1 ];

    T45 = [ cos(theta5)   0  -sin(theta5)   0;
            sin(theta5)   0   cos(theta5)   0;
                0        -1       0         0;
                0         0       0         1 ];

    T56 = [ -cos(theta6)   sin(theta6)   0    0;
            -sin(theta6)  -cos(theta6)   0    0;
                 0             0         1   d6;
                 0             0         0    1 ];

    % Full symbolic transformation
    T06 = simplify(T01 * T12 * T23 * T34 * T45 * T56);

    % Apply numeric joint values and constants
    theta_vals = num2cell(joint_angles);
    T06_eval = double(subs(T06, ...
        {theta1, theta2, theta3, theta4, theta5, theta6, ...
         d1, a2, a3, d4, d6}, ...
        {theta_vals{:}, d1_val, a2_val, a3_val, d4_val, d6_val}));

    % Extract position
    position = T06_eval(1:3, 4);
end
