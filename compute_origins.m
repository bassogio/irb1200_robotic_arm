function origins = compute_origins(joint_angles_deg)
    % Load symbolic FK definitions
    run('irb1200_ForwardKinematics.m');

    % Define link lengths in meters
    d_vals = [0.399, 0.35, 0.42, 0.351, 0.082];  % d1, d2, d3, d4, d6
    joint_angles_rad = deg2rad(joint_angles_deg);

    % Substitute numerical values into symbolic matrices
    T01_num = double(subs(T01, [theta1, d1], [joint_angles_rad(1), d_vals(1)]));
    T12_num = double(subs(T12, [theta2, d2], [joint_angles_rad(2), d_vals(2)]));
    T23_num = double(subs(T23, [theta3, d3], [joint_angles_rad(3), d_vals(3)]));
    T34_num = double(subs(T34, [theta4, d4], [joint_angles_rad(4), d_vals(4)]));
    T45_num = double(subs(T45, theta5, joint_angles_rad(5)));
    T56_num = double(subs(T56, [theta6, d6], [joint_angles_rad(6), d_vals(5)]));

    % Compute full transforms
    T02 = T01_num * T12_num;
    T03 = T02 * T23_num;
    T04 = T03 * T34_num;
    T05 = T04 * T45_num;
    T06 = T05 * T56_num;

    % Extract joint origins
    origins = zeros(3, 7);  % 6 joints + base
    origins(:, 1) = [0; 0; 0];
    origins(:, 2) = T01_num(1:3, 4);
    origins(:, 3) = T02(1:3, 4);
    origins(:, 4) = T03(1:3, 4);
    origins(:, 5) = T04(1:3, 4);
    origins(:, 6) = T05(1:3, 4);
    origins(:, 7) = T06(1:3, 4);
end
