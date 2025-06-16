clc;
clear;
close all;

% Step 1: Run the symbolic forward kinematics script
run('irb1200_ForwardKinematics.m');

% Step 2: Define test joint angles (in degrees)
test_cases_deg = [
    0    0    0    0    0    0;
    90   0    0    0    0    0;
    0    90   0    0    0    0;
    0    0    90   0    0    0;
    0    0    0    90   0    0;
    0    0    0    0    90   0;
    0    0    0    0    0    90;
    90  90   90   90   90   90
];

% Convert to radians
test_cases_rad = deg2rad(test_cases_deg);

% Step 3: Set link lengths 
d_vals = [0.4, 0.3, 0.2, 0.15, 0.1];  % [d1, d2, d3, d4, d6]

% Step 4: Evaluate T06 for each test case
fprintf('==== Testing Forward Kinematics ====\n');
for i = 1:size(test_cases_rad, 1)
    joint_vals = test_cases_rad(i, :);
    T06_eval = subs(T06, {
        theta1, theta2, theta3, theta4, theta5, theta6, ...
        d1, d2, d3, d4, d6}, ...
        {joint_vals(1), joint_vals(2), joint_vals(3), ...
         joint_vals(4), joint_vals(5), joint_vals(6), ...
         d_vals(1), d_vals(2), d_vals(3), d_vals(4), d_vals(5)});
    
    % Convert symbolic to double for display
    T06_eval = double(vpa(T06_eval, 4));
    
    fprintf('\n--- Test Case %d ---\n', i);
    disp(['Joint angles (deg): ', mat2str(test_cases_deg(i,:))]);
    disp('End-effector Transformation Matrix:');
    disp(T06_eval);
    disp(['End-effector position (x,y,z): ', ...
        mat2str(T06_eval(1:3, 4)', 4)]);
end
