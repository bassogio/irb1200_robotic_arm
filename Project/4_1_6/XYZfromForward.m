% printIRB1200  –  Aligned table of XYZ positions for test joint angles
clc; clear; close all;

%% 1. Robot DH constants (cm)
params = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

%% 2. Test joint configs (deg)
test_angles = [ ...
     0   0    0   0   0   0;
    90   0    0   0   0   0;
    90  90    0   0   0   0;
    90  90  -90   0   0   0;
    90  90  -90  90   0   0;
    90  90  -90  90  90   0;
    90  90  -90  90  90  90 ];

%% 3. Column widths
wAng = 6;          % width for each θ column
wPos = 10;         % width for each XYZ column

%% 4. Header
fprintf('\n%-36s | %-30s\n','Joint angles (deg)','End-effector position (cm)');
fprintf('%6s%6s%6s%6s%6s%6s |%10s%10s%10s\n','θ1','θ2','θ3','θ4','θ5','θ6','X','Y','Z');
fprintf('%s\n', repmat('-',1,36 + 1 + 30));   % 36 = 6*6, 30 = 3*10, +1 for " | "

%% 5. Loop through each configuration
for k = 1:size(test_angles,1)
    % Forward kinematics
    ang_rad = deg2rad(test_angles(k,:));
    [~, origins, ~] = forwardKinematics(params, ang_rad);
    p = origins(:, end);   % XYZ of end-effector (cm)

    % Print aligned row
    fprintf('%6.0f%6.0f%6.0f%6.0f%6.0f%6.0f |%10.2f%10.2f%10.2f\n', ...
            test_angles(k,:), p);
end
