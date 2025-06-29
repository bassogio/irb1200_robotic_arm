clc; clear; close all;

% === DH constants ===
params = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

% ===Test joint configs (deg) ===
test_angles = [ ...
     0   0    0   0   0   0;
    90   0    0   0   0   0;
    90  90    0   0   0   0;
    90  90  -90   0   0   0;
    90  90  -90  90   0   0;
    90  90  -90  90  90   0;
    90  90  -90  90  90  90 ];
% test_angles = [0, -45, 45, 0, 90, 0];

axisLen = 50;                      % arrow length in cm
quivOpts = {'AutoScale','off','MaxHeadSize',1,'LineWidth',1.2};
cols3    = {'r','g','b'};         % X=red, Y=green, Z=blue
labels   = {'X','Y','Z'};

for k = 1:size(test_angles,1)
    % New figure per configuration
    fig = figure('Name',sprintf('Pose [%s]°',num2str(test_angles(k,:))), ...
                 'NumberTitle','off');
    
    % Compute forward kinematics
    ang_rad = deg2rad(test_angles(k,:));
    [T06, origins, rots] = forwardKinematics(params, ang_rad);
    
    % Print end-effector rotation matrix
    R06 = T06(1:3,1:3);
    fprintf('Rotation matrix for config [%s]°:\n', num2str(test_angles(k,:)));
    disp(R06);

    % Extract and print end-effector position
    P06 = T06(1:3,4);
    fprintf('Position for config [%s]°: [%.2f, %.2f, %.2f] cm\n\n', ...
            num2str(test_angles(k,:)), P06(1), P06(2), P06(3));


    % Plot links
    plot3(origins(1,:),origins(2,:),origins(3,:),'-ok', ...
          'LineWidth',2,'MarkerSize',4);
    hold on;
    
    % Plot joint frames
    for j = 1:size(origins,2)
        O = origins(:,j);
        R = rots(:,:,j);
        for ii = 1:3
            v = R(:,ii)*axisLen;
            quiver3(O(1),O(2),O(3), v(1),v(2),v(3), ...
                    cols3{ii}, quivOpts{:});
            tip = O + v*1.1;
            text(tip(1),tip(2),tip(3), sprintf('%s%d',labels{ii},j-1), ...
                 'FontSize',9,'FontWeight','bold');
        end
    end
    
    % Auto-scale axes (no padding)
    axis equal; grid on; 
    xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
    view(45,30);
    
    % **Add title with the joint angles**
    title(sprintf('Joint angles: [%s]°', num2str(test_angles(k,:))));
    
    hold off;
end
