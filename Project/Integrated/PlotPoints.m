function PlotPoints(dh, angles, ang_offset)
% Plots simulated robotic arm in all the orientations required
% Input:
%   dh – struct with fields a, d, alpha. Contains arrays of same size.
%   angles - Array of all the desired joint angles

axisLen = 50;                      % arrow length in mm
quivOpts = {'AutoScale','off','MaxHeadSize',1,'LineWidth',1.2};
cols3    = {'r','g','b'};         % X=red, Y=green, Z=blue
labels   = {'X','Y','Z'};

for k = 1:size(angles,1)
    % New figure per configuration
    fig = figure('Name',sprintf('Pose [%s]°',num2str(angles(k,:))), ...
        'NumberTitle','off');

    % Compute forward kinematics
    ang = deg2rad(angles(k,:)) + ang_offset;
    [~, orgs, rots] = ForwardKinematics(dh, ang);

    % Plot links
    plot3(orgs(1,:),orgs(2,:),orgs(3,:),'-ok', ...
        'LineWidth',2,'MarkerSize',4);
    hold on;

    % Plot joint frames
    for j = 1:size(orgs,2)
        O = orgs(:,j);
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
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    view(45,30);

    % **Add title with the joint angles**
    title(sprintf('Joint angles: [%s]°', num2str(angles(k,:))));

    hold off;
end
end