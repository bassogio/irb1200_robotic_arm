% testInverseIK.m
clc; clear; close all;

% DH constants (cm)
dh = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

% Define test positions [x, y, z] in cm
test_pts = [ ...
    200,   0, 300;   % in front, above base
      0, 200, 300;   % to the side
    -200, 100, 200;  % behind & to the side
    100, -150, 250   % diagonal down
];

axisLen = 50;                      % axis arrow length
quivOpts = {'AutoScale','off','MaxHeadSize',1,'LineWidth',1.2};
cols3    = {'r','g','b'};          % X=red, Y=green, Z=blue
labels   = {'X','Y','Z'};

for k = 1:size(test_pts,1)
    P = test_pts(k,:)';
    
    % Build T06: identity rotation + desired position
    T06 = [ eye(3), P; 
            0 0 0    1 ];
    
    % Compute IK solutions (expects inverseKinematics.m on path)
    sols = InverseKinematic(dh, T06);
    
    % New figure per test point
    fig = figure('Name',sprintf('Target (%.0f,%.0f,%.0f)',P), ...
                 'NumberTitle','off');
    hold on; grid on; axis equal;
    xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
    view(45,30);
    
    % Plot the desired point
    plot3(P(1),P(2),P(3),'kp','MarkerSize',12,'MarkerFaceColor','y');
    text(P(1),P(2),P(3)+10,'Target','FontWeight','bold');
    
    % Loop over IK candidates
    colors = lines(numel(sols));  % distinct color per solution
    for si = 1:numel(sols)
        th = sols(si).theta;       % 6×1 joint vector
        
        % Forward‐kinematics to get joint frames
        [T06_fk, origins, rots] = forwardKinematics(dh, th);
        
        % Plot the stick figure
        plot3(origins(1,:),origins(2,:),origins(3,:),'-o', ...
              'Color',colors(si,:),'LineWidth',1.5,'MarkerSize',3);
        
        % Optionally plot joint frames
        for j = 1:7
            O = origins(:,j);
            Rj = rots(:,:,j);
            for ii = 1:3
                v = Rj(:,ii)*axisLen;
                quiver3(O(1),O(2),O(3),v(1),v(2),v(3), ...
                        cols3{ii}, quivOpts{:});
            end
        end
        
        % Annotate solution number
        text(origins(1,end),origins(2,end),origins(3,end), ...
             sprintf('Sol %d',si), 'Color',colors(si,:), ...
             'FontSize',9,'FontWeight','bold');
    end
    
    title(sprintf('IK for Target [%.0f,%.0f,%.0f] (cm)',P));
    hold off;
end
