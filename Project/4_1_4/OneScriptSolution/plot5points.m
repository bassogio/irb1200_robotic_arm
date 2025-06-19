% plotIRB1200.m
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

axisLen = 50;                      % arrow length in cm
quivOpts = {'AutoScale','off','MaxHeadSize',1,'LineWidth',1.2};
cols3    = {'r','g','b'};         % X=red, Y=green, Z=blue
labels   = {'X','Y','Z'};

for k = 1:size(test_angles,1)
    % New figure per configuration
    fig = figure('Name',sprintf('Pose [%s]°',num2str(test_angles(k,:))), ...
                 'NumberTitle','off');
    
    % 3. Convert angles to radians and apply DH offsets
    ang = deg2rad(test_angles(k,:));
    t = [ ang(1);
          ang(2) - pi/2;
          ang(3);
          ang(4);
          ang(5);
          ang(6) - pi ];
    
    % 4. Prepare DH arrays
    a     = [ 0, params.a2, params.a3, 0, 0, 0 ];
    d     = [ params.d1, 0, 0, params.d4, 0, params.d6 ];
    alpha = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0];
    
    % 5. Initialize kinematic chain
    T       = eye(4);
    origins = zeros(3,7);
    rots    = repmat(eye(3),[1,1,7]);
    origins(:,1) = [0;0;0];  % base frame origin
    
    % 6. Build chain with inline DHFUNCTION
    for i = 1:6
        Ai = DHFUNCTION(a(i), alpha(i), d(i), t(i));
        T  = T * Ai;
        origins(:,i+1) = T(1:3,4);
        rots(:,:,i+1)  = T(1:3,1:3);
    end
    
    % 7. Plot links
    plot3(origins(1,:),origins(2,:),origins(3,:),'-ok', ...
          'LineWidth',2,'MarkerSize',4);
    hold on;
    
    % 8. Plot joint frames
    for j = 1:7
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
    
    % 9. Formatting
    axis equal; grid on;
    xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
    view(45,30);
    title(sprintf('Joint angles: [%s]°', num2str(test_angles(k,:))));
    hold off;
end


%% Inline DH transform function
function A = DHFUNCTION(a, alpha, d, theta)
    ct = cos(theta);    st = sin(theta);
    ca = cos(alpha);    sa = sin(alpha);
    A = [ ...
      ct,    -st*ca,   st*sa,   a*ct;
      st,     ct*ca,  -ct*sa,   a*st;
       0,        sa,      ca,      d;
       0,         0,       0,      1 ];
end
