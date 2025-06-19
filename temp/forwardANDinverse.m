% forwardANDinverse.m
clc; clear; close all;

% DH constants (cm)
params = struct('d1',39.9,'a2',35,'a3',4.2,'d4',35.1,'d6',8.2);

% Test joint configs (deg)
test_angles = [ ...
     0   0    0   0   0   0;
    90   0    0   0   0   0;
    90  90    0   0   0   0;
    90  90  -90   0   0   0;
    90  90  -90  90   0   0;
    90  90  -90  90  90   0;
    90  90  -90  90  90  90];

axisLen = 5;                      
quivOpts = {'AutoScale','off','MaxHeadSize',1,'LineWidth',1.2};
cols3    = {'r','g','b'};         
labels   = {'X','Y','Z'};

for k = 1:size(test_angles,1)
    figure('Name',sprintf('Pose [%s]°',num2str(test_angles(k,:))), ...
           'NumberTitle','off');

    ang_rad = deg2rad(test_angles(k,:));
    [T06, origins, rots] = forwardKinematics(params, ang_rad);

    % Plot links & frames (same as before)…
    plot3(origins(1,:),origins(2,:),origins(3,:),'-ok','LineWidth',2,'MarkerSize',4);
    hold on;
    for j = 1:size(origins,2)
        O = origins(:,j);
        R = rots(:,:,j);
        for ii = 1:3
            v = R(:,ii)*axisLen;
            quiver3(O(1),O(2),O(3),v(1),v(2),v(3),cols3{ii},quivOpts{:});
            tip = O + v*1.1;
            text(tip(1),tip(2),tip(3),sprintf('%s%d',labels{ii},j-1),...
                 'FontSize',9,'FontWeight','bold');
        end
    end

    % End-effector position
    P06 = origins(:,end);
    text(P06(1),P06(2),P06(3),sprintf('  [%.1f, %.1f, %.1f]',P06),...
         'FontSize',10,'FontWeight','bold','Color','m');
    fprintf('End-effector (cm) for pose %d: X=%.2f, Y=%.2f, Z=%.2f\n', ...
            k, P06(1), P06(2), P06(3));

    % Position-only IK (no subtraction of d6)
    try
        th = inverseKinematicsPositionOnly(params, P06, true);
        fprintf('Recovered angles (deg): [%s]\n\n', num2str(rad2deg(th)'));
    catch ME
        fprintf('  -> IK failed for pose %d: %s\n\n', k, ME.message);
    end

    axis equal; grid on; xlabel('X'); ylabel('Y'); zlabel('Z');
    view(45,30);
    title(sprintf('Joint angles: [%s]°', num2str(test_angles(k,:))));
    hold off;
end

%% Position-Only IK (first 3 joints)
function thetas = inverseKinematicsPositionOnly(params, Pw, elbowUp)
    % Treat Pw as the wrist center already
    d1 = params.d1; a2 = params.a2; a3 = params.a3;

    Xw = Pw(1); Yw = Pw(2); Zw = Pw(3);

    % 1) θ1
    theta1 = atan2(Yw, Xw);

    % 2) Distances
    r = hypot(Xw, Yw);
    s = Zw - d1;

    % 3) θ3 by law of cosines
    D = (r^2 + s^2 - a2^2 - a3^2)/(2*a2*a3);
    if abs(D) > 1
        error('Position unreachable');
    end
    signElbow = elbowUp*2 - 1;  % +1 or –1
    theta3 = atan2(signElbow*sqrt(1 - D^2), D);

    % 4) θ2
    phi = atan2(s, r);
    psi = atan2(a3*sin(theta3), a2 + a3*cos(theta3));
    theta2 = phi - psi;

    % 5) Other joints = 0 by default
    thetas = [theta1; theta2; theta3; 0; 0; 0];
end
