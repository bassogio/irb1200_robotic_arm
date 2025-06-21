% plotIRB1200_IK  –  Draw the ABB-IRB1200 for every IK solution
clc; clear; close all;

%% 1.  Robot DH constants (cm)  (must match forwardKinematics.m)
dh = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

%% 2.  Desired tool-centre-point in centimetres
goal_cm = [0.5; 0.5; 0.8];        

%% 3.  Inverse kinematics  → 6×N joint sets (radians)
Q = InverseKinematics(dh, goal_cm); 

%% 4.  Pretty table of all θ-solutions (deg) and the XYZ from the ForwardKinematics  ----------------------------
Qdeg = rad2deg(Q);            % angles for display
N    = size(Q,2);

fprintf('\n%-6s | %6s%6s%6s%6s%6s%6s || %9s%9s%9s   (from FK)\n',...
        'Sol#','θ1','θ2','θ3','θ4','θ5','θ6','X(cm)','Y(cm)','Z(cm)');
fprintf('%s\n', repmat('-',1,6 + 1 + 6*6 + 3 + 3*9));  % divider

for k = 1:N
    % Run forward kinematics to get XYZ in *centimetres*
    [~,origins,~] = forwardKinematics(dh, Q(:,k));
    p_cm = origins(:,end);          % FK position
    
    fprintf('%6d | %6.1f%6.1f%6.1f%6.1f%6.1f%6.1f || %9.2f%9.2f%9.2f\n',...
            k, Qdeg(:,k), p_cm);
end

%% 5.  Plot each solution  ----------------------------------------------
axisLen = 5;                       % arrow length in cm
quivOpt = {'AutoScale','off','MaxHeadSize',0.9,'LineWidth',1.2};
cols    = {'r','g','b'};           % X,Y,Z colour code
lbl     = {'X','Y','Z'};

for k = 1:N
    figure('Name',sprintf('Solution %d',k), 'NumberTitle','off');
    
    % Forward kinematics for this θ-set
    [~,orig,rots] = forwardKinematics(dh, Q(:,k));
    
    % Draw links
    plot3(orig(1,:),orig(2,:),orig(3,:),'-ok','LineWidth',2,'MarkerSize',4);
    hold on;
    
    % Draw joint frames
    for j = 1:size(orig,2)
        O = orig(:,j);  R = rots(:,:,j);
        for ax = 1:3
            v = R(:,ax)*axisLen;
            quiver3(O(1),O(2),O(3), v(1),v(2),v(3), cols{ax}, quivOpt{:});
            text(O(1)+1.1*v(1), O(2)+1.1*v(2), O(3)+1.1*v(3), ...
                 sprintf('%s%d', lbl{ax}, j-1), ...
                 'FontSize',8,'FontWeight','bold','Color',cols{ax});
        end
    end
    
    % Cosmetics
    axis equal;  grid on;
    xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
    view(45,30);
    title(sprintf('Solution %d  –  θ = [%s]°', k, num2str(Qdeg(:,k).', '%.1f ')));
    hold off;
end
