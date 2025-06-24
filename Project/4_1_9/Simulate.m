clc; clear; close all;

% === 1. Robot DH constants ===
dh = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

% === 2. Desired pose: [Px; Py; Pz; roll; pitch; yaw] in radians ===
goal = [500.79; -150; 500.32; deg2rad(0); deg2rad(90); deg2rad(0)];
goal_pos = goal(1:3);
sg = sign(goal_pos);

% === 3. Run inverse kinematics (returns 6×N joint solutions in radians) ===
Q = ik_irb1200(dh, goal);
Qdeg = rad2deg(Q);
N = size(Q,2);

% === 4. Joint limits for each joint (degrees) ===
lim1 = [-170, 170];
lim2 = [-100, 135];
lim3 = [-200,  70];
lim4 = [-270, 270];
lim5 = [-130, 130];
lim6 = [-400, 400];

% === 5. Check joint limits ===
jointValid = ...
    Qdeg(1,:) >= lim1(1) & Qdeg(1,:) <= lim1(2) & ...
    Qdeg(2,:) >= lim2(1) & Qdeg(2,:) <= lim2(2) & ...
    Qdeg(3,:) >= lim3(1) & Qdeg(3,:) <= lim3(2) & ...
    Qdeg(4,:) >= lim4(1) & Qdeg(4,:) <= lim4(2) & ...
    Qdeg(5,:) >= lim5(1) & Qdeg(5,:) <= lim5(2) & ...
    Qdeg(6,:) >= lim6(1) & Qdeg(6,:) <= lim6(2);

% === 6. Compute FK and position of end-effector ===
P_all = zeros(3, N);
for k = 1:N
    [~, orig, ~] = forwardKinematics(dh, Q(:,k));
    P_all(:,k) = orig(:,end);
end

% === 7. Sign match validation ===
signValid = true(1,N);
for k = 1:N
    for ax = 1:3
        if sg(ax)==0, continue; end
        signValid(k) = signValid(k) && (sign(P_all(ax,k)) == sg(ax));
    end
end

% === 8. Combine masks ===
validIdx = jointValid & signValid;
invalidIdx = ~validIdx;

% === 9. Print INVALID solutions ===
fprintf('\n=== Invalid IK Solutions and Reasons ===\n');
fprintf('%-6s | %s\n','Sol#','Reason');
fprintf('%s\n', repmat('-',1,40));
for k = find(invalidIdx)
    reasons = {};
    if Qdeg(1,k)<lim1(1)||Qdeg(1,k)>lim1(2), reasons{end+1}='θ1 out of limits'; end
    if Qdeg(2,k)<lim2(1)||Qdeg(2,k)>lim2(2), reasons{end+1}='θ2 out of limits'; end
    if Qdeg(3,k)<lim3(1)||Qdeg(3,k)>lim3(2), reasons{end+1}='θ3 out of limits'; end
    if Qdeg(4,k)<lim4(1)||Qdeg(4,k)>lim4(2), reasons{end+1}='θ4 out of limits'; end
    if Qdeg(5,k)<lim5(1)||Qdeg(5,k)>lim5(2), reasons{end+1}='θ5 out of limits'; end
    if Qdeg(6,k)<lim6(1)||Qdeg(6,k)>lim6(2), reasons{end+1}='θ6 out of limits'; end

    if ~signValid(k)
        for ax = 1:3
            if sg(ax)==0, continue; end
            if sign(P_all(ax,k)) ~= sg(ax)
                reasons{end+1} = sprintf('axis %d sign mismatch', ax);
            end
        end
    end
    fprintf('%6d | %s\n', k, strjoin(reasons, ', '));
end

% === 10. Print VALID solutions table ===
validList = find(validIdx);
fprintf('\n=== Valid IK Solutions (within joint limits & sign match) ===\n');
fprintf('%-6s | %8s %8s %8s %8s %8s %8s || %10s %10s %10s\n', ...
    'Sol#','t1','t2','t3','t4','t5','t6','X','Y','Z');
fprintf('%s\n', repmat('-',1,92));
for i = 1:numel(validList)
    k = validList(i);
    P = P_all(:,k);
    fprintf('%6d | %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f || %10.2f %10.2f %10.2f\n', ...
        i, Qdeg(1,k),Qdeg(2,k),Qdeg(3,k),Qdeg(4,k),Qdeg(5,k),Qdeg(6,k), ...
        P(1),P(2),P(3));
end

% === 11. Distance from goal and best match ===
rawDists = vecnorm(P_all - goal_pos, 2, 1);
distsValid = rawDists(validList);
[~, bestI] = min(distsValid);
bestK = validList(bestI);

fprintf('\n=== Distances from goal (VALID solutions) ===\n');
fprintf('%-6s | %10s\n', 'Sol#','Dist(cm)');
fprintf('%s\n', repmat('-',1,24));
for i = 1:numel(validList)
    k = validList(i);
    fprintf('%6d | %10.2f\n', i, rawDists(k));
end

fprintf('\n=== Best IK Solution ===\n');
P_best = P_all(:,bestK);
fprintf('%-6s | %8s %8s %8s %8s %8s %8s || %10s %10s %10s\n', ...
    'Sol#','t1','t2','t3','t4','t5','t6','X','Y','Z');
fprintf('%s\n', repmat('-',1,92));
fprintf('%6d | %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f || %10.2f %10.2f %10.2f\n', ...
    bestI, Qdeg(1,bestK),Qdeg(2,bestK),Qdeg(3,bestK),Qdeg(4,bestK),Qdeg(5,bestK),Qdeg(6,bestK), ...
    P_best(1),P_best(2),P_best(3));

% === 12. Plot all VALID IK solutions ===
axisLen = 50;
cols = {'r','g','b'};
lbls = {'X','Y','Z'};
quivOpts = {'AutoScale','off','MaxHeadSize',0.9,'LineWidth',1.2};

for i = 1:numel(validList)
    k = validList(i);
    [~, orig, rots] = forwardKinematics(dh, Q(:,k));
    thetaStr = num2str(Qdeg(:,k).', '%.1f ');

    figure('Name', sprintf('IK Solution %d', i), 'NumberTitle','off');
    plot3(orig(1,:), orig(2,:), orig(3,:), '-ok', ...
          'LineWidth',2, 'MarkerSize',4);
    hold on;

    % Goal marker
    plot3(goal(1), goal(2), goal(3), 'p', ...
          'MarkerSize',12, 'MarkerFaceColor','m', 'MarkerEdgeColor','k');
    text(goal(1), goal(2), goal(3), '  Goal', ...
         'FontSize',10, 'Color','m', 'FontWeight','bold');

    % Draw joint frames
    for j = 1:size(orig,2)
        O = orig(:,j); R = rots(:,:,j);
        for ax = 1:3
            v = R(:,ax)*axisLen;
            quiver3(O(1),O(2),O(3), v(1),v(2),v(3), ...
                    cols{ax}, quivOpts{:});
            text(O(1)+v(1),O(2)+v(2),O(3)+v(3), ...
                sprintf('%s%d', lbls{ax}, j-1), ...
                'FontSize',8,'FontWeight','bold');
        end
    end

    axis equal; grid on;
    xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
    title(sprintf('Valid Solution %d – θ = [%s]°', i, thetaStr));
    view(45,30);
    hold off;
end
