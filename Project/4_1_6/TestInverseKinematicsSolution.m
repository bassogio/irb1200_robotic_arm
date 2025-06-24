clc; clear; close all;

%% 1. Robot DH constants (cm)
dh = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

%% 2. Desired tool‐centre‐point in cm
goal = [500.79; -150; 500.32];

%% 3. Inverse kinematics → 6×N joint sets (rad)
Q = InverseKinematics(dh, goal);

%% 4. Pretty table of ALL θ‐solutions (deg) + FK XYZ
Qdeg = rad2deg(Q);
N    = size(Q,2);

fprintf('\n%-6s | %8s %8s %8s %8s %8s %8s || %10s %10s %10s\n', ...
        'Sol#','t1','t2','t3','t4','t5','t6','X(cm)','Y(cm)','Z(cm)');
fprintf('%s\n', repmat('-',1,6+3+7*8+2+3*10));
for k = 1:N
    [~, orig, ~] = forwardKinematics(dh, Q(:,k));
    P = orig(:,end);
    fprintf('%6d | %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f || %10.2f %10.2f %10.2f\n', ...
            k, Qdeg(1,k),Qdeg(2,k),Qdeg(3,k),Qdeg(4,k),Qdeg(5,k),Qdeg(6,k), ...
            P(1),P(2),P(3));
end

%% 5. Joint limits (deg) & initial masks
lim1 = [-170, 170];
lim2 = [-100, 135];
lim3 = [-200,  70];
jointValid = ...
    Qdeg(1,:) >= lim1(1) & Qdeg(1,:) <= lim1(2) & ...
    Qdeg(2,:) >= lim2(1) & Qdeg(2,:) <= lim2(2) & ...
    Qdeg(3,:) >= lim3(1) & Qdeg(3,:) <= lim3(2);

%% 6. Compute all end‐effector positions
P_all = zeros(3, N);
for k = 1:N
    [~, orig, ~] = forwardKinematics(dh, Q(:,k));
    P_all(:,k) = orig(:,end);
end

%% 7. Sign‐match mask (nonzero axes only)
sg = sign(goal);
signValid = true(1,N);
for k = 1:N
    for ax = 1:3
        if sg(ax)==0, continue; end
        signValid(k) = signValid(k) && (sign(P_all(ax,k))==sg(ax));
    end
end

%% 8. Combine masks
validIdx   = jointValid & signValid;
invalidIdx = ~validIdx;

%% 9. Print INVALID solutions with reasons
fprintf('\n=== Invalid Solutions and Reasons ===\n');
fprintf('%-6s | %s\n','Sol#','Reason');
fprintf('%s\n', repmat('-',1,6+3+20));
for k = find(invalidIdx)
    reasons = {};
    if ~jointValid(k)
        if Qdeg(1,k)<lim1(1)||Qdeg(1,k)>lim1(2), reasons{end+1}='θ1 out of limits'; end
        if Qdeg(2,k)<lim2(1)||Qdeg(2,k)>lim2(2), reasons{end+1}='θ2 out of limits'; end
        if Qdeg(3,k)<lim3(1)||Qdeg(3,k)>lim3(2), reasons{end+1}='θ3 out of limits'; end
    end
    if ~signValid(k)
        for ax = 1:3
            if sg(ax)==0, continue; end
            if sign(P_all(ax,k))~=sg(ax)
                reasons{end+1} = sprintf('axis %d sign mismatch',ax);
            end
        end
    end
    fprintf('%6d | %s\n', k, strjoin(reasons, ', '));
end

%% 10. Print VALID solutions table (renumbered)
fprintf('\n=== Valid Solutions (within joint limits & sign match) ===\n');
fprintf('%-6s | %8s %8s %8s %8s %8s %8s || %10s %10s %10s   (from FK)\n', ...
        'Sol#','t1','t2','t3','t4','t5','t6','X(cm)','Y(cm)','Z(cm)');
fprintf('%s\n', repmat('-',1,6+3+7*8+2+3*10));
validList = find(validIdx);
for i = 1:numel(validList)
    k = validList(i);
    P = P_all(:,k);
    fprintf('%6d | %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f || %10.2f %10.2f %10.2f\n', ...
            i, Qdeg(1,k),Qdeg(2,k),Qdeg(3,k),Qdeg(4,k),Qdeg(5,k),Qdeg(6,k), ...
            P(1),P(2),P(3));
end

%% 11. Compute distances only for valid solutions & pick best
rawDists     = vecnorm(P_all - goal,2,1);
distsValid   = rawDists(validList);
[~, bestI]   = min(distsValid);      % index in renumbered list 1…numel(validList)
bestK        = validList(bestI);     % original solution index

%% 12. Print distances for each VALID solution
fprintf('\n=== Distance from goal for VALID solutions ===\n');
fprintf('%-6s | %10s\n','Sol#','Dist(cm)');
fprintf('%s\n', repmat('-',1,6+3+12));
for i = 1:numel(validList)
    k = validList(i);
    fprintf('%6d | %10.2f\n', i, rawDists(k));
end

%% 13. Print the BEST solution (closest to goal)
fprintf('\n=== Best Solution (closest to goal) ===\n');
fprintf('%-6s | %8s %8s %8s %8s %8s %8s || %10s %10s %10s   (from FK)\n', ...
        'Sol#','t1','t2','t3','t4','t5','t6','X(cm)','Y(cm)','Z(cm)');
fprintf('%s\n', repmat('-',1,6+3+7*8+2+3*10));
P_best = P_all(:,bestK);
fprintf('%6d | %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f || %10.2f %10.2f %10.2f\n', ...
        bestI, Qdeg(1,bestK),Qdeg(2,bestK),Qdeg(3,bestK),Qdeg(4,bestK),Qdeg(5,bestK),Qdeg(6,bestK), ...
        P_best(1),P_best(2),P_best(3));

%% 14. Plot only the BEST solution
axisLen = 50;
quivOpt = {'AutoScale','off','MaxHeadSize',0.9,'LineWidth',1.2};
cols    = {'r','g','b'};
lbl     = {'X','Y','Z'};
thetaStr = num2str(Qdeg(:,bestK).', '%.1f ');

figure('Name','Best IK Solution','NumberTitle','off');
[~, orig, rots] = forwardKinematics(dh, Q(:,bestK));
plot3(orig(1,:),orig(2,:),orig(3,:),'-ok','LineWidth',2,'MarkerSize',4);
hold on;
for j = 1:size(orig,2)
    O = orig(:,j); R = rots(:,:,j);
    for ax = 1:3
        v = R(:,ax)*axisLen;
        quiver3(O(1),O(2),O(3),v(1),v(2),v(3),cols{ax},quivOpt{:});
        text(O(1)+1.1*v(1),O(2)+1.1*v(2),O(3)+1.1*v(3), ...
             sprintf('%s%d',lbl{ax},j-1),'FontSize',8,'FontWeight','bold');
    end
end
axis equal; grid on;
xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
view(45,30);
title(sprintf('Best Solution – θ = [%s]°', thetaStr));
hold off;
