clc; clear; close all;

%% 1. Robot DH constants (cm)
dh = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

%% 2. Desired tool‐centre‐point in cm
goal = [514.79; 0; 711.32];

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

%% 5. Joint limits (deg) & initial valid index
lim1 = [-170, 170];
lim2 = [-100, 135];
lim3 = [-200,  70];
validIdx = Qdeg(1,:) >= lim1(1) & Qdeg(1,:) <= lim1(2) & ...
           Qdeg(2,:) >= lim2(1) & Qdeg(2,:) <= lim2(2) & ...
           Qdeg(3,:) >= lim3(1) & Qdeg(3,:) <= lim3(2);

%% 6. Enforce matching signs of X,Y,Z to goal
% 6.1 Compute all end‐effector positions:
P_all = zeros(3, N);
for k = 1:N
    [~, orig, ~] = forwardKinematics(dh, Q(:,k));
    P_all(:,k) = orig(:,end);
end

% 6.2 Sign mask:
tol = 1e-6;
sg = sign(goal);
sg(abs(goal) < tol) = 0;

signMatch = true(1, N);
for k = 1:N
    for ax = 1:3
        if sg(ax) == 0
            signMatch(k) = signMatch(k) && (abs(P_all(ax,k)) < tol);
        else
            signMatch(k) = signMatch(k) && (sign(P_all(ax,k)) == sg(ax));
        end
    end
end

% 6.3 Combine masks
validIdx = validIdx & signMatch;

%% 7. Print VALID solutions table
fprintf('\n=== Valid Solutions (within joint limits & sign match) ===\n');
fprintf('%-6s | %8s %8s %8s %8s %8s %8s || %10s %10s %10s   (from FK)\n', ...
        'Sol#','t1','t2','t3','t4','t5','t6','X(cm)','Y(cm)','Z(cm)');
fprintf('%s\n', repmat('-',1,6+3+7*8+2+3*10));

idx = find(validIdx);
for i = 1:numel(idx)
    k = idx(i);
    P = P_all(:,k);
    fprintf('%6d | %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f || %10.2f %10.2f %10.2f\n', ...
            k, Qdeg(1,k),Qdeg(2,k),Qdeg(3,k),Qdeg(4,k),Qdeg(5,k),Qdeg(6,k), ...
            P(1),P(2),P(3));
end

%% 8. Plot only VALID solutions
axisLen = 50;                       
quivOpt = {'AutoScale','off','MaxHeadSize',0.9,'LineWidth',1.2};
cols    = {'r','g','b'};
lbl     = {'X','Y','Z'};
validSolutions = idx;

for ii = 1:numel(validSolutions)
    k = validSolutions(ii);
    figure('Name',sprintf('Valid Solution %d',k),'NumberTitle','off');
    [~, orig, rots] = forwardKinematics(dh, Q(:,k));
    plot3(orig(1,:),orig(2,:),orig(3,:),'-ok','LineWidth',2,'MarkerSize',4);
    hold on;
    for j = 1:size(orig,2)
        O = orig(:,j); R = rots(:,:,j);
        for ax = 1:3
            v = R(:,ax)*axisLen;
            quiver3(O(1),O(2),O(3),v(1),v(2),v(3),cols{ax},quivOpt{:});
            text(O(1)+1.1*v(1),O(2)+1.1*v(2),O(3)+1.1*v(3), ...
                 sprintf('%s%d',lbl{ax},j-1), 'FontSize',8,'FontWeight','bold');
        end
    end
    axis equal; grid on;
    xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
    view(45,30);
    thetaStr = num2str(Qdeg(:,k).', '%.1f ');
    title(sprintf('Valid Solution %d: θ = [%s]°', k, thetaStr));
    hold off;
end
