% plotIRB1200_Workspace
clc; clear; close all;

%% 1. Load symbolic FK & robot parameters
run('irb1200_ForwardKinematics.m');   % defines T06_simplified, θ1…θ6, d1,a2,a3,d4,d6

params = struct( ...
    'd1', 0.399, ...    % [m]
    'a2', 0.350, ...    % [m]
    'a3', 0.042, ...    % [m]
    'd4', 0.351, ...    % [m]
    'd6', 0.082 ...     % [m]
);

limits = [ -170,  170;    % θ1 (deg)
           -100,  135;    % θ2 (deg)
           -200,   70 ];  % θ3 (deg)

% fix wrist joints at zero
fixed_t4 = 0;
fixed_t5 = 0;
fixed_t6 = 0;

%% 2. Discretization for joints 1–3
angle_step  = 15;  % degrees
t1_vals     = limits(1,1):angle_step:limits(1,2);
t2_vals     = limits(2,1):angle_step:limits(2,2);
t3_vals     = limits(3,1):angle_step:limits(3,2);

n1 = numel(t1_vals);
n2 = numel(t2_vals);
n3 = numel(t3_vals);
total = n1 * n2 * n3;

% preallocate
pts = zeros(total,3);

%% 3. Prepare symbolic substitution
symbol_list = {
    theta1, theta2, theta3, theta4, theta5, theta6, ...
    d1,      a2,      a3,      d4,      d6
};
fixed_params = {
    params.d1, params.a2, params.a3, ...
    params.d4, params.d6
};

%% 4. Compute workspace (position only)
hWB = waitbar(0,'Computing workspace...','Name','Progress');
idx = 1;
for t1 = deg2rad(t1_vals)
  for t2 = deg2rad(t2_vals)
    for t3 = deg2rad(t3_vals)
      % build substitution values
      angles = {t1, t2, t3, 0, 0, 0};
      vals   = [angles, fixed_params];

      % eval T06
      Tnum = double( subs(T06_simplified, symbol_list, vals) );
      p    = Tnum(1:3,4);

      pts(idx,:) = p.';  
      idx = idx + 1;

      % update waitbar
      if mod(idx,1000)==0 || idx>total
        waitbar(idx/total,hWB);
        if ~isvalid(hWB), error('User cancelled.'); end
      end
    end
  end
end
close(hWB);

%% 5. Plot all four views
figure('Name','IRB-1200 Position Workspace','Position',[50 50 1000 600]);

% 3D
subplot(2,2,1);
scatter3(pts(:,1),pts(:,2),pts(:,3),10,'.','MarkerEdgeAlpha',0.4);
axis equal; grid on; view(45,30);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Workspace');

% Top (X–Y)
subplot(2,2,2);
scatter(pts(:,1),pts(:,2),10,'.','MarkerEdgeAlpha',0.4);
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Top View (X–Y)');

% Front (X–Z)
subplot(2,2,3);
scatter(pts(:,1),pts(:,3),10,'.','MarkerEdgeAlpha',0.4);
axis equal; grid on;
xlabel('X (m)'); ylabel('Z (m)');
title('Front View (X–Z)');

% Side (Y–Z)
subplot(2,2,4);
scatter(pts(:,2),pts(:,3),10,'.','MarkerEdgeAlpha',0.4);
axis equal; grid on;
xlabel('Y (m)'); ylabel('Z (m)');
title('Side View (Y–Z)');
