function interactive_IRB1200()
    % DH constants
    params = struct('d1',399,'a2',350,'a3',42,'d4',351,'d6',82);

    % Joint angle limits [min, max] per joint
    joint_limits = [ ...
        -170, 170;
        -100, 135;
        -200, 70;
        -270, 270;
        -130, 130;
        -400, 400
    ];

    % Create figure
    fig = figure('Name','IRB 1200 Interactive Viewer','NumberTitle','off', ...
                 'Position',[100, 100, 1000, 600]);
    set(fig, 'Color', [1 1 1]);

    % Create axes
    ax = axes('Parent',fig, 'Position',[0.35, 0.1, 0.6, 0.85]);
    set(ax, 'Color', [1 1 1]);
    xlabel(ax, 'X (cm)');
    ylabel(ax, 'Y (cm)');
    zlabel(ax, 'Z (cm)');
    view(ax, 45, 30);

    % Initialize
    theta = zeros(1,6);
    sliders = gobjects(1,6);
    edits   = gobjects(1,6);

    for i = 1:6
        minVal = joint_limits(i,1);
        maxVal = joint_limits(i,2);

        % Slider
        sliders(i) = uicontrol(fig,'Style','slider','Min',minVal,'Max',maxVal,...
            'Value',theta(i),'Units','normalized',...
            'Position',[0.05, 0.9 - (i-1)*0.13, 0.2, 0.05]);

        addlistener(sliders(i), 'Value', 'PostSet', @(src, event) update_from_slider(i));

        % Edit box
        edits(i) = uicontrol(fig,'Style','edit','String',num2str(theta(i)),...
            'Units','normalized',...
            'Position',[0.26, 0.9 - (i-1)*0.13, 0.05, 0.05],...
            'Callback',@(src,~) update_from_edit(i));

        % Label
        uicontrol(fig,'Style','text','String',sprintf('theta%d (°)',i),...
            'Units','normalized','Position',[0.01, 0.9 - (i-1)*0.13, 0.03, 0.05]);
    end

    update_plot();

    function update_from_slider(j)
        theta(j) = sliders(j).Value;
        edits(j).String = sprintf('%.1f', theta(j));
        update_plot();
    end

    function update_from_edit(j)
        val = str2double(edits(j).String);
        if isnan(val), return; end
        val = min(joint_limits(j,2), max(joint_limits(j,1), val));  % Clamp
        theta(j) = val;
        sliders(j).Value = theta(j);
        edits(j).String = sprintf('%.1f', theta(j));
        update_plot();
    end

    function update_plot()
        cla(ax);
        axis(ax, 'equal');
        xlabel(ax, 'X (cm)');
        ylabel(ax, 'Y (cm)');
        zlabel(ax, 'Z (cm)');
        view(ax, 45, 30);

        ang_rad = deg2rad(theta);
        [T06, origins, rots] = forwardKinematics(params, ang_rad);

        % Plot links
        plot3(ax, origins(1,:), origins(2,:), origins(3,:), '-ok', ...
              'LineWidth',2,'MarkerSize',4);
        hold(ax, 'on');
        axisLen = 50;
        cols3 = {'r','g','b'};
        labels = {'X','Y','Z'};

        for j = 1:size(origins,2)
            O = origins(:,j);
            R = rots(:,:,j);
            for ii = 1:3
                v = R(:,ii)*axisLen;
                line(ax, [O(1), O(1) + v(1)], ...
                         [O(2), O(2) + v(2)], ...
                         [O(3), O(3) + v(3)], ...
                         'Color', cols3{ii}, 'LineWidth', 2);
                tip = O + v*1.1;
                text(ax, tip(1), tip(2), tip(3), ...
                     sprintf('%s%d', labels{ii}, j-1), ...
                     'FontSize', 9, 'FontWeight', 'bold');
            end
        end

        P06 = T06(1:3,4);
        title(ax, sprintf('Joint Angles [°]: [%s] — TCP Position: [%.2f, %.2f, %.2f] cm', ...
                num2str(theta, '%.1f '), P06(1), P06(2), P06(3)));

        setAxesEqual3D(ax);
        hold(ax, 'off');
    end
end

function setAxesEqual3D(ax)
    xL = xlim(ax); yL = ylim(ax); zL = zlim(ax);
    xMid = mean(xL); yMid = mean(yL); zMid = mean(zL);
    xRange = diff(xL); yRange = diff(yL); zRange = diff(zL);
    maxRange = max([xRange, yRange, zRange]);
    xlim(ax, [xMid - maxRange/2, xMid + maxRange/2]);
    ylim(ax, [yMid - maxRange/2, yMid + maxRange/2]);
    zlim(ax, [zMid - maxRange/2, zMid + maxRange/2]);
end
