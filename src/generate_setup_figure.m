clear; close all; clc;

assert(exist('SerialLink','class')==8 || exist('SerialLink','class')==2, ...
    'Peter Corke Robotics Toolbox is required.');

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end
repoDir = fileparts(fileparts(scriptDir));
submissionDir = char([25237 31295]);
submissionVersionDir = char([25237 31295 29256 26412]);
outputDir = fullfile(repoDir, submissionDir, submissionVersionDir);
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

cfg = struct();
cfg.fontName = 'Helvetica';
cfg.fontSize = 14;
cfg.exportSize = [470 350];

robot = buildRobot();

pStart = [0.40 0.20 0.20];
pGoal = [0.00 0.40 0.60];
qDisplay = solveGoalPosture(robot, pGoal);

fig = figure('Color', 'w', 'Units', 'points', 'Position', [120 80 cfg.exportSize]);
set(fig, 'DefaultAxesFontName', cfg.fontName, ...
    'DefaultTextFontName', cfg.fontName, ...
    'DefaultAxesFontSize', cfg.fontSize, ...
    'DefaultTextFontSize', cfg.fontSize, ...
    'DefaultAxesLineWidth', 0.9);

ax = axes('Parent', fig, 'Position', [0.120 0.175 0.735 0.720]);
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');
axis(ax, 'equal');
view(ax, 42, 24);

robot.plot(qDisplay, ...
    'workspace', [-1 1 -1 1 -0.25 1.05], ...
    'scale', 0.42, ...
    'delay', 0, ...
    'noname');
ax = gca;
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');
axis(ax, 'equal');
view(ax, 42, 24);

hStart = scatter3(ax, pStart(1), pStart(2), pStart(3), ...
    85, 'g', 'filled', 'o', 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
hGoal = scatter3(ax, pGoal(1), pGoal(2), pGoal(3), ...
    110, 'r', 'p', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);

text(ax, pStart(1) + 0.060, pStart(2) - 0.050, pStart(3) + 0.045, ...
    sprintf('Start (%.2f, %.2f, %.2f)', pStart), ...
    'FontName', cfg.fontName, 'FontSize', cfg.fontSize - 3, ...
    'Color', [0.05 0.05 0.05], 'FontWeight', 'bold', 'Clipping', 'off');
text(ax, pGoal(1) + 0.060, pGoal(2) + 0.035, pGoal(3) + 0.050, ...
    sprintf('Goal (%.2f, %.2f, %.2f)', pGoal), ...
    'FontName', cfg.fontName, 'FontSize', cfg.fontSize - 3, ...
    'Color', [0.05 0.05 0.05], 'FontWeight', 'bold', 'Clipping', 'off');

xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
xlim(ax, [-0.90 0.90]);
ylim(ax, [-0.85 0.85]);
zlim(ax, [-0.20 1.05]);
xticks(ax, [-0.75 0.00 0.75]);
yticks(ax, [-0.75 0.00 0.75]);
zticks(ax, [0.00 0.50 1.00]);
set(ax, 'FontName', cfg.fontName, ...
    'FontSize', cfg.fontSize - 1, ...
    'LineWidth', 0.9, ...
    'TickDir', 'out', ...
    'Layer', 'top');

legend(ax, [hStart hGoal], {'Start point', 'Goal point'}, ...
    'Location', 'northeast', 'FontSize', cfg.fontSize - 2);

exportEps(fig, fullfile(outputDir, '1.eps'), cfg.exportSize);
fprintf('Setup figure saved: %s\n', fullfile(outputDir, '1.eps'));

function robot = buildRobot()
    deg = pi/180;
    mm2m = 1e-3;

    d1 = 89.2;  a1 = 0;   alpha1 = -pi/2;
    d2 = 0;     a2 = 425; alpha2 = 0;
    d3 = 0;     a3 = 392; alpha3 = 0;
    d4 = 109.3; a4 = 0;   alpha4 = pi/2;
    d5 = 94.75; a5 = 0;   alpha5 = -pi/2;
    d6 = 82.5;  a6 = 0;   alpha6 = 0;

    d1 = d1 * mm2m; d2 = d2 * mm2m; d3 = d3 * mm2m;
    d4 = d4 * mm2m; d5 = d5 * mm2m; d6 = d6 * mm2m;
    a1 = a1 * mm2m; a2 = a2 * mm2m; a3 = a3 * mm2m;
    a4 = a4 * mm2m; a5 = a5 * mm2m; a6 = a6 * mm2m;

    L(1) = Link([0, d1, a1, alpha1], 'standard');
    L(2) = Link([0, d2, a2, alpha2], 'standard');
    L(3) = Link([0, d3, a3, alpha3], 'standard');
    L(4) = Link([0, d4, a4, alpha4], 'standard');
    L(5) = Link([0, d5, a5, alpha5], 'standard');
    L(6) = Link([0, d6, a6, alpha6], 'standard');

    for i = 1:6
        L(i).offset = 0;
        L(i).qlim = [-180 180] * deg;
    end
    robot = SerialLink(L, 'name', 'UR5e');
end

function q = solveGoalPosture(robot, pGoal)
    seeds = [
        25 -55 75 -35 45 0
        0 -60 90 -30 45 0
        45 -70 95 -50 45 0
        -35 -60 85 -35 60 0
        90 -75 95 -45 30 0
        ] * pi / 180;
    qPreferred = [25 -55 75 -35 45 0] * pi / 180;
    bestQ = seeds(1, :);
    bestCost = inf;

    options = optimset('Display', 'off', ...
        'MaxIter', 2500, ...
        'MaxFunEvals', 12000, ...
        'TolX', 1e-10, ...
        'TolFun', 1e-12);

    for i = 1:size(seeds, 1)
        qCandidate = fminsearch(@(q) goalPostureCost(robot, q, pGoal, qPreferred), ...
            seeds(i, :), options);
        cost = goalPostureCost(robot, qCandidate, pGoal, qPreferred);
        if cost < bestCost
            bestCost = cost;
            bestQ = qCandidate;
        end
    end

    q = atan2(sin(bestQ), cos(bestQ));
    pEnd = endEffectorPosition(robot, q);
    fprintf('Display posture EE position: [%.4f %.4f %.4f], goal error: %.6f m\n', ...
        pEnd(1), pEnd(2), pEnd(3), norm(pEnd - pGoal(:)));
end

function cost = goalPostureCost(robot, q, pGoal, qPreferred)
    pEnd = endEffectorPosition(robot, q);
    positionError = norm(pEnd - pGoal(:))^2;
    postureError = norm(atan2(sin(q(:) - qPreferred(:)), cos(q(:) - qPreferred(:))))^2;
    limitPenalty = sum(max(abs(q(:)) - pi, 0).^2);
    cost = 1e5 * positionError + 0.02 * postureError + 10 * limitPenalty;
end

function p = endEffectorPosition(robot, q)
    T = robot.fkine(q);
    if isa(T, 'SE3')
        p = T.t;
    else
        p = T(1:3, 4);
    end
    p = p(:);
end

function exportEps(fig, filePath, exportSize)
    set(findall(fig, '-property', 'FontName'), 'FontName', 'Helvetica');
    set(fig, 'Units', 'points');
    pos = get(fig, 'Position');
    pos(3:4) = exportSize;
    set(fig, 'Position', pos);
    set(fig, 'PaperUnits', 'points');
    set(fig, 'PaperSize', exportSize);
    set(fig, 'PaperPosition', [0 0 exportSize]);
    set(fig, 'PaperPositionMode', 'auto');
    print(fig, filePath, '-depsc2', '-painters', '-loose');
end
