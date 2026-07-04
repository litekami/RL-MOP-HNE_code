% focused_experiment_runner.m
% Runs Case I with all 6 algorithms, computes new metrics, saves trajectory data
clear; close all; clc;

addpath('D:/论文投稿/机械臂轨迹规划论文投稿/代码/1-F350六自由度机械臂路径规划+运动学正解+标准D-H参数法');
outputDir = 'D:/论文投稿/机械臂轨迹规划论文投稿/投稿/投稿版本/scientific reports';

% Load all functions from generate_all_plots.m (FUNCTIONS_ONLY guard returns early)
setenv('FUNCTIONS_ONLY', '1');
run('generate_all_plots.m');
setenv('FUNCTIONS_ONLY', '');

% Build robot and cases
robot = buildRobot();
q0 = [0 90 0 0 180 0] * pi / 180;
Rref = robot.fkine(q0).R;

% Use PLOT_CASE_IDS to load only Case I
allCases = buildCases();
caseData = allCases(1);  % Case I only

algorithms = buildAlgorithms();

cfg = struct();
cfg.Ts = 0.02;
cfg.qdotMax = (60 * pi / 180) * ones(1, 6);
cfg.safetyMargin = 0.03;
cfg.pathSamples = 80;
cfg.archiveSize = 60;
cfg.maxIter = 80;
cfg.populationSize = 40;
cfg.baseSeed = 42;
cfg.bestRunWeights = [0.15 0.15 0.05 0.20 0.45];
cfg.pathStyle = 'pchip';
cfg.pathPreference = 'smooth';

fprintf('Running %s - %d obstacles\n', caseData.label, numel(caseData.obstacles));
allTraj = struct([]);

for ai = 1:numel(algorithms)
    algo = algorithms(ai);
    fprintf('\n=== %s ===\n', algo.label);
    
    solverCfg = cfg;
    if isfield(algo, 'qdotMax') && ~isempty(algo.qdotMax)
        solverCfg.qdotMax = algo.qdotMax;
    end
    if isfield(algo, 'pathStyle') && ~isempty(algo.pathStyle)
        solverCfg.pathStyle = algo.pathStyle;
    end
    if isfield(algo, 'pathPreference') && ~isempty(algo.pathPreference)
        solverCfg.pathPreference = algo.pathPreference;
    end
    if isfield(algo, 'jointSmoothWindow') && ~isempty(algo.jointSmoothWindow)
        solverCfg.jointSmoothWindow = algo.jointSmoothWindow;
    end
    
    % Run solver with timing
    tStart = tic;
    solveOut = runAlgorithmSolver(algo, caseData, solverCfg);
    compTime = toc(tStart);
    
    % Get the best solution
    bestSol = pickRepresentativeSolution(solveOut.archive, caseData, algo, solverCfg);
    traj = cartesianPathToJointTrajectory(robot, bestSol.path, q0, Rref, solverCfg);
    archiveExact = evaluateArchiveExact(robot, solveOut.archive, caseData, q0, Rref, solverCfg);
    bestExact = pickRepresentativeExact(archiveExact, algo.prefWeights);
    
    % Add new metrics
    nSamples = size(traj.q, 1);
    w = zeros(nSamples, 1);
    c = zeros(nSamples, 1);
    for k = 1:nSamples
        w(k) = yoshikawaManipulability(robot, traj.q(k, :));
        c(k) = jacobianConditionNumber(robot, traj.q(k, :));
    end
    bestExact.maxJerk = max(abs(traj.jerk(:)));
    bestExact.avgJerk = mean(abs(traj.jerk(:)));
    bestExact.maxAccel = max(abs(traj.qdd(:)));
    bestExact.avgManipulability = mean(w);
    bestExact.minManipulability = min(w);
    bestExact.maxConditionNumber = max(c);
    bestExact.minObstacleClearance = minObstacleClearance(robot, traj.q, caseData.obstacles);
    bestExact.computationTime = compTime;
    
    % Save trajectory and metrics
    trajFile = fullfile(outputDir, sprintf('%s_%s_run1_trajectory.mat', algo.id, caseData.label));
    save(trajFile, 'traj', 'bestExact');
    
    % Store for merged plots
    entry = struct();
    entry.algo = algo;
    entry.traj = traj;
    entry.bestExact = bestExact;
    if isempty(allTraj)
        allTraj = entry;
    else
        allTraj(end+1) = entry;
    end
    
    fprintf('  path=%.3f, time=%.3f, maxVel=%.3f, velVar=%.3f, compTime=%.1fs\n', ...
        bestExact.pathLength, bestExact.trajTime, bestExact.maxJointVel, bestExact.velVar, compTime);
    fprintf('  jerk=%.4f, manipulability=%.3f, clearance=%.3f\n', ...
        bestExact.maxJerk, bestExact.avgManipulability, bestExact.minObstacleClearance);
end

% Generate merged joint-angle plot
fprintf('\n=== Generating merged joint-angle plot ===\n');
fig = figure('Visible', 'off', 'Position', [100 100 900 700]);
algoColors = [
    0.0000 0.4470 0.7410;  % MOEA/D - blue
    0.8500 0.3250 0.0980;  % MOPSO - red
    0.9290 0.6940 0.1250;  % MSCLPSO - yellow
    0.4940 0.1840 0.5560;  % NSGA-II - purple
    0.4660 0.6740 0.1880;  % RL-NSGA-II - green
    0.8510 0.3250 0.0980;  % RL-MOP-HNE - orange
];
algoLabels = {allTraj.algo};
algoLabels = cellfun(@(s) s.label, algoLabels, 'UniformOutput', false);
lineStyles = {'-', '--', '-.', ':', '-', '--'};

for jointIdx = 1:6
    subplot(3, 2, jointIdx); hold on;
    for ti = 1:numel(allTraj)
        t = allTraj(ti).traj.t;
        q = allTraj(ti).traj.q(:, jointIdx) * 180/pi;
        tNorm = (t - t(1)) / max(t(end) - t(1), 1e-10);
        plot(tNorm, q, 'Color', algoColors(ti, :), 'LineWidth', 1.2, ...
            'LineStyle', lineStyles{ti});
    end
    xlabel('Normalized time');
    ylabel(sprintf('Joint %d (deg)', jointIdx));
    grid on; box on;
    set(gca, 'FontSize', 9);
    if jointIdx == 1
        title(sprintf('%s: Joint angle comparison', caseData.label));
    end
end
% Add global legend
subplot(3,2,1);
hL = legend(algoLabels, 'Orientation', 'horizontal', 'FontSize', 7);
set(hL, 'Position', [0.15 0.935 0.7 0.035]);

epsFile = fullfile(outputDir, 'merged_joint_angles_Case_I.eps');
set(fig, 'Renderer', 'painters');
print(fig, '-depsc', epsFile);
fprintf('Saved: %s\n', epsFile);
close(fig);

% Generate new metrics summary CSV
fprintf('\n=== Generating new metrics CSV ===\n');
fid = fopen(fullfile(outputDir, 'new_metrics_revision.csv'), 'w');
fprintf(fid, 'algorithm,path_length,execution_time,avg_joint_range,max_velocity,velocity_variance,');
fprintf(fid, 'max_jerk,avg_jerk,max_accel,avg_manipulability,min_manipulability,max_condition_number,');
fprintf(fid, 'min_obstacle_clearance,computation_time\n');
for ti = 1:numel(allTraj)
    e = allTraj(ti).bestExact;
    fprintf(fid, '%s,%.4f,%.4f,%.4f,%.4f,%.4f,', ...
        allTraj(ti).algo.label, e.pathLength, e.trajTime, e.avgJointRange, ...
        e.maxJointVel, e.velVar);
    fprintf(fid, '%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,', ...
        e.maxJerk, e.avgJerk, e.maxAccel, e.avgManipulability, e.minManipulability, e.maxConditionNumber);
    fprintf(fid, '%.4f,%.1f\n', e.minObstacleClearance, e.computationTime);
end
fclose(fid);
fprintf('Saved new_metrics_revision.csv\n');

fprintf('\n=== Done! ===\n');
