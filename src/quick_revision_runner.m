% quick_revision_runner.m
% Quick run: limited generations, only Case I, compute new metrics
% Uses generate_all_plots.m for its functions and solvers
clear; close all; clc;

% Configuration
MAX_GEN = 30;  % Reduced from 80 - should converge enough for trajectory shape
POP_SIZE = 40;
outputDir = 'D:/论文投稿/机械臂轨迹规划论文投稿/投稿/投稿版本/scientific reports';

% Run generate_all_plots with limited settings
setenv('RNG_SEED', '42');
setenv('MAX_ITER', num2str(MAX_GEN));
setenv('POPULATION_SIZE', num2str(POP_SIZE));
setenv('RLMOPHNE_REPEATS', '1');
setenv('EXPORT_OLD_PARETO3D', 'false');
setenv('PLOT_CASE_IDS', '1');
setenv('OUTPUT_DIR', outputDir);

fprintf('Starting generate_all_plots with %d generations...\n', MAX_GEN);
tic;
cd('D:/论文投稿/机械臂轨迹规划论文投稿/代码/1-F350六自由度机械臂路径规划+运动学正解+标准D-H参数法');
run('generate_all_plots.m');
elapsed = toc;
fprintf('Complete! Elapsed: %.1f min\n', elapsed/60);

% Check what was generated
matFiles = dir(fullfile(outputDir, '*solveOut.mat'));
fprintf('\nGenerated %d solveOut MAT files:\n', numel(matFiles));
for fi = 1:numel(matFiles)
    fprintf('  %s\n', matFiles(fi).name);
end

fprintf('\nDone. Now run post_process_revision.m to generate merged plots and CSVs.\n');
