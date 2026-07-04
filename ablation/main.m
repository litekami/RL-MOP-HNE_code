clc; clear; close all;

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end
repoDir = fileparts(fileparts(fileparts(scriptDir)));
outputDir = fullfile(repoDir, '投稿', '投稿版本');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
plotFontSize = 16;
exportSize = [420 315];

%% ================== 全局参数 ==================
rng(2025);                 % 固定随机种子（论文必须）
numRuns = 20;              % 每组重复实验次数
modeList = {'FULL','NO_RL','NO_HNE'};

% 结果存储
Result = struct();

for i = 1:length(modeList)
    Result(i).name   = modeList{i};
    Result(i).path   = zeros(numRuns,1);
    Result(i).energy = zeros(numRuns,1);
    Result(i).time   = zeros(numRuns,1);
    Result(i).smooth = zeros(numRuns,1);
end

%% ================== 批量消融实验 ==================
for m = 1:length(modeList)

    expMode = modeList{m};
    fprintf('\n=========== %s ===========\n',expMode);

    for k = 1:numRuns

        % ===== 运行规划器 =====
        metrics = runPlanner(expMode);

        % ===== 保存指标 =====
        Result(m).path(k)   = metrics.path;
        Result(m).energy(k) = metrics.energy;
        Result(m).time(k)   = metrics.time;
        Result(m).smooth(k) = metrics.smooth;

        fprintf('Run %02d: Path=%.3f  Energy=%.3f  Time=%.3f\n',...
                k,metrics.path,metrics.energy,metrics.time);
    end
end


%% ================== 绘图 ==================

labels = {'RL-MOP-HNE','No-RL','No-HNE'};

%% ---- 1 路径长度箱线图 ----
figure('Color','w','Units','pixels','Position',[100 100 exportSize]);
boxchart([Result(1).path Result(2).path Result(3).path]);
xticklabels(labels);
ylabel('Path Length (m)');
grid on;
styleFigure(gcf, plotFontSize);
exportEps(gcf, fullfile(outputDir, '7.1.eps'), exportSize);

%% ---- 2 能耗箱线图 ----
figure('Color','w','Units','pixels','Position',[120 120 exportSize]);
boxchart([Result(1).energy Result(2).energy Result(3).energy]);
xticklabels(labels);
ylabel('Energy Consumption (rad)');
grid on;
styleFigure(gcf, plotFontSize);
exportEps(gcf, fullfile(outputDir, '7.2.eps'), exportSize);

%% ---- 3 时间柱状图 ----
figure('Color','w','Units','pixels','Position',[140 140 exportSize]);
bar([mean(Result(1).time) mean(Result(2).time) mean(Result(3).time)]);
set(gca,'XTickLabel',labels);
ylabel('Trajectory Time (s)');
grid on;
styleFigure(gcf, plotFontSize);
exportEps(gcf, fullfile(outputDir, '7.3.eps'), exportSize);

%% ---- 4 雷达图----
radarData = [
 mean(Result(1).path) mean(Result(1).energy) mean(Result(1).time) mean(Result(1).smooth);
 mean(Result(2).path) mean(Result(2).energy) mean(Result(2).time) mean(Result(2).smooth);
 mean(Result(3).path) mean(Result(3).energy) mean(Result(3).time) mean(Result(3).smooth);
];

figure('Color','w','Units','pixels','Position',[160 160 exportSize]);
radarPlot(radarData,...
 {'Path','Energy','Time','Smoothness'},...
 labels);

title('Ablation Performance Comparison');
styleFigure(gcf, plotFontSize);
exportEps(gcf, fullfile(outputDir, '7.4.eps'), exportSize);

disp('========== Ablation Experiment Finished ==========');

function styleFigure(fig, fontSize)
set(findall(fig, '-property', 'FontName'), 'FontName', 'Helvetica');
set(findall(fig, '-property', 'FontSize'), 'FontSize', fontSize);
axesList = findall(fig, 'Type', 'axes');
for i = 1:numel(axesList)
    set(axesList(i), 'FontSize', fontSize, 'LineWidth', 0.9, 'Box', 'on');
end
legends = findall(fig, 'Type', 'Legend');
for i = 1:numel(legends)
    set(legends(i), 'FontSize', fontSize, 'Box', 'on');
end
end

function exportEps(fig, filePath, exportSize)
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
