% ==== 6-DOF：D-H建模 + teach + 条件式规划(无障=4点/有障=RRT)
%      + RRT后处理(捷径平滑 + 样条重采样 + 时间参数化)
%      + NSGA-II 多目标优化（路径/关节旋转/轨迹时间）
%      + 无障碍：强制经过4关键点（硬约束固定控制点）
%      + 有障碍：RRT 后仍进行 NSGA-II 再优化（含碰撞约束）
%      + 修复：关键点索引越界（正确计算 wpBaseIdx；控制点索引裁剪）
clear; close all; clc;
FAST_MODE = true;   % true: 跳过 teach、GIF 导出和 robot 弹窗动画
assert(exist('SerialLink','class')==8 || exist('SerialLink','class')==2 || exist('SerialLink')==2, ...
    '需要安装 Peter Corke Robotics Toolbox');
%% ================== 单次运行 ==================
numRuns = 1;

% 用于存储最终结果
finalPath  = zeros(numRuns,1);
finalEnergy = zeros(numRuns,1);
finalTime   = zeros(numRuns,1);

for run = 1:numRuns
    fprintf('Independent Run %d / %d\n', run, numRuns);
    
    %     rng(run);   % 固定随机种子，保证可复现（SCI强烈推荐）
    
    % =========================
    % 运行你的主算法
    % （该部分保持你现有代码不变）
    % =========================
    deg = pi/180; mm2m = 1e-3;
    
    % ---- 标准 D-H（长度 mm->m）----
    d1=89.2; a1=0;    alpha1=-pi/2;
    d2=0;    a2=425;  alpha2=0;
    d3=0;    a3=392;  alpha3=0;
    d4=109.3;a4=0;    alpha4= pi/2;
    d5=94.75;a5=0;    alpha5=-pi/2;
    d6=82.5; a6=0;    alpha6=0;
    d1=d1*mm2m; d2=d2*mm2m; d3=d3*mm2m; d4=d4*mm2m; d5=d5*mm2m; d6=d6*mm2m;
    a1=a1*mm2m; a2=a2*mm2m; a3=a3*mm2m; a4=a4*mm2m; a5=a5*mm2m; a6=a6*mm2m;
    
    L(1)=Link([0,d1,a1,alpha1],'standard');
    L(2)=Link([0,d2,a2,alpha2],'standard');
    L(3)=Link([0,d3,a3,alpha3],'standard');
    L(4)=Link([0,d4,a4,alpha4],'standard');
    L(5)=Link([0,d5,a5,alpha5],'standard');
    L(6)=Link([0,d6,a6,alpha6],'standard');
    for i=1:6, L(i).offset=0; L(i).qlim=[-180 180]*deg; end
    robot = SerialLink(L,'name','UR5e');
    
    % ---- 初始位形 & teach（可注释）----
    q0 = [0 90 0 0 180 0]*deg;
    if ~FAST_MODE
        figure(1); robot.plot(q0,'workspace',[-1 1 -1 1 -0.1 1],'scale',0.5); title('UR5e');
        figure(2); robot.plot(q0,'workspace',[-1 1 -1 1 -0.1 1],'scale',0.5); robot.teach(); title('Teach');
    end
    
    % =================== 规划条件：有障碍->RRT；无障碍->4点 ===================
    % 如需障碍（球形），取消下一行注释：obstacles = [ struct('c',[0.2 0.2 0.4],'r',0.2) ];
    % obstacles = [];
    obstacles = [ struct('c',[0.2 0.2 0.4],'r',0.2) ];%1个障碍物
%     obstacles = [ ...
%         struct('c',[0.28 0.22 0.30],'r',0.08); ...
%         struct('c',[0.20 0.28 0.40],'r',0.08); ...
%         struct('c',[0.13 0.32 0.50],'r',0.08) ...
%     ];%3个障碍物
%     obstacles = [ ...
%        struct('c',[0.34 0.38 0.28],'r',0.09); ...
%        struct('c',[0.28 0.24 0.46],'r',0.09); ...
%        struct('c',[0.20 0.36 0.58],'r',0.09); ...
%        struct('c',[0.12 0.26 0.34],'r',0.09); ...
%        struct('c',[0.06 0.34 0.50],'r',0.09) ...
%        ];%5个障碍物
    
    
    safety = 0.03;  % 安全裕度 (m)
    
    % 通用参考姿态
    Rref = robot.fkine(q0).R;
    
    % 关节速度上限 / 采样周期（用于时间参数化和动画）
    Ts = 0.02;
    qdot_max = (60*deg)*ones(1,6);
    
    % 预声明用于 NSGA 的固定点信息
    fixedBaseIdx = []; % 这些是 q_for_opt 中强制固定的行索引（用于无障碍）
    
    if isempty(obstacles)
        %% ======================= 无障碍：4 个关键点（硬约束） =======================
        Wp = [ 0.40  0.20 0.20;
            0.35  0.10 0.35;
            0.10  0.35 0.55;
            -0.05  0.30 0.40 ];
        for i=1:4, Twp(i)=SE3(Rref, Wp(i,:)); end %#ok<SAGROW>
        
        % 逆解（链式初值）
        q_wp=zeros(4,6); seed=q0;
        for i=1:4, q_wp(i,:)=robot.ikcon(Twp(i),seed); seed=q_wp(i,:); end
        
        % 基于关节速度上限的时间估计（各段）
        segT=zeros(3,1);
        for k=1:3
            dq=abs(q_wp(k+1,:)-q_wp(k,:));
            segT(k)=max(max(dq./qdot_max),Ts);
        end
        
        % === 修复1：用函数正确构造 q_base 与关键点索引 wpBaseIdx ===
        [q_base, wpBaseIdx] = buildBaseTrajWithWpIdx(q_wp, segT, Ts);
        
        % 固定点（四个关键点）
        fixedBaseIdx = wpBaseIdx;
        
        % 可视化基础轨迹
        T_all=robot.fkine(q_base); P=transl(T_all);
        fig=figure(3); clf(fig); ax=axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
                view(ax,45,25);
        xlabel(ax,'X(m)'); ylabel(ax,'Y(m)'); zlabel(ax,'Z(m)'); title(ax,'无障碍：基础轨迹（含4关键点）');
        drawPath3D(ax,P,'-',2); scatter3(ax,Wp(:,1),Wp(:,2),Wp(:,3),60,'filled');
        legend(ax,{'基础末端轨迹','关键点'},'Location','best');
        axis(ax,[-1 1 -1 1 -0.1 1]); axis(ax,'manual');
        
        % === 导出GIF（基础轨迹）— 已注释，避免 robot.plot 弹窗 ===
        % if ~FAST_MODE
        %     savePlanningGif(robot, q_base, fig, ax, 'plan_no_obstacles_base.gif', Ts);
        % end
        
        q_for_opt = q_base;   % 交给 NSGA-II 的初始轨迹
        
    else
        %% ======================= 有障碍：仅起点-终点 & RRT ==================
        pStart = [0.40  0.20 0.20];
        pGoal  = [0.00  0.40 0.60];
        Tstart = SE3(Rref, pStart);
        Tgoal  = SE3(Rref, pGoal);
        
        q_start = robot.ikcon(Tstart, q0);
        q_goal  = robot.ikcon(Tgoal,  q_start);
        
        % RRT 参数
        prm.maxNodes=3000; prm.step=8*deg; prm.goalBias=0.2;
        prm.connectTol=6*deg; prm.edgeCheckN=20;
        qlim = vertcat(robot.links.qlim);
        
        [path_q, ok] = rrtPlanJointSpace(robot, q_start, q_goal, obstacles, safety, qlim, prm);
        
        % 可视化
        fig=figure(4); clf(fig); ax=axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
            view(ax,45,25);
        xlabel(ax,'X(m)'); ylabel(ax,'Y(m)'); zlabel(ax,'Z(m)');
        drawObstacles(ax, obstacles);
        % 起点：绿色实心圆 + 标注
        scatter3(ax, pStart(1), pStart(2), pStart(3), 120, 'g', 'filled', 'o', ...
            'MarkerEdgeColor','k', 'LineWidth',1.2);
        text(ax, pStart(1)+0.02, pStart(2), pStart(3)+0.02, 'Start', ...
            'FontSize',11, 'Color','k', 'FontWeight','bold');
        % 终点：红色五角星 + 标注
        scatter3(ax, pGoal(1), pGoal(2), pGoal(3), 160, 'r', 'p', 'filled', ...
            'MarkerEdgeColor','k', 'LineWidth',1.5);
        text(ax, pGoal(1)+0.02, pGoal(2), pGoal(3)+0.02, 'Goal', ...
            'FontSize',11, 'Color','k', 'FontWeight','bold');
        
        
        if ok
            % (1) 细分原始 RRT
            q_rrt=[];
            for k=1:size(path_q,1)-1
                [qq,~,~]=jtraj(path_q(k,:), path_q(k+1,:), 10);
                if isempty(q_rrt), q_rrt=qq; else, q_rrt=[q_rrt; qq(2:end,:)]; end %#ok<AGROW>
            end
            
            % (2) 捷径平滑
            shortcut.maxIter   = 600;
            shortcut.edgeCheck = 25;
            q_short = shortcutSmoothPath(robot, q_rrt, obstacles, safety, shortcut);
            
            % (3) 样条重采样 + 碰撞校验
            splineCfg.method    = 'pchip';
            splineCfg.numPts    = 400;
            splineCfg.edgeCheck = 25;
            [q_smooth, okSpline] = splineSmoothPath(robot, q_short, obstacles, safety, splineCfg);
            if ~okSpline
                warning('样条重采样发生碰撞，回退到捷径结果。');
                q_smooth = q_short;
            end
            
            % (4) 时间参数化（速度上限）
            [q_exec, ~] = timeParamFromPath(q_smooth, qdot_max, Ts);
            
            % 可视化：原始 / 捷径 / 样条+定时
            P_rrt   = transl(robot.fkine(q_rrt));
            P_short = transl(robot.fkine(q_short));
            P_sm    = transl(robot.fkine(q_exec));
            %         drawPath3D(ax,P_rrt,  ':',1.0);
            %         drawPath3D(ax,P_short,'--',1.5);
            drawPath3D(ax,P_sm,   '-', 2.0);
            legend(ax,{'Obstacle','Start','Goal','Path'},'Location','best');
            if ~FAST_MODE
                axes(ax);
                robot.plot(q_exec,'workspace',[-1 1 -1 1 -0.1 1], 'scale',0.5,'delay',Ts);
            end
            
            % === 导出GIF（基础轨迹）— 已注释，避免 robot.plot 弹窗 ===
            % if ~FAST_MODE
            %     savePlanningGif(robot, q_exec, fig, ax, 'plan_rrt_smooth_base.gif', Ts);
            % end
            
            q_for_opt    = q_exec;      % 交给 NSGA-II 的初始轨迹
            
            % ====== 为 NSGA-II 优化前轨迹进行时间参数化 ======
            [q_pre_exec, t_pre_exec] = timeParamFromPath(q_for_opt, qdot_max, Ts);
            
            if isempty(t_pre_exec)
                t_pre_exec = (0:size(q_pre_exec,1)-1).' * Ts;
            end
            
            fixedBaseIdx = [1, size(q_for_opt,1)]; % 有障碍下：只固定起点和终点
            
        else
            legend(ax,{'障碍物','起/终点'},'Location','best');
            error('RRT 未找到可行路径，无法进行后续 NSGA-II 优化。');
        end
    end
    
    %% ======================= NSGA-II 多目标优化（含固定控制点） =======================
    nsgaCfg.popSize   = 40;         % 种群
    nsgaCfg.maxGen    = 5;         % 代数
    nsgaCfg.eliteFrac = 0.10;       % 精英保留比例
    nsgaCfg.mutProb   = 0.15;       % 变异概率
    nsgaCfg.etaC      = 15;         % SBX 交叉参数
    nsgaCfg.etaM      = 20;         % 多项式变异参数
    
    
    % 变量设计（仅对非固定内点施加扰动）：
    % x = [tscale, s_smooth, d_free1(1..6), d_free2(1..6), ...]
    NvTarget = 30;                                         % 控制点数（含两端）
    [ctrl, fixedCtrlMask, ctrlBaseIdx] = pickControlPointsConstrained(q_for_opt, NvTarget, fixedBaseIdx);
    freeMask = true(size(ctrl,1),1);
    freeMask( fixedCtrlMask ) = false;     % 固定点不扰动
    freeMask(1) = false; freeMask(end)=false; % 保险：首尾也不扰动
    freeIdx  = find(freeMask);
    Mfree    = numel(freeIdx);
    dmax     = 5*deg;                                      % 单关节扰动上限 ±5°
    lb       = [0.6, 0.0, -dmax*ones(1,6*Mfree)];          % tscale∈[0.6,1.6], s∈[0,1]
    ub       = [1.6, 1.0,  dmax*ones(1,6*Mfree)];
    q_lim    = vertcat(robot.links.qlim);
    
    % 评估器（根据 x 构造 ctrl_new：固定点零扰动，非固定内点加扰动）
    evalFun = @(x) evaluateIndividualConstrained(robot, q_for_opt, ctrl, freeIdx, x, qdot_max, Ts, obstacles, safety, q_lim);
    
    [best, pareto, hist] = nsga2OptimizePath(evalFun, lb, ub, nsgaCfg);
    
    % 取一个“拥挤距离最大”的代表解作为最终（也可交互挑选）
    [~, pickIdx] = max(pareto.crowd);
    x_star = pareto.X(pickIdx,:);
    % ====== NSGA-II 优化完成后 ======
    % === Pareto 3D/2D 图 — 已注释（plotNsgaReports 已包含） ===
    % plotParetoComparison(pareto);
    % 用最优个体重建最终轨迹，并可视化
    [~, ~, traj] = evaluateIndividualConstrained(robot, q_for_opt, ctrl, freeIdx, x_star, qdot_max, Ts, obstacles, safety, q_lim);
    % ========= 各目标函数值 =========
    finalPath(run)   = hist.best(end,1);
    finalEnergy(run) = hist.best(end,2);
    finalTime(run)   = hist.best(end,3);
    %% ========= 综合目标函数计算 =========
    F_raw = hist.best;      % G x 3
    G     = size(F_raw,1);
    epsi  = 1e-12;
    
    % ---- Min-Max 归一化（基于本次运行） ----
    F_min = min(F_raw, [], 1);
    F_max = max(F_raw, [], 1);
    F_norm = (F_raw - F_min) ./ (F_max - F_min + epsi);
    
    % ---- 综合目标函数（加权和） ----
    w = [1/3, 1/3, 1/3];
    F_comp = F_norm * w(:);     % G x 1
    
    % ---- 取最后一代作为该次实验结果 ----
    F_comp_final(run) = F_comp(end);
end
% === 曲线与帕累托图 ===
plotNsgaReports(robot, q_for_opt, traj.q, pareto, hist, Ts);

%% ================== 关节角-时间曲线（优化前 vs 优化后） ==================
figure(300); clf;

t_pre  = t_pre_exec(:);
q_pre  = q_pre_exec;
plotCompactJointStack(300, t_pre, q_pre, '#D95319', 1.5, false);

figure(301); clf;
t_post = traj.t(:);
q_post = traj.q;
plotCompactJointStack(301, t_post, q_post, '#117733', 2.0, false);
%% ================== 关节角速度-时间曲线（优化前 vs 优化后） ==================
% ---------- 速度计算 ----------
qdot_pre  = diff(q_pre_exec) / Ts;   % (N-1) x 6
qdot_post = diff(traj.q)      / Ts;

t_pre_v   = t_pre_exec(1:end-1);     % 时间对齐
t_post_v  = traj.t(1:end-1);

figure(400); clf;
plotCompactJointStack(400, t_pre_v, qdot_pre, '#D95319', 1.5, true);

figure(401); clf;
plotCompactJointStack(401, t_post_v, qdot_post, '#117733', 2.0, true);

%% ================== 目标函数随迭代代数变化曲线 ==================
figure(700); clf; hold on; grid on; box on;

gen = 1:size(hist.best,1);

plot(gen, hist.best(:,1), 'LineWidth', 2);
plot(gen, hist.best(:,2), 'LineWidth', 2);
plot(gen, hist.best(:,3), 'LineWidth', 2);

xlabel('Iteration / Generation');
ylabel('Objective Value');
title('Convergence Curves of Objective Functions');

legend({'Path Length', 'Energy Consumption', 'Trajectory Time'}, ...
    'Location','best');

set(gca,'FontSize',11);

figure(701); clf;

subplot(3,1,1);
plot(gen, hist.best(:,1),'LineWidth',2); grid on;
ylabel('Path Length (m)');
title('Convergence of Path Length');

subplot(3,1,2);
plot(gen, hist.best(:,2),'LineWidth',2); grid on;
ylabel('Energy (rad)');
title('Convergence of Energy Consumption');

subplot(3,1,3);
plot(gen, hist.best(:,3),'LineWidth',2); grid on;
xlabel('Iteration');
ylabel('Time (s)');
title('Convergence of Trajectory Time');

%% ================== 综合目标函数（归一化）随迭代变化 — 已注释（用户要求） ==================
% hist.best(:,1): Path Length
% hist.best(:,2): Energy Consumption
% hist.best(:,3): Trajectory Time

% F_raw = hist.best;          % G x 3
% G     = size(F_raw,1);
% epsi  = 1e-12;

% --------- 1) Min-Max 归一化 ---------
% F_min = min(F_raw, [], 1);
% F_max = max(F_raw, [], 1);

% F_norm = (F_raw - F_min) ./ (F_max - F_min + epsi);

% --------- 2) 综合目标函数（加权和） ---------
% w = [1/3, 1/3, 1/3];        % 权重（可在论文中说明）
% F_comp = F_norm * w(:);     % G x 1

% --------- 3) 绘制综合目标函数收敛曲线 ---------
% figure(702); clf;
% plot(gen, F_comp, 'k-', 'LineWidth', 2); grid on; box on;

% xlabel('Iteration / Generation');
% ylabel('Normalized Composite Objective');
% title('Convergence Curve of Normalized Composite Objective Function');

% set(gca,'FontSize',11);

% %% ====== 归一化目标函数箱线图 — 已注释（非论文必要） ======
% figure(703); clf;
% boxchart(F_norm);
% xticklabels({'Norm-Path', 'Norm-Energy', 'Norm-Time'});
% grid on; box on;
% ylabel('Normalized Objective Value');
% title('Boxplot of Normalized Objective Functions');
% set(gca,'FontSize',11);

% %% ================== 各目标函数值箱线图绘制 — 已注释（非论文必要） ==================
% figure(704); clf;
% subplot(1,3,1); boxchart(finalPath); xticklabels({'Path Length'}); ylabel('Length (m)');
% title('(a) Path Length'); grid on;
% subplot(1,3,2); boxchart(finalEnergy); xticklabels({'Energy'}); ylabel('Energy Metric');
% title('(b) Energy Consumption'); grid on;
% subplot(1,3,3); boxchart(finalTime); xticklabels({'Time'}); ylabel('Time (s)');
% title('(c) Trajectory Time'); grid on;
% set(gcf,'Position',[200 200 900 300]);

%% ================== 综合目标函数值箱线图绘制 — 已注释（用户要求） ==================
% figure(705); clf;
% boxchart(F_comp_final);
% xticklabels({'Norm-Path-Energy-Time'});

% grid on; box on;
% ylabel('Normalized Objective Value');
% title('Boxplot of Normalized Objective Functions');

% set(gca,'FontSize',11);
% === 播放&导出最终GIF — 已注释，避免 robot.plot 弹窗 ===
% if ~FAST_MODE
%     fig=figure(99); clf(fig); ax=axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
%         view(ax,45,25);
%     xlabel(ax,'X(m)'); ylabel(ax,'Y(m)'); zlabel(ax,'Z(m)');
%     title(ax,'NSGA-II 优化后的最终末端路径（含固定关键点约束）');
%     if ~isempty(obstacles)
%         drawObstacles(ax, obstacles);
%         scatter3(ax, pStart(1), pStart(2), pStart(3), 120, 'g', 'filled', 'o', ...
%             'MarkerEdgeColor','k', 'LineWidth',1.2);
%         text(ax, pStart(1)+0.02, pStart(2), pStart(3)+0.02, 'Start', ...
%             'FontSize',11, 'Color','k', 'FontWeight','bold');
%         scatter3(ax, pGoal(1), pGoal(2), pGoal(3), 160, 'r', 'p', 'filled', ...
%             'MarkerEdgeColor','k', 'LineWidth',1.5);
%         text(ax, pGoal(1)+0.02, pGoal(2), pGoal(3)+0.02, 'Goal', ...
%             'FontSize',11, 'Color','k', 'FontWeight','bold');
%     end
%     P1 = transl(robot.fkine(traj.q));     drawPath3D(ax,P1,'-.', 2.0);
%     legend(ax,{'Obstacle 1','Obstacle 2','Obstacle 3','Start','Goal','Path'},'Location','best');
%     axis(ax,[-1 1 -1 1 -0.1 1]); axis(ax,'manual');
%     savePlanningGif(robot, traj.q, fig, ax, 'plan_nsga2_final.gif', Ts);
% end

fprintf('\n=== NSGA-II 最优解目标 ===\n');
fprintf('末端路径长度: %.4f m\n', traj.metrics.pathLength);
fprintf('关节旋转总量: %.4f rad\n', traj.metrics.totalJointMove);
fprintf('轨迹时间(s): %.4f s\n', traj.metrics.trajTime);

%% ================== 定量性能对比（打印） ==================
% 时间
T_pre  = t_pre(end);
T_post = traj.metrics.trajTime;

% 关节总运动量
jointMove_pre  = sum(sum(abs(diff(q_pre_exec))));
jointMove_post = traj.metrics.totalJointMove;

% 最大关节速度
qdot_pre  = diff(q_pre_exec)  / Ts;
qdot_post = diff(traj.q) / Ts;

maxVel_pre  = max(abs(qdot_pre),[],'all');
maxVel_post = max(abs(qdot_post),[],'all');

fprintf('\n========== NSGA-II 优化前后对比 ==========\n');
fprintf('轨迹时间 (s):       前 = %.3f | 后 = %.3f\n', T_pre, T_post);
fprintf('关节总运动量 (rad): 前 = %.3f | 后 = %.3f\n', jointMove_pre, jointMove_post);
fprintf('最大关节角速度(rad/s): 前 = %.3f | 后 = %.3f\n', maxVel_pre, maxVel_post);
fprintf('========================================\n');

%% ======================== ———— 函数区 ———— ========================
function [q_base, wpBaseIdx] = buildBaseTrajWithWpIdx(q_wp, segT, Ts)
% 正确拼接 q_base，并返回 4 个关键点在 q_base 中的索引
% q_wp: 4x6 关节关键点; segT: 3x1 每段建议时长
q_base = [];
Nseg = zeros(3,1);
for k=1:3
    N = max(2, round(segT(k)/Ts));
    Nseg(k) = N;
    [qq,~,~] = jtraj(q_wp(k,:), q_wp(k+1,:), N);
    if isempty(q_base)
        q_base = qq;                     % 第一段完整保留 N1
    else
        q_base = [q_base; qq(2:end,:)];  %#ok<AGROW> % 之后每段去掉首样本，避免重复
    end
end
% 四个关键点的真实索引（考虑重叠去重后的长度）
idx1 = 1;
idx2 = Nseg(1);
idx3 = Nseg(1) + (Nseg(2)-1);
idx4 = Nseg(1) + (Nseg(2)-1) + (Nseg(3)-1);
wpBaseIdx = [idx1, idx2, idx3, idx4];
% 保险：裁剪到合法范围
n = size(q_base,1);
wpBaseIdx = min(max(wpBaseIdx,1), n);
end

function [path, ok] = rrtPlanJointSpace(robot, q_start, q_goal, obstacles, margin, qlim, prm)
Node.q=q_start; Node.parent=0; tree=repmat(Node,prm.maxNodes,1); n=1;
if isConfigCollide(robot,q_start,obstacles,margin) || isConfigCollide(robot,q_goal,obstacles,margin)
    ok=false; path=[]; return;
end
for it=1:prm.maxNodes
    if rand<prm.goalBias, q_rand=q_goal; else, q_rand=sampleInQlim(qlim); end
    idx=nearestNode(tree,n,q_rand); qn=tree(idx).q; v=q_rand-qn; d=norm(v); if d<1e-9, continue; end
    q_new=qn+prm.step*(v/d);
    if ~edgeCollision(robot,qn,q_new,obstacles,margin,prm.edgeCheckN)
        n=n+1; tree(n).q=q_new; tree(n).parent=idx;
        if norm(q_new-q_goal)<prm.connectTol && ~edgeCollision(robot,q_new,q_goal,obstacles,margin,prm.edgeCheckN)
            path=[q_goal]; id=n; while id~=0, path=[tree(id).q; path]; id=tree(id).parent; end
            ok=true; path=uniqueRowTol(path,1e-9); return;
        end
    end
end
ok=false; path=[];
end

function tf = edgeCollision(robot, qa, qb, obstacles, margin, N)
if nargin < 6 || isempty(N), N = 20; end
dqNorm = norm(qb-qa);
stepNorm = 2*pi / max(N,1);
Ns = max(2, ceil(dqNorm / stepNorm));
tf=false; for i=0:Ns, q=qa+(i/Ns)*(qb-qa); if isConfigCollide(robot,q,obstacles,margin), tf=true; return; end, end
end

function tf = isConfigCollide(robot, q, obstacles, margin)
tf=false; P=jointPositions(robot,q);
for oi=1:numel(obstacles)
    c=obstacles(oi).c(:); R=obstacles(oi).r+margin;
    for k=1:size(P,1)-1
        a=P(k,:).'; b=P(k+1,:).'; if distPointToSegment(c,a,b)<=R, tf=true; return; end
    end
end
end

function P = jointPositions(robot, q)
L=robot.links; T=robot.base; if ~isa(T,'SE3'), T=SE3(T); end
n=numel(L); P=zeros(n+1,3); P(1,:)=T.t.';
for i=1:n, T=T*A(L(i),q(i)); P(i+1,:)=T.t.'; end
end

function d = distPointToSegment(p,a,b)
ab=b-a; t=dot(p-a,ab)/max(dot(ab,ab),1e-16); t=max(0,min(1,t)); d=norm(p-(a+t*ab));
end

function idx = nearestNode(tree,n,q)
best=inf; idx=1; for i=1:n, d=norm(tree(i).q-q); if d<best, best=d; idx=i; end, end
end

function q = sampleInQlim(qlim)
q = qlim(:,1).' + rand(1,size(qlim,1)).*(qlim(:,2).'-qlim(:,1).');
end

function A = uniqueRowTol(A,tol)
keep=true(size(A,1),1);
for i=2:size(A,1)
    if any(vecnorm(A(1:i-1,:)-A(i,:),2,2)<tol), keep(i)=false; end
end
A=A(keep,:);
end

function drawObstacles(ax, obstacles)
[Xs, Ys, Zs] = sphere(24);

for oi = 1:numel(obstacles)
    % 颜色：浅灰蓝 + 较高透明度
    h = surf(ax, ...
        Xs * obstacles(oi).r + obstacles(oi).c(1), ...
        Ys * obstacles(oi).r + obstacles(oi).c(2), ...
        Zs * obstacles(oi).r + obstacles(oi).c(3));
    
    set(h, ...
        'FaceColor', [0.75 0.85 0.95], ...  % 很浅的蓝灰色
        'FaceAlpha', 0.18, ...               % 透明度调低一点，更通透
        'EdgeColor', 'none');
    %         % 青绿色调（科技感强）
    %         set(h, ...
    %             'FaceColor', [0.6 0.9 0.85], ...   % 青绿色
    %             'FaceAlpha', 0.15–0.22, ...         % 范围建议
    %             'EdgeColor', 'none');
    
    %         set(h, ...
    %             'FaceColor', [0.92 0.92 0.94], ...  % 极浅灰
    %             'FaceAlpha', 0.12, ...               % 非常透明
    %             'EdgeColor', 'none');
    % 可选：轻微添加边缘线（很淡），增加立体感
    set(h, 'EdgeColor', [0.6 0.7 0.8], 'EdgeAlpha', 0.3);
end


end

function drawPath3D(ax,P,ls,lw)
plot3(ax,P(:,1),P(:,2),P(:,3),ls,'LineWidth',lw,'Color','#D95319');
end

function q_out = shortcutSmoothPath(robot, q_in, obstacles, margin, cfg)
if size(q_in,1)<=2, q_out = q_in; return; end
q = q_in; n = size(q,1);
for it=1:cfg.maxIter
    if n<=2, break; end
    i = randi([1,n-2]); j = randi([i+2,n]);
    qa = q(i,:); qb = q(j,:);
    if ~edgeCollision(robot, qa, qb, obstacles, margin, cfg.edgeCheck)
        q = [q(1:i,:); qb; q(j+1:end,:)]; %#ok<AGROW>
        n = size(q,1);
    end
end
q_out = uniqueRowTol(q,1e-9);
end

function [q_out, ok] = splineSmoothPath(robot, q_in, obstacles, margin, cfg)
ok = true; q_out = q_in;
if size(q_in,1)<=2, return; end
ds = vecnorm(diff(q_in),2,2); s = [0; cumsum(ds)];
if s(end)<1e-9, return; end
s = s / s(end);
si = linspace(0,1,cfg.numPts).';
qi = zeros(cfg.numPts, size(q_in,2));
for d=1:size(q_in,2)
    switch lower(cfg.method)
        case 'pchip'
            qi(:,d) = pchip(s, q_in(:,d), si);
        case 'csaps'
            qi(:,d) = csaps(s, q_in(:,d), 0.85, si);
        otherwise
            qi(:,d) = pchip(s, q_in(:,d), si);
    end
end
for k=1:size(qi,1)-1
    qa = qi(k,:); qb = qi(k+1,:);
    if edgeCollision(robot, qa, qb, obstacles, margin, cfg.edgeCheck)
        ok = false; q_out = q_in; return;
    end
end
q_out = qi;
end

function [q_exec, t_exec] = timeParamFromPath(q_path, qdot_max, Ts)
nSeg = size(q_path,1)-1;
if nSeg < 1
    t_exec = []; q_exec = [];
    return;
end
segT = zeros(nSeg,1);
segN = zeros(nSeg,1);
for k=1:nSeg
    dq = abs(q_path(k+1,:) - q_path(k,:));
    segT(k) = max( max(dq ./ qdot_max), Ts );
    segN(k) = max(2, round(segT(k) / Ts));
end
totalN = segN(1) + sum(segN(2:end)-1);
q_exec = zeros(totalN, size(q_path,2));
t_exec = zeros(totalN, 1);
t0 = 0;
pos = 1;
for k=1:nSeg
    N = segN(k);
    [qq,~,~] = jtraj(q_path(k,:), q_path(k+1,:), N);
    tt = linspace(t0, t0+segT(k), N).';
    if k == 1
        rows = pos:pos+N-1;
        q_exec(rows,:) = qq;
        t_exec(rows) = tt;
        pos = pos + N;
    else
        rows = pos:pos+N-2;
        q_exec(rows,:) = qq(2:end,:);
        t_exec(rows) = tt(2:end);
        pos = pos + N - 1;
    end
    t0 = t0 + segT(k);
end
end

function savePlanningGif(robot, q_traj, fig, ax, gifFile, delayTime)
hold(ax,'on');
xl = xlim(ax); yl = ylim(ax); zl = zlim(ax);
ws = [xl(1) xl(2) yl(1) yl(2) zl(1) zl(2)];
P = transl(robot.fkine(q_traj));
hTrail = plot3(ax, nan, nan, nan, '-', 'LineWidth', 2);
for i=1:size(q_traj,1)
    axes(ax); %#ok<LAXES>
    robot.plot(q_traj(i,:), 'workspace', ws, 'scale', 0.5, 'delay', 0);
    set(hTrail, 'XData', P(1:i,1), 'YData', P(1:i,2), 'ZData', P(1:i,3));
    drawnow;
    frame = getframe(fig);
    [imind, cm] = rgb2ind(frame2im(frame), 256);
    if i==1
        imwrite(imind, cm, gifFile, 'gif', 'Loopcount', inf, 'DelayTime', delayTime);
    else
        imwrite(imind, cm, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
    end
end
fprintf('✅ GIF 已保存：%s\n', gifFile);
end

function plotParetoComparison(pareto)
% =========================================================
% 三目标 Pareto 前沿可视化
% f1: 路径长度 (m)
% f2: 能耗 / 关节总旋转量 (rad)
% f3: 轨迹时间 (s)
% =========================================================

F = pareto.F;

f1 = F(:,1);
f2 = F(:,2);
f3 = F(:,3);

%% ========== 1) 三维 Pareto 前沿 ==========
figure(600); clf;
scatter3(f1, f2, f3, 60, f3, 'filled');
grid on; box on;
xlabel('Path Length (m)');
ylabel('Energy Consumption (rad)');
zlabel('Trajectory Time (s)');
title('Pareto Front in Objective Space (3D)');
colormap(jet);
colorbar;
view(45, 25);

%% ========== 2) 二维投影：f1 - f2 ==========
figure(601); clf;
scatter(f1, f2, 50, 'filled');
grid on; box on;
xlabel('Path Length (m)');
ylabel('Energy Consumption (rad)');
title('Pareto Projection: Path Length vs Energy');

%% ========== 3) 二维投影：f1 - f3 ==========
figure(602); clf;
scatter(f1, f3, 50, 'filled');
grid on; box on;
xlabel('Path Length (m)');
ylabel('Trajectory Time (s)');
title('Pareto Projection: Path Length vs Time');

%% ========== 4) 二维投影：f2 - f3 ==========
figure(603); clf;
scatter(f2, f3, 50, 'filled');
grid on; box on;
xlabel('Energy Consumption (rad)');
ylabel('Trajectory Time (s)');
title('Pareto Projection: Energy vs Time');

end

%% ======================== NSGA-II：带固定控制点的实现 ========================
function [ctrl, fixedCtrlMask, ctrlBaseIdx] = pickControlPointsConstrained(q_base, Nv, fixedBaseIdx)
% 选择 Nv 个控制点；强制包含 fixedBaseIdx（在 q_base 中的行索引）
% 修复：对 ctrlBaseIdx 做去重 / 排序 / 越界裁剪
n = size(q_base,1);
Nv = min(Nv, n);

baseIdx = unique(round(linspace(1,n,Nv)));
fixedBaseIdx = unique(fixedBaseIdx(:).');

% 合并 & 去重
allIdx = unique([baseIdx, fixedBaseIdx]);
% 越界裁剪
allIdx = allIdx(allIdx>=1 & allIdx<=n);
% 如果数量超出 Nv：优先保留固定点，再在非固定候选里等距抽样补齐
if numel(allIdx) > Nv
    nonFixed = setdiff(allIdx, fixedBaseIdx);
    need = Nv - numel(fixedBaseIdx);
    need = max(0, need);
    if need > 0 && ~isempty(nonFixed)
        take = unique(round(linspace(1,numel(nonFixed),need)));
        allIdx = sort([fixedBaseIdx, nonFixed(take)]);
    else
        allIdx = sort(fixedBaseIdx);
        allIdx = allIdx(1:Nv);
    end
elseif numel(allIdx) < Nv
    remain = setdiff(1:n, allIdx);
    if ~isempty(remain)
        take = unique(round(linspace(1,numel(remain), Nv-numel(allIdx))));
        allIdx = sort([allIdx, remain(take)]);
    end
end

% 最终再次裁剪 & 排序
allIdx = unique(allIdx);
allIdx = allIdx(allIdx>=1 & allIdx<=n);
% 若仍不足，兜底补到 Nv
if numel(allIdx) < Nv
    extra = setdiff(1:n, allIdx);
    add = extra(1:min(numel(extra), Nv-numel(allIdx)));
    allIdx = sort([allIdx, add]);
elseif numel(allIdx) > Nv
    allIdx = allIdx(1:Nv);
end

ctrlBaseIdx   = allIdx(:);
ctrl          = q_base(ctrlBaseIdx, :);
fixedCtrlMask = ismember(ctrlBaseIdx, fixedBaseIdx(:));

% 保证首尾固定（保险）
fixedCtrlMask(1)   = true;
fixedCtrlMask(end) = true;
end

function [f, pen, traj] = evaluateIndividualConstrained(robot, q_base, ctrl, freeIdx, x, qdot_max, Ts, obstacles, safety, q_lim)
tscale = x(1); s = x(2);
Mfree = numel(freeIdx);
d = reshape(x(3:end), 6, Mfree).';
ctrl_new = ctrl;
for k=1:Mfree
    idx = freeIdx(k);
    ctrl_new(idx,:) = ctrl_new(idx,:) + d(k,:);
end
ctrl_new = max(min(ctrl_new, q_lim(:,2).'), q_lim(:,1).');
ds = vecnorm(diff(ctrl_new),2,2); sacc=[0; cumsum(ds)];
if sacc(end)<1e-9, sacc(end)=1; end
sacc = sacc / sacc(end);
baseN = size(q_base,1);
N = max(20, round(baseN * tscale));
si = linspace(0,1,N).';
qi_pchip = zeros(N,6);
qi_csaps = zeros(N,6);
usePchip = s < 1;
useCsaps = s > 0 && hasCsaps();
for j=1:6
    if usePchip || ~useCsaps
        qi_pchip(:,j) = pchip(sacc, ctrl_new(:,j), si);
    end
    if useCsaps
        qi_csaps(:,j) = csaps(sacc, ctrl_new(:,j), 0.8, si);
    else
        qi_csaps(:,j) = qi_pchip(:,j);
    end
end
qi = (1-s)*qi_pchip + s*qi_csaps;

% ===== 关键修复：强制起点和终点严格等于原始路径的起点和终点 =====
qi(1,:) = ctrl_new(1,:);
qi(end,:) = ctrl_new(end,:);
% ========================================================

[q_exec, t_exec] = timeParamFromPath(qi, qdot_max, Ts);
T = robot.fkine(q_exec); P = transl(T);
f1 = sum( vecnorm(diff(P),2,2) );
f2 = sum( sum(abs(diff(q_exec))) );
if ~isempty(t_exec)
    f3 = t_exec(end);
else
    f3 = (size(q_exec,1)-1) * Ts;
end
pen = 0;
if ~isempty(obstacles)
    for k=1:size(q_exec,1)-1
        qa = q_exec(k,:); qb = q_exec(k+1,:);
        if edgeCollision(robot, qa, qb, obstacles, safety, 20)
            pen = pen + 1;
        end
    end
end
bigM = 1e3;
f = [f1, f2, f3] + pen*bigM;
if nargout >= 3
    traj.q = q_exec;
    traj.t = t_exec;
    traj.metrics.pathLength = f1;
    traj.metrics.totalJointMove = f2;
    traj.metrics.trajTime = f3;
    traj.penalty = pen;
end
end
function has = hasCsaps()
persistent cachedHas
if isempty(cachedHas)
    cachedHas = ~isempty(which('csaps'));
end
has = cachedHas;
end
function [best, pareto, hist] = nsga2OptimizePath(evalFun, lb, ub, cfg)
% 简洁 NSGA-II（实数编码 + SBX 交叉 + 多项式变异）
D = numel(lb); NP = cfg.popSize; G = cfg.maxGen;
Pop = repmat(lb,NP,1) + rand(NP,D).*repmat((ub-lb),NP,1);
[F,~] = batchEval(evalFun, Pop);
[fronts, rank, crowd] = fastNonDominatedSort(F);
hist.best = nan(G,3);

for gen=1:G
    hist.best(gen,:) = min(F,[],1);               % 本代最优
    Mating = tournamentSelect(rank, crowd, NP);   % 选择
    % 交叉 + 变异
    Off = zeros(NP,D);
    for i=1:2:NP
        a = Pop(Mating(i),:); b = Pop(Mating(i+1),:);
        [c1,c2] = sbxCrossover(a,b,lb,ub,cfg.etaC);
        c1 = polyMutation(c1,lb,ub,cfg.mutProb,cfg.etaM);
        c2 = polyMutation(c2,lb,ub,cfg.mutProb,cfg.etaM);
        Off(i,:) = c1; Off(i+1,:) = c2;
    end
    % 合并选择
    Pool = [Pop; Off];
    [Fpool,~] = batchEval(evalFun, Pool);
    [fronts, rankPool, crowdPool] = fastNonDominatedSort(Fpool);
    % 生成下一代
    PopNew = zeros(NP,D); FNew = zeros(NP,size(F,2));
    idxFill = 0;
    for fi=1:numel(fronts)
        front = fronts{fi};
        if idxFill + numel(front) <= NP
            PopNew(idxFill+1:idxFill+numel(front),:) = Pool(front,:);
            FNew(idxFill+1:idxFill+numel(front),:)   = Fpool(front,:);
            idxFill = idxFill + numel(front);
        else
            [~,ord] = sort(crowdPool(front),'descend');
            take = NP - idxFill;
            pick = front(ord(1:take));
            PopNew(idxFill+1:NP,:) = Pool(pick,:);
            FNew(idxFill+1:NP,:)   = Fpool(pick,:);
            break;
        end
    end
    Pop = PopNew; F = FNew;
    [fronts, rank, crowd] = fastNonDominatedSort(F);
    fprintf('Gen %2d | Pareto size: %d | f_min = [%.4f %.4f %.4f]\n', ...
        gen, numel(fronts{1}), min(F(:,1)), min(F(:,2)), min(F(:,3)));
end

pareto.idx = fronts{1};
pareto.X   = Pop(pareto.idx,:);
pareto.F   = F(pareto.idx,:);
pareto.crowd = crowd(pareto.idx);

[~, pick] = max(pareto.crowd);
best.x = pareto.X(pick,:);
best.f = pareto.F(pick,:);
end

function [F,Pen] = batchEval(evalFun, Pop)
NP = size(Pop,1);
F = zeros(NP,3); Pen = zeros(NP,1);
if NP >= 16 && canUseParallelPool()
    parfor i=1:NP
        [fi,pen] = evalFun(Pop(i,:));
        F(i,:) = fi; Pen(i) = pen;
    end
else
    for i=1:NP
        [fi,pen] = evalFun(Pop(i,:));
        F(i,:) = fi; Pen(i) = pen;
    end
end
end

function tf = canUseParallelPool()
% 稳健检查并行池是否可用（兼容 MATLAB 2022b 到 2025b）
persistent cachedCanUse
if ~isempty(cachedCanUse)
    tf = cachedCanUse;
    return;
end
cachedCanUse = false;
% 检查 Parallel Computing Toolbox 是否安装
if exist('gcp', 'file') ~= 2 || exist('parpool', 'file') ~= 2
    tf = false;
    return;
end
try
    pool = gcp('nocreate');
    if isempty(pool)
        % MATLAB 2025b: parpool 可能需要显式指定 profile
        try
            pool = parpool('local');
        catch
            % 回退到默认
            pool = parpool;
        end
    end
    cachedCanUse = ~isempty(pool) && isvalid(pool);
catch ME
    % 并行池不可用，静默回退到串行
    warning(ME.identifier, 'Parallel pool unavailable, using serial evaluation. Reason: %s', ME.message);
    cachedCanUse = false;
end
tf = cachedCanUse;
end

function [fronts, rank, crowd] = fastNonDominatedSort(F)
N = size(F,1);
S = cell(N,1); n = zeros(N,1);
rank = zeros(N,1);
fronts = {};
if N == 0
    crowd = zeros(0,1);
    return;
end
Fp = reshape(F, N, 1, []);
Fq = reshape(F, 1, N, []);
dominates = all(Fp <= Fq, 3) & any(Fp < Fq, 3);
dominates(1:N+1:end) = false;
for p=1:N
    S{p} = find(dominates(p,:));
end
n = sum(dominates,1).';
fronts{1} = find(n==0).';
rank(fronts{1}) = 1;
i=1;
while ~isempty(fronts{i})
    Q=[];
    for p=fronts{i}
        for q=S{p}
            n(q)=n(q)-1;
            if n(q)==0
                rank(q)=i+1; Q=[Q q]; %#ok<AGROW>
            end
        end
    end
    i=i+1; fronts{i}=Q; %#ok<AGROW>
    if isempty(Q), break; end
end
fronts = fronts(~cellfun(@isempty,fronts));

crowd = zeros(N,1);
for fi=1:numel(fronts)
    I = fronts{fi};
    if isempty(I), continue; end
    if numel(I)<=2, crowd(I)=inf; continue; end
    f = F(I,:);
    for m=1:size(F,2)
        [fm,ord] = sort(f(:,m));
        crowd(I(ord(1)))   = inf;
        crowd(I(ord(end))) = inf;
        denom = fm(end)-fm(1);
        if denom < 1e-12, denom = 1; end
        for k=2:numel(I)-1
            crowd(I(ord(k))) = crowd(I(ord(k))) + (fm(k+1)-fm(k-1))/denom;
        end
    end
end
end

function sel = tournamentSelect(rank, crowd, NP)
sel = zeros(NP,1);
for i=1:NP
    a = randi(NP); b = randi(NP);
    if rank(a) < rank(b), sel(i)=a;
    elseif rank(b) < rank(a), sel(i)=b;
    else
        if crowd(a) > crowd(b), sel(i)=a; else, sel(i)=b; end
    end
end
end

function [c1,c2] = sbxCrossover(p1,p2,lb,ub,etaC)
u = rand(size(p1));
beta = zeros(size(p1));
beta(u<=0.5) = (2*u(u<=0.5)).^(1/(etaC+1));
beta(u>0.5)  = (2*(1-u(u>0.5))).^(-1/(etaC+1));
c1 = 0.5*((1+beta).*p1 + (1-beta).*p2);
c2 = 0.5*((1-beta).*p1 + (1+beta).*p2);
c1 = min(max(c1,lb),ub); c2 = min(max(c2,lb),ub);
end

function c = polyMutation(c,lb,ub,pm,etaM)
for i=1:numel(c)
    if rand < pm
        u = rand;
        if u < 0.5
            delta = (2*u)^(1/(etaM+1)) - 1;
        else
            delta = 1 - (2*(1-u))^(1/(etaM+1));
        end
        c(i) = c(i) + delta*(ub(i)-lb(i));
        c(i) = min(max(c(i),lb(i)),ub(i));
    end
end
end

%% ======================== 报表与曲线 ========================
function plotCompactJointStack(figNo, t, yData, lineColor, lineWidth, symmetricY)
figure(figNo); clf;
fig = gcf;
set(fig, 'Color', 'w', 'Units', 'points', 'Position', [160 100 540 405]);

for j = 1:6
    ax = compactJointAxes(fig, j);
    plot(ax, t, yData(:, j), '-', 'Color', lineColor, 'LineWidth', lineWidth);
    styleCompactJointAxes(ax, t, yData(:, j), sprintf('J%d', j), symmetricY);
end
end

function ax = compactJointAxes(fig, idx)
nRows = 3;
nCols = 2;
left = 0.155;
right = 0.045;
bottom = 0.095;
top = 0.075;
colGap = 0.090;
rowGap = 0.125;

row = ceil(idx / nCols);
col = idx - (row - 1) * nCols;
axW = (1 - left - right - (nCols - 1) * colGap) / nCols;
axH = (1 - top - bottom - (nRows - 1) * rowGap) / nRows;
x = left + (col - 1) * (axW + colGap);
y = 1 - top - row * axH - (row - 1) * rowGap;
ax = axes('Parent', fig, 'Position', [x y axW axH]);
end

function styleCompactJointAxes(ax, t, y, jointLabel, symmetricY)
grid(ax, 'on');
box(ax, 'on');
xlim(ax, [0 t(end)]);
setThreeTicksLocal(ax, 'x', [0 t(end)], false);
setThreeTicksLocal(ax, 'y', y, symmetricY);
xlabel(ax, '');
ylabel(ax, '');
title(ax, jointLabel, 'FontWeight', 'bold');
set(ax, 'FontName', 'Helvetica', ...
    'FontSize', 13, ...
    'LineWidth', 0.8, ...
    'TickDir', 'out', ...
    'Layer', 'top');
xtickformat(ax, '%.1f');
ytickformat(ax, '%.2g');
end

function setThreeTicksLocal(ax, axisName, values, symmetricY)
if strcmp(axisName, 'x')
    lim = [values(1), values(2)];
elseif symmetricY
    ymax = max(abs(values(:)));
    if ymax < eps
        ymax = 1;
    end
    lim = [-ymax, ymax];
else
    ymin = min(values(:));
    ymax = max(values(:));
    pad = 0.05 * max(ymax - ymin, 1e-6);
    lim = [ymin - pad, ymax + pad];
end

ticks = [lim(1), mean(lim), lim(2)];
if strcmp(axisName, 'x')
    xlim(ax, lim);
    xticks(ax, ticks);
else
    ylim(ax, lim);
    yticks(ax, ticks);
end
end

function plotNsgaReports(robot, q_base, q_opt, pareto, hist, Ts)
if nargin < 6, Ts = 0.02; end

figure(201); clf;
subplot(2,2,1);
scatter3(pareto.F(:,1), pareto.F(:,2), pareto.F(:,3), 36, pareto.crowd, 'filled'); grid on;
xlabel('路径长度(m)'); ylabel('关节总角度(rad)'); zlabel('轨迹时间(s)'); title('Pareto 前沿(3D)');

subplot(2,2,2);
plot(hist.best(:,1),'-','LineWidth',1.5); hold on;
plot(hist.best(:,2),'-','LineWidth',1.5);
plot(hist.best(:,3),'-','LineWidth',1.5); grid on;
legend({'路径长度','关节总角度','轨迹时间'},'Location','best');
xlabel('代'); ylabel('最优目标'); title('每代最优目标曲线');

subplot(2,2,3);
scatter(pareto.F(:,1), pareto.F(:,2), 28, 'filled'); grid on;
xlabel('路径长度'); ylabel('关节总角度'); title('Pareto 投影：f1 vs f2');

subplot(2,2,4);
scatter(pareto.F(:,1), pareto.F(:,3), 28, 'filled'); grid on;
xlabel('路径长度'); ylabel('轨迹时间(s)'); title('Pareto 投影：f1 vs f3');

% 末端路径对比 + 累计曲线
figure(202); clf;
P0 = transl(robot.fkine(q_base));
P1 = transl(robot.fkine(q_opt));
subplot(1,2,1); plot3(P0(:,1),P0(:,2),P0(:,3),'--','LineWidth',1.5); hold on;
plot3(P1(:,1),P1(:,2),P1(:,3),'-','LineWidth',2); grid on; axis equal;
xlabel X; ylabel Y; zlabel Z; title('末端路径：基础 vs 优化'); legend('基础','优化');

% 沿轨迹的累计指标（时间改为累计秒数）
subplot(1,2,2);
len = [0; cumsum(vecnorm(diff(P1),2,2))];
joint = [0; cumsum(sum(abs(diff(q_opt)),2))];
t_cum = (0:size(q_opt,1)-1).' * Ts;
plot(len,'-','LineWidth',2); hold on;
plot(joint,'-','LineWidth',2);
plot(t_cum,'-','LineWidth',2); grid on;
legend({'路径长度累计','关节角累计','时间累计(s)'},'Location','best');
xlabel('轨迹采样点'); title('沿轨迹的累计指标');
end
