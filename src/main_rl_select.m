% ========= 6-DOF UR5e: 选择 Tabular Q-learning 或 DQN 进行训练 =========
% R2022b 兼容；DQN 需 Reinforcement Learning Toolbox。
% 状态/任务：顺序到达4个关键点（无障碍），末端姿态用 q0 的 Rref。
% Tabular Q: 离散化“与当前子目标的距离(dist)”为状态之一，轻量可跑
% DQN: 连续观测 [q-qDes; posErr; rpyErr]，动作与之前一致(13个)
clear; close all; clc;

%% ========== 选择算法 ==========
% 'q'   : 表格 Q-learning（无需RL Toolbox）
% 'dqn' : 深度Q学习 DQN（需要RL Toolbox）
use_algo = 'q';   % ← 在此切换

%% ========== 机械臂建模 (UR5e 风格, 标准D-H, 单位 m/rad) ==========
deg = pi/180; mm2m = 1e-3;
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

q0   = [0 90 0 0 180 0]*deg;
Rref = robot.fkine(q0).R;

%% ========== 任务：4个关键点（无障碍） ==========
Wp = [ 0.40  0.20 0.20;
       0.35  0.10 0.35;
       0.10  0.35 0.55;
      -0.05  0.30 0.40 ];
for i=1:4, Twp(i)=SE3(Rref, Wp(i,:)); end %#ok<SAGROW>

% 逆解（链式初值）
q_wp=zeros(4,6); seed=q0;
for i=1:4, q_wp(i,:)=robot.ikcon(Twp(i),seed); seed=q_wp(i,:); end

% 可视化任务
figure(1); clf; ax=axes; hold on; grid on; axis equal; view(45,25);
xlabel X; ylabel Y; zlabel Z; title('任务：4个关键点（无障碍）');
scatter3(Wp(:,1),Wp(:,2),Wp(:,3),70,'filled'); axis([-1 1 -1 1 -0.1 1]);

%% ========== 公共环境参数 ==========
Ts = 0.02;                  % 动画步长
dQ = 2*deg;                 % 单步关节增量
goalPosTol = 0.015;         % 位置阈值
goalRPYTol = 5*deg;         % 姿态阈值
maxSteps = 300;             % 每回合步数上限
qlim = vertcat(robot.links.qlim);

% 共用动作集：1..12=单轴±dQ；13=不动
NUM_ACT = 13;

%% ========== 分支：表格 Q-Learning 或 DQN ==========
switch lower(use_algo)
case 'q'   % ====================== Tabular Q-learning ======================
    % 状态离散化：S = (currWp ∈ {1..4}) × (distBin ∈ {0..Nd})
    Nd = 12;                 % 距离分箱数
    gamma = 0.98;            % 折扣
    alpha = 0.6;             % 学习率
    eps = 1.0; epsMin=0.05; epsDecay=0.995;   % ε-贪心
    Q = zeros(4, Nd+1, NUM_ACT);              % Q表

    EP = 300;  % 回合数
    for ep=1:EP
        % 重置：从关键点1开始
        currWp = 1;
        q = q_wp(1,:) + (randn(1,6)*1*pi/180);   % 起点轻微扰动
        steps = 0; totalR = 0;
        while steps < maxSteps
            % 目标“参考关节”= q_des（用ikcon锁定末端姿态）
            q_des = q_wp(currWp,:);
            [posErr, rpyErr] = poseError(robot, q, q_des);
            dist = norm(posErr);
            distBin = min(Nd, floor(dist/0.03*Nd)); % 0~约0.36m线性分箱（可调）
            s = sub2ind([4, Nd+1], currWp, distBin+1);

            % ε-贪心选动作
            if rand < eps, a = randi(NUM_ACT);
            else
                [~,a] = max(Q(currWp, distBin+1, :));
            end

            % 执行动作
            q_next = applyActionClamp(q, a, dQ, qlim);
            [posErr2, rpyErr2] = poseError(robot, q_next, q_des);
            dist2 = norm(posErr2);

            % 奖励：进步奖励 + 小步惩罚 + 接近目标更小的步惩罚
            progress = 5*(dist - dist2);
            stepCost = 0.01; if dist2 < 2*goalPosTol, stepCost = 0.002; end
            moveCost = 0.05*sum(abs(q_next - q));
            R = progress - (stepCost + moveCost);

            % 是否到达当前子目标
            subReached = (dist2 < goalPosTol) && (norm(rpyErr2) < goalRPYTol);
            if subReached
                R = R + 3;    % 子目标奖励
                currWp = currWp + 1;
                if currWp > 4
                    R = R + 10; % 最终奖励
                end
            end

            % 下一个状态
            if currWp <= 4
                nextBin = min(Nd, floor(dist2/0.03*Nd));
                s2 = sub2ind([4, Nd+1], currWp, nextBin+1);
            else
                s2 = s;
            end

            % Q 更新
            if currWp <= 4
                Q(currWp, distBin+1, a) = (1-alpha)*Q(currWp, distBin+1, a) + ...
                    alpha*(R + gamma*max(Q(currWp, nextBin+1, :)));
            else
                Q(currWp-1, distBin+1, a) = (1-alpha)*Q(currWp-1, distBin+1, a) + ...
                    alpha*(R);  % 终止
            end

            q = q_next;
            steps = steps + 1;
            totalR = totalR + R;
            if currWp > 4, break; end
        end
        eps = max(epsMin, eps*epsDecay);
        fprintf('TabQ Ep %3d | steps=%3d | R=%.2f | eps=%.3f\n', ep, steps, totalR, eps);
    end

    % 评测/回放
    [traj_q, ~] = rolloutTabQ(robot, q_wp, Q, dQ, qlim, goalPosTol, goalRPYTol, maxSteps, Nd);
    figure(2); clf; ax2=axes; hold on; grid on; axis equal; view(45,25);
    xlabel X; ylabel Y; zlabel Z; title('Tabular Q-learning 轨迹');
    scatter3(Wp(:,1),Wp(:,2),Wp(:,3),70,'filled'); axis([-1 1 -1 1 -0.1 1]); 
    drawPath(ax2, robot, traj_q);
    animateArm(robot, traj_q, ax2, Ts);

case 'dqn' % ====================== 深度 Q-learning (DQN) ======================
    rehash toolboxcache;
    assert(~isempty(which('rlFunctionEnv')) && ~isempty(which('rlDQNAgent')), ...
        '未检测到 Reinforcement Learning Toolbox (rlFunctionEnv/rlDQNAgent).');

    % 观测: 12x1 = [q-qDes(6); posErr(3); rpyErr(3)]
    obsInfo = rlNumericSpec([12 1]);
    obsInfo.Name = 'state';

    actInfo = rlFiniteSetSpec(1:NUM_ACT);
    actInfo.Name = 'action';

    envP.robot = robot; envP.q_wp = q_wp; envP.dQ = dQ;
    envP.qlim = qlim; envP.goalPosTol = goalPosTol; envP.goalRPYTol = goalRPYTol;
    envP.maxSteps = maxSteps; envP.numActs = NUM_ACT;

    stepFcn  = @(a,ls) stepWaypointDQN(a,ls,envP);
    resetFcn = @() resetWaypointDQN(envP, q_wp);
    env = rlFunctionEnv(obsInfo, actInfo, stepFcn, resetFcn);

    % Critic: 双输入 Q(s,a_index)->标量（state 12, action 1）
    stateLayers = [
        featureInputLayer(12,'Normalization','none','Name','state')
        fullyConnectedLayer(128,'Name','fc1s')
        reluLayer('Name','r1s')
        fullyConnectedLayer(128,'Name','fc2s')
        reluLayer('Name','r2s')];
    actionLayers = [
        featureInputLayer(1,'Normalization','none','Name','action')
        fullyConnectedLayer(128,'Name','fc1a')
        reluLayer('Name','r1a')];
    commonLayers = [
        concatenationLayer(1,2,'Name','concat')
        fullyConnectedLayer(256,'Name','fc3')
        reluLayer('Name','r3')
        fullyConnectedLayer(1,'Name','Qvalue')];

    lgraph = layerGraph();
    lgraph = addLayers(lgraph,stateLayers);
    lgraph = addLayers(lgraph,actionLayers);
    lgraph = addLayers(lgraph,commonLayers);
    lgraph = connectLayers(lgraph,'r2s','concat/in1');
    lgraph = connectLayers(lgraph,'r1a','concat/in2');
    dlnet = dlnetwork(lgraph);

    critic = rlQValueFunction( ...
        dlnet, obsInfo, actInfo, ...
        'Observation', {'state'}, 'Action', {'action'});

    % DQN 选项（R2022b）
    epsOpt = rl.option.EpsilonGreedyExploration;
    epsOpt.Epsilon      = 1.0;
    epsOpt.EpsilonMin   = 0.05;
    epsOpt.EpsilonDecay = 0.997;

    agentOpt = rlDQNAgentOptions( ...
        "UseDoubleDQN", true, ...
        "TargetUpdateFrequency", 4, ...
        "TargetSmoothFactor", 5e-4, ...
        "ExperienceBufferLength", 2e5, ...
        "MiniBatchSize", 128, ...
        "DiscountFactor", 0.995, ...
        "EpsilonGreedyExploration", epsOpt);
    agentOpt.CriticOptimizerOptions.LearnRate = 1e-3;
    agentOpt.CriticOptimizerOptions.GradientThreshold = 1.0;

    agent = rlDQNAgent(critic, agentOpt);

    % 训练
    trainOpt = rlTrainingOptions( ...
        "MaxEpisodes", 250, ...
        "MaxStepsPerEpisode", envP.maxSteps, ...
        "ScoreAveragingWindowLength", 25, ...
        "StopTrainingCriteria", "AverageReward", ...
        "StopTrainingValue", 5.0, ...
        "Verbose", true, "Plots", "training-progress");
    trainingStats = train(agent, env, trainOpt); %#ok<NASGU>

    % 评测/回放（关闭探索：把 epsilon 设为很小值，不要设0）
    agent.AgentOptions.EpsilonGreedyExploration.Epsilon = 1e-6;
    [traj_q, ~] = rolloutDQN(agent, envP, q_wp(1,:));
    figure(3); clf; ax3=axes; hold on; grid on; axis equal; view(45,25);
    xlabel X; ylabel Y; zlabel Z; title('DQN 轨迹（评测）');
    scatter3(Wp(:,1),Wp(:,2),Wp(:,3),70,'filled'); axis([-1 1 -1 1 -0.1 1]); 
    drawPath(ax3, robot, traj_q);
    animateArm(robot, traj_q, ax3, Ts);

otherwise
    error('use_algo 只能取 ''q'' 或 ''dqn''。');
end

%% ======================= 辅助函数 =======================

function q_next = applyActionClamp(q, a, dQ, qlim)
    if a==13
        j=1; delta=0;
    else
        j = ceil(a/2);
        delta = ((mod(a,2)==0) - (mod(a,2)==1)) * dQ;  % 偶数 +dQ, 奇数 -dQ
    end
    q_next = q; q_next(j) = q_next(j) + delta;
    q_next = max(min(q_next, qlim(:,2).'), qlim(:,1).');
end

function [posErr, rpyErr] = poseError(robot, q, q_des)
    T  = robot.fkine(q);
    Td = robot.fkine(q_des);
    posErr = Td.t - T.t;
    rpyErr = tr2rpy( T.R' * Td.R );
end

function drawPath(ax, robot, q_traj)
    P = transl(robot.fkine(q_traj));
    plot3(ax, P(:,1), P(:,2), P(:,3), '-', 'LineWidth', 2);
end

function animateArm(robot, q_traj, ax, Ts)
    axes(ax);
    xl=xlim(ax); yl=ylim(ax); zl=zlim(ax);
    ws=[xl(1) xl(2) yl(1) yl(2) zl(1) zl(2)];
    for i=1:size(q_traj,1)
        robot.plot(q_traj(i,:), 'workspace', ws, 'scale', 0.5, 'delay', 0);
        drawnow; pause(Ts);
    end
end

%% =================== DQN 环境（顺序到达4关键点） ===================
function [NextObs,Reward,Done,LS] = stepWaypointDQN(Action, LS, P)
    % 归一化动作索引
    a = normalizeActionIndex(Action, P.numActs);

    % 动作执行
    q_prev = LS.q;
    q = applyActionClamp(q_prev, a, P.dQ, P.qlim);

    % 目标关节（当前子目标的参考关节）
    q_des = P.q_wp(LS.currWp, :);

    % 误差 & 进步
    [posErr, rpyErr]     = poseError(P.robot, q,     q_des);
    [posErrPrev, ~]      = poseError(P.robot, q_prev,q_des);
    dist     = norm(posErr);
    distPrev = norm(posErrPrev);

    % 奖励：进步奖励 + 小步惩罚 + 关节移动惩罚
    progress = 6*(distPrev - dist);
    stepCost = 0.01; if dist < 2*P.goalPosTol, stepCost=0.002; end
    moveCost = 0.1*sum(abs(q - q_prev));
    Reward   = progress - (stepCost + moveCost);

    % 到达/终止
    subReached = (dist < P.goalPosTol) && (norm(rpyErr) < P.goalRPYTol);
    Done = false;
    if subReached
        Reward = Reward + 6;        % 子目标奖励
        if LS.currWp < size(P.q_wp,1)
            LS.currWp = LS.currWp + 1;
        else
            Reward = Reward + 15;   % 最终奖励
            Done   = true;
        end
    end

    % 更新
    LS.q = q;
    LS.step = LS.step + 1;
    if LS.step >= P.maxSteps, Done=true; end

    % 观测
    qDes = P.q_wp(LS.currWp,:);
    [posErr, rpyErr] = poseError(P.robot, q, qDes);
    NextObs = [ (q - qDes)'; posErr(:); rpyErr(:) ];
end

function [InitialObs, LS] = resetWaypointDQN(P, q_wp)
    LS.q = q_wp(1,:) + (randn(1,6)*1*pi/180);
    LS.step = 0; LS.currWp = 1;
    qDes = q_wp(1,:);
    [posErr, rpyErr] = poseError(P.robot, LS.q, qDes);
    InitialObs = [ (LS.q - qDes)'; posErr(:); rpyErr(:) ];
end

function a = normalizeActionIndex(a, numActs)
    if iscell(a), a=a{1}; end
    if isa(a,'categorical'), a = double(a); end
    if isa(a,'dlarray'),     a = extractdata(a); end
    if isnumeric(a) && isvector(a) && numel(a)==numActs && any((a==0)|(a==1)) && sum(a)==1
        [~,a] = max(a);      % one-hot
    elseif isnumeric(a) && isvector(a) && numel(a)==numActs
        [~,a] = max(a);      % Q 向量
    end
    if ~isnumeric(a) || isempty(a) || any(~isfinite(a)), a=1; end
    a = max(1, min(numActs, round(a)));
end

%% =================== Tabular Q 回放 ===================
function [traj, ok] = rolloutTabQ(robot, q_wp, Q, dQ, qlim, posTol, rpyTol, maxSteps, Nd)
    traj = q_wp(1,:); ok=false;
    currWp=1; q = traj(end,:);
    for t=1:maxSteps
        [posErr, rpyErr] = poseError(robot, q, q_wp(currWp,:));
        dist = norm(posErr);
        distBin = min(Nd, floor(dist/0.03*Nd));
        % 选择 Q 最大的动作
        [~,a] = max(Q(currWp, distBin+1, :));
        q = applyActionClamp(q, a, dQ, qlim);
        traj(end+1,:) = q; %#ok<AGROW>
        % 切换目标
        [posErr2, rpyErr2] = poseError(robot, q, q_wp(currWp,:));
        if (norm(posErr2) < posTol) && (norm(rpyErr2) < rpyTol)
            currWp = currWp + 1;
            if currWp > size(q_wp,1), ok=true; break; end
        end
    end
end

%% =================== DQN 回放 ===================
function [traj, ok] = rolloutDQN(agent, P, q_start)
    LS.q = q_start; LS.step=0; LS.currWp=1;
    qDes = P.q_wp(1,:);
    [posErr, rpyErr] = poseError(P.robot, LS.q, qDes);
    obs = [ (LS.q - qDes)'; posErr(:); rpyErr(:) ];
    traj = LS.q; ok=false;
    for t=1:P.maxSteps
        a = getAction(agent, obs);             % R2022b：只传一个输入
        [obs,~,done,LS] = stepWaypointDQN(a, LS, P);
        traj(end+1,:) = LS.q; %#ok<AGROW>
        if done
            ok = (LS.currWp > size(P.q_wp,1));
            break;
        end
    end
end
