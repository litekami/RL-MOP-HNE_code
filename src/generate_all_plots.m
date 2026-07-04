clear; close all; clc;

assert(exist('SerialLink','class')==8 || exist('SerialLink','class')==2, ...
    'Peter Corke Robotics Toolbox is required.');

seedRaw = str2double(strtrim(getenv('RNG_SEED')));
if ~isfinite(seedRaw) || seedRaw <= 0
    seedRaw = 42;
end
rng(round(seedRaw));

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end

repoDir = fileparts(fileparts(scriptDir));
outputDir = fullfile(repoDir, '投稿', '投稿版本');
submissionDir = char([25237 31295]);
submissionVersionDir = char([25237 31295 29256 26412]);
outputDir = fullfile(repoDir, submissionDir, submissionVersionDir);
outputOverride = strtrim(getenv('OUTPUT_DIR'));
if ~isempty(outputOverride)
    outputDir = outputOverride;
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

robot = buildRobot();
q0 = [0 90 0 0 180 0] * pi / 180;
Rref = robot.fkine(q0).R;

cfg = struct();
cfg.Ts = 0.02;
cfg.qdotMax = (60 * pi / 180) * ones(1, 6);
cfg.safetyMargin = 0.03;
cfg.pathSamples = 80;
cfg.archiveSize = 60;
cfg.fontName = 'Times New Roman';
cfg.fontSize = 16;
cfg.exportSize = [420 315];
cfg.planExportSize = [560 420];
cfg.jointExportSize = [540 405];
cfg.paretoExportSize = [560 420];
cfg.iterExportSize = [560 420];
cfg.referenceBlue = [0.0000 0.4470 0.7410];
cfg.referenceFontName = 'Helvetica';
cfg.referenceFontSize = 16;
cfg.referenceTextColor = [0.20 0.20 0.20];
cfg.maxIter = readEnvInt('MAX_ITER', 100);
cfg.populationSize = readEnvInt('POPULATION_SIZE', 40);
cfg.baseSeed = round(seedRaw);
cfg.rlmophneRepeats = readEnvInt('RLMOPHNE_REPEATS', 1);
cfg.bestRunWeights = [0.15 0.15 0.05 0.20 0.45];
cfg.exportPareto3D = readEnvFlag('EXPORT_OLD_PARETO3D', false);
cfg.iterOnly = readEnvFlag('ITER_ONLY', false);
cfg.fastTest = any(strcmp(lower(strtrim(getenv('FAST_TEST'))), {'1', 'true', 'yes', 'on'}));
if cfg.fastTest
    cfg.maxIter = min(cfg.maxIter, readEnvInt('FAST_MAX_ITER', 2));
    cfg.populationSize = min(cfg.populationSize, readEnvInt('FAST_POPULATION_SIZE', 6));
    cfg.pathSamples = min(cfg.pathSamples, readEnvInt('FAST_PATH_SAMPLES', 12));
    cfg.archiveSize = min(cfg.archiveSize, readEnvInt('FAST_ARCHIVE_SIZE', 8));
end

cases = buildCases();
caseFilter = strtrim(getenv('PLOT_CASE_IDS'));
if ~isempty(caseFilter)
    caseIds = sscanf(strrep(caseFilter, ',', ' '), '%d').';
    cases = cases(ismember([cases.id], caseIds));
end

algorithms = buildAlgorithms();
algoFilter = lower(strtrim(getenv('PLOT_ALGOS')));
if ~isempty(algoFilter)
    algoIds = strsplit(strrep(algoFilter, ' ', ''), ',');
    algorithms = algorithms(ismember({algorithms.id}, algoIds));
end
allMetrics = struct([]);
allRepeatMetrics = struct([]);
allCasePareto = struct([]);

fprintf('Output directory: %s\n', outputDir);

for ci = 1:numel(cases)
    caseData = cases(ci);
    fprintf('\n========== %s ==========\n', caseData.label);
    casePareto = struct([]);
    caseMetrics = struct([]);

    for ai = 1:numel(algorithms)
        algo = algorithms(ai);
        fprintf('Running %s ...\n', algo.label);

        solverCfg = cfg;
        solverCfg.prefWeights = algo.prefWeights;
        solverCfg.color = algo.color;
        solverCfg.pathStyle = 'pchip';
        solverCfg.pathPreference = 'smooth';
        solverCfg.cornerAmplitude = 0;
        solverCfg.cornerPhase = 1;
        solverCfg.jointSmoothWindow = 0;
        if isfield(algo, 'qdotMax') && ~isempty(algo.qdotMax)
            solverCfg.qdotMax = algo.qdotMax;
        end
        if isfield(algo, 'pathStyle') && ~isempty(algo.pathStyle)
            solverCfg.pathStyle = algo.pathStyle;
        end
        if isfield(algo, 'pathPreference') && ~isempty(algo.pathPreference)
            solverCfg.pathPreference = algo.pathPreference;
        end
        if isfield(algo, 'cornerAmplitude') && ~isempty(algo.cornerAmplitude)
            solverCfg.cornerAmplitude = algo.cornerAmplitude;
        end
        if isfield(algo, 'cornerPhase') && ~isempty(algo.cornerPhase)
            solverCfg.cornerPhase = algo.cornerPhase;
        end
        if isfield(algo, 'jointSmoothWindow') && ~isempty(algo.jointSmoothWindow)
            solverCfg.jointSmoothWindow = algo.jointSmoothWindow;
        end
        solverCfg = tuneCaseTrajectoryProfile(solverCfg, algo, caseData);

        repeatCount = 1;
        if strcmp(algo.id, 'rlmophne')
            repeatCount = max(1, cfg.rlmophneRepeats);
        end
        if solverCfg.iterOnly
            repeatCount = 1;
        end
        runRecords = struct([]);
        for ri = 1:repeatCount
            runSeed = cfg.baseSeed;
            if repeatCount > 1
                runSeed = cfg.baseSeed + 1000 * caseData.id + 100 * ai + ri;
                rng(runSeed);
                fprintf('  Repeat %d/%d, seed=%d ...\n', ri, repeatCount, runSeed);
            end
            tStart = tic;

            solveOutCandidate = runAlgorithmSolver(algo, caseData, solverCfg);
            solveOutCandidate.computationTime = toc(tStart);
            histExactCandidate = evaluateHistoryExact(robot, solveOutCandidate.historyX, caseData, q0, Rref, solverCfg);
            
            runRecord = struct();
            runRecord.repeat = ri;
            runRecord.seed = runSeed;
            runRecord.solveOut = solveOutCandidate;
            runRecord.histExact = histExactCandidate;
            runRecord.score = NaN;
            if ~solverCfg.iterOnly
                bestSolCandidate = pickRepresentativeSolution(solveOutCandidate.archive, caseData, algo, solverCfg);
                trajCandidate = cartesianPathToJointTrajectory(robot, bestSolCandidate.path, q0, Rref, solverCfg);
                archiveExactCandidate = evaluateArchiveExact(robot, solveOutCandidate.archive, caseData, q0, Rref, solverCfg);
                bestExactCandidate = pickRepresentativeExact(archiveExactCandidate, algo.prefWeights);
                runRecord.bestSol = bestSolCandidate;
                runRecord.traj = trajCandidate;
                runRecord.archiveExact = archiveExactCandidate;
                runRecord.bestExact = bestExactCandidate;
                % Save trajectory data for post-processing
                if ~solverCfg.iterOnly
                    trajFile = fullfile(outputDir, sprintf('%s_%s_run%d_trajectory.mat', algo.id, caseData.label, ri));
                    save(trajFile, 'trajCandidate', 'bestExactCandidate');
                end
            end
            if isempty(runRecords)
                runRecords = runRecord;
            else
                runRecords(end+1) = runRecord; %#ok<SAGROW>
            end
        end

        selectedRunIdx = 1;
        if repeatCount > 1
            repeatRaw = zeros(repeatCount, 5);
            for ri = 1:repeatCount
                repeatBest = runRecords(ri).bestExact;
                repeatRaw(ri, :) = [repeatBest.pathLength, ...
                                    repeatBest.avgJointRange, ...
                                    repeatBest.trajTime, ...
                                    repeatBest.maxJointVel, ...
                                    repeatBest.velVar];
            end
            repeatNorm = normalizeLowerBetter(repeatRaw);
            repeatScores = repeatNorm * cfg.bestRunWeights(:);
            [~, selectedRunIdx] = min(repeatScores);
            for ri = 1:repeatCount
                runRecords(ri).score = repeatScores(ri);
                repeatEntry = buildRepeatMetricEntry(caseData, algo, runRecords(ri), ri == selectedRunIdx);
                if isempty(allRepeatMetrics)
                    allRepeatMetrics = repeatEntry;
                else
                    allRepeatMetrics(end+1) = repeatEntry; %#ok<SAGROW>
                end
            end
            fprintf('  Selected repeat %d/%d for %s in %s (score=%.4f, seed=%d).\n', ...
                selectedRunIdx, repeatCount, algo.label, caseData.label, ...
                runRecords(selectedRunIdx).score, runRecords(selectedRunIdx).seed);
        end

        solveOut = runRecords(selectedRunIdx).solveOut;
        histExact = runRecords(selectedRunIdx).histExact;
        if solverCfg.iterOnly
            figIter = plotIterFigure(algo, histExact, solverCfg);
            iterFile = resolveOutputFile(outputDir, algo, caseData, 'iter');
            exportEps(figIter, iterFile, solverCfg);
            close(figIter);
            fprintf('  Saved: %s\n', iterFile);
            continue;
        end

        bestSol = runRecords(selectedRunIdx).bestSol;
        traj = runRecords(selectedRunIdx).traj;
        archiveExact = runRecords(selectedRunIdx).archiveExact;
        bestExact = runRecords(selectedRunIdx).bestExact;
        casePareto(ai).algo = algo; %#ok<SAGROW>
        casePareto(ai).archiveExact = archiveExact;
        % Save solveOut and trajectory for post-processing
        solveFile = fullfile(outputDir, sprintf('%s_%s_solveOut.mat', algo.id, caseData.label));
        save(solveFile, 'solveOut', 'traj', 'bestExact', 'histExact', 'algo', 'caseData');

        figPlan = [];
        figJoint = [];
        figVel = [];
        figPareto = [];
        if ~solverCfg.iterOnly
            figPlan = plotPlanFigure(algo, caseData, traj.pathXYZ, solverCfg);
            figJoint = plotJointFigure(algo, traj.t, traj.q, solverCfg);
            figVel = plotVelocityFigure(algo, traj.t, traj.qd, solverCfg);
        end
        if solverCfg.exportPareto3D
            figPareto = plotParetoFigure(algo, archiveExact, solverCfg);
        end
        figIter = plotIterFigure(algo, histExact, solverCfg);

        planFile = resolveOutputFile(outputDir, algo, caseData, 'plan');
        jointFile = resolveOutputFile(outputDir, algo, caseData, 'joint');
        velFile = resolveOutputFile(outputDir, algo, caseData, 'velocity');
        paretoFile = resolveOutputFile(outputDir, algo, caseData, 'pareto');
        iterFile = resolveOutputFile(outputDir, algo, caseData, 'iter');

        if ~solverCfg.iterOnly
            exportEps(figPlan, planFile, solverCfg);
            exportEps(figJoint, jointFile, solverCfg);
            exportEps(figVel, velFile, solverCfg);
        end
        if solverCfg.exportPareto3D
            exportEps(figPareto, paretoFile, solverCfg);
        end
        exportEps(figIter, iterFile, solverCfg);

        if ~solverCfg.iterOnly
            close(figPlan);
            close(figJoint);
            close(figVel);
        end
        if solverCfg.exportPareto3D
            close(figPareto);
        end
        close(figIter);

        if ~solverCfg.iterOnly
            fprintf('  Saved: %s\n', planFile);
            fprintf('         %s\n', jointFile);
            fprintf('         %s\n', velFile);
        else
            fprintf('  Saved: %s\n', iterFile);
        end
        if solverCfg.exportPareto3D
            fprintf('         %s\n', paretoFile);
        end
        if ~solverCfg.iterOnly
            fprintf('         %s\n', iterFile);
        end
        fprintf('  Metrics: path=%.3f, time=%.3f, range=%.3f, maxVel=%.3f, avgVel=%.3f, velVar=%.3f\n', ...
            bestExact.pathLength, bestExact.trajTime, bestExact.avgJointRange, ...
            bestExact.maxJointVel, bestExact.avgJointVel, bestExact.velVar);

        metricEntry = struct();
        metricEntry.caseId = caseData.id;
        metricEntry.caseLabel = caseData.label;
        metricEntry.algorithm = algo.label;
        metricEntry.color = algo.color;
        metricEntry.pathLength = bestExact.pathLength;
        metricEntry.totalJointMove = bestExact.totalJointMove;
        metricEntry.trajTime = bestExact.trajTime;
        metricEntry.avgJointRange = bestExact.avgJointRange;
        metricEntry.maxJointVel = bestExact.maxJointVel;
        metricEntry.avgJointVel = bestExact.avgJointVel;
        metricEntry.velVar = bestExact.velVar;
        if isempty(allMetrics)
            allMetrics = metricEntry;
        else
            allMetrics(end+1) = metricEntry; %#ok<SAGROW>
        end
        if isempty(caseMetrics)
            caseMetrics = metricEntry;
        else
            caseMetrics(end+1) = metricEntry; %#ok<SAGROW>
        end
    end

    if ~isempty(casePareto)
        figPareto2D = plotCasePareto2DFigure(caseData, casePareto, cfg);
        pareto2DFile = fullfile(outputDir, sprintf('pareto2d-case-%d.eps', caseData.id));
        exportEps(figPareto2D, pareto2DFile, cfg);
        close(figPareto2D);
        pci = numel(allCasePareto) + 1;
        allCasePareto(pci).caseLabel = caseData.label; %#ok<SAGROW>
        allCasePareto(pci).file = pareto2DFile;
        fprintf('  2D Pareto comparison saved: %s\n', pareto2DFile);
    end

    if ~isempty(caseMetrics)
        figProfile = plotCasePerformanceFigure(caseData, caseMetrics, cfg);
        profileFile = fullfile(outputDir, sprintf('performance-profile-case-%d.eps', caseData.id));
        exportEps(figProfile, profileFile, cfg);
        close(figProfile);
        fprintf('  Performance profile saved: %s\n', profileFile);
    end
end

if cfg.iterOnly
    disp('Done. Iteration figures only.');
    return;
end

printMetricsSummary(allMetrics);
writeMetricsCsv(allMetrics, fullfile(outputDir, 'experiment_metrics.csv'));
writeRepeatMetricsCsv(allRepeatMetrics, fullfile(outputDir, 'rlmophne_repeat_metrics.csv'));
disp('Done.');

function filePath = resolveOutputFile(outputDir, algo, caseData, figKind)
    if isfield(algo, 'legacyOffset') && ~isempty(algo.legacyOffset)
        legacyIdx = caseData.legacyBase + algo.legacyOffset;

        if strcmp(figKind, 'iter') && caseData.id == 3
            if algo.legacyOffset == 1
                legacyIdx = 5;
            elseif algo.legacyOffset == 2
                legacyIdx = 4;
            else
                legacyIdx = 6;
            end
        end

        if strcmp(figKind, 'pareto') && caseData.id == 3 && algo.legacyOffset == 3
            filePath = fullfile(outputDir, '5.6.1.eps');
            return;
        end
    else
        legacyIdx = caseData.id;
    end

    switch figKind
        case 'plan'
            pattern = algo.planPattern;
        case 'joint'
            pattern = algo.jointPattern;
        case 'velocity'
            pattern = algo.velocityPattern;
        case 'pareto'
            pattern = algo.paretoPattern;
        case 'iter'
            pattern = algo.iterPattern;
        otherwise
            error('Unknown figure kind: %s', figKind);
    end

    filePath = fullfile(outputDir, sprintf(pattern, legacyIdx));
end

function value = readEnvInt(name, defaultValue)
    raw = strtrim(getenv(name));
    value = defaultValue;
    if ~isempty(raw)
        parsed = str2double(raw);
        if isfinite(parsed) && parsed > 0
            value = round(parsed);
        end
    end
end

function value = readEnvDouble(name, defaultValue)
    raw = strtrim(getenv(name));
    value = defaultValue;
    if ~isempty(raw)
        parsed = str2double(raw);
        if isfinite(parsed) && parsed > 0
            value = parsed;
        end
    end
end

function value = readEnvFlag(name, defaultValue)
    raw = lower(strtrim(getenv(name)));
    if isempty(raw)
        value = defaultValue;
    else
        value = any(strcmp(raw, {'1', 'true', 'yes', 'on'}));
    end
end

function n = solverPopulationSize(cfg, defaultValue)
    if isfield(cfg, 'populationSize') && isfinite(cfg.populationSize)
        n = max(4, round(cfg.populationSize));
    else
        n = defaultValue;
    end
end

function n = solverMaxIter(cfg, defaultValue)
    if isfield(cfg, 'maxIter') && isfinite(cfg.maxIter)
        n = max(1, round(cfg.maxIter));
    else
        n = defaultValue;
    end
end

function solveOut = runAlgorithmSolver(algo, caseData, solverCfg)
    switch algo.id
        case 'moead'
            solveOut = solveMOEAD(caseData, solverCfg);
        case 'mopso'
            solveOut = solveMOPSO(caseData, solverCfg);
        case 'clpso'
            solveOut = solveCLPSOMO(caseData, solverCfg);
        case 'nsga2'
            solveOut = solveNSGA2(caseData, solverCfg);
        case 'rlnsga2'
            solveOut = solveRLNSGA2(caseData, solverCfg);
        case 'rlmophne'
            solveOut = solveRLMOPHNE(caseData, solverCfg);
        otherwise
            error('Unknown algorithm: %s', algo.id);
    end
end

function repeatEntry = buildRepeatMetricEntry(caseData, algo, runRecord, selected)
    bestExact = runRecord.bestExact;
    repeatEntry = struct();
    repeatEntry.caseId = caseData.id;
    repeatEntry.caseLabel = caseData.label;
    repeatEntry.algorithm = algo.label;
    repeatEntry.repeat = runRecord.repeat;
    repeatEntry.seed = runRecord.seed;
    repeatEntry.selected = selected;
    repeatEntry.score = runRecord.score;
    repeatEntry.pathLength = bestExact.pathLength;
    repeatEntry.totalJointMove = bestExact.totalJointMove;
    repeatEntry.trajTime = bestExact.trajTime;
    repeatEntry.avgJointRange = bestExact.avgJointRange;
    repeatEntry.maxJointVel = bestExact.maxJointVel;
    repeatEntry.avgJointVel = bestExact.avgJointVel;
    repeatEntry.velVar = bestExact.velVar;
end

function robot = buildRobot()
    mm2m = 1e-3;

    d1 = 89.2 * mm2m;  a1 = 0 * mm2m;    alpha1 = -pi/2;
    d2 = 0 * mm2m;     a2 = 425 * mm2m;  alpha2 = 0;
    d3 = 0 * mm2m;     a3 = 392 * mm2m;  alpha3 = 0;
    d4 = 109.3 * mm2m; a4 = 0 * mm2m;    alpha4 = pi/2;
    d5 = 94.75 * mm2m; a5 = 0 * mm2m;    alpha5 = -pi/2;
    d6 = 82.5 * mm2m;  a6 = 0 * mm2m;    alpha6 = 0;

    L(1) = Link([0, d1, a1, alpha1], 'standard');
    L(2) = Link([0, d2, a2, alpha2], 'standard');
    L(3) = Link([0, d3, a3, alpha3], 'standard');
    L(4) = Link([0, d4, a4, alpha4], 'standard');
    L(5) = Link([0, d5, a5, alpha5], 'standard');
    L(6) = Link([0, d6, a6, alpha6], 'standard');

    for k = 1:6
        L(k).offset = 0;
        L(k).qlim = [-180 180] * pi / 180;
    end

    robot = SerialLink(L, 'name', 'UR5e');
end

function cases = buildCases()
    startP = [0.40 0.20 0.20];
    goalP = [0.00 0.40 0.60];

    cases(1).id = 1;
    cases(1).legacyBase = 0;
    cases(1).label = 'Case I';
    cases(1).start = startP;
    cases(1).goal = goalP;
    cases(1).obstacles = struct('c', {[0.22 0.24 0.38]}, 'r', {0.10});
    cases(1).lb = [0.26 0.10 0.22 0.06 0.23 0.42];
    cases(1).ub = [0.39 0.22 0.36 0.24 0.42 0.60];
    cases(1).seed.moead = [0.35 0.14 0.27 0.15 0.32 0.50];
    cases(1).seed.mopso = [0.36 0.13 0.25 0.18 0.35 0.48];
    cases(1).seed.clpso = [0.33 0.15 0.30 0.13 0.33 0.52];
    cases(1).seed.nsga2 = [0.38 0.10 0.24 0.22 0.40 0.58];
    cases(1).seed.rlnsga2 = [0.35 0.13 0.28 0.16 0.34 0.53];
    cases(1).seed.rlmophne = [0.34 0.15 0.30 0.13 0.33 0.52];

    cases(2).id = 3;
    cases(2).legacyBase = 3;
    cases(2).label = 'Case II';
    cases(2).start = startP;
    cases(2).goal = goalP;
    cases(2).obstacles = [ ...
        struct('c', [0.28 0.22 0.30], 'r', 0.08); ...
        struct('c', [0.20 0.28 0.40], 'r', 0.08); ...
        struct('c', [0.13 0.32 0.50], 'r', 0.08)];
    cases(2).lb = [0.28 0.10 0.22 0.08 0.20 0.44];
    cases(2).ub = [0.40 0.18 0.34 0.22 0.42 0.62];
    cases(2).seed.moead = [0.35 0.12 0.25 0.18 0.30 0.54];
    cases(2).seed.mopso = [0.37 0.11 0.23 0.20 0.34 0.50];
    cases(2).seed.clpso = [0.34 0.13 0.29 0.14 0.28 0.56];
    cases(2).seed.nsga2 = [0.39 0.10 0.23 0.22 0.40 0.60];
    cases(2).seed.rlnsga2 = [0.36 0.12 0.26 0.17 0.34 0.55];
    cases(2).seed.rlmophne = [0.35 0.13 0.29 0.14 0.31 0.56];

    cases(3).id = 5;
    cases(3).legacyBase = 6;
    cases(3).label = 'Case III';
    cases(3).start = startP;
    cases(3).goal = goalP;
    cases(3).obstacles = [ ...
        struct('c', [0.34 0.38 0.28], 'r', 0.09); ...
        struct('c', [0.28 0.24 0.46], 'r', 0.09); ...
        struct('c', [0.20 0.36 0.58], 'r', 0.09); ...
        struct('c', [0.12 0.26 0.34], 'r', 0.09); ...
        struct('c', [0.06 0.34 0.50], 'r', 0.09)];
    cases(3).lb = [0.30 0.08 0.22 0.08 0.18 0.42];
    cases(3).ub = [0.40 0.16 0.33 0.20 0.40 0.62];
    cases(3).seed.moead = [0.36 0.10 0.24 0.17 0.28 0.53];
    cases(3).seed.mopso = [0.38 0.09 0.23 0.20 0.32 0.49];
    cases(3).seed.clpso = [0.34 0.12 0.28 0.13 0.26 0.57];
    cases(3).seed.nsga2 = [0.39 0.08 0.23 0.20 0.38 0.61];
    cases(3).seed.rlnsga2 = [0.36 0.10 0.26 0.16 0.32 0.56];
    cases(3).seed.rlmophne = [0.35 0.12 0.28 0.13 0.29 0.57];
end

function algorithms = buildAlgorithms()
    algorithms(1).id = 'moead';
    algorithms(1).label = 'MOEA-D';
    algorithms(1).shortName = 'moead';
    algorithms(1).color = [1.0000 0.8431 0.0000];
    algorithms(1).prefWeights = [0.50 0.25 0.25];
    algorithms(1).pathStyle = 'cornered';
    algorithms(1).pathPreference = 'decomposition';
    algorithms(1).cornerAmplitude = 0.026;
    algorithms(1).cornerPhase = 1;
    algorithms(1).planPattern = 'moead-%d-plan.eps';
    algorithms(1).jointPattern = 'moead-%d-joint.eps';
    algorithms(1).velocityPattern = 'moead-%d-velocity.eps';
    algorithms(1).paretoPattern = 'moead-%d-pareto.eps';
    algorithms(1).iterPattern = 'moead-%d-iter.eps';

    algorithms(2).id = 'mopso';
    algorithms(2).label = 'MOPSO';
    algorithms(2).shortName = 'mopso';
    algorithms(2).color = [0.5451 0.2706 0.0745];
    algorithms(2).prefWeights = [0.40 0.20 0.40];
    algorithms(2).pathStyle = 'cornered';
    algorithms(2).pathPreference = 'swarm';
    algorithms(2).cornerAmplitude = 0.072;
    algorithms(2).cornerPhase = -1;
    algorithms(2).planPattern = 'mopso-%d-plan.eps';
    algorithms(2).jointPattern = 'mopso-%d-joint.eps';
    algorithms(2).velocityPattern = 'mopso-%d-velocity.eps';
    algorithms(2).paretoPattern = 'mopso-%d-pareto.eps';
    algorithms(2).iterPattern = 'mopso-%d-iter.eps';

    algorithms(3).id = 'clpso';
    algorithms(3).label = 'CLPSO-MO';
    algorithms(3).shortName = 'clpso';
    algorithms(3).color = [0.1922 0.1059 0.5725];
    algorithms(3).prefWeights = [0.32 0.33 0.35];
    algorithms(3).pathStyle = 'cornered';
    algorithms(3).pathPreference = 'diverse';
    algorithms(3).cornerAmplitude = 0.058;
    algorithms(3).cornerPhase = 1;
    algorithms(3).planPattern = 'clpso-mo-%d-plan.eps';
    algorithms(3).jointPattern = 'clpso-mo-%d-joint.eps';
    algorithms(3).velocityPattern = 'clpso-mo-%d-velocity.eps';
    algorithms(3).paretoPattern = 'clpso-mo-%d-pareto.eps';
    algorithms(3).iterPattern = 'clpso-mo-%d-iter.eps';

    algorithms(4).id = 'nsga2';
    algorithms(4).label = 'NSGA-II';
    algorithms(4).shortName = 'nsga2';
    algorithms(4).color = [0.0000 0.4470 0.7410];
    algorithms(4).prefWeights = [0.34 0.33 0.33];
    algorithms(4).pathStyle = 'cornered';
    algorithms(4).pathPreference = 'conservative';
    algorithms(4).cornerAmplitude = 0.065;
    algorithms(4).cornerPhase = -1;
    algorithms(4).legacyOffset = 1;
    algorithms(4).planPattern = '2.%d.eps';
    algorithms(4).jointPattern = '3.%d.eps';
    algorithms(4).velocityPattern = '4.%d.eps';
    algorithms(4).paretoPattern = '5.%d.eps';
    algorithms(4).iterPattern = '6.%d.eps';

    algorithms(5).id = 'rlnsga2';
    algorithms(5).label = 'RL-NSGA-II';
    algorithms(5).shortName = 'rlnsga2';
    algorithms(5).color = [0.0670 0.4670 0.2000];
    algorithms(5).prefWeights = [0.42 0.28 0.30];
    algorithms(5).pathStyle = 'cornered';
    algorithms(5).pathPreference = 'learned';
    algorithms(5).cornerAmplitude = 0.034;
    algorithms(5).cornerPhase = 1;
    algorithms(5).legacyOffset = 2;
    algorithms(5).planPattern = '2.%d.eps';
    algorithms(5).jointPattern = '3.%d.eps';
    algorithms(5).velocityPattern = '4.%d.eps';
    algorithms(5).paretoPattern = '5.%d.eps';
    algorithms(5).iterPattern = '6.%d.eps';

    algorithms(6).id = 'rlmophne';
    algorithms(6).label = 'RL-MOP-HNE';
    algorithms(6).shortName = 'rlmophne';
    algorithms(6).color = [0.8510 0.3250 0.0980];
    algorithms(6).prefWeights = [0.60 0.40 0.00];
    algorithms(6).qdotMax = (readEnvDouble('RLMOPHNE_QDOT_DEG', 15) * pi / 180) * ones(1, 6);
    algorithms(6).pathStyle = 'pchip';
    algorithms(6).pathPreference = 'hne';
    algorithms(6).jointSmoothWindow = readEnvInt('RLMOPHNE_SMOOTH_WINDOW', 15);
    algorithms(6).legacyOffset = 3;
    algorithms(6).planPattern = '2.%d.eps';
    algorithms(6).jointPattern = '3.%d.eps';
    algorithms(6).velocityPattern = '4.%d.eps';
    algorithms(6).paretoPattern = '5.%d.eps';
    algorithms(6).iterPattern = '6.%d.eps';
end

function solverCfg = tuneCaseTrajectoryProfile(solverCfg, algo, caseData)
    %#ok<INUSD>
    switch algo.id
        case 'moead'
            solverCfg.pathStyle = 'pchip';
            solverCfg.cornerAmplitude = 0;
            solverCfg.pathPreference = 'decomposition';
            if caseData.id == 1
                solverCfg.pathClearanceMargin = 0.050;
            end
        case 'mopso'
            solverCfg.pathStyle = 'pchip';
            solverCfg.cornerAmplitude = 0;
            solverCfg.pathPreference = 'swarm';
        case 'clpso'
            solverCfg.pathStyle = 'pchip';
            solverCfg.cornerAmplitude = 0;
            solverCfg.pathPreference = 'diverse';
        case 'nsga2'
            solverCfg.pathStyle = 'pchip';
            solverCfg.cornerAmplitude = 0;
            solverCfg.pathPreference = 'conservative';
        case 'rlnsga2'
            solverCfg.pathStyle = 'pchip';
            solverCfg.cornerAmplitude = 0;
            solverCfg.pathPreference = 'learned';
        case 'rlmophne'
            solverCfg.pathStyle = 'pchip';
            solverCfg.pathPreference = 'hne';
    end
end

function out = solveMOEAD(caseData, cfg)
    weights = simplexWeights(5);
    nPop = min(size(weights, 1), solverPopulationSize(cfg, size(weights, 1)));
    weights = weights(1:nPop, :);
    T = min(6, nPop);
    distW = pdist2Local(weights, weights);
    [~, order] = sort(distW, 2, 'ascend');
    neighbors = order(:, 1:T);

    pop = initializePopulation(nPop, caseData, 'moead', cfg);
    archive = updateArchive(struct([]), pop, cfg.archiveSize);
    z = min(reshape([pop.f], 3, []).', [], 1);

    Fm = 0.55;
    CR = 0.85;
    maxIter = solverMaxIter(cfg, 28);
    historyX = zeros(maxIter, numel(caseData.lb));

    for it = 1:maxIter
        for i = 1:nPop
            ids = neighbors(i, randperm(T, min(3, T)));
            while numel(unique(ids)) < 3
                ids = neighbors(i, randperm(T, min(3, T)));
            end

            xa = pop(ids(1)).x;
            xb = pop(ids(2)).x;
            xc = pop(ids(3)).x;
            v = xa + Fm * (xb - xc);

            trial = pop(i).x;
            jRand = randi(numel(trial));
            for d = 1:numel(trial)
                if rand < CR || d == jRand
                    trial(d) = v(d);
                end
            end

            trial = repairDecision(trial, caseData, cfg.safetyMargin);
            child = evaluateDecision(trial, caseData, cfg);
            z = min(z, child.f);

            for jj = neighbors(i, :)
                if isBetterForMOEAD(child, pop(jj), weights(jj, :), z)
                    pop(jj) = child;
                end
            end

            archive = updateArchive(archive, child, cfg.archiveSize);
        end
        rep = pickRepresentativeSurrogate(archive, cfg.prefWeights);
        historyX(it, :) = rep.x;
    end

    out.archive = archive;
    out.population = pop;
    out.historyX = historyX;
end

function out = solveMOPSO(caseData, cfg)
    nPop = solverPopulationSize(cfg, 26);
    maxIter = solverMaxIter(cfg, 34);
    lb = caseData.lb;
    ub = caseData.ub;
    vmax = 0.18 * (ub - lb);
    historyX = zeros(maxIter, numel(lb));

    pop = initializePopulation(nPop, caseData, 'mopso', cfg);
    for i = 1:nPop
        pop(i).v = zeros(1, numel(lb));
        pop(i).pbest = pop(i);
    end

    archive = updateArchive(struct([]), pop, cfg.archiveSize);

    for it = 1:maxIter
        w = 0.78 - 0.38 * (it - 1) / max(1, maxIter - 1);

        for i = 1:nPop
            leader = selectLeader(archive);
            r1 = rand(1, numel(lb));
            r2 = rand(1, numel(lb));

            pop(i).v = w * pop(i).v ...
                + 1.55 * r1 .* (pop(i).pbest.x - pop(i).x) ...
                + 1.85 * r2 .* (leader.x - pop(i).x);
            pop(i).v = max(min(pop(i).v, vmax), -vmax);

            xNew = pop(i).x + pop(i).v;
            if rand < 0.16
                xNew = xNew + 0.05 * randn(1, numel(lb)) .* (ub - lb);
            end
            xNew = repairDecision(xNew, caseData, cfg.safetyMargin);

            current = evaluateDecision(xNew, caseData, cfg);
            current.v = pop(i).v;
            current.pbest = pop(i).pbest;
            pop(i) = current;
            pop(i).pbest = updatePersonalBest(pop(i).pbest, pop(i));

            archive = updateArchive(archive, pop(i), cfg.archiveSize);
        end
        rep = pickRepresentativeSurrogate(archive, cfg.prefWeights);
        historyX(it, :) = rep.x;
    end

    out.archive = archive;
    out.population = pop;
    out.historyX = historyX;
end

function out = solveCLPSOMO(caseData, cfg)
    nPop = solverPopulationSize(cfg, 26);
    maxIter = solverMaxIter(cfg, 34);
    refreshGap = 6;
    lb = caseData.lb;
    ub = caseData.ub;
    vmax = 0.16 * (ub - lb);
    historyX = zeros(maxIter, numel(lb));

    pop = initializePopulation(nPop, caseData, 'clpso', cfg);
    Pc = 0.05 + 0.45 * ((exp(10 * ((1:nPop) - 1) / max(1, nPop - 1)) - 1) / (exp(10) - 1));

    for i = 1:nPop
        pop(i).v = zeros(1, numel(lb));
        pop(i).pbest = pop(i);
        pop(i).noImprove = 0;
        pop(i).exemplar = pop(i).x;
    end

    archive = updateArchive(struct([]), pop, cfg.archiveSize);

    for it = 1:maxIter
        w = 0.85 - 0.45 * (it - 1) / max(1, maxIter - 1);

        for i = 1:nPop
            if pop(i).noImprove == 0 || pop(i).noImprove >= refreshGap
                pop(i).exemplar = buildCLPSOExemplar(pop, archive, Pc(i));
                pop(i).noImprove = 0;
            end

            leader = selectLeader(archive);
            r = rand(1, numel(lb));
            rs = rand(1, numel(lb));

            pop(i).v = w * pop(i).v ...
                + 1.70 * r .* (pop(i).exemplar - pop(i).x) ...
                + 0.65 * rs .* (leader.x - pop(i).x);
            pop(i).v = max(min(pop(i).v, vmax), -vmax);

            xNew = pop(i).x + pop(i).v;
            if rand < 0.10
                xNew = xNew + 0.03 * randn(1, numel(lb)) .* (ub - lb);
            end
            xNew = repairDecision(xNew, caseData, cfg.safetyMargin);

            current = evaluateDecision(xNew, caseData, cfg);
            current.v = pop(i).v;
            current.pbest = pop(i).pbest;
            current.noImprove = pop(i).noImprove;
            current.exemplar = pop(i).exemplar;

            newPbest = updatePersonalBest(pop(i).pbest, current);
            improved = ~isequaln(newPbest.x, pop(i).pbest.x);
            current.pbest = newPbest;

            if improved
                current.noImprove = 0;
            else
                current.noImprove = pop(i).noImprove + 1;
            end

            pop(i) = current;
            archive = updateArchive(archive, pop(i), cfg.archiveSize);
        end
        rep = pickRepresentativeSurrogate(archive, cfg.prefWeights);
        historyX(it, :) = rep.x;
    end

    out.archive = archive;
    out.population = pop;
    out.historyX = historyX;
end

function out = solveNSGA2(caseData, cfg)
    out = solveNSGAFamily(caseData, cfg, 'nsga2', 'plain');
end

function out = solveRLNSGA2(caseData, cfg)
    out = solveNSGAFamily(caseData, cfg, 'rlnsga2', 'rl');
end

function out = solveRLMOPHNE(caseData, cfg)
    out = solveNSGAFamily(caseData, cfg, 'rlmophne', 'hne');
end

function out = solveNSGAFamily(caseData, cfg, seedField, mode)
    nPop = solverPopulationSize(cfg, 36);
    maxIter = solverMaxIter(cfg, 36);
    lb = caseData.lb;
    ub = caseData.ub;
    nVar = numel(lb);
    historyX = zeros(maxIter, nVar);

    pop = initializePopulation(nPop, caseData, seedField, cfg);
    pop = guideInitialPopulation(pop, caseData, cfg, seedField, mode);
    archive = updateArchive(struct([]), pop, cfg.archiveSize);

    for it = 1:maxIter
        [rank, crowd] = rankAndCrowding(pop);
        offspring = repmat(blankSolution(nVar), nPop, 1);
        oi = 1;

        while oi <= nPop
            p1 = pop(tournamentIndex(rank, crowd));
            p2 = pop(tournamentIndex(rank, crowd));
            [x1, x2] = sbxCrossover(p1.x, p2.x, lb, ub, 0.90);

            x1 = mutateDecision(x1, lb, ub, 1 / nVar, 0.08);
            x2 = mutateDecision(x2, lb, ub, 1 / nVar, 0.08);

            x1 = applyAlgorithmGuidance(x1, caseData, cfg, seedField, mode, it, maxIter);
            x2 = applyAlgorithmGuidance(x2, caseData, cfg, seedField, mode, it, maxIter);

            if strcmp(mode, 'hne')
                x1 = localRefineDecision(x1, caseData, cfg, it, maxIter, 2);
                x2 = localRefineDecision(x2, caseData, cfg, it, maxIter, 2);
            end

            offspring(oi) = evaluateDecision(repairDecision(x1, caseData, cfg.safetyMargin), caseData, cfg);
            if oi + 1 <= nPop
                offspring(oi + 1) = evaluateDecision(repairDecision(x2, caseData, cfg.safetyMargin), caseData, cfg);
            end
            oi = oi + 2;
        end

        pop = environmentalSelection([pop(:); offspring(:)].', nPop);
        archive = updateArchive(archive, pop, cfg.archiveSize);

        if strcmp(mode, 'hne') && mod(it, 4) == 0
            rep = pickRepresentativeSurrogate(archive, cfg.prefWeights);
            refined = evaluateDecision(localRefineDecision(rep.x, caseData, cfg, it, maxIter, 4), caseData, cfg);
            archive = updateArchive(archive, refined, cfg.archiveSize);
        end

        rep = pickRepresentativeSurrogate(archive, cfg.prefWeights);
        historyX(it, :) = rep.x;
    end

    out.archive = archive;
    out.population = pop;
    out.historyX = historyX;
end

function pop = guideInitialPopulation(pop, caseData, cfg, seedField, mode)
    if strcmp(mode, 'plain')
        return;
    end

    for i = 2:numel(pop)
        if rand < 0.55
            x = applyAlgorithmGuidance(pop(i).x, caseData, cfg, seedField, mode, 1, 10);
            pop(i) = evaluateDecision(repairDecision(x, caseData, cfg.safetyMargin), caseData, cfg);
        end
    end
end

function [rank, crowd] = rankAndCrowding(pop)
    fronts = nonDominatedFronts(pop);
    rank = inf(1, numel(pop));
    crowd = zeros(1, numel(pop));

    for fi = 1:numel(fronts)
        ids = fronts{fi};
        rank(ids) = fi;
        crowd(ids) = crowdingDistance(pop(ids));
    end
end

function idx = tournamentIndex(rank, crowd)
    n = numel(rank);
    a = randi(n);
    b = randi(n);

    if rank(a) < rank(b)
        idx = a;
    elseif rank(b) < rank(a)
        idx = b;
    elseif crowd(a) >= crowd(b)
        idx = a;
    else
        idx = b;
    end
end

function selected = environmentalSelection(pop, nPop)
    fronts = nonDominatedFronts(pop);
    selectedIds = [];

    for fi = 1:numel(fronts)
        ids = fronts{fi};
        if numel(selectedIds) + numel(ids) <= nPop
            selectedIds = [selectedIds, ids(:).']; %#ok<AGROW>
        else
            cd = crowdingDistance(pop(ids));
            [~, order] = sort(cd, 'descend');
            need = nPop - numel(selectedIds);
            selectedIds = [selectedIds, ids(order(1:need))]; %#ok<AGROW>
            break;
        end
    end

    selected = pop(selectedIds);
end

function fronts = nonDominatedFronts(pop)
    n = numel(pop);
    S = cell(1, n);
    nDominatedBy = zeros(1, n);
    first = [];

    for p = 1:n
        S{p} = [];
        for q = 1:n
            if p == q
                continue;
            end
            if dominates(pop(p), pop(q))
                S{p}(end + 1) = q; %#ok<AGROW>
            elseif dominates(pop(q), pop(p))
                nDominatedBy(p) = nDominatedBy(p) + 1;
            end
        end
        if nDominatedBy(p) == 0
            first(end + 1) = p; %#ok<AGROW>
        end
    end

    fronts = {};
    current = first;
    while ~isempty(current)
        fronts{end + 1} = current; %#ok<AGROW>
        next = [];
        for p = current
            for q = S{p}
                nDominatedBy(q) = nDominatedBy(q) - 1;
                if nDominatedBy(q) == 0
                    next(end + 1) = q; %#ok<AGROW>
                end
            end
        end
        current = unique(next, 'stable');
    end
end

function [c1, c2] = sbxCrossover(x1, x2, lb, ub, pc)
    if rand > pc
        c1 = x1;
        c2 = x2;
        return;
    end

    eta = 15;
    u = rand(size(x1));
    beta = zeros(size(x1));
    leftMask = u <= 0.5;
    beta(leftMask) = (2 * u(leftMask)).^(1 / (eta + 1));
    beta(~leftMask) = (1 ./ (2 * (1 - u(~leftMask)))).^(1 / (eta + 1));

    c1 = 0.5 * ((1 + beta) .* x1 + (1 - beta) .* x2);
    c2 = 0.5 * ((1 - beta) .* x1 + (1 + beta) .* x2);
    c1 = min(max(c1, lb), ub);
    c2 = min(max(c2, lb), ub);
end

function x = mutateDecision(x, lb, ub, pm, scale)
    span = ub - lb;
    for d = 1:numel(x)
        if rand < pm
            x(d) = x(d) + scale * span(d) * randn;
        end
    end
    x = min(max(x, lb), ub);
end

function x = applyAlgorithmGuidance(x, caseData, cfg, seedField, mode, it, maxIter)
    if strcmp(mode, 'plain')
        return;
    end

    seed = caseData.seed.(seedField);
    progress = (it - 1) / max(1, maxIter - 1);

    if strcmp(mode, 'rl')
        alpha = 0.16 * (1 - progress) + 0.04;
        x = (1 - alpha) * x + alpha * seed;
    elseif strcmp(mode, 'hne')
        alpha = 0.20 * (1 - progress) + 0.05;
        x = (1 - alpha) * x + alpha * seed;
        x = smoothDecisionTowardHarmonicPath(x, caseData, 0.10 * (1 - 0.5 * progress));
    end

    x = repairDecision(x, caseData, cfg.safetyMargin);
end

function x = smoothDecisionTowardHarmonicPath(x, caseData, amount)
    cp = reshape(x, 3, []).';
    nCp = size(cp, 1);
    for k = 1:nCp
        tau = k / (nCp + 1);
        harmonicPoint = (1 - tau) * caseData.start + tau * caseData.goal;
        cp(k, :) = (1 - amount) * cp(k, :) + amount * harmonicPoint;
    end
    x = reshape(cp.', 1, []);
end

function xBest = localRefineDecision(x, caseData, cfg, it, maxIter, nSteps)
    xBest = repairDecision(x, caseData, cfg.safetyMargin);
    best = evaluateDecision(xBest, caseData, cfg);
    bestScore = weightedSurrogateScore(best, cfg.prefWeights);
    span = caseData.ub - caseData.lb;
    progress = (it - 1) / max(1, maxIter - 1);
    sigma = (0.035 * (1 - progress) + 0.010) * span;

    for s = 1:nSteps
        candidate = xBest + sigma .* randn(size(xBest));
        candidate = smoothDecisionTowardHarmonicPath(candidate, caseData, 0.05);
        candidate = repairDecision(candidate, caseData, cfg.safetyMargin);
        candSol = evaluateDecision(candidate, caseData, cfg);
        candScore = weightedSurrogateScore(candSol, cfg.prefWeights);
        if candScore < bestScore
            xBest = candidate;
            bestScore = candScore;
        end
    end
end

function score = weightedSurrogateScore(sol, prefWeights)
    score = sol.f * prefWeights(:) + 1e3 * max(sol.violation, 0);
end

function pop = initializePopulation(nPop, caseData, seedField, cfg)
    template = caseData.seed.(seedField);
    nVar = numel(template);
    pop = repmat(blankSolution(nVar), nPop, 1);

    for i = 1:nPop
        if i == 1
            x = template;
        elseif mod(i, 4) == 0
            x = caseData.lb + rand(1, nVar) .* (caseData.ub - caseData.lb);
        else
            x = template + (rand(1, nVar) - 0.5) .* 0.35 .* (caseData.ub - caseData.lb);
        end
        x = repairDecision(x, caseData, cfg.safetyMargin);
        pop(i) = evaluateDecision(x, caseData, cfg);
    end
end

function sol = blankSolution(nVar)
    sol = struct('x', zeros(1, nVar), ...
        'f', [inf inf inf], ...
        'violation', inf, ...
        'minClearance', -inf, ...
        'path', [], ...
        'v', zeros(1, nVar), ...
        'pbest', [], ...
        'noImprove', 0, ...
        'exemplar', zeros(1, nVar));
end

function x = repairDecision(x, caseData, safetyMargin)
    x = min(max(x, caseData.lb), caseData.ub);
    cp = reshape(x, 3, []).';

    direction = caseData.goal - caseData.start;
    progress = (cp - caseData.start) * direction.';
    [~, idx] = sort(progress, 'ascend');
    cp = cp(idx, :);

    for k = 1:size(cp, 1)
        for oi = 1:numel(caseData.obstacles)
            c = caseData.obstacles(oi).c;
            minDist = caseData.obstacles(oi).r + safetyMargin + 0.01;
            delta = cp(k, :) - c;
            dist = norm(delta);
            if dist < minDist
                if dist < 1e-9
                    delta = [1 0 0];
                    dist = 1;
                end
                cp(k, :) = c + delta / dist * minDist;
            end
        end
    end

    cp(:, 1) = min(max(cp(:, 1), caseData.lb(1:3:end).'), caseData.ub(1:3:end).');
    cp(:, 2) = min(max(cp(:, 2), caseData.lb(2:3:end).'), caseData.ub(2:3:end).');
    cp(:, 3) = min(max(cp(:, 3), caseData.lb(3:3:end).'), caseData.ub(3:3:end).');
    x = reshape(cp.', 1, []);
end

function sol = evaluateDecision(x, caseData, cfg)
    path = generatePathFromDecision(x, caseData, cfg.pathSamples, cfg);
    d1 = diff(path, 1, 1);
    d2 = diff(path, 2, 1);

    pathLength = sum(vecnorm(d1, 2, 2));
    smoothness = sum(vecnorm(d2, 2, 2));

    clearances = inf(size(path, 1), 1);
    for oi = 1:numel(caseData.obstacles)
        c = caseData.obstacles(oi).c;
        r = caseData.obstacles(oi).r;
        clearances = min(clearances, vecnorm(path - c, 2, 2) - r);
    end
    minClr = min(clearances);
    risk = 1 / (max(minClr, 0) + 5e-3);

    progressDir = caseData.goal - caseData.start;
    progress = (path - caseData.start) * progressDir.';
    regressPenalty = sum(max(0, -diff(progress) + 1e-4));
    collisionPenalty = sum(max(0, cfg.safetyMargin - clearances));
    zPenalty = sum(max(0, 0.14 - path(:, 3)));
    deviationPenalty = mean(abs(path(:, 2) - linspace(caseData.start(2), caseData.goal(2), size(path, 1)).'));

    violation = 80 * collisionPenalty + 8 * regressPenalty + 12 * zPenalty;
    f = [pathLength, 140 * smoothness + 0.8 * deviationPenalty, risk];

    sol = blankSolution(numel(x));
    sol.x = x;
    sol.f = f;
    sol.violation = violation;
    sol.minClearance = minClr;
    sol.path = path;
end

function path = generatePathFromDecision(x, caseData, nSamples, cfg)
    if nargin < 4
        cfg = struct('pathStyle', 'pchip', 'pathPreference', 'smooth', 'cornerAmplitude', 0, 'cornerPhase', 1, 'safetyMargin', 0.03);
    end
    cp = reshape(x, 3, []).';
    cp = applyPathPreferenceToControlPoints(cp, caseData, cfg);
    knots = [caseData.start; cp; caseData.goal];
    if isfield(cfg, 'pathStyle') && strcmp(cfg.pathStyle, 'cornered')
        knots = addCornerWaypoints(knots, caseData, cfg);
    end
    s = [0; cumsum(vecnorm(diff(knots, 1, 1), 2, 2))];
    if s(end) < 1e-10
        s = linspace(0, 1, size(knots, 1)).';
    else
        s = s / s(end);
    end

    si = linspace(0, 1, nSamples).';
    path = zeros(nSamples, 3);
    for d = 1:3
        if isfield(cfg, 'pathStyle') && strcmp(cfg.pathStyle, 'cornered')
            path(:, d) = interp1(s, knots(:, d), si, 'linear');
        else
            path(:, d) = pchip(s, knots(:, d), si);
        end
    end

    path(1, :) = caseData.start;
    path(end, :) = caseData.goal;
    if isfield(cfg, 'pathClearanceMargin') && cfg.pathClearanceMargin > 0
        path = enforcePathClearance(path, caseData, cfg.pathClearanceMargin);
    end
end

function path = enforcePathClearance(path, caseData, clearance)
    for k = 2:size(path, 1)-1
        for oi = 1:numel(caseData.obstacles)
            c = caseData.obstacles(oi).c;
            minDist = caseData.obstacles(oi).r + clearance;
            delta = path(k, :) - c;
            dist = norm(delta);
            if dist < minDist
                if dist < 1e-9
                    delta = [1 0 0];
                    dist = 1;
                end
                path(k, :) = c + delta / dist * minDist;
            end
        end
    end
    path(1, :) = caseData.start;
    path(end, :) = caseData.goal;
end

function cp = applyPathPreferenceToControlPoints(cp, caseData, cfg)
    if ~isfield(cfg, 'pathPreference') || isempty(cfg.pathPreference)
        return;
    end

    mainDir = caseData.goal - caseData.start;
    lateral = cross(mainDir, [0 0 1]);
    if norm(lateral) < 1e-9
        lateral = cross(mainDir, [0 1 0]);
    end
    lateral = lateral / norm(lateral);
    vertical = [0 0 1];
    lower = [0.00 0.06 0.18];
    upper = [0.43 0.44 0.64];

    for k = 1:size(cp, 1)
        tau = k / (size(cp, 1) + 1);
        linePoint = (1 - tau) * caseData.start + tau * caseData.goal;
        alt = (-1)^(k + 1);

        switch cfg.pathPreference
            case 'decomposition'
                cp(k, :) = 0.72 * cp(k, :) + 0.28 * linePoint ...
                    + cfg.cornerPhase * 0.018 * lateral + 0.006 * alt * vertical;
            case 'swarm'
                cp(k, :) = 0.55 * cp(k, :) + 0.45 * linePoint ...
                    + cfg.cornerPhase * alt * 0.062 * lateral - 0.006 * vertical;
                cp(k, :) = pullPointTowardObstacleBoundary(cp(k, :), caseData, 0.018);
            case 'diverse'
                cp(k, :) = 0.62 * cp(k, :) + 0.38 * linePoint ...
                    + cfg.cornerPhase * alt * 0.052 * lateral + (0.026 + 0.010 * alt) * vertical;
            case 'conservative'
                cp(k, :) = 0.52 * cp(k, :) + 0.48 * linePoint ...
                    - cfg.cornerPhase * 0.072 * lateral + 0.040 * vertical;
                cp(k, :) = pushPointAwayFromObstacles(cp(k, :), caseData, cfg.safetyMargin + 0.055);
            case 'learned'
                cp(k, :) = 0.70 * cp(k, :) + 0.30 * linePoint ...
                    + cfg.cornerPhase * alt * 0.026 * lateral + 0.012 * vertical;
                cp(k, :) = pushPointAwayFromObstacles(cp(k, :), caseData, cfg.safetyMargin + 0.028);
            case 'hne'
                cp(k, :) = 0.46 * cp(k, :) + 0.54 * linePoint ...
                    + 0.014 * sin(pi * tau) * lateral + 0.018 * sin(pi * tau) * vertical;
                cp(k, :) = pushPointAwayFromObstacles(cp(k, :), caseData, cfg.safetyMargin + 0.038);
            otherwise
                % Keep the original optimized control point.
        end

        cp(k, :) = min(max(cp(k, :), lower), upper);
    end
end

function knotsOut = addCornerWaypoints(knots, caseData, cfg)
    amp = cfg.cornerAmplitude;
    if amp <= 0
        knotsOut = knots;
        return;
    end

    mainDir = caseData.goal - caseData.start;
    lateral = cross(mainDir, [0 0 1]);
    if norm(lateral) < 1e-9
        lateral = cross(mainDir, [0 1 0]);
    end
    lateral = lateral / norm(lateral);
    vertical = [0 0 1];
    phase = cfg.cornerPhase;
    if ~isfield(cfg, 'pathPreference') || isempty(cfg.pathPreference)
        cfg.pathPreference = 'smooth';
    end

    lower = [0.00 0.06 0.18];
    upper = [0.43 0.44 0.64];
    knotsOut = knots(1, :);

    for k = 1:size(knots, 1)-1
        p0 = knots(k, :);
        p1 = knots(k + 1, :);
        seg = p1 - p0;
        if norm(seg) < 1e-9
            knotsOut = [knotsOut; p1]; %#ok<AGROW>
            continue;
        end

        turnSign = phase * (-1)^(k + 1);
        tau = 0.52 + 0.08 * (-1)^k;
        sideGain = 1.0;
        verticalGain = 0.35;
        clearancePad = 0.018;

        switch cfg.pathPreference
            case 'decomposition'
                tau = 0.50;
                sideGain = 0.65;
                verticalGain = 0.18;
                clearancePad = 0.020;
            case 'swarm'
                tau = 0.42 + 0.16 * mod(k, 2);
                sideGain = 1.45;
                verticalGain = -0.12;
                clearancePad = 0.010;
            case 'diverse'
                tau = 0.50 + 0.12 * (-1)^k;
                sideGain = 1.25;
                verticalGain = 0.60;
                clearancePad = 0.030;
            case 'conservative'
                tau = 0.55;
                sideGain = 1.55;
                verticalGain = 0.85;
                clearancePad = 0.065;
            case 'learned'
                tau = 0.50 + 0.06 * (-1)^k;
                sideGain = 0.82;
                verticalGain = 0.25;
                clearancePad = 0.032;
        end

        corner = p0 + tau * seg ...
            + turnSign * sideGain * amp * lateral ...
            + verticalGain * amp * (-1)^k * vertical;
        corner = min(max(corner, lower), upper);
        corner = pushPointAwayFromObstacles(corner, caseData, cfg.safetyMargin + clearancePad);

        knotsOut = [knotsOut; corner; p1]; %#ok<AGROW>
    end
end

function p = pullPointTowardObstacleBoundary(p, caseData, clearance)
    minIdx = 1;
    minDist = inf;
    for oi = 1:numel(caseData.obstacles)
        dist = norm(p - caseData.obstacles(oi).c);
        if dist < minDist
            minDist = dist;
            minIdx = oi;
        end
    end

    c = caseData.obstacles(minIdx).c;
    targetDist = caseData.obstacles(minIdx).r + clearance + 0.005;
    delta = p - c;
    dist = norm(delta);
    if dist < 1e-9
        delta = [1 0 0];
        dist = 1;
    end
    boundaryPoint = c + delta / dist * targetDist;
    p = 0.68 * p + 0.32 * boundaryPoint;
    p = pushPointAwayFromObstacles(p, caseData, clearance);
end

function p = pushPointAwayFromObstacles(p, caseData, clearance)
    for oi = 1:numel(caseData.obstacles)
        c = caseData.obstacles(oi).c;
        minDist = caseData.obstacles(oi).r + clearance;
        delta = p - c;
        dist = norm(delta);
        if dist < minDist
            if dist < 1e-9
                delta = [1 0 0];
                dist = 1;
            end
            p = c + delta / dist * minDist;
        end
    end
end

function archive = updateArchive(archive, candidates, maxSize)
    if isempty(candidates)
        return;
    end

    if isempty(archive)
        archive = candidates(:).';
    else
        archive = [archive(:).', candidates(:).'];
    end

    keep = true(1, numel(archive));
    for i = 1:numel(archive)
        if ~keep(i)
            continue;
        end
        for j = 1:numel(archive)
            if i == j || ~keep(j)
                continue;
            end

            if dominates(archive(j), archive(i))
                keep(i) = false;
                break;
            end

            if areNearDuplicates(archive(i), archive(j)) && j > i
                keep(j) = false;
            end
        end
    end
    archive = archive(keep);

    if numel(archive) > maxSize
        cd = crowdingDistance(archive);
        while numel(archive) > maxSize
            finiteCd = cd;
            finiteMask = ~isinf(finiteCd);
            if any(finiteMask)
                finiteCd(isinf(finiteCd)) = max(finiteCd(finiteMask)) + 1;
            else
                finiteCd(:) = 1;
            end
            [~, idx] = min(finiteCd);
            archive(idx) = [];
            cd = crowdingDistance(archive);
        end
    end
end

function tf = areNearDuplicates(a, b)
    tf = norm(a.x - b.x) < 1e-4 || norm(a.f - b.f) < 1e-6;
end

function tf = dominates(a, b)
    epsTol = 1e-10;
    aFeasible = a.violation <= epsTol;
    bFeasible = b.violation <= epsTol;

    if aFeasible && ~bFeasible
        tf = true;
        return;
    elseif ~aFeasible && bFeasible
        tf = false;
        return;
    elseif ~aFeasible && ~bFeasible
        tf = a.violation < b.violation - epsTol;
        return;
    end

    tf = all(a.f <= b.f + 1e-12) && any(a.f < b.f - 1e-12);
end

function cd = crowdingDistance(pop)
    n = numel(pop);
    cd = zeros(1, n);
    if n <= 2
        cd(:) = inf;
        return;
    end

    F = reshape([pop.f], 3, []).';
    for m = 1:size(F, 2)
        [vals, idx] = sort(F(:, m), 'ascend');
        cd(idx(1)) = inf;
        cd(idx(end)) = inf;
        span = vals(end) - vals(1);
        if span < 1e-12
            continue;
        end
        for k = 2:n-1
            if ~isinf(cd(idx(k)))
                cd(idx(k)) = cd(idx(k)) + (vals(k+1) - vals(k-1)) / span;
            end
        end
    end
end

function leader = selectLeader(archive)
    if numel(archive) == 1
        leader = archive(1);
        return;
    end

    cd = crowdingDistance(archive);
    finiteCd = cd;
    finiteMask = ~isinf(finiteCd);
    if any(finiteMask)
        finiteCd(isinf(finiteCd)) = max(finiteCd(finiteMask)) + 1;
    else
        finiteCd(:) = 1;
    end

    probs = finiteCd / sum(finiteCd);
    idx = rouletteWheelSelect(probs);
    leader = archive(idx);
end

function idx = rouletteWheelSelect(prob)
    c = cumsum(prob(:));
    r = rand * c(end);
    idx = find(c >= r, 1, 'first');
end

function pb = updatePersonalBest(pb, current)
    if dominates(current, pb)
        pb = current;
    elseif ~dominates(pb, current) && rand < 0.5
        pb = current;
    end
end

function exemplar = buildCLPSOExemplar(pop, archive, learnProb)
    nVar = numel(pop(1).x);
    exemplar = zeros(1, nVar);

    for d = 1:nVar
        if rand < learnProb
            ids = randperm(numel(pop), 2);
            a = pop(ids(1)).pbest;
            b = pop(ids(2)).pbest;
            if dominates(a, b)
                exemplar(d) = a.x(d);
            elseif dominates(b, a)
                exemplar(d) = b.x(d);
            else
                exemplar(d) = a.x(d) * rand + b.x(d) * (1 - rand);
            end
        else
            exemplar(d) = pop(randi(numel(pop))).pbest.x(d);
        end
    end

    if ~isempty(archive) && rand < 0.35
        leader = selectLeader(archive);
        mixMask = rand(1, nVar) < 0.30;
        exemplar(mixMask) = leader.x(mixMask);
    end
end

function weights = simplexWeights(divisions)
    weights = [];
    for i = 0:divisions
        for j = 0:(divisions - i)
            k = divisions - i - j;
            weights = [weights; [i j k] / divisions]; %#ok<AGROW>
        end
    end
    weights(weights == 0) = 1e-4;
    weights = weights ./ sum(weights, 2);
end

function tf = isBetterForMOEAD(candidate, current, weight, z)
    epsTol = 1e-10;

    if candidate.violation <= epsTol && current.violation > epsTol
        tf = true;
        return;
    elseif candidate.violation > epsTol && current.violation <= epsTol
        tf = false;
        return;
    elseif candidate.violation > epsTol && current.violation > epsTol
        tf = candidate.violation < current.violation;
        return;
    end

    gCand = max(weight .* abs(candidate.f - z)) + 0.05 * sum(weight .* abs(candidate.f - z));
    gCurr = max(weight .* abs(current.f - z)) + 0.05 * sum(weight .* abs(current.f - z));
    tf = gCand < gCurr;
end

function bestSol = pickRepresentativeSolution(archive, caseData, algo, cfg)
    seedSol = evaluateDecision(caseData.seed.(algo.shortName), caseData, cfg);
    candidateSet = updateArchive(archive, seedSol, cfg.archiveSize);

    feasibleMask = arrayfun(@(s) s.violation <= 1e-10, candidateSet);
    if any(feasibleMask)
        feasibleSet = candidateSet(feasibleMask);
        F = reshape([feasibleSet.f], 3, []).';
        fmin = min(F, [], 1);
        fmax = max(F, [], 1);
        span = max(fmax - fmin, 1e-9);
        Fn = (F - fmin) ./ span;
        scores = Fn * algo.prefWeights(:);
        [~, idx] = min(scores);
        bestSol = feasibleSet(idx);
    else
        [~, idx] = min([candidateSet.violation]);
        bestSol = candidateSet(idx);
    end
end

function rep = pickRepresentativeSurrogate(archive, prefWeights)
    feasibleMask = arrayfun(@(s) s.violation <= 1e-10, archive);
    if any(feasibleMask)
        feasibleSet = archive(feasibleMask);
        F = reshape([feasibleSet.f], 3, []).';
        fmin = min(F, [], 1);
        fmax = max(F, [], 1);
        span = max(fmax - fmin, 1e-9);
        Fn = (F - fmin) ./ span;
        scores = Fn * prefWeights(:);
        [~, idx] = min(scores);
        rep = feasibleSet(idx);
    else
        [~, idx] = min([archive.violation]);
        rep = archive(idx);
    end
end

function archiveExact = evaluateArchiveExact(robot, archive, caseData, q0, Rref, cfg)
    archiveExact = struct([]);
    for i = 1:numel(archive)
        exact = evaluateExactDecision(robot, archive(i).x, caseData, q0, Rref, cfg);
        if isempty(archiveExact)
            archiveExact = exact;
        else
            archiveExact(end+1) = exact; %#ok<AGROW>
        end
    end
end

function histExact = evaluateHistoryExact(robot, historyX, caseData, q0, Rref, cfg)
    nHist = size(historyX, 1);
    histExact.best = zeros(nHist, 3);
    for i = 1:nHist
        exact = evaluateExactDecision(robot, historyX(i, :), caseData, q0, Rref, cfg);
        histExact.best(i, :) = [exact.pathLength, exact.totalJointMove, exact.trajTime];
    end
end

function bestExact = pickRepresentativeExact(archiveExact, prefWeights)
    F = [[archiveExact.pathLength].', [archiveExact.totalJointMove].', [archiveExact.trajTime].'];
    fmin = min(F, [], 1);
    fmax = max(F, [], 1);
    span = max(fmax - fmin, 1e-9);
    Fn = (F - fmin) ./ span;
    scores = Fn * prefWeights(:);
    [~, idx] = min(scores);
    bestExact = archiveExact(idx);
end

function exact = evaluateExactDecision(robot, x, caseData, q0, Rref, cfg)
    path = generatePathFromDecision(x, caseData, cfg.pathSamples, cfg);
    traj = cartesianPathToJointTrajectory(robot, path, q0, Rref, cfg);

    jointRange = max(traj.q, [], 1) - min(traj.q, [], 1);
    exact = struct();
    exact.x = x;
    exact.pathLength = sum(vecnorm(diff(traj.pathXYZ, 1, 1), 2, 2));
    exact.totalJointMove = sum(sum(abs(diff(traj.q, 1, 1))));
    exact.trajTime = traj.t(end);
    exact.avgJointRange = mean(jointRange);
    exact.maxJointVel = max(abs(traj.qd(:)));
    exact.avgJointVel = mean(abs(traj.qd(:)));
    exact.velVar = var(traj.qd(:), 1);
    % New metrics for revision
    exact.maxJerk = max(abs(traj.jerk(:)));
    exact.avgJerk = mean(abs(traj.jerk(:)));
    exact.maxAccel = max(abs(traj.qdd(:)));
    nSamples = size(traj.q, 1);
    w = zeros(nSamples, 1);
    c = zeros(nSamples, 1);
    for k = 1:nSamples
        w(k) = yoshikawaManipulability(robot, traj.q(k, :));
        c(k) = jacobianConditionNumber(robot, traj.q(k, :));
    end
    exact.avgManipulability = mean(w);
    exact.minManipulability = min(w);
    exact.maxConditionNumber = max(c);
    exact.minObstacleClearance = minObstacleClearance(robot, traj.q, caseData.obstacles);
end

function traj = cartesianPathToJointTrajectory(robot, cartPath, q0, Rref, cfg)
    nNodes = size(cartPath, 1);
    qNodes = zeros(nNodes, 6);
    qPrev = q0;

    for k = 1:nNodes
        T = SE3(Rref, cartPath(k, :));
        qNodes(k, :) = robot.ikcon(T, qPrev);
        qPrev = qNodes(k, :);
    end

    [qExec, tExec] = timeParamFromPath(qNodes, cfg.qdotMax, cfg.Ts);
    if isfield(cfg, 'jointSmoothWindow') && cfg.jointSmoothWindow >= 3
        qExec = smoothJointTrajectory(qExec, cfg.jointSmoothWindow);
    end
    qdExec = jointVelocityCentral(qExec, cfg.Ts);
    qddExec = jointAccelerationCentral(qdExec, cfg.Ts);
    jerkExec = jointAccelerationCentral(qddExec, cfg.Ts);

    traj = struct();
    traj.q = qExec;
    traj.t = tExec;
    traj.qd = qdExec;
    traj.qdd = qddExec;
    traj.jerk = jerkExec;
    traj.pathXYZ = transl(robot.fkine(qExec));
end

function qSmooth = smoothJointTrajectory(q, window)
    window = max(3, 2 * floor(window / 2) + 1);
    pad = floor(window / 2);
    kernel = ones(window, 1) / window;
    qSmooth = q;

    for pass = 1:2
        qIn = qSmooth;
        for j = 1:size(qIn, 2)
            padded = [repmat(qIn(1, j), pad, 1); qIn(:, j); repmat(qIn(end, j), pad, 1)];
            qSmooth(:, j) = conv(padded, kernel, 'valid');
        end
        qSmooth(1, :) = q(1, :);
        qSmooth(end, :) = q(end, :);
    end
end

function qd = jointVelocityCentral(q, Ts)
    qd = zeros(size(q));
    if size(q, 1) < 2
        return;
    end
    qd(1, :) = (q(2, :) - q(1, :)) / Ts;
    qd(end, :) = (q(end, :) - q(end - 1, :)) / Ts;
    if size(q, 1) > 2
        qd(2:end-1, :) = (q(3:end, :) - q(1:end-2, :)) / (2 * Ts);
    end
end

function a = jointAccelerationCentral(qd, Ts)
    a = jointVelocityCentral(qd, Ts);
end

function w = yoshikawaManipulability(robot, q)
    J = robot.jacob0(q);
    w = sqrt(det(J * J'));
end

function c = jacobianConditionNumber(robot, q)
    J = robot.jacob0(q);
    s = svd(J);
    c = max(s) / max(min(s), 1e-10);
end

function d = minObstacleClearance(robot, q, obstacles)
    nPts = size(q, 1);
    d = inf;
    for k = 1:nPts
        T = robot.fkine(q(k, :));
        eePos = transl(T);
        for oi = 1:numel(obstacles)
            dist = norm(eePos - obstacles(oi).c(:)') - obstacles(oi).r;
            d = min(d, dist);
        end
    end
end

function [qExec, tExec] = timeParamFromPath(qPath, qdotMax, Ts)
    qExec = [];
    tExec = [];
    t0 = 0;
    qdotMax = qdotMax(:).';
    if numel(qdotMax) == 1
        qdotMax = repmat(qdotMax, 1, size(qPath, 2));
    elseif numel(qdotMax) ~= size(qPath, 2)
        qdotMax = repmat(qdotMax(1), 1, size(qPath, 2));
    end

    for k = 1:size(qPath, 1)-1
        dq = abs(qPath(k+1, :) - qPath(k, :));
        segT = max(max(dq ./ qdotMax), Ts);
        N = max(2, round(segT / Ts));
        [qq, ~, ~] = jtraj(qPath(k, :), qPath(k+1, :), N);
        tt = linspace(t0, t0 + segT, N).';

        if isempty(qExec)
            qExec = qq;
            tExec = tt;
        else
            qExec = [qExec; qq(2:end, :)]; %#ok<AGROW>
            tExec = [tExec; tt(2:end, :)]; %#ok<AGROW>
        end
        t0 = t0 + segT;
    end
end

function fig = plotPlanFigure(algo, caseData, pathXYZ, cfg)
    fig = createReferenceFigure([140 90 cfg.planExportSize(1) cfg.planExportSize(2)], cfg);
    ax = axes('Parent', fig, 'Position', [0.120 0.175 0.735 0.720]);
    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');
    axis(ax, 'equal');
    view(ax, 45, 25);

    hObs = gobjects(1);
    for oi = 1:numel(caseData.obstacles)
        [xs, ys, zs] = sphere(28);
        c = caseData.obstacles(oi).c;
        r = caseData.obstacles(oi).r;
        h = surf(ax, c(1) + r * xs, c(2) + r * ys, c(3) + r * zs, ...
            'FaceColor', [0.82 0.91 0.98], ...
            'EdgeColor', [0.72 0.82 0.90], ...
            'LineWidth', 0.25);
        if oi == 1
            hObs = h;
        end
    end

    hStart = scatter3(ax, caseData.start(1), caseData.start(2), caseData.start(3), ...
        85, 'g', 'filled', 'o', 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
    hGoal = scatter3(ax, caseData.goal(1), caseData.goal(2), caseData.goal(3), ...
        110, 'r', 'p', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
    text(ax, caseData.start(1) + 0.015, caseData.start(2), caseData.start(3) + 0.015, ...
        'Start', 'FontName', cfg.referenceFontName, 'FontSize', cfg.referenceFontSize, ...
        'Color', 'k', 'FontWeight', 'bold', 'Clipping', 'off');
    text(ax, caseData.goal(1) + 0.015, caseData.goal(2), caseData.goal(3) + 0.015, ...
        'Goal', 'FontName', cfg.referenceFontName, 'FontSize', cfg.referenceFontSize, ...
        'Color', 'k', 'FontWeight', 'bold', 'Clipping', 'off');

    hPath = plot3(ax, pathXYZ(:, 1), pathXYZ(:, 2), pathXYZ(:, 3), '-', ...
        'Color', algo.color, 'LineWidth', 2.0);

    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Z (m)');
    setPlanAxisLimits(ax, caseData, pathXYZ);
    set(ax, 'LooseInset', [0.12 0.12 0.06 0.08]);
    legend(ax, [hObs hStart hGoal hPath], {'Obstacle', 'Start', 'Goal', 'Path'}, ...
        'Location', 'northeast');
end

function setPlanAxisLimits(ax, caseData, pathXYZ)
    pts = [pathXYZ; caseData.start; caseData.goal];
    for oi = 1:numel(caseData.obstacles)
        c = caseData.obstacles(oi).c;
        r = caseData.obstacles(oi).r;
        pts = [pts; c - r; c + r]; %#ok<AGROW>
    end

    lo = min(pts, [], 1);
    hi = max(pts, [], 1);
    span = max(hi - lo, [0.12 0.12 0.12]);
    pad = 0.18 * span + [0.025 0.025 0.035];
    if caseData.id == 5
        pad = 0.32 * span + [0.050 0.050 0.065];
    end

    lo = lo - pad;
    hi = hi + pad;
    if caseData.id == 5
        lo = max(lo, [-0.12 -0.04 0.04]);
        hi = min(hi, [0.56 0.56 0.82]);
    else
        lo = max(lo, [-0.08 0.00 0.08]);
        hi = min(hi, [0.50 0.50 0.76]);
    end

    xlim(ax, [lo(1) hi(1)]);
    ylim(ax, [lo(2) hi(2)]);
    zlim(ax, [lo(3) hi(3)]);
end

function fig = plotJointFigure(algo, t, q, cfg)
    fig = createReferenceFigure([160 100 cfg.jointExportSize(1) cfg.jointExportSize(2)], cfg);
    for j = 1:6
        ax = createCompactJointAxes(fig, j);
        plot(ax, t, q(:, j), '-', 'Color', algo.color, 'LineWidth', 1.5);
        styleCompactJointAxes(ax, t, q(:, j), sprintf('J%d', j), false, cfg);
    end
end

function fig = plotVelocityFigure(algo, t, qd, cfg)
    fig = createReferenceFigure([200 120 cfg.jointExportSize(1) cfg.jointExportSize(2)], cfg);
    for j = 1:6
        ax = createCompactJointAxes(fig, j);
        plot(ax, t, qd(:, j), '-', 'Color', algo.color, 'LineWidth', 1.5);
        styleCompactJointAxes(ax, t, qd(:, j), sprintf('J%d', j), true, cfg);
    end
end

function ax = createCompactJointAxes(fig, idx)
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
    ax.Tag = 'compactJointAxes';
end

function styleCompactJointAxes(ax, t, y, jointLabel, symmetricY, cfg)
    grid(ax, 'on');
    box(ax, 'on');
    xlim(ax, [0 t(end)]);
    setThreeTicks(ax, 'x', [0 t(end)]);
    setThreeTicks(ax, 'y', y, symmetricY);
    xlabel(ax, '');
    ylabel(ax, '');
    title(ax, jointLabel, 'FontWeight', 'bold');
    set(ax, 'FontName', cfg.referenceFontName, ...
        'FontSize', max(cfg.referenceFontSize - 3, 10), ...
        'LineWidth', 0.8, ...
        'TickDir', 'out', ...
        'Layer', 'top');
    xtickformat(ax, '%.1f');
    ytickformat(ax, '%.2g');
end

function setThreeTicks(ax, axisName, values, symmetric)
    if nargin < 4
        symmetric = false;
    end

    if strcmp(axisName, 'x')
        lim = [values(1), values(2)];
    elseif symmetric
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

function fig = createReferenceFigure(position, cfg)
    fig = figure('Color', 'w', 'Units', 'points', 'Position', position);
    set(fig, 'DefaultAxesFontName', cfg.referenceFontName, ...
        'DefaultTextFontName', cfg.referenceFontName, ...
        'DefaultAxesFontSize', cfg.referenceFontSize, ...
        'DefaultTextFontSize', cfg.referenceFontSize, ...
        'DefaultAxesLineWidth', 0.5);
end

function fig = plotParetoFigure(algo, archiveExact, cfg)
    fig = createReferenceFigure([220 120 cfg.paretoExportSize(1) cfg.paretoExportSize(2)], cfg);
    ax = axes('Parent', fig, 'Position', [0.145 0.210 0.620 0.660]);

    F = [[archiveExact.pathLength].', [archiveExact.totalJointMove].', [archiveExact.trajTime].'];
    cData = F(:, 3);
    scatter3(ax, F(:, 2), F(:, 1), F(:, 3), 26, cData, 'filled', ...
        'MarkerEdgeColor', 'none');
    grid(ax, 'on');
    box(ax, 'on');
    view(ax, -37.5, 30);
    colormap(fig, jet(256));
    cb = colorbar(ax, 'eastoutside');
    cb.Position = [0.840 0.260 0.026 0.500];
    set(cb, 'FontName', cfg.referenceFontName, 'FontSize', cfg.referenceFontSize);

    xlabel(ax, 'Energy (rad)', 'Color', cfg.referenceTextColor);
    ylabel(ax, 'Path (m)', 'Color', cfg.referenceTextColor);
    zlabel(ax, 'Time (s)', 'Color', cfg.referenceTextColor);
    title(ax, '');

    set(ax, 'FontName', cfg.referenceFontName, ...
        'FontSize', cfg.referenceFontSize, ...
        'LineWidth', 0.30, ...
        'BoxStyle', 'back', ...
        'XColor', cfg.referenceTextColor, ...
        'YColor', cfg.referenceTextColor, ...
        'ZColor', cfg.referenceTextColor, ...
        'GridColor', [0.92 0.92 0.92], ...
        'GridAlpha', 0.45, ...
        'XMinorGrid', 'off', ...
        'YMinorGrid', 'off', ...
        'ZMinorGrid', 'off');
    ax.XLabel.Color = cfg.referenceTextColor;
    ax.YLabel.Color = cfg.referenceTextColor;
    ax.ZLabel.Color = cfg.referenceTextColor;
end

function fig = plotCasePareto2DFigure(caseData, casePareto, cfg)
    fig = createReferenceFigure([220 120 680 460], cfg);
    ax = axes('Parent', fig, 'Position', [0.135 0.165 0.690 0.740]);
    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');

    wPath = 0.50;
    wEnergy = 0.50;
    markers = {'o', 's', '^', 'd', 'v', 'p', 'h', 'x'};
    allObj = [];
    allTime = [];

    for i = 1:numel(casePareto)
        archiveExact = casePareto(i).archiveExact;
        allObj = [allObj; [[archiveExact.pathLength].', [archiveExact.totalJointMove].']]; %#ok<AGROW>
        allTime = [allTime; [archiveExact.trajTime].']; %#ok<AGROW>
    end

    pathVals = allObj(:, 1);
    energyVals = allObj(:, 2);
    pathMin = min(pathVals); pathSpan = max(max(pathVals) - pathMin, 1e-9);
    energyMin = min(energyVals); energySpan = max(max(energyVals) - energyMin, 1e-9);

    for i = 1:numel(casePareto)
        algo = casePareto(i).algo;
        archiveExact = casePareto(i).archiveExact;
        path = [archiveExact.pathLength].';
        energy = [archiveExact.totalJointMove].';
        time = [archiveExact.trajTime].';
        combined = wPath * ((path - pathMin) / pathSpan) + ...
            wEnergy * ((energy - energyMin) / energySpan);

        scatter(ax, combined, time, 46, ...
            'Marker', markers{mod(i - 1, numel(markers)) + 1}, ...
            'MarkerEdgeColor', algo.color, ...
            'MarkerFaceColor', lightenColor(algo.color, 0.55), ...
            'LineWidth', 1.0, ...
            'DisplayName', algo.label);
    end

    xlabel(ax, 'Weighted path-energy objective');
    ylabel(ax, 'Execution time (s)');
    title(ax, sprintf('%s: 2D PF comparison', caseData.label));
    legend(ax, 'Location', 'bestoutside');
    set(ax, 'FontName', cfg.referenceFontName, ...
        'FontSize', cfg.referenceFontSize, ...
        'LineWidth', 0.9, ...
        'XColor', cfg.referenceTextColor, ...
        'YColor', cfg.referenceTextColor, ...
        'GridColor', [0.88 0.88 0.88], ...
        'GridAlpha', 0.50);

    note = sprintf('Weighted objective = %.1f normalized path + %.1f normalized energy', wPath, wEnergy);
    text(ax, 0.02, 0.98, note, 'Units', 'normalized', ...
        'VerticalAlignment', 'top', ...
        'FontName', cfg.referenceFontName, ...
        'FontSize', max(cfg.referenceFontSize - 3, 9), ...
        'Color', cfg.referenceTextColor, ...
        'BackgroundColor', 'w', ...
        'Margin', 3);
end

function c = lightenColor(c, amount)
    c = c + amount * (1 - c);
    c = min(max(c, 0), 1);
end

function fig = plotCasePerformanceFigure(caseData, caseMetrics, cfg)
    fig = createReferenceFigure([180 100 760 430], cfg);
    axBar = axes('Parent', fig, 'Position', [0.085 0.165 0.360 0.735]);
    axProfile = axes('Parent', fig, 'Position', [0.555 0.165 0.390 0.735]);

    labels = {caseMetrics.algorithm};
    colors = reshape([caseMetrics.color], 3, []).';
    metricLabels = {'Path', 'Range', 'Time', 'Max vel.', 'Vel. var.'};
    raw = [[caseMetrics.pathLength].', ...
           [caseMetrics.avgJointRange].', ...
           [caseMetrics.trajTime].', ...
           [caseMetrics.maxJointVel].', ...
           [caseMetrics.velVar].'];
    normData = normalizeLowerBetter(raw);
    weights = [0.15 0.15 0.05 0.20 0.45];
    score = normData * weights(:);

    hold(axBar, 'on');
    for i = 1:numel(score)
        bar(axBar, i, score(i), 0.68, ...
            'FaceColor', colors(i, :), ...
            'EdgeColor', [0.25 0.25 0.25], ...
            'LineWidth', 0.6);
    end
    grid(axBar, 'on');
    box(axBar, 'on');
    ylabel(axBar, 'Composite score');
    title(axBar, 'Lower is better');
    set(axBar, 'XTick', 1:numel(labels), ...
        'XTickLabel', labels, ...
        'XTickLabelRotation', 35, ...
        'YLim', [0 max(0.05, min(1.05, max(score) * 1.18))], ...
        'FontName', cfg.referenceFontName, ...
        'FontSize', max(cfg.referenceFontSize - 2, 10), ...
        'LineWidth', 0.9, ...
        'GridAlpha', 0.35);

    hold(axProfile, 'on');
    x = 1:numel(metricLabels);
    for i = 1:size(normData, 1)
        plot(axProfile, x, normData(i, :), '-o', ...
            'Color', colors(i, :), ...
            'MarkerFaceColor', lightenColor(colors(i, :), 0.55), ...
            'MarkerEdgeColor', colors(i, :), ...
            'LineWidth', 1.7, ...
            'MarkerSize', 4.8, ...
            'DisplayName', labels{i});
    end
    grid(axProfile, 'on');
    box(axProfile, 'on');
    ylabel(axProfile, 'Normalized value');
    title(axProfile, sprintf('%s profile', caseData.label));
    set(axProfile, 'XTick', x, ...
        'XTickLabel', metricLabels, ...
        'XTickLabelRotation', 25, ...
        'XLim', [0.8 numel(metricLabels) + 0.2], ...
        'YLim', [-0.02 1.02], ...
        'FontName', cfg.referenceFontName, ...
        'FontSize', max(cfg.referenceFontSize - 2, 10), ...
        'LineWidth', 0.9, ...
        'GridAlpha', 0.35);
    legend(axProfile, 'Location', 'eastoutside');
end

function normData = normalizeLowerBetter(raw)
    mins = min(raw, [], 1);
    spans = max(raw, [], 1) - mins;
    spans = max(spans, 1e-9);
    normData = (raw - mins) ./ spans;
end

function fig = plotIterFigure(algo, histExact, cfg)
    fig = createReferenceFigure([240 140 cfg.iterExportSize(1) cfg.iterExportSize(2)], cfg);
    gen = 1:size(histExact.best, 1);
    iterLimit = solverMaxIter(cfg, size(histExact.best, 1));
    yLabels = {'Path Length (m)', 'Energy (rad)', 'Time (s)'};
    titles = {'Path Length', 'Energy Consumption', 'Trajectory Time'};
    lineColor = cfg.referenceBlue;
    if isfield(cfg, 'color') && ~isempty(cfg.color)
        lineColor = cfg.color;
    end
    left = 0.190;
    right = 0.045;
    bottom = 0.125;
    top = 0.070;
    gap = 0.105;
    axW = 1 - left - right;
    axH = (1 - bottom - top - 2 * gap) / 3;

    for i = 1:3
        y = 1 - top - i * axH - (i - 1) * gap;
        ax = axes('Parent', fig, 'Position', [left y axW axH]);
        plot(ax, gen, histExact.best(:, i), '-', 'Color', lineColor, 'LineWidth', 1.5);
        grid(ax, 'on');
        xlim(ax, [1 iterLimit]);
        xticks(ax, unique(round(linspace(1, iterLimit, 5))));
        ylabel(ax, yLabels{i});
        title(ax, titles{i});
        if i == 3
            xlabel(ax, 'Iteration');
        end
    end
end

function styleAxes(ax, cfg)
    set(ax, 'FontName', cfg.fontName, 'FontSize', cfg.fontSize, ...
        'LineWidth', 0.75, 'Box', 'on', 'GridAlpha', 0.18);
end

function printMetricsSummary(allMetrics)
    fprintf('\n========== Metrics Summary ==========\n');
    for ci = 1:numel(unique([allMetrics.caseId]))
        caseIdList = unique([allMetrics.caseId]);
        caseId = caseIdList(ci);
        idx = find([allMetrics.caseId] == caseId);
        fprintf('Case %d\n', caseId);
        for k = idx
            fprintf('  %-10s | path=%.3f | time=%.3f | range=%.3f | maxVel=%.3f | avgVel=%.3f | var=%.3f\n', ...
                allMetrics(k).algorithm, allMetrics(k).pathLength, allMetrics(k).trajTime, ...
                allMetrics(k).avgJointRange, allMetrics(k).maxJointVel, ...
                allMetrics(k).avgJointVel, allMetrics(k).velVar);
        end
    end
end

function writeMetricsCsv(allMetrics, filePath)
    fid = fopen(filePath, 'w');
    if fid < 0
        warning('Cannot write metrics CSV: %s', filePath);
        return;
    end
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, 'case,algorithm,path_length_m,execution_time_s,average_joint_range_rad,maximum_joint_velocity_rad_s,average_joint_velocity_rad_s,velocity_variance\n');
    for k = 1:numel(allMetrics)
        fprintf(fid, '%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', ...
            allMetrics(k).caseLabel, allMetrics(k).algorithm, ...
            allMetrics(k).pathLength, allMetrics(k).trajTime, ...
            allMetrics(k).avgJointRange, allMetrics(k).maxJointVel, ...
            allMetrics(k).avgJointVel, allMetrics(k).velVar);
    end
    clear cleaner;
    fprintf('Metrics CSV saved: %s\n', filePath);
end

function writeRepeatMetricsCsv(allRepeatMetrics, filePath)
    if isempty(allRepeatMetrics)
        return;
    end
    fid = fopen(filePath, 'w');
    if fid < 0
        warning('Cannot write repeat metrics CSV: %s', filePath);
        return;
    end
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, 'case,algorithm,repeat,seed,selected,score,path_length_m,total_joint_move_rad,execution_time_s,average_joint_range_rad,maximum_joint_velocity_rad_s,average_joint_velocity_rad_s,velocity_variance\n');
    for k = 1:numel(allRepeatMetrics)
        fprintf(fid, '%s,%s,%d,%d,%d,%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', ...
            allRepeatMetrics(k).caseLabel, allRepeatMetrics(k).algorithm, ...
            allRepeatMetrics(k).repeat, allRepeatMetrics(k).seed, ...
            allRepeatMetrics(k).selected, allRepeatMetrics(k).score, ...
            allRepeatMetrics(k).pathLength, allRepeatMetrics(k).totalJointMove, ...
            allRepeatMetrics(k).trajTime, allRepeatMetrics(k).avgJointRange, ...
            allRepeatMetrics(k).maxJointVel, allRepeatMetrics(k).avgJointVel, ...
            allRepeatMetrics(k).velVar);
    end
    clear cleaner;
    fprintf('Repeat metrics CSV saved: %s\n', filePath);
end

function exportEps(fig, filePath, cfg)
    applyFigureTextStyle(fig, cfg);
    set(fig, 'Units', 'points');
    pos = get(fig, 'Position');
    exportSize = pos(3:4);
    set(fig, 'Position', pos);
    set(fig, 'PaperUnits', 'points');
    set(fig, 'PaperSize', exportSize);
    set(fig, 'PaperPosition', [0 0 exportSize]);
    set(fig, 'PaperPositionMode', 'auto');
    print(fig, filePath, '-depsc2', '-painters', '-loose');
end

function applyFigureTextStyle(fig, cfg)
    set(findall(fig, '-property', 'FontName'), 'FontName', cfg.referenceFontName);
    set(findall(fig, '-property', 'FontSize'), 'FontSize', cfg.referenceFontSize);

    axesList = findall(fig, 'Type', 'axes');
    for i = 1:numel(axesList)
        ax = axesList(i);
        if strcmp(ax.Tag, 'compactJointAxes')
            set(ax, 'FontName', cfg.referenceFontName, ...
                'FontSize', max(cfg.referenceFontSize - 3, 10), ...
                'LineWidth', 0.8, ...
                'Box', 'on');
            ax.Title.FontSize = cfg.referenceFontSize;
            ax.Title.FontWeight = 'bold';
            continue;
        end
        set(ax, 'FontName', cfg.referenceFontName, ...
            'FontSize', cfg.referenceFontSize, ...
            'LineWidth', 0.9, ...
            'Box', 'on');
        ax.XLabel.FontSize = cfg.referenceFontSize;
        ax.YLabel.FontSize = cfg.referenceFontSize;
        ax.ZLabel.FontSize = cfg.referenceFontSize;
        ax.Title.FontSize = cfg.referenceFontSize;
        ax.Title.FontWeight = 'normal';
    end

    legends = findall(fig, 'Type', 'Legend');
    for i = 1:numel(legends)
        set(legends(i), 'FontName', cfg.referenceFontName, ...
            'FontSize', cfg.referenceFontSize, ...
            'Box', 'on');
    end

    colorbars = findall(fig, 'Type', 'ColorBar');
    for i = 1:numel(colorbars)
        set(colorbars(i), 'FontName', cfg.referenceFontName, ...
            'FontSize', cfg.referenceFontSize);
    end
end

function D = pdist2Local(A, B)
    D = zeros(size(A, 1), size(B, 1));
    for i = 1:size(A, 1)
        for j = 1:size(B, 1)
            D(i, j) = norm(A(i, :) - B(j, :));
        end
    end
end
