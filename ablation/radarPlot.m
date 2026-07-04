function radarPlot(data,labels,legendNames)
% ===============================
% SCI 风格雷达图（完全兼容版本）
% ===============================

numAlg = size(data,1);
numDim = size(data,2);

% ---- 归一化（雷达图必须，否则尺度失真）----
data = data ./ max(data,[],1);

theta = linspace(0,2*pi,numDim+1);

% ===== 创建 polaraxes（关键修复）=====
pax = polaraxes;
hold(pax,'on');

% ---- 绘制 ----
for i = 1:numAlg
    rho = [data(i,:) data(i,1)];
    polarplot(pax, theta, rho, '-o', ...
        'LineWidth', 2, ...
        'MarkerSize', 6);
end

% ---- 标签 ----
thetaticks(rad2deg(theta(1:end-1)));
thetaticklabels(labels);

% ---- 美化 ----
pax.FontSize = 16;
pax.RLim = [0 1];
pax.GridAlpha = 0.3;

legend(legendNames, 'Location','southoutside', 'Orientation','horizontal', 'FontSize', 16);

title('Ablation Performance Radar Chart');

end
