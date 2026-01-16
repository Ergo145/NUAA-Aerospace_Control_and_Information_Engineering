% 清除工作区
clc; clear; close all;

%% 1. 数据准备
T_raw = readmatrix('data.xlsx');

w = T_raw(1,3:end);           % 频率 w
L_theo = T_raw(4,3:end);      % 理论幅值 L
Phi_theo = T_raw(5,3:end);    % 理论相位 Phi 
L_exp = T_raw(8,3:end);       % 实验幅值 L' 
Phi_exp = T_raw(10,3:end);    % 实验相位 Phi'

%% 2. 绘图 (Bode 图)
figure('Color', 'w', 'Name', '系统对数频率特性曲线对比');

% --- 第一个子图：幅频特性 (L - w) ---
subplot(2, 1, 1);
semilogx(w, L_theo, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 6); hold on;
semilogx(w, L_exp, 'r--s', 'LineWidth', 1.5, 'MarkerSize', 6);
grid on;
title('对数幅频特性曲线 (Logarithmic Magnitude)');
ylabel('幅值 L(\omega) / dB');
legend('理论值', '实验值', 'Location', 'Best');
xlim([min(w)*0.8, max(w)*1.2]); 

% --- 第二个子图：相频特性 (Phi - w) ---
subplot(2, 1, 2);
semilogx(w, Phi_theo, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 6); hold on;
semilogx(w, Phi_exp, 'r--s', 'LineWidth', 1.5, 'MarkerSize', 6);
grid on;
title('对数相频特性曲线 (Phase)');
xlabel('频率 \omega (rad/s)');
ylabel('相位 \phi(\omega) / 度');
legend('理论值', '实验值', 'Location', 'Best');
xlim([min(w)*0.8, max(w)*1.2]);

set(findall(gcf,'-property','FontSize'),'FontSize',12);

%% 3. 计算实验值带宽频率 (-3dB)
% 定义参考增益：通常取低频第一个点 (L_exp(1)) 或最大值 (max(L_exp))
% 这里假设为低通系统，以低频起始值为基准
L_ref = L_exp(1); 
L_target = L_ref - 3; % 目标幅值 (-3dB点)

% 寻找数据中幅值首次下降并穿过 L_target 的位置
% find 返回的是第一个满足条件的数据点索引
idx_below = find(L_exp < L_target, 1);

if isempty(idx_below)
    fprintf('错误：在当前频率范围内，实验幅值未下降到 -3dB (目标值 %.2fdB)。\n', L_target);
elseif idx_below == 1
    fprintf('错误：起始幅值已经低于 -3dB 目标值，无法计算带宽。\n');
else
    % 获取穿越点前后的两个数据点：(w1, L1) 为 -3dB 之上，(w2, L2) 为 -3dB 之下
    idx_above = idx_below - 1;
    
    w1 = w(idx_above);
    L1 = L_exp(idx_above);
    w2 = w(idx_below);
    L2 = L_exp(idx_below);
    
    % 进行插值计算
    % 原理：在半对数坐标系 (log10(w), L) 中，两点连线是一条直线
    % 公式：(L_target - L1) / (L2 - L1) = (log10(w_bw) - log10(w1)) / (log10(w2) - log10(w1))
    
    % 计算插值比例 ratio
    ratio = (L_target - L1) / (L2 - L1);
    
    % 计算带宽频率的对数值
    log_w_bw = log10(w1) + ratio * (log10(w2) - log10(w1));
    
    % 还原为实际频率 w_bw
    w_bw = 10^log_w_bw;
    
    % --- 输出结果 ---
    fprintf('------------------------------------\n');
    fprintf('参考幅值 (0dB基准): %.4f dB\n', L_ref);
    fprintf('目标幅值 (-3dB点):  %.4f dB\n', L_target);
    fprintf('计算得到的实验带宽频率: %.4f rad/s\n', w_bw);
    fprintf('------------------------------------\n');
    
    % --- 在图中标记结果 (可选) ---
    subplot(2, 1, 1);
    hold on;
    % 画出 -3dB 辅助线
    yline(L_target, 'g--', 'LineWidth', 1);
    % 标记计算出的带宽点
    plot(w_bw, L_target, 'mp', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    text(w_bw, L_target, sprintf('  \\omega_b = %.2f', w_bw), ...
        'Color', 'm', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
end
