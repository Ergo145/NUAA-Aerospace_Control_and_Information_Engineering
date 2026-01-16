% 清除工作区
clc; clear; close all;

%% 1. 系统定义
s = tf('s');

% (1) 校正前系统 (Uncompensated)
% G0(s) = 30 / [s(0.1s+1)(0.2s+1)]
G0 = 30 / (s * (0.1*s + 1) * (0.2*s + 1));

% (2) 滞后校正装置 (Compensator)
% Gc(s) = (3.65s + 1) / (33.18s + 1)
Gc = (3.65*s + 1) / (33.18*s + 1);

% (3) 校正后系统 (Compensated)
G_corr = Gc * G0;

%% 2. 计算裕度参数 (用于标注)
[Gm0, Pm0, Wcg0, Wcp0] = margin(G0);       % 校正前
[Gm_c, Pm_c, Wcg_c, Wcp_c] = margin(G_corr); % 校正后

%% 3. 绘制伯德图对比
figure('Name', 'Bode Plot Comparison', 'Color', 'w');

% 使用 bode 函数同时绘制两个系统
% 蓝色虚线 = 校正前，红色实线 = 校正后
bode(G0, 'b--', G_corr, 'r-');
grid on;

% 4. 图表美化与标注
hLines = findall(gcf, 'type', 'line'); 
set(hLines, 'LineWidth', 1.5); % 加粗线条

legend('校正前 G_0(s)', '校正后 G_{corr}(s)');
title('校正前后开环频率特性曲线对比');

% 在图上添加关键数据文本框
annotation('textbox', [0.15, 0.15, 0.3, 0.2], 'String', ...
    {sprintf('【校正前】'), ...
     sprintf('截止频率: %.2f rad/s', Wcp0), ...
     sprintf('相角裕度: %.2f^o', Pm0), ...
     '', ...
     sprintf('【校正后】'), ...
     sprintf('截止频率: %.2f rad/s', Wcp_c), ...
     sprintf('相角裕度: %.2f^o', Pm_c)}, ...
    'FitBoxToText', 'on', 'BackgroundColor', 'white', 'EdgeColor', 'black');

% 提示：查看幅频特性变化
fprintf('绘图完成。\n');
fprintf('请观察幅频特性曲线(上图)：校正后的红色曲线在中高频段明显低于蓝色曲线，\n');
fprintf('这体现了滞后校正的“高频衰减”特性，从而将截止频率左移。\n');