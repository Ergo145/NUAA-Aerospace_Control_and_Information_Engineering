% 清除环境
clc; clear; close all;

%% 1. 定义系统
s = tf('s');

% 原系统传递函数
G0 = 30 / (s * (0.1*s + 1) * (0.2*s + 1));

% 指定的滞后校正装置
% Gc = (3.65s + 1) / (33.18s + 1)
Gc = (3.65*s + 1) / (33.18*s + 1);

% 校正后的开环传递函数
G_corr = Gc * G0;

%% 2. 计算截止频率、相角裕度和幅值裕度
[Gm, Pm, Wcg, Wcp] = margin(G_corr);
Gm_dB = 20 * log10(Gm); % 线性值转分贝

%% 3. 输出结果到命令行
fprintf('==================================================\n');
fprintf('已知校正装置: Gc(s) = (3.65s + 1) / (33.18s + 1)\n');
fprintf('==================================================\n');
fprintf('校正后系统指标计算结果:\n');
fprintf('--------------------------------------------------\n');
fprintf('1. 截止频率 (Wc):       %.4f rad/s\n', Wcp);
fprintf('2. 相角裕度 (PM):        %.4f deg\n', Pm);
fprintf('3. 幅值裕度 (GM):        %.4f dB\n', Gm_dB);
fprintf('--------------------------------------------------\n');
fprintf('其他参考指标:\n');
fprintf('4. 相角交界频率 (Wcg):   %.4f rad/s\n', Wcg);
fprintf('5. 速度误差系数 (Kv):    30 (保持不变)\n');
fprintf('==================================================\n');

%% 4. 绘制伯德图
% 图1: 直接使用 margin 函数绘制校正后的带标注伯德图
figure(1);
margin(G_corr);
title('校正后系统的伯德图 (G_{corr} = G_c * G_0)');
grid on;
% 优化线条粗细
set(findall(gcf,'type','line'), 'LineWidth', 1.5);

% 图2: (可选) 绘制校正前后的对比图，方便观察效果
figure(2);
bode(G0, 'b--', G_corr, 'r-');
grid on;
legend('校正前 (G0)', '校正后 (Gcorr)');
title('校正前后频率特性对比');
% 优化线条粗细
set(findall(gcf,'type','line'), 'LineWidth', 1.5);