% % 清除环境
% clc; clear; close all;
% 
% % 1. 定义传递函数
% % 方法一：使用 zpk (零极点增益模型) 或 tf (传递函数模型)
% s = tf('s'); % 定义 s 变量
% K = 30;
% 
% % 开环传递函数 G0(s)
% G0 = K / (s * (0.1*s + 1) * (0.2*s + 1));
% 
% % 2. 绘制闭环阶跃响应时域图
% % 这是一个单位反馈系统，建立闭环模型
% Sys_Closed = feedback(G0, 1); 
% 
% figure(1);
% step(Sys_Closed);
% title('闭环系统阶跃响应 (Step Response)');
% grid on;
% 
% % 3. 绘制对数幅频特性曲线图 (Bode Plot)
% % 注意：Bode图包含了“对数幅频特性”和“相频特性”
% figure(2);
% bode(G0);
% title('开环系统对数幅频特性 (Bode Plot)');
% grid on;
% % 如果只需要显示幅频特性，可以右键点击图表选择 "Magnitude"
% 
% % 4. 绘制幅频特性曲线图 (Nyquist Plot / 极坐标图)
% % 通常"幅频特性曲线"在不特指对数时，常指代奈奎斯特图(极坐标)
% figure(3);
% nyquist(G0);
% title('开环系统幅频特性 (Nyquist Plot)');
% grid on;
% axis equal; % 设置坐标轴比例一致，方便观察围线
% 
% % --- 补充：如果你需要线性的幅值-频率曲线 (非对数) ---
% figure(4);
% [mag, phase, w] = bode(G0);
% mag = squeeze(mag); % 去除多余的维度
% plot(w, mag, 'LineWidth', 1.5);
% title('线性幅频特性曲线 A(\omega)');
% xlabel('Frequency (rad/s)');
% ylabel('Amplitude A(\omega)');
% grid on;
% xlim([0, 20]); % 限制x轴范围以便观察
% 清除环境
clc; clear; close all;

% 1. 定义传递函数
s = tf('s');
G0 = 30 / (s * (0.1*s + 1) * (0.2*s + 1));

% 2. 计算裕度和频率
% Gm: 幅值裕度 (线性值)
% Pm: 相角裕度 (度)
% Wcg: 截止频率 / 剪切频率 (Gain Crossover Frequency, 幅值为0dB时的频率)
% Wcp: 相角交界频率 (Phase Crossover Frequency, 相角为-180度时的频率)
[Gm, Pm, Wcg, Wcp] = margin(G0);

% 将幅值裕度转换为分贝 (dB)
Gm_dB = 20 * log10(Gm);

% 3. 输出计算结果到命令行
fprintf('--------------------------------------------------\n');
fprintf('系统稳定性分析结果 (K=30):\n');
fprintf('--------------------------------------------------\n');
fprintf('1. 截止频率 (Wcg, 0dB处):       %.4f rad/s\n', Wcg);
fprintf('2. 相角交界频率 (Wcp, -180度处): %.4f rad/s\n', Wcp);
fprintf('--------------------------------------------------\n');
fprintf('3. 幅值裕度 (GM):               %.4f dB\n', Gm_dB);
fprintf('4. 相角裕度 (PM):               %.4f deg\n', Pm);
fprintf('--------------------------------------------------\n');

% 判断稳定性
if Gm_dB > 0 && Pm > 0
    disp('结论: 系统是稳定的 (闭环稳定)');
else
    disp('结论: 系统是不稳定的 (闭环不稳定)');
end

% 4. 绘制带有标注的伯德图
figure(1);
margin(G0); % margin函数直接调用时会自动绘图并标注裕度和频率
grid on;
title('带裕度和频率标注的伯德图 (K=30)');

% 优化图表显示 (可选，使线更清晰)
h = findobj(gcf, 'type', 'line');
set(h, 'LineWidth', 1.5);

%% === 附加功能：寻找特定相角对应的频率 ===

% 1. 设定目标相角
Target_Phase = -140; 

% 2. 定义无卷绕的相角方程 (推荐方法)
% 这是一个单调递减函数，不会出现跳变，fzero 可以完美求解
% atand 是求反正切并直接输出度数
phase_equation = @(w) -90 - atand(0.1*w) - atand(0.2*w) - Target_Phase;

% 3. 求解频率
% 现在区间 [0.1, 50] 是安全的，因为高频时方程结果会变成很大的负数
W_new = fzero(phase_equation, [0.1, 50]);

% 4. 计算该频率处的幅值 (用于计算滞后校正的 Beta)
Mag_at_Wnew = abs(evalfr(G0, 1j*W_new));
Mag_at_Wnew_dB = 20*log10(Mag_at_Wnew);

% 5. 输出结果
fprintf('\n--------------------------------------------------\n');
fprintf('特定相角查找结果:\n');
fprintf('目标相角:               %.2f deg\n', Target_Phase);
fprintf('对应的频率 (Wc''):       %.4f rad/s\n', W_new);
fprintf('该频率处的幅值:         %.4f (%.4f dB)\n', Mag_at_Wnew, Mag_at_Wnew_dB);
fprintf('--------------------------------------------------\n');

% 6. 在图上标记出来 (可视化验证)
figure(1); % 切换回刚才画的图
% 找到所有 axes (坐标轴)
all_axes = findall(gcf, 'type', 'axes'); 
%通常 Bode 图生成的 axes 中，第 1 个是幅频图(下)，第 2 个是相频图(上)
% 或者反过来，取决于版本。我们直接利用频率 W_new 绘点

% 绘制相频图上的点
subplot(2,1,2); hold on;
plot(W_new, Target_Phase, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
text(W_new, Target_Phase, sprintf('  (%.2f, %.0f^o)', W_new, Target_Phase), 'Color', 'red');

% 绘制幅频图上的点
subplot(2,1,1); hold on;
plot(W_new, Mag_at_Wnew_dB, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
text(W_new, Mag_at_Wnew_dB, sprintf('  %.2f dB', Mag_at_Wnew_dB), 'Color', 'red');