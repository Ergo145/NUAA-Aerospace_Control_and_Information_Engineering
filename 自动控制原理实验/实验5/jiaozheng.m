% =========================================================================
% 题目：串联滞后校正设计
% 要求：Kv=30, 相角裕度>=40度, 截止频率>=2.3 rad/s, 幅值裕度>=10dB
% =========================================================================
clc; clear; close all;

%% 1. 系统定义
s = tf('s');
K = 30; % 满足 Kv = 30
G0 = K / (s * (0.1*s + 1) * (0.2*s + 1));

fprintf('================ STEP 1: 原系统分析 ================\n');
[Gm0, Pm0, Wcg0, Wcp0] = margin(G0);
fprintf('原系统截止频率 Wcg: %.4f rad/s\n', Wcg0);
fprintf('原系统相角裕度 Pm:  %.4f deg\n', Pm0);
if Pm0 > 0 && 20*log10(Gm0) > 0
    disp('-> 原系统状态: 稳定，但可能指标不达标');
else
    disp('-> 原系统状态: 不稳定或临界稳定');
end

%% 2. 滞后校正设计参数计算
fprintf('\n================ STEP 2: 校正参数计算 ================\n');
% 2.1 确定目标相角
% 目标相角裕度 = 40度 + 5度(安全裕量) = 45度
% 对应的相角值 = -180 + 45 = -135度
% (注：你之前的代码找的是-140度，这里我按你的思路设为-140度，即留了40度裕度)
Target_Phase = -134; 

% 2.2 寻找该相角对应的频率 (校正后的新截止频率)
% 使用无卷绕的相角方程，避免 fzero 报错
% G0相角公式: -90 - arctan(0.1w) - arctan(0.2w)
phase_eqn = @(w) -90 - atand(0.1*w) - atand(0.2*w) - Target_Phase;

% 在 [0.1, 10] 范围内搜索
Wc_new = fzero(phase_eqn, [0.1, 10]);

% 2.3 计算该频率处的幅值，确定衰减因子 Beta
Mag_at_Wc = abs(evalfr(G0, 1j*Wc_new));
Mag_dB = 20*log10(Mag_at_Wc);
Beta = Mag_at_Wc; % 需要衰减这么多倍才能使幅值为1 (0dB)

fprintf('1. 选定新截止频率 (Wc''): %.4f rad/s\n', Wc_new);
if Wc_new >= 2.3
    fprintf('   -> 满足题目 Wc >= 2.3 的要求\n');
else
    fprintf('   -> 警告！不满足 Wc >= 2.3 的要求\n');
end
fprintf('2. 该频率处原幅值:       %.4f dB\n', Mag_dB);
fprintf('3. 计算分度系数 (Beta):  %.4f\n', Beta);

% 2.4 计算滞后网络时间常数 T
% 零点频率选在新截止频率的 0.1 倍处 (尽量减小滞后网络带来的相位滞后)
Zero_freq = 0.1 * Wc_new;
T = 1 / Zero_freq;
% 极点频率
Pole_freq = 1 / (Beta * T);

fprintf('4. 校正网络设计:         T = %.4f\n', T);
fprintf('   零点 = %.4f, 极点 = %.4f\n', Zero_freq, Pole_freq);

% 2.5 构造校正装置 Gc(s)
% Gc = (T*s + 1) / (Beta*T*s + 1);
Gc = (3.65*s + 1) / (33.18*s + 1);

fprintf('   校正传函 Gc(s) = \n');
display(Gc);

%% 3. 校正后系统验证
fprintf('================ STEP 3: 验证校正后系统 ================\n');
G_comp = Gc * G0; % 校正后的开环传递函数

[Gm_c, Pm_c, Wcg_c, Wcp_c] = margin(G_comp);
Gm_c_dB = 20*log10(Gm_c);

fprintf('校正后截止频率: %.4f rad/s (要求>=2.3) -> %s\n', ...
    Wcg_c, string(Wcg_c>=2.3));
fprintf('校正后相角裕度: %.4f deg   (要求>=40)  -> %s\n', ...
    Pm_c, string(Pm_c>=40));
fprintf('校正后幅值裕度: %.4f dB    (要求>=10)  -> %s\n', ...
    Gm_c_dB, string(Gm_c_dB>=10));

%% 4. 绘图 (修复覆盖问题)
% 图1：原系统 Bode 图 + 选定点标记
figure(1);
margin(G0); 
grid on;
title('原系统 Bode 图与选定截止频率点');

% --- 获取坐标轴句柄 (修复 subplot 覆盖问题的核心) ---
all_axes = findall(gcf, 'type', 'axes');
% 自动识别幅频和相频坐标轴
ax_mag = []; ax_phase = [];
for i = 1:length(all_axes)
    ylabel_str = get(get(all_axes(i), 'YLabel'), 'String');
    if contains(ylabel_str, 'deg'), ax_phase = all_axes(i); end
    if contains(ylabel_str, 'dB'), ax_mag = all_axes(i); end
end

% 标记选定的频率点
if ~isempty(ax_phase)
    axes(ax_phase); hold on;
    plot(Wc_new, Target_Phase, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    text(Wc_new, Target_Phase, sprintf('  选定点\n  (%.2f rad/s, %.0f^o)', Wc_new, Target_Phase), ...
        'Color', 'red', 'VerticalAlignment', 'top');
end
if ~isempty(ax_mag)
    axes(ax_mag); hold on;
    plot(Wc_new, Mag_dB, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    text(Wc_new, Mag_dB, sprintf('  需衰减 %.2f dB', Mag_dB), ...
        'Color', 'red', 'VerticalAlignment', 'bottom');
end

% 图2：校正前后对比
figure(2);
bode(G0, 'b--', G_comp, 'r-');
grid on;
legend('校正前 (Uncompensated)', '校正后 (Compensated)');
title('串联滞后校正前后对比');
% 在图上标注最终结果
text(0.1, 0.2, sprintf('Final PM = %.2f^o', Pm_c), 'Units', 'normalized', 'Color', 'red');