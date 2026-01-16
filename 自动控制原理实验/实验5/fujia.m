%% 附加题五：飞行控制律频域校正 (含执行机构与内环反馈)
clc; clear; close all;

% ==========================================
% 1. 系统建模：构建广义被控对象 G_fix
% ==========================================
s = tf('s');

% (1) 飞机模型 G0 (原始对象)
num_G0 = 30;
den_G0 = conv([1, 0], conv([0.1, 1], [0.2, 1])); % s(0.1s+1)(0.2s+1)
G0 = tf(num_G0, den_G0);

% (2) 执行机构 G_act
% 描述：G0前端接了一个 -1/(0.1s+1)
G_act = tf(-1, [0.1, 1]);

% (3) 内环前向通道
G_inner_fwd = G_act * G0;

% (4) 内环反馈通道 (俯仰角速率反馈)
% 描述：负反馈 K 接在执行机构前面
% 注意：如果 G_act 是负值，G0 是正值，总前向增益为负。
% 为了形成“负反馈”的稳定阻尼效果，反馈回路的极性非常关键。
% 若 K 取正值，feedback(G_neg, K) = G_neg / (1 + G_neg*K) = G_neg / (1 - |GK|) -> 极点右移，不稳定。
% 因此，这里假设 K 为负值 (例如 -0.5) 或者物理连接上构成了阻尼。
K_value = -0.5; % 【可修改】请根据你的具体参数调整 K 值
H_inner = K_value * s; 

% (5) 构建广义被控对象 G_fix (内环闭环)
% 这是外环控制器 Gc 真正看到的“对象”
G_fix = feedback(G_inner_fwd, H_inner);
G_fix = minreal(G_fix); % 消除微小零极点

disp('================================================');
disp('【1】 广义被控对象 G_fix(s) (已包含内环和执行机构)：');
display(G_fix);
if max(real(pole(G_fix))) > 0
    disp('警告：当前的 K 值导致内环不稳定！请检查 K 的符号或数值。');
end

% ==========================================
% 2. 外环串联滞后校正设计
% ==========================================
% --- 设计指标 ---
Expected_PM = 45;       % 期望相角裕度 (度)
Margin_Safety = 8;      % 补偿角度 (滞后校正需多留一点余量)

% 计算目标相角
phi_target = -180 + Expected_PM + Margin_Safety;

% 获取 G_fix 的 Bode 数据
w_log = logspace(-2, 2, 1000);
[mag, phase, w] = bode(G_fix, w_log);
mag = squeeze(mag); 
phase = squeeze(phase);

% --- 步骤 A: 查找新截止频率 wc_new ---
% 使用 min 函数查找最接近 phi_target 的频率
[~, idx] = min(abs(phase - phi_target));
wc_new = w(idx);
mag_at_wc = 20*log10(mag(idx)); 

% --- 步骤 B: 计算校正参数 ---
% 滞后校正器 Gc(s) = (1 + Ta*s) / (1 + Tb*s)
% 需要衰减 mag_at_wc 分贝 -> 20log(beta) = mag_at_wc
beta = 10^(mag_at_wc / 20); % 分度系数 (beta > 1)

% 零点转折频率 1/Ta 通常选在 0.1 * wc_new
Ta = 10 / wc_new;       % 分子时间常数
Tb = beta * Ta;         % 分母时间常数

% 构造校正网络 Gc
Gc = (1 + Ta*s) / (1 + Tb*s);

disp('------------------------------------------------');
disp('【2】 校正网络 Gc(s) 设计结果：');
fprintf('   新截止频率 wc" = %.4f rad/s\n', wc_new);
fprintf('   该处幅值 L(wc")= %.4f dB\n', mag_at_wc);
display(Gc);

% ==========================================
% 3. 验证与绘图 (分开绘制 4 张图)
% ==========================================
% 构建系统
sys_outer_open_old = G_fix;        % 校正前 (但已有内环)
sys_outer_open_new = Gc * G_fix;   % 校正后
sys_cl_old = feedback(sys_outer_open_old, 1);
sys_cl_new = feedback(sys_outer_open_new, 1);

% --- 图1：校正前 Bode ---
figure('Name', 'Fig1: Uncompensated Bode', 'Color', 'w');
bode(sys_outer_open_old, 'b--'); 
grid on; 
title('校正前 Bode 图 (含内环)');
subtitle('G_{fix}(s)');

% --- 图2：校正后 Bode (带指标) ---
figure('Name', 'Fig2: Compensated Bode', 'Color', 'w');
margin(sys_outer_open_new); % 自动标注 GM, PM, Wcg, Wcp
grid on;
title('校正后 Bode 图');
subtitle(sprintf('G_c(s) * G_{fix}(s) (PM \\approx %.1f^o)', Expected_PM));
h = findobj(gcf, 'type', 'line'); set(h, 'LineWidth', 1.5);

% --- 图3：校正前 阶跃响应 ---
figure('Name', 'Fig3: Uncompensated Step', 'Color', 'w');
step(sys_cl_old, 'b--', 20);
grid on;
title('校正前 阶跃响应');
subtitle('System with Inner Loop only');

% --- 图4：校正后 阶跃响应 ---
figure('Name', 'Fig4: Compensated Step', 'Color', 'w');
step(sys_cl_new, 'r-', 20);
grid on;
title('校正后 阶跃响应');
subtitle('Final Closed Loop System');

% ==========================================
% 4. 输出填表数据
% ==========================================
fprintf('\n============ 填表数据 [Source 39] ============\n');
fprintf('Ta (分子时间常数)   = %.4f\n', Ta);
fprintf('a  (即 1/Ta)        = %.4f\n', 1/Ta);
fprintf('Tb (分母时间常数)   = %.4f\n', Tb);
fprintf('b  (即 Ta/Tb)       = %.4f\n', 1/beta);
fprintf('----------------------------------------------\n');
fprintf('注意：请检查 K 值 (当前 K=%.1f) 是否符合你的设定。\n', K_value);
fprintf('==============================================\n');