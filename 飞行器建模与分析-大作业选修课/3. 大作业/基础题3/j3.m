
clear; clc; close all;

%% 系统参数
input_signal = 1;          % 输入信号
initial_condition = -50;   % 初始条件
reset_threshold = 20;      % 重置阈值
reset_value = -100;        % 重置值
sim_time = 100;            % 仿真时间

%% 使用离散时间仿真
dt = 0.1;                  % 时间步长
t = 0:dt:sim_time;         % 时间向量
N = length(t);             % 时间点数

% 初始化输出
output = zeros(1, N);
output(1) = initial_condition;

% 仿真循环
for i = 2:N
    % 积分器输出 = 前一个输出 + 输入 × 时间步长
    output(i) = output(i-1) + input_signal * dt;
    
    % 检查重置条件
    if output(i) > reset_threshold
        output(i) = reset_value;
    end
end

%% 绘制结果
figure('Position', [100, 100, 1000, 600]);

% 绘制积分器输出
plot(t, output, 'b-', 'LineWidth', 2);
hold on;

% 标记重置阈值线
line([0 sim_time], [reset_threshold reset_threshold], ...
     'Color', 'red', 'LineStyle', '--', 'LineWidth', 1.5);
 
% 标记重置值线
line([0 sim_time], [reset_value reset_value], ...
     'Color', 'green', 'LineStyle', '--', 'LineWidth', 1.5);

grid on;
title('带重置条件的积分器仿真', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('时间 [s]', 'FontSize', 12);
ylabel('积分器输出', 'FontSize', 12);

% 添加图例和说明
legend('积分器输出', ...
       sprintf('重置阈值 (%d)', reset_threshold), ...
       sprintf('重置值 (%d)', reset_value), ...
       'Location', 'best');

% 添加系统参数文本框
annotation('textbox', [0.15, 0.15, 0.3, 0.2], 'String', ...
    {'系统参数:', ...
     sprintf('输入 = %d', input_signal), ...
     sprintf('初始条件 = %d', initial_condition), ...
     sprintf('重置阈值 = %d', reset_threshold), ...
     sprintf('重置值 = %d', reset_value)}, ...
    'FontSize', 11, ...
    'BackgroundColor', 'white', 'EdgeColor', 'black');

%% 标记重置事件
% 找到重置发生的时间点
reset_indices = find(diff(output) < -50) + 1;  % 输出大幅下降表示重置

for i = 1:length(reset_indices)
    idx = reset_indices(i);
    plot(t(idx), output(idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red');
    text(t(idx)+2, output(idx)+10, sprintf('重置 #%d\n时间: %.1fs', i, t(idx)), ...
         'FontSize', 9, 'BackgroundColor', 'white');
end

%% 输出仿真统计
fprintf('带重置条件积分器仿真结果:\n');
fprintf('========================\n');
fprintf('仿真时间: 0 到 %.0f 秒\n', sim_time);
fprintf('重置次数: %d\n', length(reset_indices));
fprintf('第一次重置时间: %.1f 秒\n', t(reset_indices(1)));
fprintf('最终输出值: %.1f\n', output(end));