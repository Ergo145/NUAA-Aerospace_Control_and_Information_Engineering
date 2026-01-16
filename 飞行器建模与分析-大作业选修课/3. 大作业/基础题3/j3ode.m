% 方法2: 使用ODE求解器实现带重置条件的积分器

clear; clc; close all;

%% 系统参数
input_signal = 1;
initial_condition = -50;
reset_threshold = 20;
reset_value = -100;
sim_time = 100;

%% 使用ODE事件检测
options = odeset('Events', @reset_event);
t_span = [0 sim_time];
y0 = initial_condition;

% 存储所有时间段的结果
t_all = [];
y_all = [];
reset_events = [];

% 循环仿真，处理重置事件
while t_span(1) < sim_time
    [t_segment, y_segment, te, ye, ie] = ode45(@(t,y) input_signal, t_span, y0, options);
    
    % 存储当前段的结果
    t_all = [t_all; t_segment];
    y_all = [y_all; y_segment];
    
    % 如果有事件发生（达到阈值）
    if ~isempty(te)
        % 记录重置事件
        reset_events = [reset_events; te, ye];
        
        % 重置初始条件继续仿真
        y0 = reset_value;
        t_span = [te, sim_time];
    else
        break;
    end
end

%% 绘制结果
figure('Position', [200, 200, 1000, 600]);
plot(t_all, y_all, 'b-', 'LineWidth', 2);
hold on;

% 标记阈值和重置值
yline(reset_threshold, 'r--', 'LineWidth', 1.5, 'Label', '重置阈值 (20)');
yline(reset_value, 'g--', 'LineWidth', 1.5, 'Label', '重置值 (-100)');

% 标记重置点
if ~isempty(reset_events)
    plot(reset_events(:,1), reset_events(:,2), 'ro', ...
         'MarkerSize', 8, 'MarkerFaceColor', 'red');
end

grid on;
title('带重置条件的积分器 (ODE方法)', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('时间 [秒]', 'FontSize', 12);
ylabel('积分器输出', 'FontSize', 12);

%% 事件函数定义
function [value, isterminal, direction] = reset_event(t, y)
    reset_threshold = 20;
    value = y - reset_threshold;      % 当 y > 20 时触发事件
    isterminal = 1;                   % 终止积分
    direction = 1;                    % 仅在上升时检测
end