% 力-质量系统仿真
% 运动方程: f - b*dx/dt = M*d²x/dt²
% 参数: f=1N, M=1kg, b=0.4N/(m/s)
% 拉力作用时间: 2秒

clear; clc; close all;

%% 系统参数
M = 1;          % 质量 [kg]
b = 0.4;        % 摩擦系数 [N/(m/s)]
f = 1;          % 拉力 [N]
t_pull = 2;     % 拉力作用时间 [s]
sim_time = 10;  % 总仿真时间 [s]

%% ode45
% 定义时间范围
tspan = [0 sim_time];

% 初始条件 [x(0), dx/dt(0)]
initial_conditions = [0; 0];  % 假设初始位置和速度都为0

% 求解微分方程
[t, state] = ode45(@(t, y) mass_system_ode(t, y, M, b, f, t_pull), tspan, initial_conditions);

% 提取状态变量
x = state(:, 1);      % 位置 [m]
v = state(:, 2);      % 速度 [m/s]

% 计算加速度 [m/s²]
a = zeros(size(t));
for i = 1:length(t)
    if t(i) <= t_pull
        a(i) = (f - b*v(i)) / M;
    else
        a(i) = (-b*v(i)) / M;
    end
end


%% 绘制结果

% 位置曲线
figure;
plot(t, x, 'b-', 'LineWidth', 2);
grid on;
title('力-质量系统响应', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('位置 x(t) [m]', 'FontSize', 12);
xlabel('时间 t [s]', 'FontSize', 12);
xlim([0 sim_time]);
% 标记拉力作用结束时间
line([t_pull t_pull], ylim, 'Color', 'red', 'LineStyle', '--', 'LineWidth', 1);
text(t_pull+0.1, max(x)*0.8, '拉力停止', 'Color', 'red', 'FontSize', 10);

% 速度曲线
figure;
plot(t, v, 'r-', 'LineWidth', 2);
grid on;
ylabel('速度 v(t) [m/s]', 'FontSize', 12);
xlabel('时间 t [s]', 'FontSize', 12);

% 标记拉力作用结束时间
line([t_pull t_pull], ylim, 'Color', 'red', 'LineStyle', '--', 'LineWidth', 1);
text(t_pull+0.1, max(v)*0.8, '拉力停止', 'Color', 'red', 'FontSize', 10);

% 加速度曲线
figure;

plot(t, a, 'g-', 'LineWidth', 2);
grid on;
ylabel('加速度 a(t) [m/s²]', 'FontSize', 12);
xlabel('时间 t [s]', 'FontSize', 12);
xlim([0 sim_time]);
% 标记拉力作用结束时间
line([t_pull t_pull], ylim, 'Color', 'red', 'LineStyle', '--', 'LineWidth', 1);
text(t_pull+0.1, max(a)*0.8, '拉力停止', 'Color', 'red', 'FontSize', 10);



%% 绘制在一张图中的对比图
figure('Position', [100, 100, 1000, 600]);
plot(t, x, 'b-', 'LineWidth', 2);
hold on;
plot(t, v, 'r-', 'LineWidth', 2);
plot(t, a, 'g-', 'LineWidth', 2);

grid on;
title('力-质量系统状态响应', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('时间 t [s]', 'FontSize', 12);

% 标记拉力作用结束时间
line([t_pull t_pull], ylim, 'Color', 'black', 'LineStyle', '--', 'LineWidth', 1);
text(t_pull+0.1, max(v)*0.7, '拉力停止', 'Color', 'black', 'FontSize', 10);

legend('位置 x(t)', '速度 v(t)', '加速度 a(t)', 'Location', 'best');

%% 输出系统响应特性
fprintf('力-质量系统仿真结果:\n');
fprintf('========================\n');
fprintf('最大位置: %.4f m\n', max(x));
fprintf('最大速度: %.4f m/s\n', max(v));
fprintf('最大加速度: %.4f m/s²\n', max(a));
fprintf('最终位置: %.4f m\n', x(end));
fprintf('最终速度: %.4f m/s\n', v(end));
fprintf('系统达到最大速度的时间: %.2f s\n', t(v == max(v)));

%% 定义微分方程函数
function dstate = mass_system_ode(t, state, M, b, f, t_pull)
    % state(1) = x (位置)
    % state(2) = dx/dt (速度)
    
    x = state(1);
    v = state(2);
    
    % 分段外力函数：前2秒有拉力，之后无拉力
    if t <= t_pull
        F = f;
    else
        F = 0;
    end
    
    % 计算加速度
    a = (F - b*v) / M;
    
    % 返回状态导数 [dx/dt; dv/dt]
    dstate = [v; a];
end