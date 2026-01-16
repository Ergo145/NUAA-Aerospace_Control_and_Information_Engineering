clear; clc; close all;

%% 系统参数
mu = 1/82.45;           % μ值
mu_star = 1 - mu;       % μ*值

%% 初始条件
x0 = 1.2;               % 初始x位置
dx0 = 0;                % 初始x速度
y0 = 0;                 % 初始y位置
dy0 = -1.04935751;      % 初始y速度

initial_conditions = [x0; dx0; y0; dy0];

%% 仿真时间设置
t_span = [0 10];        % 仿真时间范围 [0, 10] 单位时间

%% 求解微分方程组
[t, state] = ode45(@(t, y) spacecraft_equations(t, y, mu, mu_star), t_span, initial_conditions);

%% 提取状态变量
x = state(:, 1);        % x位置
dx = state(:, 2);       % x速度
y = state(:, 3);        % y位置
dy = state(:, 4);       % y速度

%% 计算加速度 - 修复后的方法
ddx = zeros(size(t));
ddy = zeros(size(t));

for i = 1:length(t)
    dstate = spacecraft_equations(t(i), state(i,:)', mu, mu_star);
    ddx(i) = dstate(2);
    ddy(i) = dstate(4);
end

%% 绘制轨迹图


% 主图：飞行器轨迹
figure;
plot(x, y, 'b-', 'LineWidth', 1.5);
hold on;
% 标记起点和终点
plot(x(1), y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(x(end), y(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

grid on;
axis equal;
title('空间飞行器运动轨迹', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('x 坐标', 'FontSize', 12);
ylabel('y 坐标', 'FontSize', 12);
legend('飞行轨迹', '起点', '终点', 'Location', 'best');

% 添加参数信息
text(0.02, 0.98, sprintf('μ = %.6f\nμ* = %.6f', mu, mu_star), ...
     'Units', 'normalized', 'VerticalAlignment', 'top', ...
     'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 10);

% x位置随时间变化
figure;
plot(t, x, 'r-', 'LineWidth', 1.5);
grid on;
title('x位置随时间变化', 'FontSize', 12);
xlabel('时间', 'FontSize', 10);
ylabel('x坐标', 'FontSize', 10);

% y位置随时间变化
figure;
plot(t, y, 'g-', 'LineWidth', 1.5);
grid on;
title('y位置随时间变化', 'FontSize', 12);
xlabel('时间', 'FontSize', 10);
ylabel('y坐标', 'FontSize', 10);

%% 绘制速度变化图
figure;
plot(t, dx, 'b-', 'LineWidth', 1.5);
grid on;
title('x方向速度变化', 'FontSize', 12);
xlabel('时间', 'FontSize', 10);
ylabel('dx/dt', 'FontSize', 10);

figure;
plot(t, dy, 'm-', 'LineWidth', 1.5);
grid on;
title('y方向速度变化', 'FontSize', 12);
xlabel('时间', 'FontSize', 10);
ylabel('dy/dt', 'FontSize', 10);

%% 绘制3D轨迹图
figure('Position', [100, 100, 800, 600]);
plot3(x, y, t, 'b-', 'LineWidth', 1.5);
hold on;
plot3(x(1), y(1), t(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(x(end), y(end), t(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
grid on;
title('空间飞行器轨迹 (x-y-t)', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('x坐标', 'FontSize', 12);
ylabel('y坐标', 'FontSize', 12);
zlabel('时间', 'FontSize', 12);
legend('飞行轨迹', '起点', '终点', 'Location', 'best');

%% 输出系统信息
fprintf('空间飞行器运动仿真结果:\n');
fprintf('========================\n');
fprintf('参数: μ = %.6f, μ* = %.6f\n', mu, mu_star);
fprintf('初始条件: x(0)=%.2f, dx/dt(0)=%.2f, y(0)=%.2f, dy/dt(0)=%.8f\n', ...
        x0, dx0, y0, dy0);
fprintf('仿真时间: %.1f 到 %.1f\n', t_span(1), t_span(2));
fprintf('轨迹范围: x ∈ [%.3f, %.3f], y ∈ [%.3f, %.3f]\n', ...
        min(x), max(x), min(y), max(y));
fprintf('最终位置: (%.4f, %.4f)\n', x(end), y(end));

%% 定义微分方程函数
function dstate = spacecraft_equations(t, state, mu, mu_star)
    % state = [x; dx/dt; y; dy/dt]
    x = state(1);
    dx = state(2);
    y = state(3);
    dy = state(4);
    
    % 计算 r₁ 和 r₂
    r1 = sqrt((x + mu)^2 + y^2);
    r2 = sqrt((x - mu_star)^2 + y^2);
    
    % 计算加速度项
    ddx = 2*dy + x - (mu_star*(x + mu))/(r1^3) - (mu*(x - mu_star))/(r2^3);
    ddy = -2*dx + y - (mu_star*y)/(r1^3) - (mu*y)/(r2^3);
    
    % 返回状态导数
    dstate = [dx; ddx; dy; ddy];
end