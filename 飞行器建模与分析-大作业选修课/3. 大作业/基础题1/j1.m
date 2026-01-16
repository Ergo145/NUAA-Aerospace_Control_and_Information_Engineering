clear; clc; close all;
%% ode45
% 定义时间范围
tspan = [0 20];

% 初始条件 [x(0), x'(0)]
initial_conditions = [1; 1];

% 求解微分方程
[t, state] = ode45(@system_equation, tspan, initial_conditions);

% 提取状态变量
x = state(:, 1);      % 位置
x_dot = state(:, 2);  % 速度

% 计算加速度 (从微分方程推导)
x_ddot = sin(pi*t/2) - abs(x.^2 - 1).*(x_dot.^3) - x;

%% 绘制结果

% 位置曲线
figure;
plot(t, x, 'b-', 'LineWidth', 2);
grid on;
title('系统状态响应', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('位置 x(t)', 'FontSize', 12);
legend('位置', 'Location', 'best');
xlim([0 20]);

% 速度曲线
figure;
plot(t, x_dot, 'r-', 'LineWidth', 2);
grid on;
title('系统状态响应', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('速度 dx/dt', 'FontSize', 12);
legend('速度', 'Location', 'best');
xlim([0 20]);

% 加速度曲线
figure;
plot(t, x_ddot, 'g-', 'LineWidth', 2);
grid on;
title('系统状态响应', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('加速度 d²x/dt²', 'FontSize', 12);
xlabel('时间 t (秒)', 'FontSize', 12);
legend('加速度', 'Location', 'best');
xlim([0 20]);

%% 绘制对比图
figure('Position', [100, 100, 1000, 600]);

plot(t, x, 'b-', 'LineWidth', 2);
hold on;
plot(t, x_dot, 'r-', 'LineWidth', 2);
plot(t, x_ddot, 'g-', 'LineWidth', 2);
grid on;

title('系统状态响应对比', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('时间 t (秒)', 'FontSize', 12);
ylabel('状态变量', 'FontSize', 12);
legend('位置 x(t)', '速度 dx/dt', '加速度 d²x/dt²', 'Location', 'best');
xlim([0 20]);

%% 输出统计信息
fprintf('系统仿真结果统计:\n');
fprintf('仿真时间: 0 到 %.1f 秒\n', t(end));
fprintf('位置范围: [%.3f, %.3f]\n', min(x), max(x));
fprintf('速度范围: [%.3f, %.3f]\n', min(x_dot), max(x_dot));
fprintf('加速度范围: [%.3f, %.3f]\n', min(x_dot), max(x_dot));

%% 定义微分方程函数
function dstate = system_equation(t, state)
    x = state(1);
    x_dot = state(2);

    F = sin(pi*t/2);
    
    nonlinear_damping = abs(x^2 - 1) * (x_dot^3);
    
    x_ddot = F - nonlinear_damping - x;
    
    dstate = [x_dot; x_ddot];
end