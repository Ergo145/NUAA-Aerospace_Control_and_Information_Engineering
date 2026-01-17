%% 实验内容1：惯性坐标系与旋转坐标系下的飞行轨迹对比
clc; clear; close all;

% 1. 系统参数 (地月系统)
mu = 0.01215; % 质量比
t_span = [0, 15]; % 仿真时间

% 2. 初始状态 (旋转坐标系下: x, y, vx, vy)
% 这里选取一个典型的轨道初值以便观察
Y0 = [0.9; 0; 0; 0.6]; 

% 3. 积分求解 (使用 ode45)
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);
[t, Y] = ode45(@(t,y) cr3bp_func(t,y,mu), t_span, Y0, options);

% 4. 坐标变换：旋转系 -> 惯性系
x_rot = Y(:,1);
y_rot = Y(:,2);

% 变换公式：
% X_in = x_rot * cos(t) - y_rot * sin(t)
% Y_in = x_rot * sin(t) + y_rot * cos(t)
x_in = x_rot .* cos(t) - y_rot .* sin(t);
y_in = x_rot .* sin(t) + y_rot .* cos(t);

% 5. 绘图对比
figure('Name', '实验内容1：轨迹对比', 'Color', 'w');

% 子图1：旋转坐标系
subplot(1, 2, 1);
plot(x_rot, y_rot, 'b', 'LineWidth', 1.5); hold on;
plot(-mu, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); % 地球
plot(1-mu, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'c'); % 月球
axis equal; grid on;
title('旋转坐标系下的轨迹 (Rotating Frame)');
xlabel('x'); ylabel('y');

% 子图2：惯性坐标系
subplot(1, 2, 2);
plot(x_in, y_in, 'r', 'LineWidth', 1.5); hold on;
plot(0, 0, 'k+'); % 质心
axis equal; grid on;
title('惯性坐标系下的轨迹 (Inertial Frame)');
xlabel('X'); ylabel('Y');

% --- 动力学方程函数 ---
function dY = cr3bp_func(~, Y, mu)
    x = Y(1); y = Y(2); vx = Y(3); vy = Y(4);
    
    r1 = sqrt((x+mu)^2 + y^2);
    r2 = sqrt((x-(1-mu))^2 + y^2);
    
    % 完整动力学方程 (包含科里奥利力、离心力、引力)
    ax = 2*vy + x - (1-mu)*(x+mu)/r1^3 - mu*(x-(1-mu))/r2^3;
    ay = -2*vx + y - (1-mu)*y/r1^3 - mu*y/r2^3;
    
    dY = [vx; vy; ax; ay];
end