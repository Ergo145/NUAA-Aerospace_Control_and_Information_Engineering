% 飞行器运动方程求解
clear; clc; close all;

% 定义时间范围
tspan = [0, 100]; % 时间范围，可以根据需要调整
theta0 = 0; % 初始角度，假设从theta=0开始

% 定义微分方程：dtheta/dt = 1/sqrt((dx/dtheta)^2 + (dy/dtheta)^2)
% dx/dtheta = -20*sin(theta), dy/dtheta = 5*cos(theta)
% 所以速度大小 = sqrt(400*sin(theta)^2 + 25*cos(theta)^2)
% 由于单位速率，所以 dtheta/dt = 1/sqrt(400*sin(theta)^2 + 25*cos(theta)^2)

odefun = @(t, theta) 1./sqrt(400*sin(theta).^2 + 25*cos(theta).^2);

% 求解微分方程
[t, theta] = ode45(odefun, tspan, theta0);

% 计算飞行器位置
x = 10 + 20*cos(theta);
y = 20 + 5*sin(theta);

% 绘制轨迹
figure;
plot(x, y, 'b-', 'LineWidth', 2);
hold on;
plot(x(1), y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(x(end), y(end), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
xlabel('X坐标');
ylabel('Y坐标');
title('飞行器椭圆轨迹');
legend('轨迹', '起点', '终点', 'Location', 'best');
grid on;
axis equal;

% 绘制位置随时间变化
figure;
subplot(2,1,1);
plot(t, x, 'r-', 'LineWidth', 2);
xlabel('时间 t');
ylabel('X坐标');
title('飞行器X坐标随时间变化');
grid on;

subplot(2,1,2);
plot(t, y, 'b-', 'LineWidth', 2);
xlabel('时间 t');
ylabel('Y坐标');
title('飞行器Y坐标随时间变化');
grid on;

% 显示一些数值结果
fprintf('飞行器运动方程:\n');
fprintf('x(t) = 10 + 20*cos(theta(t))\n');
fprintf('y(t) = 20 + 5*sin(theta(t))\n\n');
fprintf('初始位置: x(0) = %.2f, y(0) = %.2f\n', x(1), y(1));
fprintf('椭圆中心: (10, 20)\n');
fprintf('长半轴: 20, 短半轴: 5\n');

% 保存数据（可选）
% save('flight_path.mat', 't', 'x', 'y', 'theta');