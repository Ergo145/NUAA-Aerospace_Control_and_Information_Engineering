%% 实验内容3：双坐标系综合动态仿真
clc; clear; close all;

% 参数设置
mu = 0.01215;
t_span = 0:0.05:20; % 设置固定步长以便动画同步
Y0 = [0.8; 0; 0; 0.5]; 

% 求解轨迹
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[t, Y] = ode45(@(t,y) cr3bp_func_full(t,y,mu), t_span, Y0, options);

x_rot = Y(:,1); y_rot = Y(:,2);
% 转换到惯性系
x_in = x_rot .* cos(t) - y_rot .* sin(t);
y_in = x_rot .* sin(t) + y_rot .* cos(t);

% --- 动画设置 ---
figure('Name', '实验内容3：轨迹生成过程动态演示', 'Color', 'w', 'Position', [100, 100, 1000, 500]);

% 惯性系绘图句柄
subplot(1, 2, 1);
h_in_traj = plot(NaN, NaN, 'r-', 'LineWidth', 1.5); hold on; % 轨迹
h_in_sc = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4); % 航天器
h_in_e = plot(NaN, NaN, 'b.', 'MarkerSize', 20); % 地球
h_in_m = plot(NaN, NaN, 'c.', 'MarkerSize', 10); % 月球
axis equal; grid on;
xlim([-1.5 1.5]); ylim([-1.5 1.5]);
title('惯性坐标系 (Inertial)');
xlabel('X'); ylabel('Y');

% 旋转系绘图句柄
subplot(1, 2, 2);
h_rot_traj = plot(NaN, NaN, 'b-', 'LineWidth', 1.5); hold on;
h_rot_sc = plot(NaN, NaN, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 4); % 航天器
% 旋转系中天体位置固定
plot(-mu, 0, 'b.', 'MarkerSize', 20); text(-mu, -0.1, 'Earth');
plot(1-mu, 0, 'c.', 'MarkerSize', 10); text(1-mu, -0.1, 'Moon');
axis equal; grid on;
xlim([-1.5 1.5]); ylim([-1.5 1.5]);
title('旋转坐标系 (Rotating)');
xlabel('x'); ylabel('y');

% --- 动画循环 ---
for i = 1:length(t)
    % 1. 更新惯性系数据
    set(h_in_traj, 'XData', x_in(1:i), 'YData', y_in(1:i));
    set(h_in_sc, 'XData', x_in(i), 'YData', y_in(i));
    
    % 计算惯性系下地球和月球的位置（随时间旋转）
    E_pos = [-mu*cos(t(i)); -mu*sin(t(i))];
    M_pos = [(1-mu)*cos(t(i)); (1-mu)*sin(t(i))];
    set(h_in_e, 'XData', E_pos(1), 'YData', E_pos(2));
    set(h_in_m, 'XData', M_pos(1), 'YData', M_pos(2));
    
    % 2. 更新旋转系数据
    set(h_rot_traj, 'XData', x_rot(1:i), 'YData', y_rot(1:i));
    set(h_rot_sc, 'XData', x_rot(i), 'YData', y_rot(i));
    
    drawnow limitrate;
    pause(0.01); 
end

function dY = cr3bp_func_full(~, Y, mu)
    x = Y(1); y = Y(2); vx = Y(3); vy = Y(4);
    r1 = sqrt((x+mu)^2 + y^2);
    r2 = sqrt((x-(1-mu))^2 + y^2);
    ax = 2*vy + x - (1-mu)*(x+mu)/r1^3 - mu*(x-(1-mu))/r2^3;
    ay = -2*vx + y - (1-mu)*y/r1^3 - mu*y/r2^3;
    dY = [vx; vy; ax; ay];
end