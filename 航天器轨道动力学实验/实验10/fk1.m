%% 复刻图 13.1 & 13.2：惯性系与旋转系轨迹对比
clc; clear; close all;

% --- 1. 参数设置 ---
mu = 0.01215;         % 地月系统质量比
t_span = [0, 25];     % 仿真时间足够长以形成闭合花瓣
% 选取一个能产生类似图13.1(b)花瓣效果的初值
Y0 = [0.85; 0; 0; 0.6]; 

% --- 2. 积分计算 ---
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);
[t, Y] = ode45(@(t,y) cr3bp_func(t,y,mu), t_span, Y0, options);

% --- 3. 坐标变换 ---
x_rot = Y(:,1); y_rot = Y(:,2);
% 旋转系 -> 惯性系变换
x_in = x_rot .* cos(t) - y_rot .* sin(t);
y_in = x_rot .* sin(t) + y_rot .* cos(t);

% --- 4. 绘图复刻 ---
figure('Name', '复刻图13.1与13.2', 'Color', 'w', 'Position', [100, 100, 1000, 450]);

% 左图：复刻图 13.2 (旋转系)
subplot(1, 2, 1);
plot(x_rot, y_rot, 'b-', 'LineWidth', 1.5); hold on;
p_earth = plot(-mu, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); 
p_moon = plot(1-mu, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'c');
axis equal; grid on; box on;
xlim([-1.2 1.2]); ylim([-1.2 1.2]);
title('图 13.2 旋转坐标系下轨迹 (Rotating)');
legend([p_earth, p_moon], {'Earth (地球)', 'Moon (月球)'}, 'Location', 'best');
xlabel('x (\xi)'); ylabel('y (\eta)');

% 右图：复刻图 13.1 (惯性系)
subplot(1, 2, 2);
plot(x_in, y_in, 'k-', 'LineWidth', 1.2); hold on; % 使用黑色线条复刻书本风格
plot(0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 2);
axis equal; grid on; box on;
title('图 13.1 惯性坐标系下轨迹 (Inertial)');
legend({'Spacecraft Trajectory (航天器轨迹)', 'Barycenter (质心)'}, 'Location', 'northeast');
xlabel('X'); ylabel('Y');

% 动力学方程
function dY = cr3bp_func(~, Y, mu)
    x = Y(1); y = Y(2); vx = Y(3); vy = Y(4);
    r1 = sqrt((x+mu)^2 + y^2);
    r2 = sqrt((x-(1-mu))^2 + y^2);
    ax = 2*vy + x - (1-mu)*(x+mu)/r1^3 - mu*(x-(1-mu))/r2^3;
    ay = -2*vx + y - (1-mu)*y/r1^3 - mu*y/r2^3;
    dY = [vx; vy; ax; ay];
end