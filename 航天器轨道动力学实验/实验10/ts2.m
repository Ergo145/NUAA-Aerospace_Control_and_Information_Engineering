%% 实验内容2：动力学方程各项物理意义仿真分析
clc; clear; close all;

mu = 0.01215;
t_span = [0, 10];
Y0_base = [0.5; 0.5; 0.1; -0.1]; % 基础初始条件

figure('Name', '实验内容2：各项力单独作用效果', 'Color', 'w');

% --- 1. 仅受科里奥利力 (Coriolis Force Only) ---
% 方程: x'' = 2y', y'' = -2x'
[t1, Y1] = ode45(@(t,y) [y(3); y(4); 2*y(4); -2*y(3)], t_span, Y0_base);

subplot(1, 3, 1);
plot(Y1(:,1), Y1(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(Y1(1,1), Y1(1,2), 'go', 'MarkerFaceColor', 'g'); % 起点
title({'仅受科里奥利力';'(特征：改变方向不改速度，圆弧)'});
axis equal; grid on; xlabel('x'); ylabel('y');

% --- 2. 仅受离心力 (Centrifugal Force Only) ---
% 方程: x'' = x, y'' = y
[t2, Y2] = ode45(@(t,y) [y(3); y(4); y(1); y(2)], [0, 2], Y0_base); % 时间缩短防止发散过远

subplot(1, 3, 2);
plot(Y2(:,1), Y2(:,2), 'r-', 'LineWidth', 1.5); hold on;
plot(Y2(1,1), Y2(1,2), 'go', 'MarkerFaceColor', 'g'); % 起点
title({'仅受离心力';'(特征：沿径向向外发散)'});
axis equal; grid on; xlabel('x'); ylabel('y');

% --- 3. 仅受引力 (Gravitational Force Only) ---
% 方程: 去掉科氏力和离心力项
[t3, Y3] = ode45(@(t,y) gravity_only(t,y,mu), t_span, Y0_base);

subplot(1, 3, 3);
plot(Y3(:,1), Y3(:,2), 'k-', 'LineWidth', 1.5); hold on;
plot(-mu, 0, 'bo', 'MarkerFaceColor', 'b'); % 地球
plot(1-mu, 0, 'co', 'MarkerFaceColor', 'c'); % 月球
title({'仅受双星引力';'(特征：类似二体/三体引力运动)'});
axis equal; grid on; xlabel('x'); ylabel('y');

function dY = gravity_only(~, Y, mu)
    x = Y(1); y = Y(2); vx = Y(3); vy = Y(4);
    r1 = sqrt((x+mu)^2 + y^2);
    r2 = sqrt((x-(1-mu))^2 + y^2);
    % 仅保留引力项
    ax = -(1-mu)*(x+mu)/r1^3 - mu*(x-(1-mu))/r2^3;
    ay = -(1-mu)*y/r1^3 - mu*y/r2^3;
    dY = [vx; vy; ax; ay];
end