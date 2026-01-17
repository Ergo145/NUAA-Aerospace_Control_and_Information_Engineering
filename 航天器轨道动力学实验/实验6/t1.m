%% Task1_Numerical_Integration.m
% 实验内容 (1): 利用MATLAB解微分方程求解卫星运行轨迹，画出三维轨迹图

clc; clear; close all;

% 1. 物理常数
mu = 3.986004418e14; % 地球引力常数 m^3/s^2

% 2. 初始状态 (来自PDF Page 135 Eq 8.38)
r0 = [-6311227.13644808; -1112839.6255322; 3700000]; % 位置 (m)
v0 = [1274.45143292937; -7227.77323794544; 2.2471043881515e-13]; % 速度 (m/s)
y0 = [r0; v0]; % 状态向量

% 3. 计算轨道周期 (用于确定积分时间)
r_mag = norm(r0);
v_mag = norm(v0);
energy = v_mag^2/2 - mu/r_mag; % 活力公式
a = -mu / (2*energy); % 半长轴
T = 2*pi * sqrt(a^3/mu); % 周期 (s)

fprintf('轨道半长轴 a = %.2f km\n', a/1000);
fprintf('轨道周期 T = %.2f s\n', T);

% 4. 数值积分 (ode45)
tspan = [0, T]; % 积分一个周期
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
[t, y] = ode45(@(t,y) TwoBodyODE(t, y, mu), tspan, y0, options);

% 5. 绘图
figure('Color', 'w');
plot3(y(:,1), y(:,2), y(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % 地心
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Task 1: 主星三维运行轨迹 (数值积分法)');
legend('卫星轨迹', '地心');
view(3);

% --- 内部函数: 二体微分方程 ---
function dydt = TwoBodyODE(~, y, mu)
    r = y(1:3);
    v = y(4:6);
    r_norm = norm(r);
    
    drdt = v;
    dvdt = -mu / r_norm^3 * r;
    
    dydt = [drdt; dvdt];
end